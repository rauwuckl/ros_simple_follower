#!/usr/bin/env python

from __future__ import division
import rospy
import message_filters
import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge 

from sensor_msgs.msg import Image
from simple_follower.msg import position as PositionMsg
from std_msgs.msg import String as StringMsg
np.seterr(all='raise')  
displayImage=False

plt.close('all')
class visualTracker:
	def __init__(self):
		self.bridge = CvBridge()
		self.targetUpper = np.array(rospy.get_param('~target/upper'))
		self.targetLower = np.array(rospy.get_param('~target/lower'))
		self.pictureHeight= rospy.get_param('~pictureDimensions/pictureHeight')
		self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth')
		vertAngle =rospy.get_param('~pictureDimensions/verticalAngle')
		horizontalAngle =  rospy.get_param('~pictureDimensions/horizontalAngle')
		# precompute tangens since thats all we need anyways:
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)
		
		self.lastPosition =None

		# one callback that deals with depth and rgb at the same time
		im_sub = message_filters.Subscriber('/orbbec_astra/rgb/image_rect_color', Image)
		dep_sub = message_filters.Subscriber('/orbbec_astra/depth/image', Image)
		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
		self.timeSynchronizer.registerCallback(self.trackObject)


		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
		self.infoPublisher = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)
		rospy.logwarn(self.targetUpper)


	def trackObject(self, image_data, depth_data):
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')

		#convert both images to numpy arrays
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")

		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))

		# blure a little and convert to HSV color space
		blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
		
		# select all the pixels that are in the range specified by the target
		org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)	

		# clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=4)
		mask = cv2.dilate(mask,None, iterations=3)
		
		# find contours of the object
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		newPos = None #if no contour at all was found the last position will again be set to none

		# lets you display the image for debuging. Not in realtime though
		if displayImage:
			backConverted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
			#cv2.imshow('frame', backConverted)
			#cv2.waitKey(0)
			#print(backConverted)
			plt.figure()
			plt.subplot(2,2,1)
			plt.imshow(frame)
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,2)
			plt.imshow(org_mask, cmap='gray', interpolation = 'bicubic')
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,3)
			plt.imshow(mask, cmap='gray', interpolation = 'bicubic')
			plt.xticks([]),plt.yticks([])
			plt.show()
			rospy.sleep(0.2)

		# go threw all the contours. starting with the bigest one
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			# get position of object for this contour
		 	pos = self.analyseContour(contour, depthFrame)

			# if it's the first one we found it will be the fall back for the next scan if we don't find a plausible one
			if newPos is None:
				newPos = pos
			
			# check if the position is plausible
			if self.checkPosPlausible(pos):
				self.lastPosition = pos
				self.publishPosition(pos)
				return

		self.lastPosition = newPos #we didn't find a plossible last position, so we just save the biggest contour 
		# and publish warnings
		rospy.loginfo('no position found')
		self.infoPublisher.publish(StringMsg('visual:nothing found'))

			
			
			

	def publishPosition(self, pos):
		# calculate the angles from the raw position
		angleX = self.calculateAngleX(pos)
		angleY = self.calculateAngleY(pos)
		# publish the position (angleX, angleY, distance)
		posMsg = PositionMsg(angleX, angleY, pos[1])
		self.positionPublisher.publish(posMsg)

	def checkPosPlausible(self, pos):
		'''Checks if a position is plausible. i.e. close enough to the last one.'''

		# for the first scan we cant tell
		if self.lastPosition is None:
			return False

		# unpack positions
		((centerX, centerY), dist)=pos	
		((PcenterX, PcenterY), Pdist)=self.lastPosition
		
		if np.isnan(dist):
			return False

		# distance changed to much
		if abs(dist-Pdist)>0.5:
			return False

		# location changed to much (5 is arbitrary)
		if abs(centerX-PcenterX)>(self.pictureWidth /5):
			return False

		if abs(centerY-PcenterY)>(self.pictureHeight/5):
			return False
		
		return True
				
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''

		centerX = pos[0][0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		centerY = pos[0][1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle

	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour

		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object
		'''
		# get a rectangle that completely contains the object
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# get the center of that rounded to ints (so we can index the image)
		center = np.round(centerRaw).astype(int)

		# find out how far we can go in x/y direction without leaving the object (min of the extension of the bounding rectangle/2 (here 3 for safety)) 
		minSize = int(min(size)/3)

		# get all the depth points within this area (that is within the object)
		depthObject = depthFrame[(center[1]-minSize):(center[1]+minSize), (center[0]-minSize):(center[0]+minSize)]

		# get the average of all valid points (average to have a more reliable distance measure)
		depthArray = depthObject[~np.isnan(depthObject)]
		averageDistance = np.mean(depthArray)

		if len(depthArray) == 0:
			rospy.logwarn('empty depth array. all depth values are nan')

		return (centerRaw, averageDistance)
		

if __name__ == '__main__':
        print('starting')
        rospy.init_node('visual_tracker')
	tracker=visualTracker()
        print('seems to do something')
        try:
                rospy.spin()
        except rospy.ROSInterruptException:
                print('exception')

