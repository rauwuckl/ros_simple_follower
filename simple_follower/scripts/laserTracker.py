#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from first_experiments.msg import position as PositionMsg
		
class laserTracker:
	def __init__(self):
		self.lastScan=None
		self.winSize = rospy.get_param('~winSize')
		self.deltaDist = rospy.get_param('~deltaDist')
		self.scanSubscriber = rospy.Subscriber('/hokuyo_base/scan', LaserScan, self.registerScan)
		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg,queue_size=3)
		self.infoPublisher = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)

	def registerScan(self, scan_data):
		# registers laser scan and publishes position of closest object (or point rather)
		ranges = np.array(scan_data.ranges)
		# sort by distance to check from closer to further away points if they might be something real
		sortedIndices = np.argsort(ranges)
		
		minDistanceID = None
		minDistance   = float('inf')		

		if(not(self.lastScan is None)):
			# if we already have a last scan to compare to:
			for i in sortedIndices:
				# check all distance measurements starting from the closest one
				tempMinDistance   = ranges[i]
				
				# now we check if this might be noise:
				# get a window. in it we will check if there has been a scan with similar distance
				# in the last scan within that window
				
				# we kneed to clip the window so we don't have an index out of bounds
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1],0,len(self.lastScan))
				window = self.lastScan[windowIndex[0]:windowIndex[1]]

				with np.errstate(invalid='ignore'):
					# check if any of the scans in the window (in the last scan) has a distance close enough to the current one
					if(np.any(abs(window-tempMinDistance)<=self.deltaDist)):
					# this will also be false for all tempMinDistance = NaN or inf

						# we found a plausible distance
						minDistanceID = i
						minDistance = ranges[minDistanceID]
						break # at least one point was equally close
						# so we found a valid minimum and can stop the loop
			
		self.lastScan=ranges	
		
		#catches no scan, no minimum found, minimum is actually inf
		if(minDistance > scan_data.range_max):
			#means we did not really find a plausible object
			
			# publish warning that we did not find anything
			rospy.logwarn('laser no object found')
			self.infoPublisher.publish(StringMsg('laser:nothing found'))
			
		else:
			# calculate angle of the objects location. 0 is straight ahead
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
			# here we only have an x angle, so the y is set arbitrarily
			self.positionPublisher.publish(PositionMsg(minDistanceAngle, 42, minDistance))
			



if __name__ == '__main__':
	print('starting')
	rospy.init_node('laser_tracker')
	tracker = laserTracker()
	print('seems to do something')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


