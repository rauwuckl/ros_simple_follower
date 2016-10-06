#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3

switchMode=False # if this is set to False the O button has to be kept pressed in order for it to move
scansToAverageOver = 2 #how many scans we average from to get the closest object
tolleratedDistance = 2 # disance from which it start running away
max_speed = 0.55 # float('inf')


# make it Safe: max_speed, switch mode, self.active, controllerLossTimer.start

class Runaway:
	def __init__(self):
		self.cmdVelPublisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size =3)
		self.joySubscriber = rospy.Subscriber('joy', Joy, self.buttonCallback)
		self.scanSubscriber = rospy.Subscriber('/hokuyo_base/scan', LaserScan, self.scanCallback)
		self.active=False
		self.buttonCallbackBusy=False
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
		self.controllerLossTimer.start()
		self.tracker = simpleTracker()
		self.PID_controller = simplePID([0, 1.0], [1.0,0.9], [0.09,0.06], [0.00, 0.000])
		#this worked in simulation without weird reallive omni controll
		#self.PID_controller = simplePID([0, 1.0], [1.5,1.5], [0.001,1.1], [0.001, 0.001])
		#first one shall be anle=0 second distance=0.8
		rospy.on_shutdown(self.controllerLoss)
	
	def scanCallback(self, scan_data):
		if(not(self.active)):
			return #if we are not active we will return imediatly without doing anything

		#ranges = np.array(scan_data.ranges)
		try:
			angle, distance = self.tracker.registerScan(scan_data)	
		except UserWarning: 
			#this means we haven't found a valid closest point
			self.stopMoving()
			return
		

		#self.lastAngles.append(minDistanceAngle)
		#self.lastDistances.append(ranges[minDistanceID])

		
		rospy.loginfo('Angle: {}, Distance: {}, '.format(angle, distance))
		
		#if((len(self.lastDistances)<scansToAverageOver) and (len(self.lastAngles)<scansToAverageOver)):
		#	return #we don't have enough data yet to averate

		#if((len(self.lastDistances) != len(self.lastAngles))):	
		#	rospy.logwarn('NOT THE SAME LENGTH: length of lastDistance: {}, lesngth of lastRanges: {}'.format(len(self.lastDistances), len(self.lastAngles)))

		#if(len(self.lastAngles)>scansToAverageOver):
		#	_ = self.lastAngles.pop(0)
		#if(len(self.lastDistances)>scansToAverageOver):
			#_ = self.lastDistances.pop(0)

 				
		
		#/scan_data.scan_time
		
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle, distance])
			
		angularSpeed = np.clip(-uncliped_ang_speed, -max_speed, max_speed)
		linearSpeed  = np.clip(-uncliped_lin_speed, -max_speed, max_speed)
		#linearSpeed = 0
		
		velocity = Twist()	
		#velocity.linear = Vector3(linearSpeed,0.,0.)
		#velocity.angular= Vector3(0., 0.,angularSpeed)
		velocity.linear = Vector3(linearSpeed,angularSpeed,0.)
		velocity.angular= Vector3(0., 0.,0)
		rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
		self.cmdVelPublisher.publish(velocity)
		

	def buttonCallback(self, joy_data):
		# set timer and cancel it. if it runs out stop everything
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()
		
		
		if self.buttonCallbackBusy:
			return 
		else:
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))

	def threadedButtonCallback(self, joy_data):
		self.buttonCallbackBusy = True

		if(joy_data.buttons[-4]==switchMode and self.active):
			rospy.loginfo('stoping')
			self.stopMoving()
			self.active = False
			rospy.sleep(0.5)
		elif(joy_data.buttons[-4]==True and not(self.active)):
			rospy.loginfo('activating')
			self.active = True #enable response
			rospy.sleep(0.5)

		self.buttonCallbackBusy = False

	def stopMoving(self):
		velocity = Twist()
		velocity.linear = Vector3(0.,0.,0.)
		velocity.angular= Vector3(0.,0.,0.)
		self.cmdVelPublisher.publish(velocity)

	def controllerLoss(self):
		self.stopMoving()
		self.active = False
		rospy.loginfo('lost connection')


class simpleTracker:
	def __init__(self):
		self.lastScan=None
		self.winSize = 1
		self.deltaDist = 0.2

	def registerScan(self, scan_data):
		ranges = np.array(scan_data.ranges)
		sortedIndices = np.argsort(ranges)
		
		minDistanceID = None
		minDistance   = float('inf')		

		if(not(self.lastScan is None)):
			print('after if')
			for i in sortedIndices:
				tempMinDistance   = ranges[i]
				
				# clip to not overstep bounds
				# get a window. in it we will check if there has been a scan with similar distance
				# in the last scan within that window
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1],0,len(self.lastScan))
				window = self.lastScan[windowIndex[0]:windowIndex[1]]
				#print('last scan: {}'.format(abs(window-tempMinDistance)))
				with np.errstate(invalid='ignore'):
					if(np.any(abs(window-tempMinDistance)<=self.deltaDist)):
					# this will also be false for all tempMinDistance = NaN/inf
						minDistanceID = i
						minDistance = ranges[minDistanceID]
						break # at least one point was equally close
						# so we found a valid minimum and can stop the loop
			
			#how to deal with corners
		self.lastScan=ranges	
		
		#catches no scan, no minimum found, minimum is actually inf
		if(minDistance > scan_data.range_max):
			minDistance = scan_data.range_max
			minDistanceAngle=0
			#self.stopMoving() #this has to go
			print('nothing found')
			raise UserWarning('no minimum distance found')
		else:
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
		print('{}, {}'.format(minDistanceAngle, minDistance))

		return minDistanceAngle, minDistance	
		
class simplePID:
	def __init__(self, target, P, I, D):
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		#self.integrationTimeWindow=50
		self.last_error=0
		self.integrator = 0#[]
		self.integrator_max = float('inf')
		self.timeOfLastCall = None #thats gona be far to big for the first call
		
		
	def update(self, current_value):
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))

		error = self.setPoint - current_value
		
		P =  error
		
		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		#self.integrator.append(error*deltaT)
		#if(len(self.integrator)>=self.integrationTimeWindow):
		#	_ = self.integrator.pop(0)

		#I = np.sum(self.integrator)

		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		D = (error-self.last_error)/deltaT
		#requires that this function is called at fixed time intervalls
	#	print('P:{}, I:{}, D:{}'.format(P,self.integrator,D))
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		return self.Kp*P + self.Ki*I + self.Kd*D
		
		
	

			




if __name__ == '__main__':
	print('starting')
	rospy.init_node('runaway')
	runaway = Runaway()
	print('seems to do something')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


