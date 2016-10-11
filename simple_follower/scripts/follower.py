#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from first_experiments.msg import position as PositionMsg
from std_msgs.msg import String as StringMsg

class Follower:
	def __init__(self):
		# as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
		self.controllerLossTimer.start()
		self.switchMode= rospy.get_param('~switchMode') # if this is set to False the O button has to be kept pressed in order for it to move
		self.max_speed = rospy.get_param('~maxSpeed') 
		self.controllButtonIndex = rospy.get_param('~controllButtonIndex')

		self.buttonCallbackBusy=False
		self.active=False

		self.cmdVelPublisher = rospy.Publisher('/cmd_vel/yolo', Twist, queue_size =3)
		# the topic for the messages from the ps3 controller (game pad)
		self.joySubscriber = rospy.Subscriber('joy', Joy, self.buttonCallback)

		# the topic for the tracker that gives us the current position of the object we are following
		self.positionSubscriber = rospy.Subscriber('/object_tracker/current_position', PositionMsg, self.positionUpdateCallback)
		# an info string from that tracker. E.g. telling us if we lost the object
		self.trackerInfoSubscriber = rospy.Subscriber('/object_tracker/info', StringMsg, self.trackerInfoCallback)

		# PID parameters first is angular, dist
		targetDist = rospy.get_param('~targetDist')
		PID_param = rospy.get_param('~PID_controller')
		# the first parameter is the angular target (0 degrees always) the second is the target distance (say 1 meter)
		self.PID_controller = simplePID([0, targetDist], PID_param['P'], PID_param['I'], PID_param['D'])

		# this method gets called when the process is killed with Ctrl+C
		rospy.on_shutdown(self.controllerLoss)

	def trackerInfoCallback(self, info):
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		rospy.logwarn(info.data)
	
	def positionUpdateCallback(self, position):
		# gets called whenever we receive a new position. It will then update the motorcomand
	
		if(not(self.active)):
			return #if we are not active we will return imediatly without doing anything

		angleX= position.angleX
		distance = position.distance

		rospy.loginfo('Angle: {}, Distance: {}, '.format(angleX, distance))
		
		# call the PID controller to update it and get new speeds
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
			
		# clip these speeds to be less then the maximal speed specified above
		angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
		linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)
		
		# create the Twist message to send to the cmd_vel topic
		velocity = Twist()	
		velocity.linear = Vector3(linearSpeed,0,0.)
		velocity.angular= Vector3(0., 0.,angularSpeed)
		rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
		self.cmdVelPublisher.publish(velocity)
		

	def buttonCallback(self, joy_data):
		# this method gets called whenever we receive a message from the joy stick

		# there is a timer that always gets reset if we have a new joy stick message
		# if it runs out we know that we have lost connection and the controllerLoss function
		# will be called
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()

		# if we are in switch mode, one button press will make the follower active / inactive 
		# but 'one' button press will be visible in roughly 10 joy messages (since they get published to fast) 
		# so we need to drop the remaining 9
		
		if self.buttonCallbackBusy:
			# we are busy with dealing with the last message
			return 
		else:
			# we are not busy. i.e. there is a real 'new' button press
			# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean
			# time
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))

	def threadedButtonCallback(self, joy_data):
		self.buttonCallbackBusy = True

		if(joy_data.buttons[self.controllButtonIndex]==self.switchMode and self.active):
			# we are active
			# switchMode = false: we will always be inactive whenever the button is not pressed (buttons[index]==false)
			# switchMode = true: we will only become inactive if we press the button. (if we keep pressing it, 
			# we would alternate between active and not in 0.5 second intervalls)
			rospy.loginfo('stoping')
			self.stopMoving()
			self.active = False
			rospy.sleep(0.5)
		elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)):
			# if we are not active and just pressed the button (or are constantly pressing it) we become active
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
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		self.active = False
		rospy.loginfo('lost connection')


		
class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):
		'''Create a discrete PID controller
		each of the parameters may be a vector if they have the same length
		
		Args:
		target (double) -- the target value(s)
		P, I, D (double)-- the PID parameter

		'''

		# check if parameter shapes are compatabile. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 
		
		
	def update(self, current_value):
		'''Updates the PID controller. 

		Args:
			current_value (double): vector/number of same legth as the target given in the constructor

		Returns:
			controll signal (double): vector of same length as the target

		'''
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))

		
		error = self.setPoint - current_value
		P =  error
		
		currentTime = time.clock()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D
		
		
	

			




if __name__ == '__main__':
	print('starting')
	rospy.init_node('follower')
	follower = Follower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')


