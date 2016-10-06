import numpy as np
import copy
from unsafe_runaway import *
import time
from nose.tools import assert_raises




class mockup:
	pass
	

def makeLaserData():
	laser_data = mockup()
	laser_data.range_max = 5.6
	laser_data.angle_min = -2.1
	laser_data.angle_increment = 0.06136
	laser_data.ranges = list(1+np.random.rand(69)*5)
	return laser_data

class Test_simpleTracker:

	
	def setUp(self):
		self.tracker = simpleTracker() 
	
	def test_ignor_first_scan(self):
		laser_data = makeLaserData()
		tracker = simpleTracker()
		assert_raises(UserWarning, self.tracker.registerScan,laser_data)

	def test_unmuted(self):
		laser_data = makeLaserData()
		backup = copy.copy(laser_data)
		try:
			angle, distance = self.tracker.registerScan(laser_data)
		except:
			pass
		assert backup.ranges == laser_data.ranges
		#print(laser_data.ranges)
		#print('angle: {}, dist: {}'.format(angle, distance))	

	def test_nan(self):
		laser_data = makeLaserData()
		assert_raises(UserWarning, self.tracker.registerScan,laser_data)
		laser_data.ranges[12] = float('nan')
		angle, dist=self.tracker.registerScan(laser_data)
		#print('angle: {}, dist: {}'.format(angle, dist))	

	def test_only_nan(self):
		laser_data = makeLaserData()
		laser_data.ranges = [float('nan') for _ in laser_data.ranges]
		assert_raises(UserWarning, self.tracker.registerScan,laser_data)
		assert_raises(UserWarning, self.tracker.registerScan,laser_data)

		

	def test_real_real_min(self):
		laser_data = makeLaserData()
		laser_data.ranges[-1]=0.5 #real min
		assert_raises(UserWarning, self.tracker.registerScan,laser_data)
		laser_data.ranges[-1]=0.6 
		laser_data.ranges[42]=0.1 #fake min
		ang, dist = self.tracker.registerScan(laser_data)
		assert dist == 0.6
		#print('ang: {}, target: {}'.format(ang, (laser_data.angle_min+ 23*laser_data.angle_increment)))	
		assert ang == laser_data.angle_min+ 68*laser_data.angle_increment


class Test_PID:
	def setUp(self): 
		pass

	def test_convergence(self):
		self.pid = simplePID([0,30], 0.8, 0.001, 0.0001)
		x =np.array([23, 12])
		for i in range(20):
			update= self.pid.update(x)
			print('added {} to current x {}'.format(update, x))
			x = x+update 
			time.sleep(0.1)
		assert np.all(abs(x-[0,30])<=0.01)

	def test_convergence_differentParamShape(self):
		self.pid = simplePID([0,30],0.8, 0.001, 0.0001)
		x =np.array([23, 12])
		for i in range(20):
			update= self.pid.update(x)
			print('added {} to current x {}'.format(update, x))
			x = x+update 
			time.sleep(0.1)
		assert np.all(abs(x-[0,30])<=0.01)

	 
	def test_raises_unequal_param_shape_at_creation(self):
		assert_raises(TypeError, simplePID, [0,30],[0.8, 0.7, 0.1], 0.001, 0.0001)
		assert_raises(TypeError, simplePID, [0,30],[0.8, 0.7], 0.001, 0.0001)
		assert_raises(TypeError, simplePID, 0,[0.8, 0.7], 0.001, 0.0001)
		assert_raises(TypeError, simplePID, 0, [0.8, 0.7], [0.001, 0.001], [0.0001, 0,0001])
		_ =  simplePID([0,30],[0.8, 0.7], [0.001, 0.001], [0.0001, 0.0001])
		_ =  simplePID([0,30],0.8, 0.001, 0.0001)
		_ =  simplePID(0,0.8, 0.001, 0.0001)

	def test_raise_incompatable_input(self):
		self.pid = simplePID([0,30], 0.8, 0.001, 0.0001)
		_ = assert_raises(TypeError, self.pid.update, 3)
		x =np.array([23, 12])
		for i in range(50):
			update= self.pid.update(x)
			print('added {} to current x {}'.format(update, x))
			x = x+update 
			time.sleep(0.1)
		assert np.all(abs(x-[0,30])<=0.001)
