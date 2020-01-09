#!/usr/bin/env python3

import time
import smbus
from scipy import integrate

def s16(value):		#符号無し16進数 to 符号有り16進数
	return -(value & 0x8000) | (value & 0x7FFF)

class ICM():	#class of ICM_20689
	def __init__(self):
		# I2C interface
		self._imu_addr = 0x68
		self._ax_addr = 0x3B
		self._bus = smbus.SMBus(1)
		self._bus.write_i2c_block_data(self._imu_addr, 0x75, [0])
		self._bus.write_i2c_block_data(self._imu_addr, 0x6B, [0])

		time.sleep(0.1)
		if( self._bus ):
			print( 'IMU_I2C bus available' )
		else:
			print( 'No IMU_I2C bus ' )

		self.time = time.time()
		self.alpha = 0.8	#lowpassfilter
		self.gravityx = 0
		self.gravityy = 0
		self.gravityz = 0

	def get_imu_status(self):
		read_imu()
		cal_acc_gyr()
		cal_vel_comp()
#		cal_loc()

	def read_imu(self):
		buf = self._bus.read_i2c_block_data(self._imu_addr, self._ax_addr, 14)

		self.rawax = s16((buf[0] << 8) | (buf[1] << 0))
		self.raway = s16((buf[2] << 8) | (buf[3] << 0))
		self.rawaz = s16((buf[4] << 8) | (buf[5] << 0))
		Temperture = (buf[6] << 8) | (buf[7] << 0)
		self.rawgx = s16((buf[8] << 8) | (buf[9] << 0))
		self.rawgy = s16((buf[10] << 8) | (buf[11] << 0))
		self.rawgz = s16((buf[12] << 8) | (buf[13] << 0))

	def cal_acc_gyr(self):
		self.ax = self.rawax / 16384.0
		self.ay = self.raway / 16384.0
		self.az = self.rawaz / 16384.0
		self.gx = self.rawgx / 131.0
		self.gy = self.rawgy / 131.0
		self.gz = self.rawaz / 131.0

		self.gravityx = alpha * self.gravityx + (1 - alpha) * self.preax;
		self.gravityy = alpha * self.gravityy + (1 - alpha) * self.preay;
		self.gravityz = alpha * self.gravityz + (1 - alpha) * self.preaz;

		self.comax = self.ax - self.gravityx
		self.comay = self.ay - self.gravityy
		self.comaz = self.az - self.gravityz

	def cal_vel_comp(self):
		self.dt = self.time - time.time()
		self.time = time.time()

		self.vx = vx + comax * dt
		self.vy = vy + comay * dt
		self.vz = vz + comaz * dt

		self.cx
		self.cy
		self.cz



def test_imu():
	o = ICM()
	for i in range(20):
		o.get_imu_status()
		print(int(o.ax), int(o.ay), int(o.az), int(o.gx), int(o.gy), int(o.gz))
		print(int(o.vx), int(o.vy), int(o.vz), int(o.cx), int(o.cy), int(o.cz))
		print(int(o.lx), int(o.ly), int(o.lz))
		time.sleep(1)



if __name__ == '__main__':
	test_imu()
#	amd01.motor_test()
