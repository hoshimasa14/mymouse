#!/usr/bin/env python3

import time
import smbus
import amd01

def s16(value):
	return -(value & 0x8000) | (value & 0x7FFF)

class AMD01_IMU(amd01.AMD01):
	def __init__(self, addr=0x68, ax_addr = 0x3B):
		# I2C interface
		self._imu_addr = addr
		self._ax_addr = ax_addr
		self._bus = smbus.SMBus(1)
		self._bus.write_i2c_block_data(self._imu_addr, 0x75, [0])
		self._bus.write_i2c_block_data(self._imu_addr, 0x6B, [0])
		time.sleep(0.1)
		if( self._bus ):
			print( 'IMU_I2C bus available' )
		else:
			print( 'No IMU_I2C bus ' )

	def get_imu_status(self):
		print("get_imu_status")

		buf = self._bus.read_i2c_block_data(self._imu_addr, self._ax_addr, 14)

		axRaw = s16((buf[0] << 8) | (buf[1] << 0))
		ayRaw = s16((buf[2] << 8) | (buf[3] << 0))
		azRaw = s16((buf[4] << 8) | (buf[5] << 0))
		Temperture = (buf[6] << 8) | (buf[7] << 0)
		gxRaw = s16((buf[8] << 8) | (buf[9] << 0))
		gyRaw = s16((buf[10] << 8) | (buf[11] << 0))
		gzRaw = s16((buf[12] << 8) | (buf[13] << 0))

		self.ax = axRaw / 16384.0
		self.ay = ayRaw / 16384.0
		self.az = azRaw / 16384.0
		self.gx = gxRaw / 131.0
		self.gy = gyRaw / 131.0
		self.gz = gzRaw / 131.0

def imu_test():
	m = AMD01_IMU()
	while(1):
		m.get_imu_status()
		print(int(m.gx), int(m.gy), int(m.gz))
		time.sleep(1)


if __name__ == '__main__':
	imu_test()
#	amd01.motor_test()
