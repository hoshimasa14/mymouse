#!/usr/bin/env python3

import time
import smbus
import ctypes

# structure
class st_amd01_status(ctypes.Structure):
    _fields_ = [ ('control_state', ctypes.c_uint16),
    ('m1_ref_speed',  ctypes.c_int16 ),
    ('m2_ref_speed',  ctypes.c_int16 ),
    ('m1_encoder',    ctypes.c_int16 ),
    ('m2_encoder',    ctypes.c_int16 ),
    ('m1_speed',      ctypes.c_int16 ),
    ('m2_speed',      ctypes.c_int16 ),
    ('m1_current',    ctypes.c_uint16),
    ('m2_current',    ctypes.c_uint16),
    ('m1_duty',       ctypes.c_int16 ),
    ('m2_duty',       ctypes.c_int16 ),
    ('batt_vol',      ctypes.c_uint16),
    ('pid_kp',        ctypes.c_uint16),
    ('pid_ki',        ctypes.c_uint16),
    ('gear_rate',     ctypes.c_uint8 ),
    ('encoder_cpr',   ctypes.c_uint8 ),
    ('current_limit', ctypes.c_uint16),
    ('timeout_sec',   ctypes.c_uint8 ),
    ('firm_version',  ctypes.c_uint8 ),
    ('dummy',         ctypes.c_uint8 * 30) ]

class amd01_status(ctypes.Union):   #unionを使うことで64byteのbuffer情報を各status変数に割り振る
    _fields_ = [ ('buf', ctypes.c_uint8 * 64), ('reg', st_amd01_status) ]

class AMD01(object):
	def __init__(self):
		# I2C interface
		self._addr = 0x58
		self._bus = smbus.SMBus(1)
		time.sleep(0.1)
		if( self._bus ):
			print( 'I2C bus available' )
		else:
			print( 'No I2C bus ' )

		# status
		self.status = amd01_status()

	def get_state(self):
		buf = []
#		self._bus.write_i2c_block_data(self._addr, 0x80, [0x40])
		buf = self._bus.read_i2c_block_data( self._addr,  0, 64 )

		for i in range(len(buf)):
			self.status.buf[i] = buf[i]
		return buf

	def control_state( self, ctrl ):
		buf = [ (ctrl>>0)&0xFF, (ctrl>>8)&0xFF ]
		self._bus.write_i2c_block_data( self._addr, 0x00, buf )

	def gain( self, kp, ki ):
		buf = [ (kp>>0)&0xFF, (kp>>8)&0xFF, (ki>>0)&0xFF, (ki>>8)&0xFF ]
		self._bus.write_i2c_block_data( self._addr, 0x18, buf )

	def drive(self, m1_rpm, m2_rpm):
		buf = [ (m1_rpm>>0)&0xFF, (m1_rpm>>8)&0xFF, (m2_rpm>>0)&0xFF, (m2_rpm>>8)&0xFF ]
		self._bus.write_i2c_block_data( self._addr, 0x02, buf )

	def stop(self):
		self.drive(0, 0)

	def get_state2(self):
		buf = self._bus.read_i2c_block_data(self._addr, 0x02, 4)
		print("b")
		for i in range(len(buf)):
			self.status.buf[i] = buf[i]
		return buf


def test_motor():
	m = AMD01()
	for i in range(50):
		m.drive(10,-10)
		time.sleep(0.01)


def test_encoder():
	m =AMD01()
#	for i in range(50):
	while(1):
		print("a")
#		m.drive(10, -10)
		m.get_state2()
		print(m.status.reg.control_state, m.status.reg.m1_ref_speed)
		time.sleep(0.01)



if __name__ == '__main__':
	test_motor()
#	encoder_test()
