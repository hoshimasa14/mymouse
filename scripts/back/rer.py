#!/usr/bin/env python
#encoding: utf8
from pyftdi.i2c import I2cController
import time
# 8ビット符号無し整数を符号あり整数に
def s8( value ):
	return -(value & 0x80) | (value & 0x7F)
# 16ビット符号無し整数を符号あり整数に	
def s16( value ):
	return -(value & 0x8000) | (value & 0x7FFF)
	
# 32ビット符号無し整数を符号あり整数に	
def s32( value ):
	return -(value & 0x80000000) | (value & 0x7FFFFFFF)
# 64ビット符号無し整数を符号あり整数に	
def s64( value ):
	return -(value & 0x8000000000000000) | (value & 0x7FFFFFFFFFFFFFFF)
# AMD01用クラス
class amd01:
	_i2c 			= I2cController()
	_buf				=	[]
	addr			=	0x58
	control_state	=	0   # ( 2 byte) (R/W)[-] control status
	m1_ref_speed	=	0   # ( 2 byte) (R/W)[rpm/duty rate] M1 reference speed
	m2_ref_speed	=	0   # ( 2 byte) (R/W)[rpm/duty rate] M2 reference speed
	m1_encoder		=	0   # ( 2 byte) (R)[count] M1 encoder value
	m2_encoder		=	0   # ( 2 byte) (R)[count] M2 encoder value
	m1_speed		=	0   # ( 2 byte) (R)[rpm] M1 speed
	m2_speed		=	0   # ( 2 byte) (R)[rpm] M2 speed
	m1_current		=	0   # ( 2 byte) (R)[mA] M1 current
	m2_current		=	0   # ( 2 byte) (R)[mA] M2 current
	m1_duty			=	0   # ( 2 byte) (R)[-] M1 duty
	m2_duty			=	0   # ( 2 byte) (R)[-] M2 duty
	batt_vol		=	0	# ( 2 byte) (R)[mV] battery voltage
	pid_kp			=	0	# ( 2 byte) (R/W)[-] PID P gain
	pid_ki			=	0   # ( 2 byte) (R/W)[-] PID I gain
	gear_rate		=   0   # ( 1 byte) (R/W)[-] gear rate
	encoder_cpr		=	0   # ( 1 byte) (R/W)[cpr] encoder resolution
	current_limit	=	0	# ( 2 byte) (R/W)[-] current limit
	timeout_sec		=	0   # ( 1 byte) (R)[-] software version
	firm_version	=	0   # ( 1 byte) (R)[-] software version
	
	def __init__(self):
		self._i2c.set_retry_count(10)
		self._i2c.configure(url='ftdi://ftdi:232h/1')
		print('AMD01 connect')
		
	def __del__(self):
		self._i2c.terminate()
		print('AMD01 terminate')
		
	def update_state(self):
		buf = []	
		self._i2c.write( self.addr, [0x80, 0x40] )
		buf = self._i2c.read(self.addr, 64)
		
		self.control_state	=	( buf[1]  << 8 ) | ( buf[0]  << 0 )
		self.m1_ref_speed	=	s16( ( buf[3]  << 8 ) | ( buf[2]  << 0 ) )
		self.m2_ref_speed	=	s16( ( buf[5]  << 8 ) | ( buf[4]  << 0 ) )
		self.m1_encoder		=	s16( ( buf[7]  << 8 ) | ( buf[6]  << 0 ) )
		self.m2_encoder		=	s16( ( buf[9]  << 8 ) | ( buf[8]  << 0 ) )
		self.m1_speed		=	s16( ( buf[11] << 8 ) | ( buf[10] << 0 ) )
		self.m2_speed		=	s16( ( buf[13] << 8 ) | ( buf[12] << 0 ) )
		self.m1_current		=	     ( buf[15] << 8 ) | ( buf[14] << 0 )
		self.m2_current		=	     ( buf[17] << 8 ) | ( buf[16] << 0 )
		self.m1_duty		=	s16( ( buf[19] << 8 ) | ( buf[18] << 0 ) )
		self.m2_duty		=	s16( ( buf[21] << 8 ) | ( buf[20] << 0 ) )
		self.batt_vol		=	     ( buf[23] << 8 ) | ( buf[22] << 0 )
		self.pid_kp			=	     ( buf[25] << 8 ) | ( buf[24] << 0 )
		self.pid_ki			=	     ( buf[27] << 8 ) | ( buf[26] << 0 )
		self.gear_rate		=	       buf[28]
		self.encoder_cpr	=	       buf[29]
		self.current_limit	=	     ( buf[31] << 8 ) | ( buf[30] << 0 )
		self.timeout_sec	=	       buf[32]
		self.firm_version	=	       buf[33]
		
	def speed_command(self, m1, m2):
		buf = []	
		
		self.m1_ref_speed = m1
		self.m2_ref_speed = m2
		
		buf += [0x02]
		buf += [0x04]
		buf += [ ( self.m1_ref_speed >> 0 ) & 0xFF ]
		buf += [ ( self.m1_ref_speed >> 8 ) & 0xFF ]
		buf += [ ( self.m2_ref_speed >> 0 ) & 0xFF ]
		buf += [ ( self.m2_ref_speed >> 8 ) & 0xFF ]
		self._i2c.write( self.addr, buf )
		
		time.sleep(0.001)
		
def amd01_test():
	md = amd01()
	for i in range(50):
		md.update_state()
		md.speed_command( 5, 5 )
		print(md.m1_encoder)
		time.sleep(0.01)
# メイン関数
if( __name__ == '__main__' ):
	amd01_test()
