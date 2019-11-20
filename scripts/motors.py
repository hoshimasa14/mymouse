#!/usr/bin/env python3

import rospy
import time
import smbus
import ctypes
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import Int32MultiArray

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

class amd01_status(ctypes.Union):
    _fields_ = [ ('buf', ctypes.c_uint8 * 64), ('reg', st_amd01_status) ]

class AMD01(object):
    def __init__(self, addr=0x58):
        # I2C interface
        self._addr = addr
        self._bus = smbus.SMBus(1)
        time.sleep(0.1)
        if( self._bus ):
            print( 'I2C bus available' )
        else:
            print( 'No I2C bus ' )

        # status
        self.status = amd01_status()

        self.max_vel = 2
        self.max_rot = 10
        self.twist = Twist()

    def get_state(self):
        buf = []
        tmp = []

        tmp  = self._bus.read_i2c_block_data( self._addr,  0, 17 )
        buf += tmp[1:]
        tmp  = self._bus.read_i2c_block_data( self._addr, 16, 17 )
        buf += tmp[1:]
        tmp  = self._bus.read_i2c_block_data( self._addr, 32, 17 )
        buf += tmp[1:]
        tmp  = self._bus.read_i2c_block_data( self._addr, 48, 17 )
        buf += tmp[1:]

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

    def callback(self, data):
        self.rpm_left = (self.max_vel * data.linear.x * 60 / (0.4 * pi)) / 2 - self.max_rot * data.angular.z * 2
        self.rpm_right = (self.max_vel * data.linear.x * 60 / (0.4 * pi)) / 2 + self.max_rot * data.angular.z * 2

        self.drive(int(self.rpm_right ), int(self.rpm_left * -1))



if __name__ == '__main__':
    m = AMD01()
    pub = rospy.Publisher("encoder", Int32MultiArray)
    rospy.init_node('motors')
    rospy.Subscriber("cmd_vel", Twist, m.callback)
    rospy.spin()

    while(1):
        m.get_state()
        pub.publish([m.status.reg.m1_speed, m.status.reg.m2_speed])
        time.sleep(0.02)
