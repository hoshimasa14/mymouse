#!/usr/bin/env python3

import rospy
from imu import ICM
from std_msgs.msg import Float32MultiArray

def test_imu_ros():
    o = ICM()
    buf = Float32MultiArray()
    pub = rospy.Publisher('imu', Float32MultiArray, queue_size=10)
    rospy.init_node("imu")
    rate = rospy.Rate(30*10**3) # 30kHz
    while not rospy.is_shutdown():
        o.get_imu_status()
        buf.data = [o.ax, o.ay, o.az, o.gx, o.gy, o.gz]
        pub.publish(buf)
        rate.sleep()

if __name__ == '__main__':
    test_imu_ros()
