#!/usr/bin/env python3

import time
import amd01
import rospy
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import Int32MultiArray

class AMD01_ROS(amd01.AMD01):
	max_vel = 2
	max_rot = 10

	def remote(self, Twist):
		rpm_left = (self.max_vel * Twist.linear.x * 60 / (0.4 * pi)) / 2 - self.max_rot * Twist.angular.z * 2
		rpm_right = (self.max_vel * Twist.linear.x * 60 / (0.4 * pi)) / 2 + self.max_rot * Twist.angular.z * 2
		self.drive(int(rpm_right ), int(rpm_left * -1))
		print(int(rpm_right), int(rpm_left))
		time.sleep(0.01)


def remote_test():
	m = AMD01_ROS()

	rospy.init_node("motors")
	rospy.Subscriber("cmd_vel", Twist, m.remote)
	while not rospy.is_shutdown():
		time.sleep(0.01)



def encoder_test():
	m = AMD01()
	s = m.status.reg
	pub = rospy.Publisher("encoder", Int32MultiArray, queue_size=100)

	rospy.init_node('motors')
	rospy.Subscriber("cmd_vel", Twist, m.callback)

	pub_enc = Int32MultiArray()
	pub_enc.data = [0, 0, 0, 0]

	time.sleep(0.05)

	while not rospy.is_shutdown():
		m.get_state()
		pub_enc.data = [s.m1_ref_speed, s.m2_ref_speed, s.m1_encoder, s.m2_encoder]
		pub.publish(pub_enc)
		time.sleep(0.05)

if __name__ == '__main__':
#	amd01.test()
	remote_test()
