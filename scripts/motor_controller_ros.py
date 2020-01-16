#!/usr/bin/env python3

import time
import motors
import rospy
from geometry_msgs.msg import Twist
from math import pi

class AMD01_ROS(motors.AMD01):
	max_vel = 2		#m/s
	max_rot = 10	#rot/m

	def remote(self, Twist):
		rpm_left = (self.max_vel * Twist.linear.x * 60 / (0.4 * pi)) / 2 - self.max_rot * Twist.angular.z * 2
		rpm_right = (self.max_vel * Twist.linear.x * 60 / (0.4 * pi)) / 2 + self.max_rot * Twist.angular.z * 2
		self.drive(int(rpm_right ), int(rpm_left * -1))
		print(int(rpm_right), int(rpm_left))
		time.sleep(0.01)

def test_remote():
	m = AMD01_ROS()

	rospy.init_node("motors")
	rospy.Subscriber("cmd_vel", Twist, m.remote)
	while not rospy.is_shutdown():
		time.sleep(0.01)



if __name__ == '__main__':
	test_remote()
