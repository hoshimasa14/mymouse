#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class sensordata_convertor():
    def __init__(self):
        rospy.init_node("sensordata_convertor")
        rospy.Subscriber("sub_scan", LaserScan, callback_scan)
        rospy.Subscriber("sub_odo", Float32MultiArray, callback_odo)

        i = 0
        flag = 1
        while(flag):
            i = i + 1
            try:
                with open("sensordata("+str(i)+").lsc", "x") as f:
                    pass
                flag = 0
            except FileExistsError:
                pass

        self.filename = "sensordata("+str(i)+").lsc"

    def callback_scan(data):
        self.raw_scan_data = data
        self.convert()
        self.write()

    def callback_odo(data):
        self.raw_odo_data = data

    def convert():
        self.sensordata = "LASERSCAN sid sec nsec "

        self.pnum = round((self.raw_scan_data.angle_max - self.raw_scan_data.angle_min) / self.raw_scan_data.angle_increment) + 1
        self.sensordata += self.pnum + " "
        for i in range(self.pnum):
            self.sensordata += str(self.raw_scan_data.angle_min + self.raw_scan_data.angle_increment * (i - 1)) + " "
            self.sensordata += str(self.raw_scan_data.ranges[i]) + " "

        self.sensordata += str(self.raw_odo_data.data[0]) + " " + str(self.raw_odo_data.data[1]) + " " + str(self.raw_odo_data.data[2]) + "\n"  # x, y, radの順

    def write():
        with open(self.filename, "a") as f:
            f.write(self.sensordata)

def convert_sensordata():   #クラス内のサブスクライバのコールバックでファイルにセンサデータを追記
    con = sensordata_convertor()
    rospy.spin()

if __name__ == "main":
    convert_sensordata()
