#!/usr/bin/env python
# -*- coding: utf-8 -*- 

# @Author: Qihao Yao
# @Date: Fri Apr 08 2022
# @Last Modified by: DaiKai
# @Last Modified time: Fri Apr 08 2022 10:56:50 AM
# @Comment: 将激光雷达传感器传入的数据转化为Autoware需求的数据格式

import rospy
from sensor_msgs.msg import PointCloud2

class Topic_transform():
    def __init__(self):
        rospy.init_node('transfer_topic_from_rslidar_points_to_points_raw')

        self.points_raw_pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=100)

        rospy.Subscriber('/rslidar_points', PointCloud2, self.callback_lidar)

        rate = rospy.Rate(10)


    def callback_lidar(self,data):
            
        data.header.frame_id="velodyne"
        self.points_raw_pub.publish(data)
        # print(data)

if __name__ == '__main__':
    Topic_transform()
    rospy.spin()
