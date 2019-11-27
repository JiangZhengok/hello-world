#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys

#filename = 'array_reflection_2D_TM_vertical_normE_center.txt'

file_handle=open('times.txt',mode='w')
with rosbag.Bag('/media/jiang/X032/0926_kbd_test_bag/2019-09-26-14-07-00.bag', 'r') as bag: 
    for topic,msg,t in bag.read_messages():
        if topic == "/pandar_points":          #/kbd_cam_node/raw_kbd_cam_rear               
                timestr = "%.6f" %  msg.header.stamp.to_sec()
                file_handle.write(timestr)
                file_handle.write('\n')
file_handle.close()
                       


