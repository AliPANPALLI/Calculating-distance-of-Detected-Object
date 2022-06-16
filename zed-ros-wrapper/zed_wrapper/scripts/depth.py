#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# Ali PANPALLI
from std_msgs.msg import String
from std_msgs.msg import Int32
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math

class depth_processing():

    def __init__(self):

        rospy.init_node('listener', anonymous=True)
        self.bridge = CvBridge()
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0
        self.dist = 0
        rospy.Subscriber("zed2i/zed_node/depth/depth_registered", Image, self.callback)
     
        rospy.spin()
    def distanceListen_x(self, data):
        self.start_x = data.data
    def distanceListen_y(self, data):
        self.start_y = data.data
    def distanceListen_end_x(self, data):
        self.end_x = data.data
    def distanceListen_end_y(self, data):
        self.end_y = data.data

    def callback(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError:
            pass
        #print(self.msg.data)
        depth_image = cv2.resize(depth_image, (600, 360))
        depth_array = np.array(depth_image, dtype=np.float32)
        pub = rospy.Publisher("distance",String,queue_size=1)

        #print('Image size: {width}x{height}'.format(width=depth_data.width,height=depth_data.height))
        rospy.Subscriber('/start_x',Int32,self.distanceListen_x)
        #print(start_x.data_class)
        rospy.Subscriber('/start_y',Int32,self.distanceListen_y)
        #print(start_y.data_class)
        rospy.Subscriber('/end_x',Int32,self.distanceListen_end_x)
        rospy.Subscriber('/end_y',Int32,self.distanceListen_end_y)

        
        u = (self.end_x + self.start_x)/2
        v = (self.end_y+self.start_y)/2
        u = math.ceil(u)
        v = math.ceil(v)
        image = cv2.circle(depth_image, (u,v), 1, (0,0,0), 10)
        
        self.dist=(depth_array[u,v])
        self.dist=str(self.dist)
        rospy.loginfo('Dist: {}'.format(self.dist))
        pub.publish(self.dist)
        cv2.imshow("image", image)
        cv2.waitKey(1)
        
depth_processing()
