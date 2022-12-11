#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class robot_camera():
    def __init__(self):
        rospy.init_node("kamera")
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_cb)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        self.bridge = CvBridge()
        rospy.spin()

    def camera_cb(self, mesaj):
        self.cap = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        
        ######################3 kirmizi_Bolge bulma ###############################
        lower_red = np.array([0,50,50]) #example value
        upper_red = np.array([10,255,255])
        
        hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)
    
        mask = cv2.inRange(hsv, lower_red, upper_red)
        mask = cv2.erode(mask, (5, 5), iterations=9)
        mask = cv2.medianBlur(mask, 7)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))
        mask = cv2.dilate(mask, (5, 5), iterations=1)

        _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    
        cnts,_ = cv2.findContours(thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

        frame_merkez_x = self.cap.shape[1]/2
        frame_merkez_y = self.cap.shape[0]/2

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            cv2.circle(self.cap, (int(x), int(y)),int(radius),(0, 0, 255), 2)
            cv2.putText(self.cap, "X : " + str(round(x,2)), (int(x)+int(radius)+5,int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(self.cap, "Y : " + str(round(y,2)), (int(x)+int(radius)+5,int(y)+35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.line(self.cap, (int(frame_merkez_x),int(frame_merkez_y)),(int(x),int(y)),(0,0,0),3)
        ###############################################################################

        ####################### hizalama takip etme #############################################

            hata_x = int(x) - int(frame_merkez_x)
            
            self.hiz_mesaji.linear.x = 0.2 # ileri gitmesi icin
            self.hiz_mesaji.angular.z = -hata_x/100
            self.pub.publish(self.hiz_mesaji)
            if radius > 150:
                self.hiz_mesaji.linear.x = 0.0
                self.hiz_mesaji.angular.z = 0.0
                self.pub.publish(self.hiz_mesaji)

        else:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.5
            self.pub.publish(self.hiz_mesaji)


        #########################################################################################

        cv2.imshow("frame", self.cap)
        cv2.waitKey(1)


  
nesne = robot_camera()