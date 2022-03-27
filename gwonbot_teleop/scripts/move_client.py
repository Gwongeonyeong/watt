#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import sys, select, os
import numpy as np, cv2
import math
from cv_bridge import CvBridge,CvBridgeError

import gwonbot_control.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#client
class Movingclient(object):
    def __init__(self):   
        self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odom_callback)
        
        self.client = actionlib.SimpleActionClient("Move", gwonbot_control.msg.MovingAction)
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        self.goal = gwonbot_control.msg.MovingGoal()
        self.goal.x = self.robot_pose_x
        self.goal.y = self.robot_pose_y
        self.goal.z = self.robot_pose_z
        self.goal.vel = 0.3
        self.goal.position = "search"

        if not wait:
            rospy.logerr("Action server not available!")

            return

    def odom_callback(self,msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        self.robot_pose_z = msg.pose.pose.position.z

    def callback_feedback(self,msg):

        print("0: search 1: positioning 2: end", msg.sequence)
        if msg.sequence == 1:
            self.goal = gwonbot_control.msg.MovingGoal()
            self.goal.x = self.robot_pose_x
            self.goal.y = self.robot_pose_y
            self.goal.z = self.robot_pose_z
            self.goal.vel = 0.1
            self.goal.position = "positioning"

        elif msg.sequence == 2:
            self.goal.vel = 0
            self.goal.position = "end"

    def callback_active(self):
        rospy.loginfo("Active")
        
    def callback_done(self,state, result):
       rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def moving_robot(self):

        self.client.send_goal(self.goal,active_cb=self.callback_active,feedback_cb=self.callback_feedback, done_cb=self.callback_done)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        # Prints out the result of executing the action
        return self.client.get_result()  

if __name__=="__main__":
    rospy.init_node('moving_client')
    move_client = Movingclient()
    while not rospy.is_shutdown():
        move_client.moving_robot()
    
    rospy.spin()



'''
################
        
        for i in ranges:
            if i == max(ranges):
                pass
            elif max_range == 0 or i > max_range:
                max_range = i

        # converting distance angle to x,y
        for i in range(num_pts):
            if (ranges[i] > 10 ) or (math.isnan(ranges[i])):
                pass
            else:
                angle = angle_min + float(i)*angle_icr
                xy_scan[i][0] = float(ranges[i]*math.cos(angle))
                xy_scan[i][1] = float(ranges[i]*math.sin(angle))
                
        for i in range(num_pts):
            pt_x = xy_scan[i,0]
            pt_y = xy_scan[i,1]    
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > image_size) or (pix_y > image_size):
                    print "Error"
                else:
                    blank_image[pix_y,pix_x] = [0,0,255] 

        # Convert CV2 Image to ROS Message
		img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        

        # # find point
        # img_point = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img_back = np.float32(blank_image)
       
        # result = cv2.cornerHarris(img_back,2,3,0.04)
        # result = cv2.dilate(result,None,iterations=6)
        # img_point[result>0.01*result.max()]=[0,255,0]


		# Publish image
        cv2.imshow('result', blank_image), cv2.waitKey(3)

		# self.pub.publish(img)

        # count = msg.scan_time/msg.time_increment
        # for i in count:
        #     if i == max(self.range_):
        #         pass
        #     elif self.max_range == 0 or i > self.max_range:
        #         self.max_range = i
        #         degree = self.rad2(self.angle_min + self.angle_icr*i)
        #         print(degree)
        #         ang = int(degree)
        #         dis = self.range_[i]
        #         if ang >= -70 & ang <= 70:
        #             if ang%10 == 0:
        #                 self.pub_data.append([ang,dis])
        # print(self.pub_data)
        
################
'''