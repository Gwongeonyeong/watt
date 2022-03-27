#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import sys, select, os, time

import gwonbot_control.msg 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#server
class MovingAction(object):
    _feedback = gwonbot_control.msg.MovingFeedback()
    _result = gwonbot_control.msg.MovingResult()
    def __init__(self, name):
        self._action_name = name
        self._as=actionlib.SimpleActionServer(self._action_name, gwonbot_control.msg.MovingAction, execute_cb=self.execute_cb, auto_start = False) 
        self._as.start()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.check_position = False
        self.max_distance = 0.0
        self.direction = 0.0
        self._feedback.sequence = 0
        
    def scan_callback(self, msg):
        angle_max = msg.angle_max
        angle_min = msg.angle_min
        angle_icr = msg.angle_increment
        
        ranges = msg.ranges
        range_max = msg.range_max
        max_range = 0.0
        pub_data = []

        resoultion = 0.015 * 20
        count = 0

        #except infinity
        for i in ranges:
            if i == max(ranges):
                pass
            else:
                if max_range == 0 or i > max_range:
                    max_range = i
                pub_data.append(i)

        #find check position
        for i in range(len(pub_data)):

            sum_front = 0
            sum_middle = 0
            sum_back = 0
            average_back = 0
            average_front = 0
            average_middle = 0
            compare = 0.0
  
            for j in range(i,i+9):
                sum_front = sum_front + pub_data[j]
                average_front = sum_front/8

            for k in range(i+9,i+13):
                sum_middle = sum_middle + pub_data[k]
                average_middle = sum_middle/4

            for s in range(i+13,i+21):
                sum_back = sum_back + pub_data[k]
                average_back = sum_back/8

            # print(average_back,average_middle,average_front)
            if average_back > average_front:
                compare = average_back - average_front
                if compare < 0.1 and average_middle > average_front:
                    count = count + 1
                    self.max_distance = pub_data[i]
                    self.direction = (i/len(pub_data))*100       
                    
            else:
                compare = average_front - average_back
                if compare < 0.1 and average_middle > average_back:
                    count = count + 1
                    if self.max_distance > pub_data[i]:
                        self.max_distance = self.max_distance
                        self.direction = (i/len(pub_data))*100
                    else:
                        self.max_distance = pub_data[i]
                        self.direction = (i/len(pub_data))*100

            if i+30 >= len(pub_data):
                break

        if count > 5:
            self.check_position = True

    def right_turn(self,vel,deg):
        cmd_vel = Twist()
        cmd_vel.angular.y = vel
        self.cmd_vel_pub.publish(cmd_vel)
    def left_turn(self,vel,angluar):
        cmd_vel = Twist()
        cmd_vel.angular.y = vel
        self.cmd_vel_pub.publish(cmd_vel)
    def forward(self,vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = vel
        self.cmd_vel_pub.publish(cmd_vel)
    def backward(self,vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = vel
        self.cmd_vel_pub.publish(cmd_vel)
    def stop_(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        self.cmd_vel_pub.publish(cmd_vel)
   
    def linear_move(self,vel):
        self.forward(vel)

    def positioning(self,vel):
        wall_param = rospy.get_param("wall_distance","")
        print("max_distance: ",self.max_distance)
        if self.direction < 30.0:
            print("right :", self.direction)
        elif self.direction > 60.0:
            print("left : ",self.direction)
        else:
            print("straight :", self.direction)
            if self.max_distance > wall_param:
                self.forward(vel)
                time.sleep(5)
                self.forward(0)
   
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True
        if goal.position == "search":
            self.linear_move(goal.vel)
            if self.check_position:
                self.linear_move(0)
                print(goal)
                self._feedback.sequence = 1
        elif goal.position == "positioning":
            self.positioning(goal.vel)
            print(goal)


        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        self._as.publish_feedback(self._feedback)
        r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__=='__main__':
    rospy.init_node("move")

    server = MovingAction("Move")
    print('sever open')

    rospy.spin()