#!/usr/bin/env python

import rospy
import yaml                       
import sys 
import math 
import actionlib 
#importing requirements
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose,Point,Twist                
from goal_publisher.msg import PointArray

class Crisis_goal_sorting(object):

    def __init__(self): 
        rate = rospy.Rate(20)   # Defines the frequency of execution in Hz 
        self.sub = rospy.Subscriber('/goals',PointArray,self.goals) #Subscribing the goals
        while not rospy.is_shutdown():                        
            rate.sleep()  

    def goals(self,msg):
        self.goal = msg.goals   #assigning subscribed goal
        self.goal_list = []
        for i in range(len(self.goal)):
            if self.goal[i].reward > 0:
                self.goal_list.append([self.goal[i].x,self.goal[i].y,self.goal[i].z,self.goal[i].reward])
        self.goal_list.sort(key=lambda  item:item[3], reverse=True) #arranging goals on rewards
        print(self.goal_list)

if __name__ == '__main__':
    rospy.init_node('Crisis_sort_goal')
    Crisis_goal_sorting()
    rospy.spin()

