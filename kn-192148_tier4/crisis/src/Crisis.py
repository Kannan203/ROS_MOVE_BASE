#!/usr/bin/env python

import rospy
import yaml                       
import sys 
import math 
import actionlib 
import time
#importing requirements
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose,Point,Twist                
from goal_publisher.msg import PointArray

class Crisis_goal_sorting(object):

    def __init__(self): 
        self.c = time.time()/1000000000
        rate = rospy.Rate(20)                                               # Defines the frequency of execution in Hz 
        self.point = rospy.wait_for_message('/goals',PointArray)            #Subscribing the goals
        self.i = 0
        self.state = 0
        self.goal_list = []
        self.total_rewards = 0
        while not rospy.is_shutdown():
            if self.state == 0 :
                
                self.sort_goal(self.point)
            if self.state == 1:                                                #State definition 
                self.send_goals(self.i) 
            if self.state == 2:
                self.go_to_goal(self.x_pos,self.y_pos)
            if self.state == 3:
                print('Goal reached and reward collected',self.rew)
                self.total_rewards += self.rew
                print('Total reward collected',self.total_rewards)
                self.i += 1
                if self.i == (len(self.goal)-1):
                    quit()
                else:
                    self.change_state(1)

            rate.sleep()  
    
    def send_goals(self,i):
        self.x_pos = self.goal_list[i][0]
        self.y_pos = self.goal_list[i][1]                                          #send goals to move_base function
        self.rew = self.goal_list[i][3]
        self.change_state(2)

    def go_to_goal(self,x,y):
        a_client = actionlib.SimpleActionClient('robot'+str(sys.argv[1])+'/move_base', MoveBaseAction)       # To send goal requests to the 'move_base' server through a SimpleActionClient

        while(not a_client.wait_for_server(rospy.Duration.from_sec(5.0))):         # Wait for the Action Server to come up
            print('Waiting for the move_base to react')

        goal = MoveBaseGoal()                                                       # Assigns movebase function to 'goal' variable

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()                            # Setting up the frame parameters

        goal.target_pose.pose.position.x =  x
        goal.target_pose.pose.position.y =  y
        goal.target_pose.pose.position.z =  0                                       # Defining the Goal Positions x and y

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0                                   # Defining Goal Orientation, which is equivalet to '0' degree Yaw here

        print("Moving towards the Goal point")
        a_client.send_goal(goal)                                                    # Publishing the goal location

        a_client.wait_for_result(rospy.Duration(70))                                # Timeout for a Goal is 70 seconds

        if(a_client.get_state() ==  GoalStatus.SUCCEEDED):
            print(sys.argv)
            self.change_state(3)

        else:
            print("Desitination Failed to reach") 
            self.change_state(3)
            
                
    def change_state(self,state):
        self.state = state

    def sort_goal(self,goal): 
        self.goal = goal.goals
        for i in range(len(self.goal)):
            if self.goal[i].reward > 0:
                self.goal_list.append([self.goal[i].x,self.goal[i].y,self.goal[i].z,self.goal[i].reward])
        self.goal_list.sort(key=lambda  item:item[3], reverse=True)                  #arranging goals on rewards
        self.change_state(1)
        

if __name__ == '__main__':
    rospy.init_node('Crisis')                                                         #initiating node
    Crisis_goal_sorting()
    rospy.spin()

