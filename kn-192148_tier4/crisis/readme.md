# MAIN PROJECT

[[_TOC_]]

## Team details

Team name : **CRISIS**

Name| Matr.No.| Address|
:--- | :---| :---|
KANNAN NEELAMEGAM| 32973| @kn-192148

This repository is part of the AAMR course, it contains codes to achieve the task of moving the robot in a gazebo environment to reach the goals by avoiding all the obstacles in the environment.

## Objective

We have given a practice arena, in this arena we have to reach the goals published by the `goal_publisher` package. The aim is to reach all the goals and collect the reward provided to each goal in arena within 10 minutes by avoiding the obscatcles using `move_base` package.

## Getting started

This project consists of two main tasks which have to be accomplished by the robot:

1. To reach the goals published by /goals topic from goal_publisher package, in minimum time getting maximum rewards.

2. To avoid the obstacles while reaching these goal points.

This project can be started by achieving these tasks independently and then combining the codes to achieve the main task.
The prerequisites include:


Check if goals are being published when the topic /goals are published.
Good knowledge about ROS, Topics, Subscribers, Publishers.
Gazebo environment with obstacles.
A map.yaml file is required to get the map of the environment for navigation.

## Short Description

As the aim is to collect the maximum reward points we came up with sorting algorithm which is based on the cost function wihch is dependent on global path distance and rewards. We subscribed to amcl and move_base nodes to navigate through the environment. AMCL (adaptive Monte Carlo localization) package has amcl node which takes in a laser-based map(/map), laser scans(/scan), and transformations of robots(/tf), and estimates position in the map . The move_base node acts like a simple action server links together a global and local planner to accomplish its global navigation task. In this project we used default global and local planners i.e Navfn as base global planner and DWA planner as base local planner.

## General description
The goal of the project is to move the robot to reach as many goals as possible while avoiding all the obstacles in the environment.
Each goal has a reward associated with the point. The robot has to cover the goals to get the maximum number of these rewards and in the minimum number of times. 

The topics being used are:

/cmd_vel: To provide motion to the robot. The robot moves in the x-axis and rotates along the z-axis.

/amcl_pose: This is used to get the current position of the robot while in the environment.

/goals: This is used to get the goals to which the robot has to move.

/scan: This is used to get the Laser scan information.

/move_base: It is a navigation stack, which when given a point determines a path and navigates to the point, from the current robot position.

## Sorting algorithm

- Subscribing to the `/goals` topic and storing all the goals and its respective rewards in seperate list using msg.goals.
- Sorting the positions based on highest rewards


## Move_Base Package

- move_base package contains a node called move_base node , which works like simple action server
- The main function of the move_base node is to move the robot from its current position to a goal position i.e. providing the trajectory.
- This node takes a goal pose with msg type geometry_msg/Pose stamped
- This Action server provides the topic move_base/goal from which it receive goals.
- When the node receives the goal pose. it links to components such as global planner, local planner, recovery behavior and costmaps and generates an output which is velocity command with msg type     geometry_msg/Twist() ie from cmd_vel topic.


## AMCL(adaptive Monte Carlo Localization) Package

As we know the robot may not always moves as we expected, it makes some random guesses to move to next position. These random guesses are called particles(the green arrows that appear under the robot in rviz).The robot observes environment and discards particles that doesn't match with the readings and generates more particles towards the probable one.The more we move the robot, the more data from sensors and localization will be more precise. MCL Localization is also called particle filter localization.

The amcl (Adaptive Monte Carlo Localization)node:
This node subscribe to the data of laser(/scan), the laser based map(/map), the transformations of robot (/tf) and publishes to the position in the map using topic (/amcl_pose)

## Global and Local Planners

### Global Planners

When a new goal is received by the move_base,this goal is immediately sent to the global planner then the global planner is incharge of calculating safe path in order to arrive at the goal position. The path is calculated before the robot moves to the goal. So, it will not take into account the readings that the laser scaner are doing while moving.Ech time new path planned by global planner is published into topic (/path).There are different global palnners like `carrot_planner`, `sbpl_lattice_planner` and `navfn_planner`. As per requirements of our project `navfn_planner` is the best suited and most used as well. The Carrot planner allows the robot to get as close to a user-specified goal point as possible. It can be used in situation where the goals are present on the obstacle.

### Local Planners
We came across `teb_Local_planner` (Timed Elastic Band) and `dwa_local_planner`(Dynamic Window Approach). Afer testing and changing some of the parameters using ROS dynamic reconfigure finally we selected dwa local planner for our project. 





 

