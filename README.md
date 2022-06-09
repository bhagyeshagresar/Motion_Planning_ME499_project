# ME499 - Motion Planning using Adroit Manipulator Arm
The aim of this project was to use Adroit Robot Arm and build a pyramid of cylinders. This is a motion planning task which was achieved by using ROS and Moveit on  HDT Adroit. 

# Planning Scene Setup
The planning scene consists of a table with 6 cylinders place in sequence. The task of the robot is to perform motion planning using Move Group C++ Interface without knocking other cylinders in the process and place them in a pattern such that the arm builds a pyramid.

# Setup

1) Install ROS Noetic - http://wiki.ros.org/noetic/Installation/Ubuntu

2) Make a Catkin workspace as follows:
    `mkdir -p catkin_ws/src\`
    `cd catkin_ws/src`
    `git clone git@github.com:bhagyeshagresar/me499_project.git`
    `cd catkin_ws`
    `catkin_make`

3) Make sure moveit is installed on the machine:
    `sudo apt install ros-noetic-moveit`

4) Clone the hdt-adroit repository in the custom_ws

5) Follow the instructions to setup the HDT Adroit on your machine - https://nu-msr.github.io/me495_site/adroit.html

6) The robot arm can be controlled using a xbox controller by running the following launch file:
    `roslaunch hdt_6dof_a24_pincer_bringup hdt_arm_bringup_1.launch controller_type:=xbox`

# Running the actual Demonstration

In order to visualise the robot in Rviz window, run the following command:

a) `roslaunch arm_planner robot_control.launch`

b) In order to bring the robot to home configuration call the following service in separate terminal:
    `rosservice call /reset restart:true`

c) In order to run the full automated sequence of picking up and placing the cylinders, call the following service in separate  terminal:
    `rosservice call /follow {}`


# Demonstration of the HDT Adroit Arm in Rviz
    https://youtu.be/nonTPkbvQE0

# Demonstration of the HDT Adroit Arm in real world
    https://youtu.be/uECW0llTjYU



