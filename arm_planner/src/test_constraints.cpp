#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <string.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_constraints");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();               
    static const std::string PLANNING_GROUP = "arm";

     //control the planning_group 'arm'    
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

      
      
      
      
      
      }