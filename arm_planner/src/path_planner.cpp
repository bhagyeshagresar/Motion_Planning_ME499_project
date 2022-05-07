#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "arm_planner/Gripper.h"
#include <std_srvs/Empty.h>


// const double tau = 2*M_PI;
// static float pincer_angle{0.8};




// bool gripper_fn(arm_planner::Gripper::Request &req, std_srvs::Empty::Response &res){

//   if(req.state == 1){
//     std_msgs::Float64 msg;

//     msg.data = 0.8;
//     std::cout << "gripper opened" << std::endl;

//     pub.publish(msg);

//   }
//   else{
//     std_msgs::Float64 msg;

//     msg.data = 0.1;
//     std::cout << "gripper closed" << std::endl;

//     pub.publish(msg);

//   }

//   return true;
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint_position_controller/command", 10);

  // ros::ServiceServer gripper_service = nh.advertiseService("gripper", gripper_fn);


  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());


  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());


  // ROS_INFO_NAMED("Available Planning Groups:");

  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


  //add brick 1

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  

  for (int i = 0; i < 8; i++){
    moveit_msgs::CollisionObject collision_object;
    
    // std::cout << "header1" << std::endl;

    collision_object.header.frame_id = "base_link";
    // std::cout << "header" << collision_object << std::endl;
    collision_object.id = i;
    // std::cout << "header" << collision_objects.at(i) << std::endl;

    shape_msgs::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.24;
    primitive.dimensions[primitive.BOX_Y] = 0.07;//0.07
    primitive.dimensions[primitive.BOX_Z] = 0.112;

  //   std::cout << "primitives" << primitives.at(i) << std::endl;

    geometry_msgs::Pose brick_pose;

    if(i % 2 == 0){
      brick_pose.orientation.w = 1.0;
      brick_pose.position.x = 0.5;
      brick_pose.position.y = 0.5 + i/2.0;
      brick_pose.position.z = 0.056;
      collision_object.primitive_poses.push_back(brick_pose);

    }

    else{
      brick_pose.orientation.w = 1.0;
      brick_pose.position.x = -0.5;
      brick_pose.position.y = 0.5 + i/2.0;
      brick_pose.position.z = 0.056;
      collision_object.primitive_poses.push_back(brick_pose);

    }
    collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(brick_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);



  }

  // // ROS_INFO_NAMED("add object into world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // pub.publish(pincer_angle);
  std_msgs::Float64 msg;

  msg.data = 0.8;
  std::cout << "state reached" << std::endl;

  pub.publish(msg);


  
  ros::Rate r(120);

  while(ros::ok()){

    // std_msgs::Float64 msg;

    // msg.data = pincer_angle;
    // std::cout << "state reached" << std::endl;

    // pub.publish(msg);

    ros::spinOnce();

    r.sleep();



  }




  return 0;


}