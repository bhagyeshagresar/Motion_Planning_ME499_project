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
#include "arm_planner/Reset.h"
#include "arm_planner/Step.h"


static int flag{0};
static bool reset_val{false};
static std::vector<double> waypoints;
static bool step_val{false};
static std::vector<double> joint_group_positions;
// static bool gripper_step{false};

// static const std::string PLANNING_GROUP = "arm";
// static moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
// static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;




bool reset_fn(arm_planner::Reset::Request &req, arm_planner::Reset::Response &res){

  

  reset_val = req.restart;

  if(reset_val == true){
    waypoints.clear();

  }


  return true;
}




bool gripper_fn(arm_planner::Gripper::Request &req, arm_planner::Gripper::Response &res){

  if(req.state == true){
    
    flag = 1;

  }
  else{
   
    flag = 2;

  }

  return true;
}


bool step_fn(arm_planner::Step::Request &req, arm_planner::Step::Response &res){
  
  step_val = true;
  joint_group_positions.at(0) = req.j1;
  joint_group_positions.at(1) = req.j2;
  joint_group_positions.at(2) = req.j3;
  joint_group_positions.at(3) = req.j4;
  joint_group_positions.at(4) = req.j5;
  joint_group_positions.at(5) = req.j6;
  if(req.gripper_status == true){
    flag = 1;
  }
  else{
    flag = 2;
  }



  return true;


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint_position_controller/command", 10);

  ros::ServiceServer gripper_service = nh.advertiseService("gripper", gripper_fn);
  ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
  ros::ServiceServer step_service = nh.advertiseService("step", step_fn);

  static const std::string PLANNING_GROUP = "arm";
  static moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  ROS_INFO_NAMED("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());


  ROS_INFO_NAMED("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // ROS_INFO_NAMED("Get named targets: %s", move_group_interface.getNamedTargets());

  // std::copy(move_group_interface.getNamedTargets().begin(),
  //           move_group_interface.getNamedTargets().end(), std::ostream_iterator<std::string>(std::cout, ", "));



  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));



  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //get waypoints
  nh.getParam("waypoints", waypoints);
  std::cout << "first waypoints: " << waypoints.size() << std::endl;
  

 //Add Ground

  moveit_msgs::CollisionObject ground_collision;
  ground_collision.header.frame_id = "base_link";
  
  shape_msgs::SolidPrimitive ground;
  ground.type = ground.BOX;
  ground.dimensions.resize(3);
  ground.dimensions[ground.BOX_X] = 1.5;
  ground.dimensions[ground.BOX_Y] = 1.5;
  ground.dimensions[ground.BOX_Z] = 0.0;

  geometry_msgs::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.orientation.x = 0.0;
  ground_pose.orientation.y = 0.0;
  ground_pose.orientation.z = -1.0;
  ground_collision.primitive_poses.push_back(ground_pose);

  ground_collision.primitives.push_back(ground);
  ground_collision.operation = ground_collision.ADD;
  collision_objects.push_back(ground_collision);



  //Add bricks
  for (int i = 0; i < 1; i++){
    moveit_msgs::CollisionObject collision_object;
    

    collision_object.header.frame_id = "base_link";
    collision_object.id = i;

    shape_msgs::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.24;
    primitive.dimensions[primitive.BOX_Y] = 0.07;
    primitive.dimensions[primitive.BOX_Z] = 0.112;


    geometry_msgs::Pose brick_pose;

    if(i % 2 == 0){
      brick_pose.orientation.w = 1.0;
      brick_pose.position.x = 0.2;
      brick_pose.position.y = 0.2 + i/2.0;
      brick_pose.position.z = 0.056;
      collision_object.primitive_poses.push_back(brick_pose);

    }

    else{
      brick_pose.orientation.w = 1.0;
      brick_pose.position.x = -0.2;
      brick_pose.position.y = 0.2 + i/2.0;
      brick_pose.position.z = 0.056;
      collision_object.primitive_poses.push_back(brick_pose);

    }
    collision_object.primitives.push_back(primitive);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  //Add cylinders

  












  //Add objects to planning scene
  planning_scene_interface.addCollisionObjects(collision_objects);


  ros::Rate r(120);

  while(ros::ok()){

  

    if (flag == 1){
      std_msgs::Float64 msg;
      msg.data = 0.8;
      std::cout << "gripper openeed" << std::endl;
      pub.publish(msg);
      flag = 0;

    }

    if (flag == 2){
      std_msgs::Float64 msg;
      msg.data = 0.1;
      std::cout << "gripper closed" << std::endl;
      pub.publish(msg);
      flag = 0;
    }

    if(reset_val == true){
        std::cout << "reset waypoints: " << waypoints.size() << std::endl;
        nh.setParam("waypoints", waypoints);
        move_group_interface.setNamedTarget("ready");
        move_group_interface.move();
        reset_val = false;

    }


    // if(step_val == true){
      
    //   moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //   std::vector<double> joint_group_positions;
    //   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // }

    // if (gripper_step == true){

    // }



    ros::spinOnce();

    r.sleep();



  }




  return 0;


}