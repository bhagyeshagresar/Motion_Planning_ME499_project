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
#include "arm_planner/Follow.h"
#include "arm_planner/Test.h"


static int flag{0};
static bool reset_val{false};
static bool step_val{false};
static std::vector<double> joint_group_positions;
static bool set_gripper{false};
static std::vector<std::vector<double>> waypoints;
static std::vector<double> waypoints_list;
static bool set_follow{false};
static bool test_val{false};

const double tau = 2 * M_PI;


// static bool gripper_step{false};

// static const std::string PLANNING_GROUP = "arm";
// static moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
// static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;




bool reset_fn(arm_planner::Reset::Request &req, arm_planner::Reset::Response &res){

  

  reset_val = req.restart;

  if(reset_val == true){
    waypoints.clear();
    waypoints_list.clear();
    joint_group_positions.clear();


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

// bool test_fn(arm_planner::Test::Request &req, arm_planner::Test::Response &res){
//   test_val = true;

// }


bool step_fn(arm_planner::Step::Request &req, arm_planner::Step::Response &res){
  
  step_val = true;
  std::cout << "step value reached" << std::endl;
  joint_group_positions.push_back(req.j1);
  std::cout << "filled first joint state" << std::endl;
  joint_group_positions.push_back(req.j2);
  joint_group_positions.push_back(req.j3);
  joint_group_positions.push_back(req.j4);
  joint_group_positions.push_back(req.j5);
  joint_group_positions.push_back(req.j6);
  set_gripper = (double)req.gripper_status;

  std::cout << "start filing waypoints" << std::endl;
  waypoints_list.push_back(joint_group_positions.at(0));
  waypoints_list.push_back(joint_group_positions.at(1));
  waypoints_list.push_back(joint_group_positions.at(2));
  waypoints_list.push_back(joint_group_positions.at(3));
  waypoints_list.push_back(joint_group_positions.at(4));
  waypoints_list.push_back(joint_group_positions.at(5));
  waypoints_list.push_back(set_gripper);
  std::cout << "waypoints filled" << std::endl;

  // set_gripper = req.gripper_status;
  std::cout << "inside the service fn" << std::endl;
  



  return true;


}



// bool follow_fn(arm_planner::Follow::Request &req, arm_planner::Follow::Response &res){

//   set_follow = req.follow;

//   if (set_follow == true){
//     while(1){
//       for(int i = 0; i < waypoints.size(); i++){
        
//         joint_group_positions.push_back()
//       }
//     }
//   }




// }


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
  // ros::ServiceServer test_service = nh.advertiseService("test", test_fn);

  //add planning group "arm"
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //add planning group "pincer"
  static const std::string PLANNING_GROUP_2 = "pincer";
  moveit::planning_interface::MoveGroupInterface move_group_interface_2(PLANNING_GROUP_2);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_2;
  
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  ROS_INFO_NAMED("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(move_group_interface.getJointModelGroupNames().begin(),
  //           move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));



  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //get waypoints
  // nh.getParam("waypoints", waypoints);
  // std::cout << "first waypoints: " << waypoints.size() << std::endl;
  

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


  std::vector <double> joints_check_1 = move_group_interface.getCurrentJointValues();

  for(int a = 0; a < joints_check_1.size(); a++){
    std::cout << "joint angle at: " << a << "is: " << joints_check_1.at(a) << std::endl;
  }


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
        // nh.setParam("waypoints", waypoints);
        move_group_interface.setNamedTarget("ready");
        move_group_interface.move();
        reset_val = false;

    }


    if(step_val == true){
      std::cout << "outside the service fn" << std::endl;
      
      moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      
      move_group_interface.setJointValueTarget(joint_group_positions);
      std::cout << "set joint" << std::endl;
      
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      std::cout << "my_plan" << std::endl;
      
      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "success: " << success << std::endl;


      
      if(success == true){
        std::cout << "success reached" << std::endl;
        move_group_interface.execute(my_plan);
        // move_group_interface.move();
        std::vector <double> joints_check = move_group_interface.getCurrentJointValues();

        for(int a = 0; a < joints_check.size(); a++){
          std::cout << "joint angle at: " << a << "is: " << joints_check.at(a) << std::endl;
        }
        

        
        waypoints.push_back(waypoints_list);
        

        // for(int i = 0; i < waypoints.size(); i++){
        //   std::cout << "waypoints at: " << i << "is: " << waypoints.at(i) << std::endl;
        // }
        // if (waypoints.size() != 0){
        //   for(int i = 0; i < waypoints.size(); i++){
            
        //     std::cout << "waypoints at: " << i << "is: " << waypoints.at(i) << std::endl;
        //   }
        // }

        // for(int z = 0; z < waypoints.size(); z++){
        //   for(int j = 0; j < waypoints[z].size(); j++){
        //     std::cout << "waypoints for 2d" << waypoints[z][j] << std::endl;
        //   }
        //   std::cout << std::endl;
        // }

        std::cout << "waypoints size: " << waypoints.size() << std::endl;



        if(set_gripper == true){
          std_msgs::Float64 msg;
          msg.data = 0.8;
          std::cout << "gripper openeed" << std::endl;
          pub.publish(msg);
        }
        
        else{
          std_msgs::Float64 msg;
          msg.data = 0.1;
          std::cout << "gripper closed" << std::endl;
          pub.publish(msg);
        
        
      }
      // nh.setParam("waypoints", waypoints);
      waypoints_list.clear();
      joint_group_positions.clear();
      

      }
    else{
      move_group_interface.stop();
    }
      step_val = false;


    }

    // if(test_val == true){
    //     moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //     joint_group_positions[4] = -1.5708;
    //     move_group_interface.setJointValueTarget(joint_group_positions);
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    //     bool success_2 = (move_group_interface.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
    //     test_val = false;




    

   

    ros::spinOnce();

    r.sleep();



  }





  return 0;
}
