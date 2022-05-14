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
#include <math.h>
#include <string.h>


static int flag{0};
static bool reset_val{false};
static bool step_val{false};
static std::vector<double> joint_group_positions;
static bool set_gripper{false};
static std::vector<std::vector <double>> waypoints;
static std::vector<double> waypoints_list;
static int set_follow{0};
static bool test_val{false};
static std::vector <double> joints_seq;
static double joint1{0.0}, joint2{0.0}, joint3{0.0}, joint4{0.0}, joint5{0.0}, joint6{0.0};
static bool gripper_req{false};
static bool bool_param{false};
static std::vector<double> sample_waypoints;

// const double tau = 2 * M_PI;


// static bool gripper_step{false};

// static const std::string PLANNING_GROUP = "arm";
// static moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
// static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;




bool reset_fn(arm_planner::Reset::Request &req, arm_planner::Reset::Response &res){

  

  reset_val = req.restart;

  if(reset_val == true){
    // waypoints.clear();
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

bool test_fn(arm_planner::Test::Request &req, arm_planner::Test::Response &res){
  test_val = true;

  return true;

}


bool step_fn(arm_planner::Step::Request &req, arm_planner::Step::Response &res){
  step_val = true;
  joint1 = req.j1;
  joint2 = req.j2;
  joint3 = req.j3;
  joint4 = req.j4;
  joint5 = req.j5;
  joint6 = req.j6;
  gripper_req = req.gripper_status;

  std::cout << "j1: %d" << std::endl;
  std::cout << "j2: %d" << std::endl;
  std::cout << "j3: %d" << std::endl;
  std::cout << "j4: %d" << std::endl;
  std::cout << "j5: %d" << std::endl;
  std::cout << "j6: %d" << std::endl;



  return true;


}



bool follow_fn(arm_planner::Follow::Request &req, arm_planner::Follow::Response &res){

  set_follow = req.follow;
  return true;
  
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint/position_controllers/JointPositionController", 10);
  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint_position_controller/command", 10);
  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("hdt_arm/pincer_joint/fake_pincer_controller", 10);
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);


  ros::ServiceServer gripper_service = nh.advertiseService("gripper", gripper_fn);
  ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
  ros::ServiceServer step_service = nh.advertiseService("step", step_fn);
  // ros::ServiceServer joints_service = nh.advertiseService("joint_control", joint_control_fn);
  // ros::ServiceClient joints_client = nh.serviceClient()
  ros::ServiceServer test_service = nh.advertiseService("test", test_fn);
  ros::ServiceServer follow_service = nh.advertiseService("follow", follow_fn);

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

  
  // ROS_INFO_NAMED("Planner id: %s", move_group_interface.getDefaultPlannerId(PLANNING_GROUP).c_str());
  // ROS_INFO_NAMED("Default planning pipeline id: %s", move_group_interface.getDefaultPlanningPipelineId().c_str());
  // // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(move_group_interface.getJointModelGroupNames().begin(),
  //           move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // move_group_interface.setPlannerId();
  

  std::string default_planner_id = move_group_interface.getPlannerId();
  // std::string def_planning_pipe_id = move_group_interface.getPlanningPipelineId();

  std::cout << "default planner id: " << default_planner_id << std::endl;
  // std::cout << "default planning pipeline id: %s" << def_planning_pipe_id << std::endl;



  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //get waypoints
  // nh.getParam("/path_planner/waypoints", sample_waypoints);
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



  //brick1
  moveit_msgs::CollisionObject collision_object;
  

  collision_object.header.frame_id = "base_link";
  collision_object.id = 1;

  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.24;
  primitive.dimensions[primitive.BOX_Y] = 0.07;
  primitive.dimensions[primitive.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose;

  brick_pose.orientation.w = 1.0;
  brick_pose.position.x = 0.0;
  brick_pose.position.y = 0.28;
  brick_pose.position.z = 0.056;
  collision_object.primitive_poses.push_back(brick_pose);


  
  collision_object.primitives.push_back(primitive);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);


  //brick2
  moveit_msgs::CollisionObject collision_object_2;
    

  collision_object_2.header.frame_id = "base_link";
  collision_object_2.id = 2;

  shape_msgs::SolidPrimitive primitive_2;

  primitive_2.type = primitive_2.BOX;
  primitive_2.dimensions.resize(3);
  primitive_2.dimensions[primitive_2.BOX_X] = 0.24;
  primitive_2.dimensions[primitive_2.BOX_Y] = 0.07;
  primitive_2.dimensions[primitive_2.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_2;

  brick_pose_2.orientation.w = 1.0;
  brick_pose_2.position.x = 0.0;
  brick_pose_2.position.y = 0.28;
  brick_pose_2.position.z = (0.112);
  collision_object_2.primitive_poses.push_back(brick_pose_2);


  
  collision_object_2.primitives.push_back(primitive_2);
  collision_object_2.operation = collision_object_2.ADD;
  collision_objects.push_back(collision_object_2);


  //brick3
  moveit_msgs::CollisionObject collision_object_3;
    

  collision_object_3.header.frame_id = "base_link";
  collision_object_3.id = 3;

  shape_msgs::SolidPrimitive primitive_3;

  primitive_3.type = primitive_3.BOX;
  primitive_3.dimensions.resize(3);
  primitive_3.dimensions[primitive_3.BOX_X] = 0.24;
  primitive_3.dimensions[primitive_3.BOX_Y] = 0.07;
  primitive_3.dimensions[primitive_3.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_3;

  brick_pose_3.orientation.w = 1.0;
  brick_pose_3.position.x = 0.0;
  brick_pose_3.position.y = 0.28;
  brick_pose_3.position.z = (0.168);
  collision_object_3.primitive_poses.push_back(brick_pose_3);


  
  collision_object_3.primitives.push_back(primitive_3);
  collision_object_3.operation = collision_object_3.ADD;
  collision_objects.push_back(collision_object_3);


  //brick4
  moveit_msgs::CollisionObject collision_object_4;
    

  collision_object_4.header.frame_id = "base_link";
  collision_object_4.id = 4;

  shape_msgs::SolidPrimitive primitive_4;

  primitive_4.type = primitive_4.BOX;
  primitive_4.dimensions.resize(3);
  primitive_4.dimensions[primitive_4.BOX_X] = 0.24;
  primitive_4.dimensions[primitive_4.BOX_Y] = 0.07;
  primitive_4.dimensions[primitive_4.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_4;

  brick_pose_4.orientation.w = 1.0;
  brick_pose_4.position.x = 0.0;
  brick_pose_4.position.y = 0.28;
  brick_pose_4.position.z = (0.224);
  collision_object_4.primitive_poses.push_back(brick_pose_4);


  
  collision_object_4.primitives.push_back(primitive_4);
  collision_object_4.operation = collision_object_4.ADD;
  collision_objects.push_back(collision_object_4);



  //brick5
  moveit_msgs::CollisionObject collision_object_5;
    

  collision_object_5.header.frame_id = "base_link";
  collision_object_5.id = 5;

  shape_msgs::SolidPrimitive primitive_5;

  primitive_5.type = primitive_5.BOX;
  primitive_5.dimensions.resize(3);
  primitive_5.dimensions[primitive_5.BOX_X] = 0.24;
  primitive_5.dimensions[primitive_5.BOX_Y] = 0.07;
  primitive_5.dimensions[primitive_5.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_5;

  brick_pose_5.orientation.w = 1.0;
  brick_pose_5.position.x = 0.0;
  brick_pose_5.position.y = -0.28;
  brick_pose_5.position.z = (0.056);
  collision_object_5.primitive_poses.push_back(brick_pose_5);


  
  collision_object_5.primitives.push_back(primitive_5);
  collision_object_5.operation = collision_object_5.ADD;
  collision_objects.push_back(collision_object_5);


  //brick6
  moveit_msgs::CollisionObject collision_object_6;
    

  collision_object_6.header.frame_id = "base_link";
  collision_object_6.id = 6;

  shape_msgs::SolidPrimitive primitive_6;

  primitive_6.type = primitive_6.BOX;
  primitive_6.dimensions.resize(3);
  primitive_6.dimensions[primitive_6.BOX_X] = 0.24;
  primitive_6.dimensions[primitive_6.BOX_Y] = 0.07;
  primitive_6.dimensions[primitive_6.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_6;

  brick_pose_6.orientation.w = 1.0;
  brick_pose_6.position.x = 0.0;
  brick_pose_6.position.y = -0.28;
  brick_pose_6.position.z = (0.112);
  collision_object_6.primitive_poses.push_back(brick_pose_6);


  
  collision_object_6.primitives.push_back(primitive_6);
  collision_object_6.operation = collision_object_6.ADD;
  collision_objects.push_back(collision_object_6);


  //brick7
  moveit_msgs::CollisionObject collision_object_7;
    

  collision_object_7.header.frame_id = "base_link";
  collision_object_7.id = 7;

  shape_msgs::SolidPrimitive primitive_7;

  primitive_7.type = primitive_7.BOX;
  primitive_7.dimensions.resize(3);
  primitive_7.dimensions[primitive_7.BOX_X] = 0.24;
  primitive_7.dimensions[primitive_7.BOX_Y] = 0.07;
  primitive_7.dimensions[primitive_7.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_7;

  brick_pose_7.orientation.w = 1.0;
  brick_pose_7.position.x = 0.0;
  brick_pose_7.position.y = -0.28;
  brick_pose_7.position.z = (0.168);
  collision_object_7.primitive_poses.push_back(brick_pose_7);


  
  collision_object_7.primitives.push_back(primitive_7);
  collision_object_7.operation = collision_object_7.ADD;
  collision_objects.push_back(collision_object_7);


  //brick8
  moveit_msgs::CollisionObject collision_object_8;
    

  collision_object_8.header.frame_id = "base_link";
  collision_object_8.id = 8;

  shape_msgs::SolidPrimitive primitive_8;

  primitive_8.type = primitive_8.BOX;
  primitive_8.dimensions.resize(3);
  primitive_8.dimensions[primitive_8.BOX_X] = 0.24;
  primitive_8.dimensions[primitive_8.BOX_Y] = 0.07;
  primitive_8.dimensions[primitive_8.BOX_Z] = 0.112;


  geometry_msgs::Pose brick_pose_8;

  brick_pose_8.orientation.w = 1.0;
  brick_pose_8.position.x = 0.0;
  brick_pose_8.position.y = -0.28;
  brick_pose_8.position.z = (0.224);
  collision_object_8.primitive_poses.push_back(brick_pose_8);


  
  collision_object_8.primitives.push_back(primitive_8);
  collision_object_8.operation = collision_object_8.ADD;
  collision_objects.push_back(collision_object_8);



  //cylinder1
  moveit_msgs::CollisionObject cylinder1;

  cylinder1.header.frame_id = "base_link";
  cylinder1.id = 9;

  shape_msgs::SolidPrimitive cylinder1_primitive;

  cylinder1_primitive.type = cylinder1_primitive.CYLINDER;
  cylinder1_primitive.dimensions.resize(2);
  cylinder1_primitive.dimensions[cylinder1_primitive.CYLINDER_HEIGHT] = 0.4;
  cylinder1_primitive.dimensions[cylinder1_primitive.CYLINDER_RADIUS] = 0.025;


  geometry_msgs::Pose cylinder1_pose;
  cylinder1_pose.orientation.w = 1.0;
  cylinder1_pose.position.x = 0.212;
  cylinder1_pose.position.y = -0.2;
  cylinder1_pose.position.z = 0.2;
  cylinder1.primitive_poses.push_back(cylinder1_pose);

  cylinder1.primitives.push_back(cylinder1_primitive);
  cylinder1.operation = cylinder1.ADD;
  collision_objects.push_back(cylinder1);


  //cylinder2

  moveit_msgs::CollisionObject cylinder2;

  cylinder2.header.frame_id = "base_link";
  cylinder2.id = 10;

  shape_msgs::SolidPrimitive cylinder2_primitive;

  cylinder2_primitive.type = cylinder2_primitive.CYLINDER;
  cylinder2_primitive.dimensions.resize(2);
  cylinder2_primitive.dimensions[cylinder2_primitive.CYLINDER_HEIGHT] = 0.4;
  cylinder2_primitive.dimensions[cylinder2_primitive.CYLINDER_RADIUS] = 0.025;


  geometry_msgs::Pose cylinder2_pose;
  cylinder2_pose.orientation.w = 1.0;
  cylinder2_pose.position.x = 0.212;
  cylinder2_pose.position.y = 0.2;
  cylinder2_pose.position.z = 0.2;
  cylinder2.primitive_poses.push_back(cylinder2_pose);

  cylinder2.primitives.push_back(cylinder2_primitive);
  cylinder2.operation = cylinder2.ADD;
  collision_objects.push_back(cylinder2);







  for(int z = 0; z < sample_waypoints.size(); z++){
    std::cout << "sample_waypoints: " << sample_waypoints[z] << std::endl;
            // std::cout << std::endl;
          }


















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
        // nh.setParam("/waypoints", waypoints);
        move_group_interface.setNamedTarget("ready");
        move_group_interface.move();
        reset_val = false;

    }


    if(step_val == true){
      std::cout << "outside the service fn" << std::endl;


      joint_group_positions = move_group_interface.getCurrentJointValues();
      move_group_interface.setStartStateToCurrentState();
      move_group_interface.setMaxVelocityScalingFactor(1.0);

      std::cout << "step value reached" << std::endl;
      joint_group_positions[0] = (joint1*M_PI)/180.0;
      std::cout << "filled first joint state" << std::endl;
      joint_group_positions[1] = (joint2*M_PI)/180.0;
      joint_group_positions[2] = (joint3*M_PI)/180.0;
      joint_group_positions[3] = (joint4*M_PI)/180.0;
      joint_group_positions[4] = (joint5*M_PI)/180.0;
      joint_group_positions[5] = (joint6*M_PI)/180.0;
      // set_gripper = gripper_req;
      if (gripper_req == true){
        joint_group_positions[6] = 1.0;
      }
      else{
        joint_group_positions[6] = 0.0;
      }
      std::cout << "joint_group_pos 0: " << joint_group_positions[0] << std::endl;
      std::cout << "joint_group_pos 1: " << joint_group_positions[1] << std::endl;
      std::cout << "joint_group_pos 2: " << joint_group_positions[2] << std::endl;
      std::cout << "joint_group_pos 3: " << joint_group_positions[3] << std::endl;
      std::cout << "joint_group_pos 4: " << joint_group_positions[4] << std::endl;
      std::cout << "joint_group_pos 5: " << joint_group_positions[5] << std::endl;
      std::cout << "joint_group_pos 6: " << joint_group_positions[6] << std::endl;


      std::cout << "start filing waypoints" << std::endl;
      
      std::cout << "filled temp: " << std::endl;

      waypoints_list.push_back(joint_group_positions[0]);
      waypoints_list.push_back(joint_group_positions[1]);
      waypoints_list.push_back(joint_group_positions[2]);
      waypoints_list.push_back(joint_group_positions[3]);
      waypoints_list.push_back(joint_group_positions[4]);
      waypoints_list.push_back(joint_group_positions[5]);
      waypoints_list.push_back(joint_group_positions[6]);
      std::cout << "waypoints filled" << std::endl;

      // set_gripper = req.gripper_status;
      std::cout << "inside the service fn" << std::endl;
      

      
      // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
      // std::vector<double> joint_group_positions;
      // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      
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
        // std::vector <double> joints_check = move_group_interface.getCurrentJointValues();

        // for(int a = 0; a < joints_check.size(); a++){
        //   std::cout << "joint angle at: " << a << "is: " << joints_check.at(a) << std::endl;
        // }
        

        
        waypoints.push_back(waypoints_list);
        // s

        
        // std::cout << "waypoints: " << waypoints[0][0] << std::endl;

        

        for(int z = 0; z < waypoints.size(); z++){
          for(int j = 0; j < waypoints[z].size(); j++){
            std::cout << "waypoints for 2d: " << waypoints[z][j] << std::endl;
          }
          // std::cout << std::endl;
        }
        // nh.setParam("/waypoints", {1.0, 2.0, 3.0, 4.0});
        std::cout << "waypoints size: " << waypoints.size() << std::endl;
        // nh.setParam("/path_planner/waypoints", waypoints);



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
      waypoints_list.clear();
      joint_group_positions.clear();
      

      }
    else{
      move_group_interface.stop();
    }
      step_val = false;


    }

    if(test_val == true){
      //reference - Tiago - planning joint space
      
      std::vector <double> joint_test = move_group_interface.getCurrentJointValues();
      move_group_interface.setStartStateToCurrentState();
      move_group_interface.setMaxVelocityScalingFactor(1.0);
      joint_test[0] = 0.0;
      joint_test[1] = 0.0;
      joint_test[2] = 0.0;
      joint_test[3] = -1.57079;
      joint_test[4] = -1.4835;
      joint_test[5] = 1.65806;
      move_group_interface.setJointValueTarget(joint_test);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group_interface.setPlanningTime(5.0);
      bool success_2 = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if ( !success_2 )
        throw std::runtime_error("No plan found");
  
      // ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      ros::Time start = ros::Time::now();

      move_group_interface.move();

      // ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

      std::vector <double> joints_move = move_group_interface.getCurrentJointValues();

      for(int a = 0; a < joints_move.size(); a++){
        std::cout << "joint angle at: " << a << "is: " << joints_move.at(a) << std::endl;
      }
      
      double goal_tolerance = move_group_interface.getGoalJointTolerance();

      std::cout << "goal tolerance is: " << goal_tolerance << std::endl;

        
      test_val = false;
    }

    
    if(set_follow == 1){
      std::cout << "reached follow service" << std::endl;
      
      while(1){
        for(int z = 0; z < waypoints.size(); z++){
            joints_seq.push_back(waypoints[z][0]);
            joints_seq.push_back(waypoints[z][1]);
            joints_seq.push_back(waypoints[z][2]);
            joints_seq.push_back(waypoints[z][3]);
            joints_seq.push_back(waypoints[z][4]);
            joints_seq.push_back(waypoints[z][5]);
            double gripper_value = waypoints[z][6];

            

            move_group_interface.setJointValueTarget(joints_seq);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            move_group_interface.setPlanningTime(5.0);
            move_group_interface.move();

            if(gripper_value == 1.0){
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
          joints_seq.clear();


          }
          set_follow = 0;


        }



        

      }

    if (set_follow == 2){
      std::cout << "follow else condition reached" << std::endl;
      for(int z = 0; z < waypoints.size(); z++){
        joints_seq.push_back(waypoints[z][0]);
        joints_seq.push_back(waypoints[z][1]);
        joints_seq.push_back(waypoints[z][2]);
        joints_seq.push_back(waypoints[z][3]);
        joints_seq.push_back(waypoints[z][4]);
        joints_seq.push_back(waypoints[z][5]);
        double gripper_value = waypoints[z][6];

        

        move_group_interface.setJointValueTarget(joints_seq);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_interface.setPlanningTime(5.0);
        move_group_interface.move();


        if(gripper_value == 1.0){
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
      joints_seq.clear();

          }
      set_follow = 0;

    }


      





    



    ros::spinOnce();

    r.sleep();



  }


  spinner.stop();


  return 0;
}
