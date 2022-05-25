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
#include "arm_planner/StepPos.h"
#include "arm_planner/FollowPos.h"
#include "arm_planner/Cartesian.h"



static int flag{0};
static bool reset_val{false};
static bool step_val{false};
static std::vector<double> joint_group_positions;
static std::vector<double> pos_group_positions;
static bool set_gripper{false};
static bool set_pos_gripper{false};
static std::vector<std::vector <double>> waypoints;
static std::vector<std::vector <double>> waypoints_pos;
static std::vector<double> waypoints_list;
static std::vector<double> waypoints_pos_list;
static int set_follow{0};
static bool test_val{false};
static std::vector <double> joints_seq;
static std::vector <double> pos_seq;
static double joint1{0.0}, joint2{0.0}, joint3{0.0}, joint4{0.0}, joint5{0.0}, joint6{0.0};
static double x_pos{0.0}, y_pos{0.0}, z_pos{0.0}, w_pos{0.0};
static bool gripper_req{false};
static bool gripper_pos_req{false};
static bool bool_param{false};
static std::vector<double> sample_waypoints;
static bool step_pos_val{false};
static int set_follow_pos{0};
static const std::string PLANNING_GROUP = "arm";
static geometry_msgs::Pose target_pose;
static bool cartesian_val{false};
// static moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
// static moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
static std::vector<geometry_msgs::Pose> cartesian_waypoints;
static geometry_msgs::Pose target_pose_2;
static double cartesian_x_pos{0.0}, cartesian_y_pos{0.0}, cartesian_z_pos{0.0};
static bool increment_pos{0};






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

  


  return true;


}


bool step_pos_fn(arm_planner::StepPos::Request &req, arm_planner::StepPos::Response &res){
  step_pos_val = true;
  x_pos = req.x;
  y_pos = req.y;
  z_pos = req.z;
  w_pos = req.w;
  gripper_pos_req = req.gripper_pos_status;



  return true;



}




bool follow_fn(arm_planner::Follow::Request &req, arm_planner::Follow::Response &res){

  set_follow = req.follow;
  return true;
  
}


bool follow_pos_fn(arm_planner::FollowPos::Request &req, arm_planner::FollowPos::Response &res){

  set_follow_pos = req.follow_pos;
  return true;
  
}

bool cartesian_pos_fn(arm_planner::Cartesian::Request &req, arm_planner::Cartesian::Response &res){

  cartesian_val = true;
  cartesian_x_pos = req.cartesian_x;
  cartesian_y_pos = req.cartesian_y;
  cartesian_z_pos = req.cartesian_z;






  return true;

}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint/position_controllers/JointPositionController", 10);
  ros::Publisher pub = nh.advertise<std_msgs::Float64>("/pincer_joint_position_controller/command", 10);
  // /hdt_arm/pincer_joint_position_controller/command
  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("hdt_arm/pincer_joint/fake_pincer_controller", 10);
  // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/hdt_arm/pincer_joint_position_controller/command", 10);


  ros::ServiceServer gripper_service = nh.advertiseService("gripper", gripper_fn);
  ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
  ros::ServiceServer step_service = nh.advertiseService("step", step_fn);
  ros::ServiceServer step_pos_service = nh.advertiseService("step_pos", step_pos_fn);
  ros::ServiceServer test_service = nh.advertiseService("test", test_fn);
  ros::ServiceServer follow_service = nh.advertiseService("follow", follow_fn);
  ros::ServiceServer follow_pos_service = nh.advertiseService("follow_pos", follow_pos_fn);
  ros::ServiceServer cartesian_pos_service = nh.advertiseService("cartesian_pos", cartesian_pos_fn);


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
  


  //Add stand
  moveit_msgs::CollisionObject collision;
  collision.header.frame_id = "base_link";
  collision.id = 1;
  
  shape_msgs::SolidPrimitive stand;
  stand.type = stand.BOX;
  stand.dimensions.resize(3);
  stand.dimensions[stand.BOX_X] = 0.05;
  stand.dimensions[stand.BOX_Y] = 0.05;
  stand.dimensions[stand.BOX_Z] = 0.45;

  geometry_msgs::Pose stand_pose;
  stand_pose.orientation.w = 1.0;
  stand_pose.position.x = 0.0;
  stand_pose.position.y = 0.0;
  stand_pose.position.z = -0.225;
  collision.primitive_poses.push_back(stand_pose);

  collision.primitives.push_back(stand);
  collision.operation = collision.ADD;
  collision_objects.push_back(collision);






 //Add Ground
  moveit_msgs::CollisionObject ground_collision;
  ground_collision.header.frame_id = "base_link";
  ground_collision.id = 2;
  
  shape_msgs::SolidPrimitive ground;
  ground.type = ground.BOX;
  ground.dimensions.resize(3);
  ground.dimensions[ground.BOX_X] = 2.0;
  ground.dimensions[ground.BOX_Y] = 2.0;
  ground.dimensions[ground.BOX_Z] = 0.001;

  geometry_msgs::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = -0.45;
  ground_collision.primitive_poses.push_back(ground_pose);

  ground_collision.primitives.push_back(ground);
  ground_collision.operation = ground_collision.ADD;
  collision_objects.push_back(ground_collision);


  //table
  moveit_msgs::CollisionObject collision_object;
  

  collision_object.header.frame_id = "base_link";
  collision_object.id = 3;

  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.58;
  primitive.dimensions[primitive.BOX_Y] = 0.60;
  primitive.dimensions[primitive.BOX_Z] = 0.54;


  geometry_msgs::Pose brick_pose;

  brick_pose.orientation.w = 1.0;
  brick_pose.position.x = 0.0;
  brick_pose.position.y = 0.565;
  brick_pose.position.z = -0.225;
  collision_object.primitive_poses.push_back(brick_pose);


  
  collision_object.primitives.push_back(primitive);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);



  //cylinder
  moveit_msgs::CollisionObject collision_cylinder1;
  

  collision_cylinder1.header.frame_id = "base_link";
  collision_cylinder1.id = 4;

  shape_msgs::SolidPrimitive cylinder_primitive;

  cylinder_primitive.type = cylinder_primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.257;
  cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder1_pose;

  cylinder1_pose.orientation.w = 1.0;
  cylinder1_pose.position.x = 0.0968;
  cylinder1_pose.position.y = 0.4188;
  cylinder1_pose.position.z = 0.1285;
  collision_cylinder1.primitive_poses.push_back(cylinder1_pose);


  
  collision_cylinder1.primitives.push_back(cylinder_primitive);
  collision_cylinder1.operation = collision_cylinder1.ADD;
  collision_objects.push_back(collision_cylinder1);

  


  //Add objects to planning scene
  planning_scene_interface.addCollisionObjects(collision_objects);


  std::vector <double> joints_check_1 = move_group_interface.getCurrentJointValues();

  

  ros::Rate r(120);

  while(ros::ok()){

  
    //call gripper open service
    if (flag == 1){
      std_msgs::Float64 msg;
      msg.data = 0.8;
      std::cout << "gripper openeed" << std::endl;
      pub.publish(msg);
      flag = 0;

    }

    //call gripper close service
    if (flag == 2){
      std_msgs::Float64 msg;
      msg.data = 0.1;
      std::cout << "gripper closed" << std::endl;
      pub.publish(msg);
      flag = 0;
    }

    //reset service - return to start configuration
    if(reset_val == true){
        std::cout << "reset waypoints: " << waypoints.size() << std::endl;
        // nh.setParam("/waypoints", waypoints);
        move_group_interface.setMaxVelocityScalingFactor(1.0);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);

        move_group_interface.setNamedTarget("ready");
        move_group_interface.move();
        reset_val = false;

    }


    //step service for joint goals
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

      
      
      move_group_interface.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


      
      if(success == true){
        std::cout << "success reached" << std::endl;
        move_group_interface.execute(my_plan);
        
        waypoints_list.push_back(joint_group_positions[0]);
        waypoints_list.push_back(joint_group_positions[1]);
        waypoints_list.push_back(joint_group_positions[2]);
        waypoints_list.push_back(joint_group_positions[3]);
        waypoints_list.push_back(joint_group_positions[4]);
        waypoints_list.push_back(joint_group_positions[5]);
        waypoints_list.push_back(joint_group_positions[6]);

        
        waypoints.push_back(waypoints_list);

        

        for(int z = 0; z < waypoints.size(); z++){
          for(int j = 0; j < waypoints[z].size(); j++){
            std::cout << "waypoints joint angles: " << waypoints[z][j] << std::endl;
          }
        }
        std::cout << "waypoints size joint angles: " << waypoints.size() << std::endl;



        if(gripper_req == true){
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

    
    //test service
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



    //follow condition 1
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


    //follow condition 2
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


    //step service for pose goal
    if(step_pos_val == true){
      // geometry_msgs::Pose target_pose;
      target_pose.orientation.w = 1.0;
      target_pose.position.x = x_pos;
      target_pose.position.y = y_pos;
      target_pose.position.z = z_pos;
      
      
      move_group_interface.setStartStateToCurrentState();
      move_group_interface.setMaxVelocityScalingFactor(1.0);
      move_group_interface.setMaxAccelerationScalingFactor(1.0);


      move_group_interface.setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

      bool success = (move_group_interface.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "success: " << success << std::endl;


      
      if(success == true){
        std::cout << "pos success reached" << std::endl;
        move_group_interface.execute(my_plan2);
        

        waypoints_pos_list.push_back(target_pose.position.x);
        waypoints_pos_list.push_back(target_pose.position.y);
        waypoints_pos_list.push_back(target_pose.position.z);
        if (gripper_pos_req == true){
          waypoints_pos_list.push_back(1.0);
        }
        else{
          waypoints_pos_list.push_back(0.0);
        }
        

        
        waypoints_pos.push_back(waypoints_pos_list);
       
        

        for(int z = 0; z < waypoints_pos.size(); z++){
          for(int j = 0; j < waypoints_pos[z].size(); j++){
            std::cout << "waypoints for 2d pos: " << waypoints_pos[z][j] << std::endl;
          }
        }
        std::cout << "waypoints size pos: " << waypoints_pos.size() << std::endl;



        if(gripper_pos_req == true){
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
      waypoints_pos_list.clear();
      

      }
    else{
      move_group_interface.stop();
    }
      step_pos_val = false;


    }



    //follow pos goal condition 1
    if(set_follow_pos == 1){
      std::cout << "reached follow service pos goal" << std::endl;
      
      while(1){
        for(int z = 0; z < waypoints_pos.size(); z++){
            geometry_msgs::Pose follow_pose;
            follow_pose.orientation.w = 1.0;
            follow_pose.position.x = waypoints_pos[z][0];
            follow_pose.position.y = waypoints_pos[z][1];
            follow_pose.position.z = waypoints_pos[z][2];
      
            double gripper_value_pos = waypoints[z][3];

            

            move_group_interface.setPoseTarget(follow_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            move_group_interface.setPlanningTime(5.0);
            move_group_interface.move();

            if(gripper_value_pos == 1.0){
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


          }
          set_follow_pos = 0;


      }

      }


    //follow pos goal condition 2
    if (set_follow_pos == 2){
      std::cout << "follow else condition reached pos" << std::endl;
      for(int z = 0; z < waypoints_pos.size(); z++){
        geometry_msgs::Pose follow_pose;
        follow_pose.orientation.w = 1.0;
        follow_pose.position.x = waypoints_pos[z][0];
        follow_pose.position.y = waypoints_pos[z][1];
        follow_pose.position.z = waypoints_pos[z][2];
  
        double gripper_value_pos = waypoints_pos[z][3];

        

        move_group_interface.setPoseTarget(follow_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_interface.setPlanningTime(5.0);
        move_group_interface.move();


        if(gripper_value_pos == 1.0){
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

          }
      set_follow_pos = 0;

    }


    if (cartesian_val == true){

      // geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
      move_group_interface.setStartStateToCurrentState();

      
      target_pose_2.position.x = target_pose.position.x;
      target_pose_2.position.y = target_pose.position.y;
      target_pose_2.position.z = target_pose.position.z;

      std::cout << "get_current_pose_x: " << target_pose_2.position.x << std::endl;
      std::cout << "get_current_pose_y: " << target_pose_2.position.y << std::endl;
      std::cout << "get_current_pose_z: " << target_pose_2.position.z << std::endl;


      target_pose_2.position.x -= cartesian_x_pos;
      target_pose_2.position.y -= cartesian_y_pos;
      target_pose_2.position.z -= cartesian_z_pos;
      cartesian_waypoints.push_back(target_pose_2);

      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_interface.computeCartesianPath(cartesian_waypoints, eef_step, jump_threshold, trajectory);

      move_group_interface.execute(trajectory);

      cartesian_val = false;



    }



    ros::spinOnce();

    r.sleep();



  }


  spinner.stop();


  return 0;
}