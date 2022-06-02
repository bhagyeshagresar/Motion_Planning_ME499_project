#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <std_msgs/String.h>
#include "arm_planner/Gripper.h"
#include <std_srvs/Empty.h>
#include "arm_planner/Reset.h"
#include "arm_planner/Step.h"
#include "arm_planner/Test.h"
#include "arm_planner/Attach.h"
#include <math.h>
#include <string.h>
#include "arm_planner/StepPos.h"
#include "arm_planner/Cartesian.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "arm_planner/Detach.h"
#include "XmlRpcValue.h"
#include "arm_planner/Follow.h"



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
static bool test_val{false};
static std::vector <double> joints_seq;
static std::vector <double> pos_seq;
static double joint1{0.0}, joint2{0.0}, joint3{0.0}, joint4{0.0}, joint5{0.0}, joint6{0.0};
static double x_pos{0.0}, y_pos{0.0}, z_pos{0.0};
static double roll{0.0}, pitch{0.0}, yaw{0.0};
static bool gripper_req{false};
static bool gripper_pos_req{false};
static bool bool_param{false};
static std::vector<double> sample_waypoints;
static bool step_pos_val{false};
static bool cartesian_val{false};
static std::vector<geometry_msgs::Pose> cartesian_waypoints;
static double cartesian_x_delta{0.0}, cartesian_y_delta{0.0}, cartesian_z_delta{0.0};
static bool increment_pos{0};
static bool attach_obj_val{false};
static double obj_x{0.0}, obj_y{0.0}, obj_z{0.0}, obj_roll{0.0}, obj_pitch{0.0}, obj_yaw{0.0};
static bool detach_obj_val{false};
static std::string cylinder_id;
static std::string cylinder_id_2;
static bool set_follow{false};
static bool trigger_trajectory{false};
static XmlRpc::XmlRpcValue cylinder1;
static XmlRpc::XmlRpcValue cylinder2;
static ros::Publisher pub;




void cylinder_fn(XmlRpc::XmlRpcValue &cylinder);
void set_pose_target_fn(XmlRpc::XmlRpcValue &cylinder, int index, moveit::planning_interface::MoveGroupInterface& move_group_interface);
void set_joint_target_fn(XmlRpc::XmlRpcValue &cylinder, int index);
void set_cartesian_fn(XmlRpc::XmlRpcValue &cylinder, int index);
void set_attach_fn(XmlRpc::XmlRpcValue &cylinder, int index);
void set_detach_fn(XmlRpc::XmlRpcValue &cylinder, int index);



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
  roll = req.roll_angle;
  pitch = req.pitch_angle;
  yaw = req.yaw_angle;
  gripper_pos_req = req.gripper_pos_status;



  return true;



}


bool cartesian_pos_fn(arm_planner::Cartesian::Request &req, arm_planner::Cartesian::Response &res){

  cartesian_val = true;
  cartesian_x_delta = req.x_delta;
  cartesian_y_delta = req.y_delta;
  cartesian_z_delta = req.z_delta;

  return true;

}

bool attach_obj_fn(arm_planner::Attach::Request &req, arm_planner::Attach::Response &res){
  
  attach_obj_val = true;
  cylinder_id = req.cylinder_name;

  return true;

}


bool detach_obj_fn(arm_planner::Detach::Request &req, arm_planner::Detach::Response &res){
  detach_obj_val = true;
  cylinder_id_2 = req.cylinder_name_2;
  return true;
}



bool follow_fn(arm_planner::Follow::Request &req, arm_planner::Follow::Response &res){
  trigger_trajectory = true;

  return true;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  pub = nh.advertise<std_msgs::Float64>("/pincer_joint_position_controller/command", 10);
  

  ros::ServiceServer gripper_service = nh.advertiseService("gripper", gripper_fn);
  ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
  ros::ServiceServer step_service = nh.advertiseService("step", step_fn);
  ros::ServiceServer step_pos_service = nh.advertiseService("step_pos", step_pos_fn);
  ros::ServiceServer test_service = nh.advertiseService("test", test_fn);
  ros::ServiceServer cartesian_pos_service = nh.advertiseService("cartesian_pos", cartesian_pos_fn);
  ros::ServiceServer attach_obj_service = nh.advertiseService("attach_obj", attach_obj_fn);
  ros::ServiceServer detach_obj_service = nh.advertiseService("detach_obj", detach_obj_fn);
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

  
  
  std::string default_planner_id = move_group_interface.getPlannerId();

  std::cout << "default planner id: " << default_planner_id << std::endl;

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  nh.getParam("cylinder1", cylinder1);
  nh.getParam("cylinder2", cylinder2);



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









  //table
  moveit_msgs::CollisionObject collision_object;
  

  collision_object.header.frame_id = "base_link";
  collision_object.id = 3;

  shape_msgs::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.58;
  primitive.dimensions[primitive.BOX_Y] = 0.7;
  primitive.dimensions[primitive.BOX_Z] = 0.54;


  geometry_msgs::Pose brick_pose;

  brick_pose.orientation.w = 1.0;
  brick_pose.position.x = 0.0;
  brick_pose.position.y = 0.57;
  brick_pose.position.z = -0.18;
  collision_object.primitive_poses.push_back(brick_pose);


  
  collision_object.primitives.push_back(primitive);
  collision_object.operation = collision_object.ADD;
  collision_objects.push_back(collision_object);



  //cylinder1
  moveit_msgs::CollisionObject collision_cylinder1;
  

  collision_cylinder1.header.frame_id = "base_link";
  collision_cylinder1.id = "1";

  shape_msgs::SolidPrimitive cylinder_primitive;

  cylinder_primitive.type = cylinder_primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.335;
  cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder1_pose;

  cylinder1_pose.orientation.w = 1.0;
  cylinder1_pose.position.x = 0.0;
  cylinder1_pose.position.y = 0.38;
  cylinder1_pose.position.z = 0.2575;
  collision_cylinder1.primitive_poses.push_back(cylinder1_pose);


  
  collision_cylinder1.primitives.push_back(cylinder_primitive);
  collision_cylinder1.operation = collision_cylinder1.ADD;
  collision_objects.push_back(collision_cylinder1);

  

  //cylinder2
  moveit_msgs::CollisionObject collision_cylinder2;
  

  collision_cylinder2.header.frame_id = "base_link";
  collision_cylinder2.id = "2";

  shape_msgs::SolidPrimitive cylinder_primitive2;

  cylinder_primitive2.type = cylinder_primitive2.CYLINDER;
  cylinder_primitive2.dimensions.resize(2);
  cylinder_primitive2.dimensions[cylinder_primitive2.CYLINDER_HEIGHT] = 0.335;
  cylinder_primitive2.dimensions[cylinder_primitive2.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder2_pose;

  cylinder2_pose.orientation.w = 1.0;
  cylinder2_pose.position.x = 0.15;
  cylinder2_pose.position.y = 0.38;
  cylinder2_pose.position.z = 0.2575;
  collision_cylinder2.primitive_poses.push_back(cylinder2_pose);


  
  collision_cylinder2.primitives.push_back(cylinder_primitive2);
  collision_cylinder2.operation = collision_cylinder2.ADD;
  collision_objects.push_back(collision_cylinder2);


  //cylinder3
  moveit_msgs::CollisionObject collision_cylinder3;
  

  collision_cylinder3.header.frame_id = "base_link";
  collision_cylinder3.id = "3";
  shape_msgs::SolidPrimitive cylinder_primitive3;

  cylinder_primitive3.type = cylinder_primitive3.CYLINDER;
  cylinder_primitive3.dimensions.resize(2);
  cylinder_primitive3.dimensions[cylinder_primitive3.CYLINDER_HEIGHT] = 0.335;
  cylinder_primitive3.dimensions[cylinder_primitive3.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder3_pose;

  cylinder3_pose.orientation.w = 1.0;
  cylinder3_pose.position.x = -0.15;
  cylinder3_pose.position.y = 0.38;
  cylinder3_pose.position.z = 0.2575;
  collision_cylinder3.primitive_poses.push_back(cylinder3_pose);


  
  collision_cylinder3.primitives.push_back(cylinder_primitive3);
  collision_cylinder3.operation = collision_cylinder3.ADD;
  collision_objects.push_back(collision_cylinder3);


  //cylinder4
  moveit_msgs::CollisionObject collision_cylinder4;
  

  collision_cylinder4.header.frame_id = "base_link";
  collision_cylinder4.id = "4";

  shape_msgs::SolidPrimitive cylinder_primitive4;

  cylinder_primitive4.type = cylinder_primitive4.CYLINDER;
  cylinder_primitive4.dimensions.resize(2);
  cylinder_primitive4.dimensions[cylinder_primitive4.CYLINDER_HEIGHT] = 0.257;
  cylinder_primitive4.dimensions[cylinder_primitive4.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder4_pose;

  cylinder4_pose.orientation.w = 1.0;
  cylinder4_pose.position.x = 0.0;
  cylinder4_pose.position.y = 0.58;
  cylinder4_pose.position.z = 0.2185;
  collision_cylinder4.primitive_poses.push_back(cylinder4_pose);


  
  collision_cylinder4.primitives.push_back(cylinder_primitive4);
  collision_cylinder4.operation = collision_cylinder4.ADD;
  collision_objects.push_back(collision_cylinder4);


  //cylinder5
  moveit_msgs::CollisionObject collision_cylinder5;
  

  collision_cylinder5.header.frame_id = "base_link";
  collision_cylinder5.id = "5";

  shape_msgs::SolidPrimitive cylinder_primitive5;

  cylinder_primitive5.type = cylinder_primitive5.CYLINDER;
  cylinder_primitive5.dimensions.resize(2);
  cylinder_primitive5.dimensions[cylinder_primitive5.CYLINDER_HEIGHT] = 0.257;
  cylinder_primitive5.dimensions[cylinder_primitive5.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder5_pose;

  cylinder5_pose.orientation.w = 1.0;
  cylinder5_pose.position.x = 0.15;
  cylinder5_pose.position.y = 0.58;
  cylinder5_pose.position.z = 0.2185;
  collision_cylinder5.primitive_poses.push_back(cylinder5_pose);


  
  collision_cylinder5.primitives.push_back(cylinder_primitive5);
  collision_cylinder5.operation = collision_cylinder5.ADD;
  collision_objects.push_back(collision_cylinder5);


  //cylinder6
  moveit_msgs::CollisionObject collision_cylinder6;
  

  collision_cylinder6.header.frame_id = "base_link";
  collision_cylinder6.id = "6";

  shape_msgs::SolidPrimitive cylinder_primitive6;

  cylinder_primitive6.type = cylinder_primitive6.CYLINDER;
  cylinder_primitive6.dimensions.resize(2);
  cylinder_primitive6.dimensions[cylinder_primitive6.CYLINDER_HEIGHT] = 0.257;
  cylinder_primitive6.dimensions[cylinder_primitive6.CYLINDER_RADIUS] = 0.04;


  geometry_msgs::Pose cylinder6_pose;

  cylinder6_pose.orientation.w = 1.0;
  cylinder6_pose.position.x = -0.15;
  cylinder6_pose.position.y = 0.58;
  cylinder6_pose.position.z = 0.2185;
  collision_cylinder6.primitive_poses.push_back(cylinder6_pose);


  
  collision_cylinder6.primitives.push_back(cylinder_primitive6);
  collision_cylinder6.operation = collision_cylinder6.ADD;
  collision_objects.push_back(collision_cylinder6);


  //Add objects to planning scene
  planning_scene_interface.addCollisionObjects(collision_objects);


  std::vector <double> joints_check_1 = move_group_interface.getCurrentJointValues();

  

  ros::Rate r(120);

  while(ros::ok()){

  
    //call gripper open service
    if (flag == 1){
      std_msgs::Float64 msg;
      msg.data = 0.3;
      std::cout << "gripper closed" << std::endl;
      pub.publish(msg);
      flag = 0;

    }

    //call gripper close service
    if (flag == 2){
      std_msgs::Float64 msg;
      msg.data = 0.8;
      std::cout << "gripper open" << std::endl;
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
      move_group_interface.setMaxVelocityScalingFactor(0.8);

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
     
      
      
      move_group_interface.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


      
      if(success == true){
        std::cout << "success reached" << std::endl;
        move_group_interface.execute(my_plan);
        
       
        if(gripper_req == true){
          std_msgs::Float64 msg;
          msg.data = 0.3;
          std::cout << "gripper closed" << std::endl;
          pub.publish(msg);
        }
        
        else{
          std_msgs::Float64 msg;
          msg.data = 0.8;
          std::cout << "gripper open" << std::endl;
          pub.publish(msg);
        
        
      }
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

    


    //step service for pose goal
    if(step_pos_val == true){
      geometry_msgs::Pose target_pose;
      tf2::Quaternion orient_pose;
      orient_pose.setRPY(roll, pitch, yaw);
      target_pose.orientation = tf2::toMsg(orient_pose);
      target_pose.position.x = x_pos;
      target_pose.position.y = y_pos;
      target_pose.position.z = z_pos;
      
      
      move_group_interface.setStartStateToCurrentState();
      move_group_interface.setMaxVelocityScalingFactor(0.8);
      move_group_interface.setMaxAccelerationScalingFactor(0.8);


      //check plan for target_pose
      move_group_interface.setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      bool success = (move_group_interface.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "pos goal success: " << success << std::endl;

     

      if(success == true){
        std::cout << "pos success reached" << std::endl;
        move_group_interface.execute(my_plan2);
      


        if(gripper_pos_req == true){
            std_msgs::Float64 msg;
            msg.data = 0.3;
            std::cout << "gripper closed" << std::endl;
            pub.publish(msg);
          }
          
        else{
            std_msgs::Float64 msg;
            msg.data = 0.8;
            std::cout << "gripper open" << std::endl;
            pub.publish(msg);
          
          
        }
      

      }
    else{
      move_group_interface.stop();
    }
      step_pos_val = false;


    }

  


    //Execute cartesian plan
    if (cartesian_val == true){

      geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
      std::vector<double> current_rpy = move_group_interface.getCurrentRPY();


      geometry_msgs::Pose cartesian_pose;
      tf2::Quaternion orient_pose3;
      orient_pose3.setRPY(current_rpy.at(0), current_rpy.at(1), current_rpy.at(2));
      cartesian_pose.orientation = tf2::toMsg(orient_pose3);
      cartesian_pose.position.x = current_pose.pose.position.x;
      cartesian_pose.position.y = current_pose.pose.position.y;
      cartesian_pose.position.z = current_pose.pose.position.z;
       
      cartesian_pose.position.x += cartesian_x_delta;
      cartesian_pose.position.y += cartesian_y_delta;
      cartesian_pose.position.z += cartesian_z_delta;


      std::vector<geometry_msgs::Pose> cartesian_waypoints;
      cartesian_waypoints.push_back(cartesian_pose);

      moveit_msgs::RobotTrajectory cartesian_traj;

      const double jump_thresh = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_interface.computeCartesianPath(cartesian_waypoints, eef_step, jump_thresh, cartesian_traj);

       

      if (fraction == 1.0) {
          move_group_interface.execute(cartesian_traj);
      }
      else {
          std::cout << "cartesian plan failed" << std::endl;
          move_group_interface.stop();
      }

     
      

      cartesian_val = false;



    }

    if(attach_obj_val == true){
      
      moveit_msgs::CollisionObject obj;

      obj.id = cylinder_id;

      //define frame pose for the gripper
      obj.header.frame_id = move_group_interface.getEndEffectorLink();
      
     
      std_msgs::Float64 msg;
      msg.data = 0.3;
      std::cout << "gripper closed" << std::endl;
      pub.publish(msg);
    

      std::vector <std::string> touch_links;
      std::string link1 = "pincerfinger_left_link";
      std::string link2 = "pincerfinger_right_link";
      touch_links.push_back(link1);
      touch_links.push_back(link2);
      move_group_interface.attachObject(obj.id, "endpoint_link", touch_links);


      if(obj.id == "4"){
        std::vector<std::string> object_ids;

        object_ids.push_back(collision_cylinder1.id);
        object_ids.push_back(collision_cylinder2.id);
        object_ids.push_back(collision_cylinder3.id);

        planning_scene_interface.removeCollisionObjects(object_ids);




      }


      attach_obj_val = false;


    }


    if(detach_obj_val == true){

      moveit_msgs::CollisionObject detach_obj;

      detach_obj.id = cylinder_id_2;

      std_msgs::Float64 msg;
      msg.data = 0.7;
      std::cout << "gripper open" << std::endl;
      pub.publish(msg);

      move_group_interface.detachObject(detach_obj.id);

      

      detach_obj_val = false;
    }

    
      // cylinder_fn(cylinder1);
      // cylinder_fn(cylinder2);

      set_pose_target_fn(cylinder1, 0, move_group_interface);
      // set_joint_target_fn(cylinder1, 1);
      // set_joint_target_fn(cylinder1, 2);
      // set_attach_fn(cylinder1, 3);
      // set_cartesian_fn(cylinder1, 4);
      // set_cartesian_fn(cylinder1, 5);
      // set_pose_target_fn(cylinder1, 6);
      // set_detach_fn(cylinder1, 7);



    

    ros::spinOnce();

    r.sleep();



  }


  spinner.stop();


  return 0;
}





// void cylinder_fn(XmlRpc::XmlRpcValue &cylinder){

//   //go to waypoint 1
//   set_pose_target_fn(cylinder, 0);
  
//   //go to waypoint2
//   set_joint_target_fn(cylinder, 1);

//   //go to waypoint3
//   set_joint_target_fn(cylinder, 2);

//   //attach
//   set_attach_fn(cylinder, 3);

//   //go to waypoint4
//   set_cartesian_fn(cylinder, 4);

//   //go to waypoint5
//   set_cartesian_fn(cylinder, 5);

//   //go to waypoint6
//   set_pose_target_fn(cylinder, 6);

//   //detach
//   set_detach_fn(cylinder, 7);


// }





void set_pose_target_fn(XmlRpc::XmlRpcValue &cylinder, int index, moveit::planning_interface::MoveGroupInterface& move_group_interface){

  geometry_msgs::Pose target_pose_follow;
  tf2::Quaternion orient_pose_follow;
  orient_pose_follow.setRPY(cylinder[index]["r"], cylinder[index]["p"], cylinder[index]["y"]);
  target_pose_follow.orientation = tf2::toMsg(orient_pose_follow);
  target_pose_follow.position.x = cylinder[index]["x"];
  target_pose_follow.position.y = cylinder[index]["y"];
  target_pose_follow.position.z = cylinder[index]["z"];
  
  
  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setMaxVelocityScalingFactor(0.8);
  move_group_interface.setMaxAccelerationScalingFactor(0.8);


  //check plan for target_pose
  move_group_interface.setPoseTarget(target_pose_follow);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2_follow;
  bool success_follow = (move_group_interface.plan(my_plan2_follow) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "pos goal success: " << success_follow << std::endl;

  

  if(success_follow == true){
    std::cout << "pos success reached" << std::endl;
    move_group_interface.execute(my_plan2_follow);
  


    if(cylinder[index]["gripper_state"] == "1"){
        std_msgs::Float64 msg;
        msg.data = 0.3;
        pub.publish(msg);
      }
      
    else{
        std_msgs::Float64 msg;
        msg.data = 0.8;
        pub.publish(msg);
      
      
    }
  

  }
  else{
    move_group_interface.stop();
  }

}


// void set_joint_target_fn(XmlRpc::XmlRpcValue &cylinder, int index){
  
//   std::vector<double> joint_group_positions_follow;
//   joint_group_positions_follow = move_group_interface.getCurrentJointValues();
//   move_group_interface.setStartStateToCurrentState();
//   move_group_interface.setMaxVelocityScalingFactor(0.8);

//   joint_group_positions_follow[0] = (cylinder[index]["j1"]*M_PI)/180.0;
//   joint_group_positions_follow[1] = (cylinder[index]["j2"]*M_PI)/180.0;
//   joint_group_positions_follow[2] = (cylinder[index]["j3"]*M_PI)/180.0;
//   joint_group_positions_follow[3] = (cylinder[index]["j4"]*M_PI)/180.0;
//   joint_group_positions_follow[4] = (cylinder[index]["j5"]*M_PI)/180.0;
//   joint_group_positions_follow[5] = (cylinder[index]["j6"]*M_PI)/180.0;
  
//   if (cylinder[index]["gripper_state"] == true){
//     joint_group_positions[6] = 1.0;
//   }
//   else{
//     joint_group_positions[6] = 0.0;
//   }
  
  
  
//   move_group_interface.setJointValueTarget(joint_group_positions_follow);
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan_follow;
//   bool success = (move_group_interface.plan(my_plan_follow) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  
//   if(success == true){
//     move_group_interface.execute(my_plan_follow);
    
    
//     if(gripper_req == true){
//       std_msgs::Float64 msg;
//       msg.data = 0.3;
//       pub.publish(msg);
//     }
    
//     else{
//       std_msgs::Float64 msg;
//       msg.data = 0.8;
//       pub.publish(msg);
    
    
//   }
//   joint_group_positions_follow.clear();
  

//   }

//   else{
    
//     move_group_interface.stop();

//   }


// }



// void set_cartesian_fn(XmlRpc::XmlRpcValue &cylinder, int index){

//   geometry_msgs::PoseStamped current_pose_follow = move_group_interface.getCurrentPose();
//   std::vector<double> current_rpy_follow = move_group_interface.getCurrentRPY();


//   geometry_msgs::Pose cartesian_pose_follow;
//   tf2::Quaternion orient_pose3_follow;
//   orient_pose3_follow.setRPY(current_rpy_follow.at(0), current_rpy_follow.at(1), current_rpy_follow.at(2));
//   cartesian_pose_follow.orientation = tf2::toMsg(orient_pose3_follow);
//   cartesian_pose_follow.position.x = current_pose_follow.pose.position.x;
//   cartesian_pose_follow.position.y = current_pose_follow.pose.position.y;
//   cartesian_pose_follow.position.z = current_pose_follow.pose.pose.position.z;
    
//   cartesian_pose_follow.position.x += cylinder[index]["x_cartesian"];
//   cartesian_pose_follow.position.y += cylinder[index]["y_cartesian"];
//   cartesian_pose_follow.position.z += cylinder[index]["z_cartesian"];


//   std::vector<geometry_msgs::Pose> cartesian_waypoints_follow;
//   cartesian_waypoints_follow.push_back(cartesian_pose_follow);

//   moveit_msgs::RobotTrajectory cartesian_traj_follow;

//   const double jump_thresh = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(cartesian_waypoints_follow, eef_step, jump_thresh, cartesian_traj_follow);

      

//   if (fraction == 1.0) {
//       move_group_interface.execute(cartesian_traj_follow);
//   }
//   else {
//       std::cout << "cartesian plan failed" << std::endl;
//       move_group_interface.stop();
//   }


// }





// void set_attach_fn(XmlRpc::XmlRpcValue &cylinder, int index){


//   moveit_msgs::CollisionObject obj_follow;

//   obj_follow.id = cylinder[index]["attach"];

//   //define frame pose for the gripper
//   obj_follow.header.frame_id = move_group_interface.getEndEffectorLink();
  
  
//   std_msgs::Float64 msg;
//   msg.data = 0.3;
//   std::cout << "gripper closed" << std::endl;
//   pub.publish(msg);


//   std::vector <std::string> touch_links;
//   std::string link1 = "pincerfinger_left_link";
//   std::string link2 = "pincerfinger_right_link";
//   touch_links.push_back(link1);
//   touch_links.push_back(link2);
//   move_group_interface.attachObject(obj_follow.id, "endpoint_link", touch_links);


//   if(obj_follow.id == "4"){
//     std::vector<std::string> object_ids;

//     object_ids.push_back(collision_cylinder1.id);
//     object_ids.push_back(collision_cylinder2.id);
//     object_ids.push_back(collision_cylinder3.id);

//     planning_scene_interface.removeCollisionObjects(object_ids);



// }

// }




// void set_detach_fn(XmlRpc::XmlRpcValue &cylinder, int index){

//   moveit_msgs::CollisionObject detach_obj_follow;

//   detach_obj_follow.id = cylinder[index]["detach"];

//   std_msgs::Float64 msg;
//   msg.data = 0.7;
//   std::cout << "gripper open" << std::endl;
//   pub.publish(msg);

//   move_group_interface.detachObject(detach_obj_follow.id);


// }











