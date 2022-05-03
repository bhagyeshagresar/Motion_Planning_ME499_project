// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <ros/ros.h>
// #include <string.h>


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv,"test_constraints");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();               
//     static const std::string PLANNING_GROUP = "arm";

//     //control the planning_group 'arm'    
//     moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

//     //set the object of planning interface for collision detection
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//     move_group_interface.setMaxVelocityScalingFactor(1.0);
//     move_group_interface.setMaxAccelerationScalingFactor(1.0);

//     const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
//     // move_group_interface.setStartState(*move_group_interface.getCurrentState());
//     // // move_group_interface.setPlannerId("RRT");
//     // ROS_INFO_NAMED("planner", "default planner: %s", move_group_interface.getDefaultPlannerId().c_str());
    
//     geometry_msgs::Pose target_pose1;
//     target_pose1.orientation.w = 1.0;
//     target_pose1.position.x = 0.28;
//     target_pose1.position.y = -0.2;
//     target_pose1.position.z = 0.5;
//     // move_group_interface.setPoseTarget(target_pose1);

//     // Now, we call the planner to compute the plan and visualize it.
//     // Note that we are just planning, not asking move_group_interface
//     // to actually move the robot.

//     bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//     ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//     //constraints
//     moveit_msgs::JointConstraint jc;
//     jc.joint_name = "drive1_link";
//     jc.position = 0.7859;
//     jc.tolerance_above = 0.0001;
//     jc.tolerance_below = 0.0001;
//     jc.weight = 1.0;

//     moveit_msgs::Constraints constraint_1;
//     constraint_1.joint_constraints.push_back(jc);
//     move_group_interface.setPathConstraints(constraint_1);

//     move_group_interface.setPoseTarget(target_pose1);

//     move_group_interface.setPlanningTime(10.0);

//     success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     ROS_INFO_NAMED("tutorial", "Visualizing plan  (constraints) %s", success ? "" : "FAILED");

      
//     move_group_interface.clearPathConstraints();

      
      
      
// }