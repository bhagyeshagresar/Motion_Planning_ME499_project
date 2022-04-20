#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <string.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv,"path_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();               
    static const std::string PLANNING_GROUP = "arm";
    
    
    //control the planning_group 'arm'    
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    
    //set the object of planning interface for collision detection
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setPlannerId("RRT");

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
    //visuals    
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("arm");
    visual_tools.deleteAllMarkers();

    
    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // We can print the name of the reference frame for this robot.
    // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // // We can also print the name of the end-effector link for this group.
    // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // ROS_INFO_NAMED("planner", "default planner: %s", move_group_interface.getDefaultPlannerId().c_str());


    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group_interface.getJointModelGroupNames().begin(),
    //             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    visual_tools.prompt("initialisation done step 0");

    //Set a goal, plan and execute - direct pose goal

    ROS_INFO_NAMED("planner", "default planner: %s", move_group_interface.getDefaultPlannerId().c_str());

    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    // move_group_interface.setPlannerId("RRT");
    ROS_INFO_NAMED("planner", "default planner: %s", move_group_interface.getDefaultPlannerId().c_str());
    geometry_msgs::Pose start_pose;
    start_pose.orientation.x = 1.0;
    start_pose.position.x = 0.432281;
    start_pose.position.y = 0.00348;
    start_pose.position.z = 0.391869;
    move_group_interface.setPoseTarget(start_pose);


    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(start_pose);

    // geometry_msgs::Pose target_pose = start_pose;

    
    // target_pose.position.x = 0.53;
    // waypoints.push_back(target_pose);  // down

    // target_pose.position.y += 0.01;
    // waypoints.push_back(target_pose); 
    
    // target_pose.position.x += 0.01;
    // waypoints.push_back(target_pose); 
    
    // target_pose.position.x += 0.01;
    // waypoints.push_back(target_pose); 

    // target_pose.position.y += 0.1;
    // waypoints.push_back(target_pose);  // right

    // // target_pose3.position.z = 0.3;
    // target_pose3.position.y += 0.2;
    // target_pose3.position.x -= 0.2;
    // waypoints.push_back(target_pose3); 

    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // visual_tools.deleteAllMarkers();
    // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // move_group_interface.execute(trajectory);


    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "success" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(start_pose, "pose1");
    visual_tools.publishText(start_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group_interface.execute(my_plan);
    // ROS_INFO_NAMED("planner", "default planner: %s", move_group_interface.getDefaultPlannerId());
    // std::cout << "default planner is :" << move_group_interface.getDefaultPlannerId() << std::endl;


    //define a collision object
    // moveit_msgs::CollisionObject collision_object;
    // collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // //object id
    // collision_object.id = "box1";

    // //define shape of collision object
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.BOX_X] = 0.01;
    // primitive.dimensions[primitive.BOX_Y] = 0.5;
    // primitive.dimensions[primitive.BOX_Z] = 0.24;

    // //define pose relative to frame_id
    // geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 0.0;
    // box_pose.position.y = 0.4;
    // box_pose.position.z = 0.12;

    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_object);
    
    // //add collision object in the world
    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);

    // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    // visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    


}