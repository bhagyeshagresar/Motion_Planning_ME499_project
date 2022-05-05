#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2*M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

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


  //add collision objects
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "brick1";

  //define the brick shape
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.2;
  primitive.dimensions[primitive.BOX_Y] = 0.06;
  primitive.dimensions[primitive.BOX_Z] = 0.04;

  //define pose for the brick
  geometry_msgs::Pose brick_pose;
  brick_pose.orientation.w = 1.0;
  brick_pose.position.x = 0.5;
  brick_pose.position.y = 0.0;
  brick_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(brick_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // ROS_INFO_NAMED("add object into world");
  planning_scene_interface.addCollisionObjects(collision_objects);




}