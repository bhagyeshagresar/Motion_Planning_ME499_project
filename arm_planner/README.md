# arm_planner package for motion planning

The arm_planner is the main package used for motion planning of the HDT Adroit arm for picking and placing cylinders to build a pyramid. 

The arm_planner package contains path_planner node which uses the moveit move_group cpp interface for pose_goal, cartesian_planning and joint_space planning. Planning_scene interface is used for including collision objects in the planning scene.

The test_planner node was used to test pose_goal, cartesian planning and joint_space planning.