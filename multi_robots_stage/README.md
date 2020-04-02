# multi_robots_stage

multi robot navigation stage simulation using ros stage, and navigation stack

required package:
stage_ros_add_pose_and_crash


## Launch

	roslaunch multi_robots_stage circle.launch 

## How to send goal
	
	python patrol_organizer.py 

or

	rostopic pub /robot_1/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "/map"}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}'

	rostopic pub /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "/map"}, pose: {position: {x: 18.0, y: 19.0, z: 0.0}, orientation: {w: 1.0}}}'


TODO

send target to multiple robots  // done
multiple robot config file // ok need scale up
resize map  // done 
