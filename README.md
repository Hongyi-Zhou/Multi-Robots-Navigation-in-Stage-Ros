# multi_robots_stage

a demo of multi robot navigation simulation using stage ros, and move_base in navigation stack <br>
<br>
required package: modified stage_ros

## Note
As mentioned in https://github.com/ros-simulation/stage_ros/pull/40.

The current stage_ros is incompatible with the tf2 naming convention since tf2 does not allow a leading slash (/) for the frame_id. <br>
The fix is quite simple. Only require you to remove a single character from the current released stageros.cpp in stage_ros package, namely,
changed line 190 from:

	snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);

to:

	snprintf(buf, sizeof(buf), "robot_%u/%s", (unsigned int)robotID, name);

The stage_ros package in this repo has the modified/ working stageros package. <br>

## Build
	mkdir -p catkin_ws/src
	git clone 
	cd catkin_ws
	catkin_make
	source devel/setup.bash

## Launch

	roslaunch multi_robots_stage circle.launch 
	roslaunch multi_robots_stage move_base_fake_localization_10cm.launch

## How to send goal
	
	python goal_organizer.py 

or
	rostopic pub /robot_1/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "/map"}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}'

	rostopic pub /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "/map"}, pose: {position: {x: 18.0, y: 19.0, z: 0.0}, orientation: {w: 1.0}}}'

You should see 6 robots start moving to the position of the opposite robot.<br>
You can also tune the local planner to avoid collision more effectively.
