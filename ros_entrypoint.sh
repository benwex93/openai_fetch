#!/bin/bash
set -e
export DISPLAY=:0

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros_ws/devel/setup.bash" --extend

roslaunch fetch_simple_description start_pick_and_place_world.launch gui:=$GAZEBO_GUI headless:=$HEADLESS &

sleep 5
roslaunch fetch_simple_description put_fetchsimple_in_world.launch &
rosrun fetch_train execute_trajectories.py &
sleep 2
#roslaunch fetch_gazebo_demo demo.launch &

chmod +x /ros_ws/src/fetch_simple_simulation/fetch_simple_description/scripts/initialization.py
rosrun fetch_simple_description initialization.py &
roslaunch fetch_simple_description spawn_objects.launch &

gz stats

bash
