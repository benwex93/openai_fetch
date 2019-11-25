#!/bin/bash
set -e
export DISPLAY=:0

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros_ws/devel/setup.bash"

roslaunch fetch_simple_description start_HER_world_push.launch headless:="true" gui:="false" &
sleep 5
roslaunch fetch_simple_description put_fetchsimple_in_world.launch &
rosrun fetch_train execute_trajectories.py &
sleep 4
gz stats
# rosrun gazebo_ros spawn_model -file /root/ros_ws/src/cart_pole/cartpole_description/urdf/camera.sdf -sdf -model camera &

# roslaunch cartpole_openai_ros_examples start_dper_training.launch &
# sleep 5
# source "/moveit_nav_workspace/projects/pr2_moveit/devel/setup.bash" --extend
# source "/moveit_nav_workspace/projects/pr2_navigation/devel/setup.bash" --extend
# source "/workspace/projects/pr2_learning_from_demonstration/devel/setup.bash" --extend

#set env variables
# export ROBOT=sim
# export GAZEBO_RESOURCE_PATH=/workspace/projects/pr2_learning_from_demonstration/src/pr2_demonstration_teleop/src/gazebo:$GAZEBO_RESOURCE_PATH
# export ip=$1
# export port=$2
# source "/workspace/projects/pr2_learning_from_demonstration/devel/setup.bash" --extend;
# export ROS_MASTER_URI=http://localhost:'$3'/;
# export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-"http://localhost:'$4'"};' >> ~/.bashrc
#RUN PR2 ROBOT SIMULATION
# cd /workspace/projects/pr2_learning_from_demonstration;


# cd /ros_ws/src/openai_ros/openai_ros/src/openai_ros/; python2 gym_env_start.py &
# sleep 3

# if [ $ACTION = "eval" ]; then
# 	cd /home/dsi/benwex93/learning_agent/pr2_agent/scripts/rosfetch/; sh rosfetch_eval
# elif [ $ACTION = "act" ]; then
# 	cd /home/dsi/benwex93/learning_agent/pr2_agent/scripts/rosfetch/; sh rosfetch_act
# else
# 	echo $ACTION "is not a valid action"
# fi
bash
# bash pr2_gazebo_load.sh 1 $3 $4
