FROM osrf/ros:melodic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

################################################
#NANO TEXT EDITOR and other utilities
################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    net-tools \
    iputils-ping \
  && rm -rf /var/lib/apt/lists/*


RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt clean && apt update

RUN apt-get install ros-melodic-position-controllers -y
RUN apt-get install ros-melodic-effort-controllers -y
RUN apt-get install ros-melodic-joint-state-controller -y


################################################
#OPENAI ROS BASE
################################################
RUN mkdir /ros_ws \
	&& mkdir /ros_ws/src \
	&& cd /ros_ws/src \
	&& git clone https://bitbucket.org/theconstructcore/openai_ros.git

# #create executables for repository
RUN /bin/bash -c \
	'source /opt/ros/$ROS_DISTRO/setup.bash; \
	cd /ros_ws; \
	catkin_make; \
	source devel/setup.bash; \
	rosdep install openai_ros; \
	echo "source /ros_ws/devel/setup.bash --extend" >> ~/.bashrc;'


RUN cd /ros_ws/src \
	&& git clone https://bitbucket.org/theconstructcore/spawn_robot_tools.git

RUN cd /ros_ws/src \
	&& git clone https://bitbucket.org/theconstructcore/fetch_simple_simulation.git

RUN cd /ros_ws/src \
	&& git clone https://github.com/benwex93/openai_fetch.git


RUN /bin/bash -c 'echo "source /opt/ros/$ROS_DISTRO/setup.bash --extend" >> ~/.bashrc;'

RUN /bin/bash -c \
	'cp -r /ros_ws/src/openai_fetch/robot_envs /ros_ws/src/openai_ros/openai_ros/src/openai_ros/robot_envs/; \
	cp -r /ros_ws/src/openai_fetch/task_envs /ros_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs/;'

RUN /bin/bash -c \
	'source /opt/ros/$ROS_DISTRO/setup.bash; \
	cd /ros_ws; \
	catkin_make; \
	source devel/setup.bash;'

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]