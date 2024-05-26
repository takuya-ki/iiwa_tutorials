FROM nvidia/opengl:1.2-glvnd-devel-ubuntu18.04

RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends tzdata
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-control-toolbox \
    ros-melodic-moveit \
    ros-melodic-effort-controllers \
    ros-melodic-forward-command-controller \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-position-controllers \
    ros-melodic-velocity-controllers \
    ros-melodic-nodelet \
    ros-melodic-tf \
    ros-melodic-tf2-tools \
    ros-melodic-pcl-ros \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-image-geometry \
    libignition-math2-dev \
    ros-melodic-jsk-visualization \
    ros-melodic-jsk-pcl-ros \
    ros-melodic-robot-state-publisher \
    ros-melodic-genpy \
    ros-melodic-realsense2-camera \
    python-catkin-tools \
    python-rosdep \
    build-essential \
    byobu \
    libudev-dev \
    wget \
    python-pip

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" && \
    cd /catkin_ws && rosdep init && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build" && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
    echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc

# Generates bash commands
WORKDIR /catkin_ws/src/
RUN echo "roslaunch iiwa_tool_moveit moveit_planning_execution.launch sim:=false" >> ~/.bash_history
RUN echo "roslaunch iiwa_controller iiwa_commander.launch sim:=false" >> ~/.bash_history
RUN echo "roslaunch iiwa_controller iiwa_commander.launch sim:=true" >> ~/.bash_history
RUN echo "rpython3 dummy/dummy_socket_client.py" >> ~/.bash_history
