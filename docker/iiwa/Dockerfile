FROM ros:noetic

# Remove any existing references to snapshots.ros.org
RUN rm -f /etc/apt/sources.list.d/*ros* && \
    sed -i '/snapshots.ros.org/d' /etc/apt/sources.list

# Add packages.ros.org for Noetic
RUN apt update && apt install -y --no-install-recommends \
    curl lsb-release wget gnupg \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list \
 && apt update

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y && apt install -y --no-install-recommends tzdata
RUN apt update -y && apt install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-image-proc \
    ros-noetic-image-view \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-control-toolbox \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools \
    ros-noetic-effort-controllers \
    ros-noetic-forward-command-controller \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-position-controllers \
    ros-noetic-velocity-controllers \
    ros-noetic-tf \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-genpy \
    ros-noetic-industrial-core \
    ros-noetic-roslint \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-urdf-tutorial \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-soem \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    python3-tk \
    software-properties-common \
    build-essential \
    libxkbcommon-x11-0 \
    libcanberra-gtk* \
    libgl1-mesa-dev \
    libudev-dev \
    libxcb* \
    xorg-dev \
    net-tools \
    psmisc \
    byobu \
    dbus \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
	/bin/bash -c "cd /catkin_ws/src && source /opt/ros/noetic/setup.bash && catkin_init_workspace" && \
	/bin/bash -c "cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin build" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc

WORKDIR /
RUN wget -P /opt https://repo.anaconda.com/archive/Anaconda3-2020.02-Linux-x86_64.sh && \
    bash /opt/Anaconda3-2020.02-Linux-x86_64.sh -b -p /opt/anaconda3 && \
    rm /opt/Anaconda3-2020.02-Linux-x86_64.sh && \
    export PATH=/opt/anaconda3/bin:$PATH && \
    . ~/.bashrc && conda init && conda update -n base -c defaults conda

RUN pip3 install --target=/opt/ros/noetic/lib/python3/dist-packages rospkg
RUN rm -f /etc/apt/sources.list.d/*ros*
RUN . ~/.bashrc && conda deactivate && pip3 uninstall netifaces -y && pip3 install netifaces==0.11.0 serial

# gazebo on ros1
WORKDIR /catkin_ws/src/
RUN git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git && \
    cd roboticsgroup_gazebo_plugins && \
    git reset --hard 509a32e

RUN pip3 install --force-reinstall "setuptools==65.5.1" "importlib_metadata==6.0.1"

# Generates bash commands
WORKDIR /catkin_ws/src/
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch iiwa_tool_moveit moveit_planning_execution.launch sim:=false" >> ~/.bash_history
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch iiwa_tool_moveit moveit_planning_execution.launch sim:=true" >> ~/.bash_history
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun iiwa_tutorials wiggle" >> ~/.bash_history
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun iiwa_tutorials sendmotion_loop" >> ~/.bash_history
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun iiwa_tutorials sendmotion_loop_lin" >> ~/.bash_history
RUN echo "conda deactivate && source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun iiwa_tutorials sendmotion_loop_action" >> ~/.bash_history