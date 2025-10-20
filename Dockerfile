FROM osrf/ros:noetic-desktop-full

# Arguments to receive the user and group ID from docker-compose
ARG UID=1000
ARG GID=1000

############################################################################## 
# Essential packages
##############################################################################
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    lsb-release \
    build-essential \
    software-properties-common \
    gnupg \
    curl \
    wget \
    apt-transport-https \
    git \
    tmux \
    terminator \
    nano \
    htop \
    net-tools \
    iputils-ping \
    sshpass \
    iproute2 \
    xserver-xorg-video-dummy \
    x11-* \
    xinit \
    x11vnc \
    xterm \
    xvfb \
    xauth \
    xorg \
    qtbase5-dev \
    python3-catkin-tools \
    python3-venv \
    python3-pip \
    python3-rosdep \
    ipython3 \
    python3-rosinstall \
    python-is-python3 \
    ros-noetic-move-base-msgs \
    ros-noetic-base-local-planner \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-control-msgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-jsk-rviz-plugins \
    psmisc \
    flex \
    libfl-dev \
    libfl2 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# Create a new group and user with the provided IDs
RUN groupadd -g $GID hrisim && \
    useradd -m -s /bin/bash -u $UID -g $GID hrisim && \
    echo "hrisim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER hrisim
WORKDIR /home/hrisim

# Install user-specific python packages
RUN pip install pyAgrum --user
RUN pip install pandas --user
RUN pip install shapely --user
RUN pip install networkx==3.1 --user
RUN pip install tmule --user

############################################################################## 
# TIAGo
##############################################################################
# Create a new workspace
RUN mkdir -p /home/hrisim/tiago_ws/src
WORKDIR /home/hrisim/tiago_ws

# Download tiago_public-noetic.rosinstall and clone required repositories
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall && \
rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

# Run user-level rosdep commands and install dependencies
RUN rosdep update --include-eol-distros && \
sudo apt-get update && \
rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys \
"urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

# Build the workspace
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash; cd /home/hrisim/tiago_ws; catkin build'

# ############################################################################## 
# PetriNetPlans
# ##############################################################################
# Create a new workspace
RUN mkdir -p /home/hrisim/ros_ws/src
WORKDIR /home/hrisim/ros_ws

RUN cd /home/hrisim/ros_ws
RUN git clone -b noetic_devel https://github.com/francescodelduchetto/PetriNetPlans.git
RUN mkdir -p /home/hrisim/ros_ws/PetriNetPlans/PNP/build && cd /home/hrisim/ros_ws/PetriNetPlans/PNP/build && cmake .. && sudo make install

RUN cd ~/ros_ws/src && \
ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs .
RUN echo "export PNP_HOME=/home/hrisim/ros_ws/src/pnp_ros/" >> ~/.bashrc
COPY HRISim_docker/src/HRISim/pnp_ros/main.cpp /home/hrisim/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_ros/src/main.cpp

# ############################################################################## 
# Shared Folders
# ##############################################################################
# Create a shared folder with the host machine
RUN mkdir -p /home/hrisim/shared
RUN mkdir -p /home/hrisim/.pal/tiago_maps/configurations/
RUN mkdir -p /home/hrisim/.gazebo/models

# Copy the entrypoint.sh script into the image and give execute permissions to the script
USER root
COPY entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint to run the script
USER hrisim
ENV DISABLE_ROS1_EOL_WARNINGS=1
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]