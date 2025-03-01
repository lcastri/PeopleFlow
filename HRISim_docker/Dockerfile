FROM osrf/ros:noetic-desktop-full

# Install essential packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    locales lsb-release build-essential software-properties-common gnupg curl wget \
    apt-transport-https git tmux terminator nano htop net-tools iputils-ping sshpass iproute2 \
    xserver-xorg-video-dummy x11-* xinit x11vnc xterm xvfb xauth xorg qtbase5-dev \
    python3-catkin-tools python3-venv python3-pip python3-rosdep ipython3 python3-rosinstall python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-move-base-msgs \
    ros-noetic-base-local-planner \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-control-msgs

# Source ROS environment in .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

############################################################################## TIAGo
# Create a new workspace
RUN mkdir -p ~/tiago_ws/src
WORKDIR /root/tiago_ws

# Download tiago_public-noetic.rosinstall and clone required repositories
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall && \
    rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

# Initialize rosdep and update
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

# Install dependencies
RUN DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys \
    "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

# Fix tiago_controller_configuration package.xml
COPY HRISim_docker/HRISim/robot/tiago_controller_conf_package.xml /root/tiago_ws/src/tiago_robot/tiago_controller_configuration/package.xml

# Copy files into the image
COPY HRISim_docker/HRISim/robot/robot_twist_mux/modified_scripts/twist_mux.launch /opt/ros/noetic/share/twist_mux/launch
COPY HRISim_docker/HRISim/hrisim_gazebo/modified_scripts/local_planner.yaml /root/tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/teb

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash; cd ~/tiago_ws; catkin build'
RUN echo "source ~/tiago_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc

# Install gazebo dependencies for animated actors
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs

############################################################################## CausalFlow
# Clone and install causalflow
RUN mkdir -p ~/repos
WORKDIR /root/repos
RUN DEBIAN_FRONTEND=noninteractive pip install --upgrade pip setuptools wheel importlib_metadata numpy
RUN DEBIAN_FRONTEND=noninteractive pip install --upgrade --ignore-installed cython

# Clone the causalflow repository
ARG CACHE=3
RUN git clone -b dev-reasoning-gmm-robot https://github.com/lcastri/causalflow.git causalflow

# Install Python dependencies
WORKDIR /root/repos/causalflow
RUN pip install --no-cache-dir -r requirements.txt

# Install the causalflow package
RUN /usr/bin/env python setup.py install
RUN echo 'export PYTHONPATH=/root/repos/causalflow:$PYTHONPATH' >> ~/.bashrc
RUN echo "source /root/.bashrc" >> ~/.bash_profile

############################################################################## HRISim
# Create a new workspace
RUN mkdir -p ~/ros_ws/src
WORKDIR /root/ros_ws

# Install pkgs
RUN DEBIAN_FRONTEND=noninteractive pip install tmule 
RUN DEBIAN_FRONTEND=noninteractive pip install shapely
RUN DEBIAN_FRONTEND=noninteractive pip install networkx
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-jsk-rviz-plugins \
    psmisc
# Clone and install PetriNetPlans
RUN cd ~/ros_ws
RUN git clone -b noetic_devel https://github.com/francescodelduchetto/PetriNetPlans.git
RUN DEBIAN_FRONTEND=noninteractive apt install -y flex libfl-dev libfl2
RUN mkdir -p ~/ros_ws/PetriNetPlans/PNP/build && cd ~/ros_ws/PetriNetPlans/PNP/build && cmake .. && \
    make install

RUN cd ~/ros_ws/src && \
    ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs .
RUN echo "export PNP_HOME=~/ros_ws/src/pnp_ros/" >> ~/.bashrc
COPY HRISim_docker/HRISim/pnp_ros/main.cpp /root/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_ros/src/main.cpp

# Copy the entrypoint.sh script into the image and give execute permissions to the script
COPY HRISim_docker/entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Create the runtime directory with appropriate permissions
RUN mkdir -p /run/user/1000 && chmod 0700 /run/user/1000

# Set the environment variable
ENV XDG_RUNTIME_DIR=/run/user/1000

# Copy files into the image
RUN mkdir -p ~/ros_ws/src/HRISim/
COPY HRISim_docker/HRISim /root/ros_ws/src/HRISim
RUN mkdir -p ~/ros_ws/src/pedsim_ros/
COPY HRISim_docker/pedsim_ros /root/ros_ws/src/pedsim_ros

############################################################################## Final
# Create a shared folder with the host machine
RUN mkdir -p ~/shared

# Transfer files to the image
# Create tiago_maps folder
RUN mkdir -p ~/.pal/tiago_maps/configurations/
COPY HRISim_docker/HRISim/hrisim_gazebo/tiago_maps/ /root/.pal/tiago_maps/configurations/

# Create .gazebo folder
RUN mkdir -p ~/.gazebo/models
COPY HRISim_docker/HRISim/hrisim_gazebo/models /root/.gazebo/models

# Set the entrypoint to run the script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]