FROM ros:jazzy

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
RUN sudo apt install -y wget nano

# Create overlay workspace
WORKDIR /home/arcus_ws

# Install required dependencies
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install missing dependencies
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     colcon build

RUN rosdep init || true && rosdep update

# Create runtime directory for GUI apps
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Add sourcing ROS to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# Add ROS2 Domain ID to .bashrc
RUN echo "ROS_DOMAIN_ID=10" >> ~/.bashrc
RUN echo "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc