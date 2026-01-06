FROM ros:jazzy AS base
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /home/arcus_ws

# Base essentials
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      wget nano \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
      build-essential \
      libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# rosdep setup
RUN rosdep init || true && rosdep update

# For GUI apps runtime
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Bashrc setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /home/arcus_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=10" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

# -------------------------
# Desktop (PC) target: simulation deps
# -------------------------
FROM base AS desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
  && rm -rf /var/lib/apt/lists/*

# -------------------------
# Jetson target: NO simulation deps
# -------------------------
FROM base AS jetson

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
  && rm -rf /var/lib/apt/lists/*
