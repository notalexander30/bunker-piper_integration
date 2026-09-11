FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=173
ENV ROS_LOCALHOST_ONLY=1
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    can-utils \
    curl \
    git \
    iproute2 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    tmux \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-moveit \
    ros-humble-aruco-ros \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true
RUN rosdep update

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

COPY dependencies.repos /tmp/nav-man-dependencies.repos
RUN vcs import /ros2_ws/src < /tmp/nav-man-dependencies.repos

COPY . /ros2_ws/src/bunker-piper_integration

RUN apt-get update && bash -lc 'source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "ament_python catkin warehouse_ros_mongo bunker_object_follower semantic_memory"' \
  && rm -rf /var/lib/apt/lists/*

RUN bash -lc 'source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --packages-up-to bunker_slam_bringup bunker_dual_piper_nav2 \
        bunker_autonomy piper_x_aruco_wall_approach'

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' \
    'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-173}' \
    'export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}' \
    'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}' \
    >> /root/.bashrc

CMD ["bash"]
