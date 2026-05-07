# Unified container: Gazebo (headless) + bridge + Nav2 + RViz + filters
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    nano \
    python3-colcon-common-extensions \
    libeigen3-dev \
    mesa-utils \
    libgl1-mesa-dri \
    libegl1 \
    libglx-mesa0 \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-tools-vendor \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-turtlebot4-msgs \
    ros-jazzy-turtlebot4-node \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-irobot-create-description \
    ros-jazzy-irobot-create-msgs \
    ros-jazzy-irobot-create-nodes \
    ros-jazzy-irobot-create-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb4-sim \
    ros-jazzy-nav2-minimal-tb4-description \
    ros-jazzy-navigation2 \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Disable RGBD camera sensor in TurtleBot4 URDF — crashes Ogre2 on WSLg D3D12.
# We don't need it for IMU-based filter development.
RUN python3 -c "\
import re;\
p='/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/sensors/oakd.urdf.xacro';\
s=open(p).read();\
s=re.sub(r'<sensor name=\"rgbd_camera\"[^<]*(?:<[^/][^>]*>.*?</[^>]+>\s*)+</sensor>', '<!-- rgbd_camera disabled -->', s, flags=re.DOTALL);\
open(p,'w').write(s)"

RUN useradd -m -s /bin/bash -u 1001 ros
USER ros
WORKDIR /home/ros
RUN mkdir -p /home/ros/ws/src /home/ros/.gz

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc && \
    echo "[ -f /home/ros/ws/install/setup.bash ] && source /home/ros/ws/install/setup.bash" >> /home/ros/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc && \
    echo "export TURTLEBOT4_MODEL=standard" >> /home/ros/.bashrc

COPY --chown=ros:ros entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
