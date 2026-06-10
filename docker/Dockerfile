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

# Disable RGBD camera sensor in TurtleBot4 URDF — heavy Ogre2 sensor we don't
# need for IMU-based filter development, and it has historically caused
# crashes on some GPU stacks.
RUN python3 -c "\
import re;\
p='/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/sensors/oakd.urdf.xacro';\
s=open(p).read();\
s=re.sub(r'<sensor name=\"rgbd_camera\"[^<]*(?:<[^/][^>]*>.*?</[^>]+>\s*)+</sensor>', '<!-- rgbd_camera disabled -->', s, flags=re.DOTALL);\
open(p,'w').write(s)"

# Match the container's ros user UID/GID to the host (set in docker/.env).
# Without this, files written by the container into bind-mounted host
# directories (./results, ws/src) are owned by a stranger UID and the host
# user can't manage them.
#
# The ros:jazzy-ros-base image already ships with `ubuntu:ubuntu` at
# UID/GID 1000, so we rename that user/group instead of creating a fresh
# one — `useradd` would collide with the existing IDs.
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -eux; \
    if getent group ${USER_GID} >/dev/null; then \
        existing_group=$(getent group ${USER_GID} | cut -d: -f1); \
        [ "$existing_group" != "ros" ] && groupmod -n ros "$existing_group" || true; \
    else \
        groupadd -g ${USER_GID} ros; \
    fi; \
    if getent passwd ${USER_UID} >/dev/null; then \
        existing_user=$(getent passwd ${USER_UID} | cut -d: -f1); \
        if [ "$existing_user" != "ros" ]; then \
            usermod -l ros -d /home/ros -m -s /bin/bash "$existing_user"; \
        fi; \
    else \
        useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ros; \
    fi
USER ros
WORKDIR /home/ros
# Pre-create the ws subdirs that named volumes will be mounted on. Docker
# creates volume mountpoints as root when the path doesn't already exist;
# pre-creating them as the ros user lets named volumes inherit ros ownership
# on first mount, so colcon can write build/install/log without sudo.
RUN mkdir -p /home/ros/ws/src /home/ros/ws/build /home/ros/ws/install \
             /home/ros/ws/log /home/ros/.gz /home/ros/results

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ros/.bashrc && \
    echo "[ -f /home/ros/ws/install/setup.bash ] && source /home/ros/ws/install/setup.bash" >> /home/ros/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/ros/.bashrc && \
    echo "export TURTLEBOT4_MODEL=standard" >> /home/ros/.bashrc

COPY --chown=ros:ros entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
