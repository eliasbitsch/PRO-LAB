#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash

# Particle sampler (subscribes /pf/pose, publishes /webxr/particles)
python3 /home/ros/bridge/particle_sampler.py &
SAMPLER_PID=$!

# rosbridge WebSocket on 9090
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:=9090 address:=0.0.0.0 &
ROSBRIDGE_PID=$!

# nginx (HTTPS on 8080)
nginx -g 'daemon off;' &
NGINX_PID=$!

trap "kill $SAMPLER_PID $ROSBRIDGE_PID $NGINX_PID 2>/dev/null" EXIT
wait
