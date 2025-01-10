#!/bin/bash
set -e

# Source ROS 2
source "/opt/ros/iron/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"