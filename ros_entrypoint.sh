#!/bin/bash
set -e

## setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$ROS_WS/install/setup.bash" --

## Verify robot_name environment variable is set; Exit otherwise
if [[ -z $robot_name ]]; then
    echo "[ERROR] robot_name environment variable is not set. Must be set to namespace ros artifacts correctly."
    exit 1
fi

## Verify robot_ip environment variable is set; Exit otherwise
if [[ -z $robot_ip ]]; then
    echo "[ERROR] robot_ip environment variable is not set. Must be set to construct the needed robot config"
    exit 1
fi

exec "$@"