#!/bin/bash

set -xe

source /opt/ros/${ROS_DISTRO}/setup.bash

rosdep update --rosdistro ${ROS_DISTRO}
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --continue-on-error || true
