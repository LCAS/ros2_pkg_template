#!/bin/bash

set -xe


function add_config_if_not_exist {
    if ! grep -F -q "$1" $HOME/.bashrc; then
        echo "$1" >> $HOME/.bashrc
    fi
}

add_config_if_not_exist "source /opt/ros/humble/setup.bash"
add_config_if_not_exist "source /opt/ros/lcas/install/setup.bash"
add_config_if_not_exist "alias rviz_sensors='rviz2 -d /opt/ros/lcas/install/limo_description/share/limo_description/rviz/model_sensors_real.rviz'"
add_config_if_not_exist "alias tidybot_sim='ros2 launch uol_tidybot tidybot.launch.py'"


source /opt/ros/humble/setup.bash
source /opt/ros/lcas/install/setup.bash

colcon build --symlink-install --continue-on-error || true

LOCAL_SETUP_FILE=`pwd`/install/setup.bash
add_config_if_not_exist "if [ -r $LOCAL_SETUP_FILE ]; then source $LOCAL_SETUP_FILE; fi"

sleep 10
DISPLAY=:1 xfconf-query -c xfce4-desktop -p $(xfconf-query -c xfce4-desktop -l | grep "workspace0/last-image") -s /usr/share/backgrounds/xfce/lcas.jpg  || true
