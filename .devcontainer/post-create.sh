#!/bin/bash

set -xe


function add_config_if_not_exist {
    if ! grep -F -q "$1" $HOME/.bashrc; then
        echo "$1" >> $HOME/.bashrc
    fi
}

function add_git_config_if_not_exist {
    if ! git config --global --get "$1" > /dev/null; then
        git config --global "$1" "$2"
    fi
}

add_config_if_not_exist "source /opt/ros/humble/setup.bash"

source /opt/ros/humble/setup.bash

colcon build --symlink-install --continue-on-error || true

LOCAL_SETUP_FILE=`pwd`/install/setup.bash
add_config_if_not_exist "if [ -r $LOCAL_SETUP_FILE ]; then source $LOCAL_SETUP_FILE; fi"


add_git_config_if_not_exist "core.autocrlf" "input"
add_git_config_if_not_exist "core.safecrlf" "warn"
add_git_config_if_not_exist "pull.rebase" "false"
add_git_config_if_not_exist "user.name" "Anonymous L-CAS DevContainer User"
add_git_config_if_not_exist "user.email" "noreply@lcas.lincoln.ac.uk"
add_git_config_if_not_exist "init.defaultBranch" "main"


sleep 10
# hack to set L-CAS background
DISPLAY=:1 xfconf-query -c xfce4-desktop -p $(xfconf-query -c xfce4-desktop -l | grep "workspace0/last-image") -s /usr/share/backgrounds/xfce/lcas.jpg  || true
