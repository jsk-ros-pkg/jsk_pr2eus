#!/usr/bin/env bash

######################################################
# install moveit! from source
######################################################
sudo -H pip install -q rosinstall_generator

rosinstall_generator --tar --rosdistro $ROS_DISTRO moveit >> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool update
