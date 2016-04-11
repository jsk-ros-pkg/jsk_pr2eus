#!/usr/bin/env bash

export ROBOT=sim
export KINECT1=true
export KINECT2=false

roslaunch pr2eus_tutorials pr2_tabletop_grasp_sim.launch $@
