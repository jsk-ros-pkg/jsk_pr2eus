#!/usr/bin/env bash

killall gzserver 2&>1 1>/dev/null || true

set -e

export ROBOT=sim
export KINECT1=true
export KINECT2=false

roslaunch pr2eus_tutorials pr2_tabletop_sim.launch $@
