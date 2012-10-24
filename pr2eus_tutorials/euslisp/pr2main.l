#!/usr/bin/env roseus
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "pr2_main")

(setup-for-pr2 t)

(warn "
(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
")
