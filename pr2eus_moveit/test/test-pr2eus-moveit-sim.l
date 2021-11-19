#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::load-ros-package "pr2eus_moveit")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

(pr2-init)
(deftest test-angle-vector-motion-plan
  (let (ret)
    (setq ret (send *ri* :angle-vector-motion-plan (send *pr2* :reset-manip-pose)))
    (assert ret "angle-vector-motion-plan returns something")
    ))

(run-all-tests)
(exit)
