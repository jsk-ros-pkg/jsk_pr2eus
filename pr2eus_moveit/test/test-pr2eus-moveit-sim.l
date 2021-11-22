#!/usr/bin/env roseus

;; test code to check pr2eus_moveit without MoveIt instance

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

;; check https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/446 to avoid jsk-ros-pkg/jsk_robot#1264 problem
(deftest test-init-collision-object-publisher
  (setq ret (instance collision-object-publisher :init))
  (assert ret "(instance collision-object-publisher :init)"))


(run-all-tests)
(exit)
