#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::load-ros-manifest "pr2eus_moveit")
(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")


;; wait for gazebo
(ros::roseus "test_pr2eus_mvoeit_client")
(ros::advertise "head_traj_controller/joint_trajectory_action/goal" pr2_controllers_msgs::JointTrajectoryActionGoal)
(while (= (ros::get-num-subscribers "head_traj_controller/joint_trajectory_action/goal") 0)
  (unix::sleep 1)
  (format *error-output* "waiting for controller ... head_traj_controller (~A)~%" (ros::time-now)))

;; setup pr2 and ri
(pr2-init)

;; setup moveit interface
(send *ri* :set-moveit-environment (instance pr2-moveit-environment :init))

;; send angle-vector with collision avoidance
(deftest test-angle-vector-motion-plan ()
  (let (av-diff)
    (send *ri* :angle-vector-motion-plan (send *pr2* :reset-pose) :move-arm :rarm :use-torso t)
    (send *ri* :angle-vector-motion-plan (send *pr2* :reset-pose) :move-arm :larm :use-torso t)
    (send *ri* :wait-interpolation)
    (setq av-diff (norm (apply #'float-vector (mapcar #'(lambda (x y) (let ((d (- x y))) (- d (* 360 (round (/ d 360)))))) (coerce (send *ri* :state :potentio-vector) cons) (coerce (send *pr2* :reset-pose) cons)))))
    (ros::ros-info "av diff ~A" av-diff)
    (assert (eps= av-diff 0 5) (format nil "send reset-pose ~A" av-diff))
    ))

;; send target coords
(deftest test-move-end-coords-plan ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 750)) :move-arm :rarm)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 500)) :move-arm :larm)
    (send *ri* :wait-interpolation)
    (send *ri* :move-end-coords-plan (make-coords :pos #f(700 0 1000)) :move-arm :larm-torso)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "collsion avoidance motion is too fast ~A" tm-diff))
    ))

(run-all-tests)
(exit)

