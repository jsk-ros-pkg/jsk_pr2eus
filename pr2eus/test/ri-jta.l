#!/usr/bin/env

(load "package://pr2eus/pr2-interface.l")
(load "unittest.l")

(init-unit-test)

(defclass dummy-robot
  :super robot-model
  :slots (link-1 link-2 joint-1))
(defmethod dummy-robot
  (:init ()
   (let ()
     (send-super :init)
     (setq link-1 (instance bodyset-link :init (make-cascoords)
                            :bodies (let ((b (make-cube 50 50 100)))
                                      (send b :set-color :red) (list b))))
     (setq link-2 (instance bodyset-link :init (make-cascoords :pos #f(0 0 50))
                            :bodies (let ((b (make-cube 40 40 100 :pos #f(0 0 100))))
                                      (send b :set-color :green) (list b))))
     (setq joint-1 (instance rotational-joint :init :name "joint_1"
                             :parent-link link-1 :child-link link-2
                             :axis :y :min *-inf* :max *inf*))
     (setq links (list link-1 link-2))
     (setq joint-list (list joint-1))
     (send self :assoc link-1)
     (send link-1 :assoc link-2)
     (send self :init-ending)))
  (:joint_1 (&rest args) (forward-message-to joint-1 args))
  )

(defun dummy-robot () (instance dummy-robot :init))

(defclass dummy-robot-interface
  :super robot-interface
  :slots ())
(defmethod dummy-robot-interface
  (:init (&rest args) (send-super* :init :robot (dummy-robot) args))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "dummy_controller/follow_joint_trajectory")
     (cons :controller-state "dummy_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names '("joint_1")))))
  )

(setq *robot* (dummy-robot))
(setq *ri* (instance dummy-robot-interface :init))

;; Tests
;; Use :wait-interpolation after :angle-vector to call :action-result-cb

(deftest test-abort
  (send *ri* :angle-vector #f(-100))
  (send *ri* :wait-interpolation))

(deftest test-preempted
  (send *ri* :angle-vector #f(200))
  (send *ri* :wait-interpolation))

(deftest test-suceeded
  (send *ri* :angle-vector #f(0))
  (send *ri* :wait-interpolation))

(run-all-tests)
(exit)
