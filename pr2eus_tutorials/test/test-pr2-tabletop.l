;; test-pr2-tabletop.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "gazebo_msgs")
(ros::roseus "pr2_tabletop_test_node")
(ros::rate 1)

(require :unittest "lib/llib/unittest.l")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun wait-for-gazebo (&key (timeout 30))
  (let ((start (car (unix:gettimeofday)))
        msg)
    (while (ros::ok)
      (setq msg (one-shot-subscribe "/gazebo/model_states" gazebo_msgs::ModelStates :timeout 10000))
      (when (and msg
                 (find "pr2" (send msg :name) :test #'string=)
                 (find "room73b2-bottle-0" (send msg :name) :test #'string=))
        (warning-message 2 "/gazebo/model_states/name = ~A~%" (send msg :name))
        (return-from wait-for-gazebo t))
      (when (> (- (car (unix:gettimeofday)) start) timeout)
        (if msg
            (warning-message 1 "/gazebo/model_states/name is invalid: ~A~%" (send msg :name))
            (warning-message 1 "/gazebo/model_states is not published~%"))
        (sys::exit 1))
      (warning-message 2 "Waiting for gazebo~%")
      (unix:sleep 1)
      (when (ros::wait-for-service "/gazebo/unpause_physics" 1)
        (warning-message 2 "Sending /gazebo/unpause_physics~%")
        (call-empty-service "/gazebo/unpause_physics"))
      )))


(init-unit-test)

(deftest test-pr2-tabletop ()
  (setq *tfl* (instance ros::transform-listener :init))
  (pr2)

  (dotimes (i 30)
    (setq *ri* (instance pr2-interface :init))
    (when (not (send *ri* :simulation-modep)) (return))
    (warning-message 2 "Waiting for spawning robot~%")
    (unix:sleep 1)
    (ros::spin-once))
  (assert (not (send *ri* :simulation-modep)) "robot interface is in simulation mode")

  (warning-message 2 "Looking at target~%")
  
  ;; (send *pr2* :reset-manip-pose)
  (send *pr2* :head :look-at #f(700 0 700))
  (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation :head-controller 10)

  (warning-message 2 "Waiting for tabletop objects~%")

  (setq start-time (ros::time-now))
  (setq foundp nil)
  (ros::rate 1)
  (while (and (not foundp)
              (< (send (ros::time- (ros::time-now) start-time) :to-sec) 180))
    (when (find-if #'(lambda (f)
                       (substringp "segmentation_decomposeroutput" f))
                   (send *tfl* :get-frame-strings))
      (setq foundp t)
      (return))
    (ros::spin-once)
    (send *ri* :spin-once)
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation :head-controller 10)
    (warning-message 2 "Current angle vector ~A~%" (send *ri* :state :potentio-vector))
    (ros::sleep))
  (assert foundp "tabletop object not found"))

(wait-for-gazebo)
(run-all-tests)
(exit)
