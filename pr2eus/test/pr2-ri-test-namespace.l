(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(setq *ri* (instance pr2-interface :init :namespace "namespace"))

(defmacro assert (pred &optional (message "") &rest args)
  `(let (failure ret)
     (warning-message 4 ";; Check ~A ~A~%" ',pred ,message)
     (setq ret ,pred)
     ;; lisp::step could not work with macros..
     ;;     (if (and (listp ',pred) (functionp (car ',pred)))
     ;;	 (setq ret (lisp::step ,pred))
     ;;       (setq ret ,pred))
     (if (not ret)
         ;; escape <> for xml
         (send *unit-test* :increment-failure ',pred (format nil ,message ,@args)
               (escape-xml-string (subseq (send *error-output* :buffer) 0 (or (position 0 (send *error-output* :buffer)) (length (send *error-output* :buffer)))))))
     ))

(deftest test-subscriber
  (let ()
    (send *ri* :spin-once)

    (send *ri* :send-cmd-vel-raw 0 0 0) ;; publish /base_controller/command

    (let ((topic (if (*ri* . namespace) (format nil "/~A/base_controller/command" (*ri* . namespace))
                   "/base_controller/command")))
      (assert (and (ros::get-num-subscribers topic)
                   (> (ros::get-num-subscribers topic) 0)) "/base_controller/command [geometry_msgs/Twist]"))

    ;; * /j_robotsound [sound_play/SoundRequest]
    ;; * /robot_interface_marker_array [visualization_msgs/MarkerArray]

    (let ((topic (if (*ri* . namespace) (format nil "/~A/base_controller/follow_joint_trajectory/status" (*ri* . namespace))
                   "/base_controller/follow_joint_trajectory/status")))
      (ros::advertise topic actionlib_msgs::GoalStatusArray)
      (assert (and (ros::get-num-subscribers topic)
                   (> (ros::get-num-subscribers topic) 0)) "/base_controller/follow_joint_trajectory/status [unknown type]"))

    (assert (send *ri* :state :odom) "/base_odometry/odom [nav_msgs/Odometry]")
    (assert (send *ri* :state :head_traj_controller/state) "/head_traj_controller/state [pr2_controllers_msgs/JointTrajectoryControllerState]")
    (assert (send *ri* :state :potentio-vector) "/joint_states [sensor_msgs/JointState]")
    (assert (send *ri* :state :l_arm_controller/state) "/l_arm_controller/state [pr2_controllers_msgs/JointTrajectoryControllerState]")
    (assert (send (*ri* . l-gripper-action) :wait-for-server 5) "/l_gripper_controller/gripper_action/status [actionlib_msgs/GoalStatusArray]")
    (assert (send *ri* :state :larm-position) "/l_gripper_controller/state [pr2_controllers_msgs/JointControllerState]")
    (assert (send (*ri* . move-base-action) :wait-for-server 5) "/move_base/status [actionlib_msgs/GoalStatusArray]")

    ;; * /pressure/l_gripper_motor [unknown type]
    (let ((topic (if (*ri* . namespace) (format nil "/~A/pressure/l_gripper_motor" (*ri* . namespace))
                   "/pressure/l_gripper_motor")))
      (ros::advertise topic pr2_msgs::PressureState)
      (assert (and (ros::get-num-subscribers topic)
                   (> (ros::get-num-subscribers topic) 0)) "/pressure/l_gripper_motor [unknown type]"))
    ;; * /pressure/r_gripper_motor [unknown type]
    (let ((topic (if (*ri* . namespace) (format nil "/~A/pressure/r_gripper_motor" (*ri* . namespace))
                   "/pressure/r_gripper_motor")))
      (ros::advertise topic pr2_msgs::PressureState)
      (assert (and (ros::get-num-subscribers topic)
                   (> (ros::get-num-subscribers topic) 0)) "/pressure/r_gripper_motor [unknown type]"))

    (assert (send *ri* :state :r_arm_controller/state) "/r_arm_controller/state [pr2_controllers_msgs/JointTrajectoryControllerState]")
    (assert (send (*ri* . r-gripper-action) :wait-for-server 5) "/r_gripper_controller/gripper_action/status [actionlib_msgs/GoalStatusArray]")
    (assert (send *ri* :state :rarm-position) "/r_gripper_controller/state [pr2_controllers_msgs/JointControllerState]")
    ;; * /tf [tf/tfMessage]
    ;; * /tf_static [tf2_msgs/TFMessage]
    (assert (send *ri* :state :torso_controller/state) "/torso_controller/state [pr2_controllers_msgs/JointTrajectoryControllerState]")
    ))

(run-all-tests)
(exit)


