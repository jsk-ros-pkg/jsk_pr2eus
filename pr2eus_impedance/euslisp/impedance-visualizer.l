#!/usr/bin/env roseus
(ros::roseus "impedance-visualizer")
(ros::load-ros-manifest "ee_cart_imped_msgs")

(setq *node-name* "impedance_visualizer")
(setq *r_arm_actual_force* "r_arm_actual_force")
(setq *l_arm_actual_force* "l_arm_actual_force")

(setq *r_arm_actual_pose_force* "r_arm_actual_pose_force")
(setq *l_arm_actual_pose_force* "l_arm_actual_pose_force")


(defun vector3-msg->vector (msg)
  (float-vector (send msg :x) (send msg :y) (send msg :z))
  )

(defun vector->vector3-msg (vec)
  (let ((msg (instance geometry_msgs::Vector3 :init)))
    (send msg :x (elt vec 0))
    (send msg :y (elt vec 1))
    (send msg :z (elt vec 2))
    msg
    )
  )

(defun rotate-wrench (wrench mat)
  (let ((ret-wrench (instance geometry_msgs::Wrench :init)))
    (send ret-wrench :force (vector->vector3-msg (transform (vector3-msg->vector (send wrench :force)) mat)))
    (send ret-wrench :torque (vector->vector3-msg (transform (vector3-msg->vector (send wrench :torque)) mat)))
    (print ret-wrench)
    ret-wrench
    ))

(defun visualize-rarm-cb (msg)
  ;;  (visualize-actual-force msg :rarm)
  (visualize-actual-pose-force msg :rarm)
  )

(defun visualize-larm-cb (msg)
  ;;  (visualize-actual-force msg :larm)
  (visualize-actual-pose-force msg :larm)
  )

#|
(defun visualize-actual-force (msg arm)
  (let (pub-topic)
    (setq pub-topic (instance geometry_msgs::WrenchStamped :init :header (send msg :header) :wrench (send msg :actual_force)))
    (case arm
      (:rarm
       (send pub-topic :header :frame_id "/r_gripper_tool_frame")
       (ros::publish *r_arm_actual_force* pub-topic)
       )
      (:larm
       (send pub-topic :header :frame_id "/l_gripper_tool_frame")
       (ros::publish *l_arm_actual_force* pub-topic)
       )
      )
    )
  )
|#

(defun visualize-actual-pose-force (msg arm)
  (let (pub-topic)
    (setq pub-topic (instance geometry_msgs::WrenchStamped :init :header (send msg :header)))
    (case arm
      (:rarm
       (let ((rotate-coords (send *tfl* :lookup-transform "/base_link" "/r_gripper_tool_frame" (ros::time 0))))
	 (unless (null rotate-coords)
	   (send pub-topic :wrench (rotate-wrench (send msg :actual_pose :wrench_or_stiffness) (send rotate-coords :rot)))
	   (send pub-topic :header :frame_id "/r_gripper_tool_frame")
	   (ros::publish (format nil "~A/~A" *node-name* *r_arm_actual_pose_force*) pub-topic)
	   )
	 )
       )
      (:larm
       (let ((rotate-coords (send *tfl* :lookup-transform "/base_link" "/l_gripper_tool_frame" (ros::time 0))))
	 (unless (null rotate-coords)
	   (send pub-topic :wrench (rotate-wrench (send msg :actual_pose :wrench_or_stiffness) (send rotate-coords :rot)))
	   (send pub-topic :header :frame_id "/l_gripper_tool_frame")
	   (ros::publish (format nil "~A/~A" *node-name* *l_arm_actual_pose_force*) pub-topic)
	   )
	 )
       )
      )
    )
  )

#|
(ros::advertise *r_arm_actual_force*
		geometry_msgs::WrenchStamped)
(ros::advertise *l_arm_actual_force*
		geometry_msgs::WrenchStamped)
|#

(ros::advertise (format nil "~A/~A" *node-name* *r_arm_actual_pose_force*)
		geometry_msgs::WrenchStamped)
(ros::advertise (format nil "~A/~A" *node-name* *l_arm_actual_pose_force*)
		geometry_msgs::WrenchStamped)


(ros::subscribe "r_arm_cart_imped_controller/state"
		ee_cart_imped_msgs::EEcartImpedFeedback
		#'visualize-rarm-cb)

(ros::subscribe "l_arm_cart_imped_controller/state"
		ee_cart_imped_msgs::EEcartImpedFeedback
		#'visualize-larm-cb)

(setq *tfl* (instance ros::transform-listener :init))
(ros::rate 30)

(do-until-key
 (ros::spin-once)
 (ros::sleep)
 (unless (ros::ok) (return))
 )