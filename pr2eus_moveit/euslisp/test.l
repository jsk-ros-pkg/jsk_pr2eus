;;(ros::load-ros-manifest "pr2eus_moveit")
(ros::roseus-add-msgs "moveit_msgs")
(ros::roseus-add-srvs "moveit_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "moveit_roseus_test")

(setq mpr (instance moveit_msgs::motionplanrequest :init))
;;
;;(send mpr :workspace_parameters)
(send mpr :workspace_parameters :header :stamp (ros::time-now))
(send mpr :workspace_parameters :header :frame_id "/odom_combined")
(send mpr :workspace_parameters :min_corner :x -1.0)
(send mpr :workspace_parameters :min_corner :y -1.0)
(send mpr :workspace_parameters :min_corner :z -1.0)
(send mpr :workspace_parameters :max_corner :x 1.0)
(send mpr :workspace_parameters :max_corner :y 1.0)
(send mpr :workspace_parameters :max_corner :z 1.0)

;;(send mpr :start_state)
;;;;; have to add joint_state
(send mpr :start_state :joint_state :header :frame_id "/odom_combined")
(send mpr :start_state :joint_state :name
      (list "bl_caster_rotation_joint" "bl_caster_l_wheel_joint" "bl_caster_r_wheel_joint" "br_caster_rotation_joint" "br_caster_l_wheel_joint" "br_caster_r_wheel_joint" "fl_caster_rotation_joint" "fl_caster_l_wheel_joint" "fl_caster_r_wheel_joint" "fr_caster_rotation_joint" "fr_caster_l_wheel_joint" "fr_caster_r_wheel_joint" "torso_lift_joint" "head_pan_joint" "head_tilt_joint" "l_shoulder_pan_joint" "l_shoulder_lift_joint" "l_upper_arm_roll_joint" "l_elbow_flex_joint" "l_forearm_roll_joint" "l_wrist_flex_joint" "l_wrist_roll_joint" "l_gripper_l_finger_joint" "l_gripper_l_finger_tip_joint" "l_gripper_motor_slider_joint" "l_gripper_motor_screw_joint" "l_gripper_r_finger_joint" "l_gripper_r_finger_tip_joint" "l_gripper_joint" "laser_tilt_mount_joint" "r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint" "r_gripper_l_finger_joint" "r_gripper_l_finger_tip_joint" "r_gripper_motor_slider_joint" "r_gripper_motor_screw_joint" "r_gripper_r_finger_joint" "r_gripper_r_finger_tip_joint" "r_gripper_joint" "torso_lift_motor_screw_joint"))
(send mpr :start_state :joint_state :position
      #f(8.916531879954448e-06 0.001902523434713643 0.0007114896340292276 1.1504606676204787e-05 -0.004408615577324859 -0.005406824099976504 2.3408729827423258e-05 0.005203134665079823 0.0044398849974323085 3.064780287864721e-05 -5.6350000126492716e-05 -0.0006982058996234386 0.01194897691791269 0.0003930625987349501 -0.023556576604458535 0.00010442550733547051 0.003199836154895941 -0.001012493898770117 -0.31173415993378484 -0.0011607062892577957 -0.42816375034795584 -0.0002080700119515555 0.0021135292061295885 0.0021135292061295885 0.0 0.0 0.0021135292061295885 0.0021135292061295885 3.185428774270542e-06 -0.1527930319704769 -7.277861638943506e-05 0.0032225452630996188 0.001153518961040767 -0.31283681948780284 0.0015777360839246413 -0.4287278728307644 9.339550654807738e-05 0.0021114933447395623 0.0021114933447395623 0.0 0.0 0.0021114933447395623 0.0021114933447395623 2.8136527798945936e-06 0.0))
(send mpr :start_state :joint_state :velocity
      #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))

;;(send mpr :start_state :multi_dof_joint_state)
(send mpr :start_state :multi_dof_joint_state :header :frame_id "/odom_combined")
(send mpr :start_state :multi_dof_joint_state :joint_names (list "virtual_joint"))
(send mpr :start_state :multi_dof_joint_state :joint_transforms
      (list (instance geometry_msgs::Transform :init :rotation (instance geometry_msgs::quaternion :init :w 1.0))))

(send mpr :goal_constraints
      (list
       (instance moveit_msgs::constraints :init :name ""
                 :joint_constraints
                 (mapcar #'(lambda (jn pos)
                             (instance moveit_msgs::jointconstraint :init
                                       :joint_name jn :position pos
                                       :tolerance_above 0.001 :tolerance_below 0.001
                                       :weight 1.0))
                         (list "r_elbow_flex_joint"
                               "r_forearm_roll_joint"
                               "r_shoulder_lift_joint"
                               "r_shoulder_pan_joint"
                               "r_upper_arm_roll_joint"
                               "r_wrist_flex_joint"
                               "r_wrist_roll_joint")
                         (list -1.05883253436
                               -2.58503687268
                               -0.100279476582
                               -0.393630678861
                               -0.111580584313
                               -0.41650041924
                               1.13848036479)))
       ))

(send mpr :planner_id "")
(send mpr :group_name "right_arm")
(send mpr :num_planning_attempts 1)
(send mpr :allowed_planning_time 5.0)

;;
;;(ros::service-call "/plan_kinematic_path" (instance moveit_msgs::GetMotionPlanRequest :init :motion_plan_request mpr))
;;
