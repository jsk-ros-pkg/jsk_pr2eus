
;;(ros::load-ros-manifest "pr2_interactive_manipulation")
(ros::load-ros-manifest "kinematics_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")

(defvar *MOVE_ARM_PLANNER_ID* "SBLkConfig1")
(defvar *MOVE_ARM_PLANNER_SERVICE_NAME* "ompl_planning/plan_kinematic_path")
(defvar *RIGHT_ARM_ACTION* "/move_right_arm")
(defvar *LEFT_ARM_ACTION*  "/move_left_arm")
(defvar *RIGHT_ARM_IK* "/pr2_right_arm_kinematics/get_constraint_aware_ik")
(defvar *LEFT_ARM_IK*  "/pr2_left_arm_kinematics/get_constraint_aware_ik")
(defvar *PLANNING_SCENE_SERVICE* "/environment_server/set_planning_scene_diff")

(defun move-arm-to-goal (&key (robot *pr2*) (arm :rarm)
                              (group-names
                               (list (cons :rarm "right_arm")
                                     (cons :larm "left_arm")))
                              (tolerance_below 0.08) (tolerance_above 0.08)
                              (planning_time 5.0)
                              (action-list
                               (list (cons :rarm *RIGHT_ARM_ACTION*)
                                     (cons :larm *LEFT_ARM_ACTION*)))
                              (wait-result t)
                              (arm-action-client))
  (let ((action_name (cdr (assoc arm action-list)))
        (group_name (cdr (assoc arm group-names)))
        (gl (instance arm_navigation_msgs::MoveArmGoal :init)))
    (when (and action_name group_name)
      (unless arm-action-client
        (setq arm-action-client
              (instance ros::simple-action-client :init
                        action_name
                        arm_navigation_msgs::MoveArmAction))
        (send arm-action-client :wait-for-server))

      ;; (send gl :planning_scene_diff :link_padding link_padding)
      ;; (send gl :operations collision_operations)
      (send gl :motion_plan_request :group_name group_name)
      (send gl :motion_plan_request :num_planning_attempts 1)
      (send gl :motion_plan_request :allowed_planning_time (ros::Time planning_time))
      (send gl :motion_plan_request :planner_id *MOVE_ARM_PLANNER_ID*)
      (send gl :planner_service_name *MOVE_ARM_PLANNER_SERVICE_NAME*)

      (send gl :motion_plan_request :goal_constraints :joint_constraints
            (mapcar #'(lambda (j)
                        (let* ((n (send j :name))
                               (nm (if (symbolp n) (symbol-name n) n)))
                          (instance arm_navigation_msgs::JointConstraint :init
                                    :joint_name nm
                                    :position (deg2rad (send j :joint-angle))
                                    :tolerance_above tolerance_above
                                    :tolerance_below tolerance_below)
                          ))
                    (send robot arm :joint-list)))

      (send arm-action-client :send-goal
            (instance arm_navigation_msgs::MoveArmActionGoal :init :goal gl))

      (when wait-result
        (send arm-action-client :wait-for-result)
        (warn ";; action state ~A / ~A~%"
              (send arm-action-client :get-state)
              (send arm-action-client :get-goal-status-text)))
      ) ;; /when
    (unless arm-action-client
      (warn ";; error ~%"))
    arm-action-client
    ))

(defun get-ik-for-pose (pose_stamped &key (robot *pr2*) (arm :rarm) (timeout 2.0)
                                     (target-link-names
                                      (list (cons :rarm "r_wrist_roll_link")
                                            (cons :larm "l_wrist_roll_link")))
                                     (use-actual-seed t)
                                     (ik-service-list
                                      (list (cons :rarm *RIGHT_ARM_IK*)
                                            (cons :larm *LEFT_ARM_IK*))))
  (let ((ik-req (instance kinematics_msgs::GetConstraintAwarePositionIKrequest :init))
        (link-name (cdr (assoc arm target-link-names)))
        (ik-service-name (cdr (assoc arm ik-service-list)))
        ik-res)
    (when (and link-name ik-service-name)
      (send ik-req :ik_request :ik_link_name link-name)
      (send ik-req :ik_request :pose_stamped pose_stamped)
      (let (names positions)

        (send ik-req :ik_request :ik_seed_state :joint_state
              (joint-list->joint_state (send robot arm :joint-list)
                                       :position (if use-actual-seed nil 0.0)
                                       :effort nil :velocity nil))

        (send ik-req :timeout (ros::time timeout))
        (setq ik-res (ros::service-call ik-service-name ik-req))
        )) ;; /when
    (unless ik-res
      (warn ";; IK service failed~%"))
    ik-res))

(defun get-planning-scene (&key (collision-operations
                                 (instance arm_navigation_msgs::OrderedCollisionOperations :init))
                                (link-padding) ;; (list (instance arm_navigation_msgs::LinkPadding :init) .. )
                                (planning-scene-service *PLANNING_SCENE_SERVICE*))
  (let ((planning_scene_req
         (instance arm_navigation_msgs::SetPlanningSceneDiffRequest :init)))
    (send planning_scene_req :planning_scene_diff :link_padding link-padding)
    (send planning_scene_req :operations collision-operations)
    (ros::service-call planning-scene-service planning_scene_req)
    ))
