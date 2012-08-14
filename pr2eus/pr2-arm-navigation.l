
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

(defvar *arm-navigation-error-code-list*
  (list
   (cons "SUCCESS" 1)
   (cons "PLANNING_FAILED" -1)
   (cons "TIMED_OUT" -2)
   ;;# start state errors
   (cons "START_STATE_IN_COLLISION" -3)
   (cons "START_STATE_VIOLATES_PATH_CONSTRAINTS" -4)
   ;;# goal errors
   (cons "GOAL_IN_COLLISION" -5)
   (cons "GOAL_VIOLATES_PATH_CONSTRAINTS" -6)
   ;;# robot state
   (cons "INVALID_ROBOT_STATE" -7)
   (cons "INCOMPLETE_ROBOT_STATE" -8)
   ;;# planning request errors
   (cons "INVALID_PLANNER_ID" -9)
   (cons "INVALID_NUM_PLANNING_ATTEMPTS" -10)
   (cons "INVALID_ALLOWED_PLANNING_TIME" -11)
   (cons "INVALID_GROUP_NAME" -12)
   (cons "INVALID_GOAL_JOINT_CONSTRAINTS" -13)
   (cons "INVALID_GOAL_POSITION_CONSTRAINTS" -14)
   (cons "INVALID_GOAL_ORIENTATION_CONSTRAINTS" -15)
   (cons "INVALID_PATH_JOINT_CONSTRAINTS" -16)
   (cons "INVALID_PATH_POSITION_CONSTRAINTS" -17)
   (cons "INVALID_PATH_ORIENTATION_CONSTRAINTS" -18)
   ;;# state/trajectory monitor errors
   (cons "INVALID_TRAJECTORY" -19)
   (cons "INVALID_INDEX" -20)
   (cons "JOINT_LIMITS_VIOLATED" -21)
   (cons "PATH_CONSTRAINTS_VIOLATED" -22)
   (cons "COLLISION_CONSTRAINTS_VIOLATED" -23)
   (cons "GOAL_CONSTRAINTS_VIOLATED" -24)
   (cons "JOINTS_NOT_MOVING" -25)
   (cons "TRAJECTORY_CONTROLLER_FAILED" -26)
   ;;# system errors
   (cons "FRAME_TRANSFORM_FAILURE" -27)
   (cons "COLLISION_CHECKING_UNAVAILABLE" -28)
   (cons "ROBOT_STATE_STALE" -29)
   (cons "SENSOR_INFO_STALE" -30)
   ;;# kinematics errors
   (cons "NO_IK_SOLUTION" -31)
   (cons "INVALID_LINK_NAME" -32)
   (cons "IK_LINK_IN_COLLISION" -33)
   (cons "NO_FK_SOLUTION" -34)
   (cons "KINEMATICS_STATE_IN_COLLISION" -35)
   ;;# general errors
   (cons "INVALID_TIMEOUT" -36)))

;;
;; call service and action
;;
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
        #|(warn ";; action state ~A / ~A~%"
              (send arm-action-client :get-state)
              (send arm-action-client :get-goal-status-text))|#
        (ros::ros-info ";; action state result ~A / ~A"
                       (goal-status-string (send arm-action-client :get-state))
                       (send arm-action-client :get-goal-status-text))
        )
      ) ;; /when
    (unless arm-action-client
      ;;(warn ";; error ~%")
      (ros::ros-info ";; motion plan error")
      )
    arm-action-client
    ))

;; collision aware
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
      ;;(warn ";; IK service failed~%")
      (ros::ros-info ";; IK service failed")
      )
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

;;
;; wrapper for PR2
;;
(defun solve-constraint-ik (&optional (cds
                                       (send (send (send *pr2* :base_footprint) :worldcoords)
                                             :transformation
                                             (send (send *pr2* :r_wrist_roll_link) :worldcoords)))
                                      &key (arm :rarm) (use-actual-seed nil) (get-scene t)
                                      (reference-robot *pr2*) (frame-id "/base_footprint"))
  "Return nil if IK succeeded and error_code if failed "
  (let* ((msg (ros::coords->tf-pose-stamped cds frame-id)))
    (setq ret (get-ik-for-pose msg :arm arm :use-actual-seed use-actual-seed
                               :reference-robot reference-robot))
    ;; (print ret)
    (cond
     ((= (send ret :error_code :val) 1)
      ;;(warn ";; success IK~%")
      (ros::ros-info ";; success IK")
      (apply-joint_state (send ret :solution :joint_state) reference-robot)
      nil)
     (t
      #|(warn ";; IK error at ~A / ~A~%"
            (send ret :error_code :val)
            (elt *arm-navigation-error-code-list* (- (send ret :error_code :val))))|#
      (ros::ros-info ";; IK error at ~A / ~A"
                     (send ret :error_code :val)
                     (elt *arm-navigation-error-code-list* (- (send ret :error_code :val))))
      (send ret :error_code :val)))
    ))

(defun planning-move-arm (&key (arm :rarm) (get-scene t) (time 5.0) (wait t) (arm-action-client))
  (when get-scene (get-planning-scene))
  (move-arm-to-goal :arm arm :planning_time time :wait-result wait
                    :arm-action-client arm-action-client))

(defun planning-move-arm-to-coords (cds &key (arm :rarm) (frame-id "/base_footprint")
                                        (reference-robot *pr2*) (end-coords t)
                                        (time 5.0) (wait t) (arm-action-client))
  (let (ret)
    (when end-coords
      (setq cds (send cds :copy-worldcoords))
      (send cds :translate (float-vector -180 0 0)))
    (when (solve-constraint-ik cds :arm arm :use-actual-seed t :frame-id frame-id
                               :robot reference-robot)
      (if (solve-constraint-ik cds :arm arm :use-actual-seed nil :frame-id frame-id
                               :robot reference-robot)
          (return nil)))
    (setq ret
          (planning-move-arm :arm arm :time time :wait wait
                             :robot reference-robot
                             :arm-action-client arm-action-client))
    (and ret
         (= (send ret :get-state) actionlib_msgs::GoalStatus::*Succeeded*))
    ))
