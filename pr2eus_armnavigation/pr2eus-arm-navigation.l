
;;(ros::load-ros-manifest "pr2_interactive_manipulation")
(ros::load-ros-manifest "kinematics_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")

(ros::load-ros-manifest "pr2eus")
(load "package://pr2eus/robot-interface.l")

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
;;
;;
(defclass arm_planning_environment
  :super propertied-object
  :slots (config-list
          planning-scene-service
          planner-id
          arm-planning-service-name
          robot default-frame-id
          ))

(defmethod arm_planning_environment
  (:init
   (&key ((:planning-scene-service pl-srv) "/environment_server/set_planning_scene_diff")
         ((:planner-id pl-id) "SBLkConfig1")
         ((:arm-planning-service-name arm-pl-srv) "ompl_planning/plan_kinematic_path")
         ((:robot rb) *pr2*) (frame-id "/base_footprint")) ;; frame-id needs to be contained in robot_model
   (setq planning-scene-service pl-srv
         planner-id pl-id
         arm-planning-service-name arm-pl-srv
         robot rb
         default-frame-id frame-id)
   (setq config-list
         (mapcar #'(lambda (conf)
                     (let* ((conf-car (car conf))
                            (conf-cdr (cdr conf))
                            (nm (cdr (assoc :action-name conf-cdr)))
                            action)
                       (cond
                        (nm
                         (setq action (instance ros::simple-action-client :init
                                                nm arm_navigation_msgs::MoveArmAction))
                         (unless (send action :wait-for-server 5)
                           (ros::ros-error ";; planning action ~A not found." nm)))
                        (t (ros::ros-error ";; :action-name not found in ~A" conf)))
                       (push (cons :action action) conf-cdr)
                       (cons conf-car conf-cdr)))
                 (send self :default-configuration)))
   self)
  (:robot (&rest args) (forward-message-to robot args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :action-name "/move_right_arm")
               (cons :target-link-name "r_wrist_roll_link")
               (cons :ik-service-name "/pr2_right_arm_kinematics/get_constraint_aware_ik")
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :action-name "/move_left_arm")
               (cons :target-link-name "l_wrist_roll_link")
               (cons :ik-service-name "/pr2_left_arm_kinematics/get_constraint_aware_ik")
               (cons :joint-list (send robot :larm :joint-list))
               )))
  (:get-planning-scene
   (&key (collision-operations (instance arm_navigation_msgs::OrderedCollisionOperations :init))
         (link-padding)) ;; (list (instance arm_navigation_msgs::LinkPadding :init) .. )
   (let ((planning_scene_req
          (instance arm_navigation_msgs::SetPlanningSceneDiffRequest :init)))
     (send planning_scene_req :planning_scene_diff :link_padding link-padding)
     (send planning_scene_req :operations collision-operations)
     (ros::service-call planning-scene-service planning_scene_req)))
  (:get-ik-for-pose-single
   (pose_stamped confkey &key (timeout 2.0) (use-actual-seed t))
   (let ((ik-req (instance kinematics_msgs::GetConstraintAwarePositionIKrequest :init))
         (link-name (cdr (assoc :target-link-name (cdr (assoc confkey config-list)))))
         (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
         (ik-service-name (cdr (assoc :ik-service-name (cdr (assoc confkey config-list)))))
         ik-res)
     (unless (and link-name ik-service-name)
       ;; error
       (ros::ros-error "configuration not found ~A" (list confkey link-name ik-service-name))
       (return-from :get-ik-for-pose-single))
     (send ik-req :ik_request :ik_link_name link-name)
     (send ik-req :ik_request :pose_stamped pose_stamped)
     (let (names positions)
       (send ik-req :ik_request :ik_seed_state :joint_state
             (joint-list->joint_state joint-list
                                      :position (if use-actual-seed nil 0.0)
                                      :effort nil :velocity nil))
       (send ik-req :timeout (ros::time timeout))
       (setq ik-res (ros::service-call ik-service-name ik-req)))
     (unless ik-res
       (ros::ros-info ";; IK service failed"))
     ik-res))
  (:get-robot-coords
   (confkey)
   (let ((link-name (cdr (assoc :target-link-name (cdr (assoc confkey config-list))))))
     (send robot (intern (string-upcase link-name) *keyword-package*) :copy-worldcoords)
     ))
  (:motion-plan
   (confkey &key (tolerance_below 0.08) (tolerance_above 0.08) (scene)
            (planning_time 5.0) (filter_time 4.0) (wait-result t))
   (let ((group_name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
         (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
         (gl (instance arm_navigation_msgs::GetMotionPlanRequest :init))
         res (ret t))
     (unless (and group_name joint-list)
       ;; error
       (ros::ros-error "configuration not found ~A" (list confkey group_name joint-list))
       (return-from :motion-plan))
     ;;
     (send gl :motion_plan_request :group_name group_name)
     (send gl :motion_plan_request :num_planning_attempts 1)
     (send gl :motion_plan_request :allowed_planning_time (ros::Time planning_time))
     (send gl :motion_plan_request :planner_id planner-id)

     (send gl :motion_plan_request :goal_constraints :joint_constraints
           (mapcar #'(lambda (j)
                       (let* ((n (send j :name))
                              (nm (if (symbolp n) (symbol-name n) n)))
                         (instance arm_navigation_msgs::JointConstraint :init
                                   :joint_name nm
                                   :position (send j :ros-joint-angle)
                                   :tolerance_above tolerance_above
                                   :tolerance_below tolerance_below)
                         )) joint-list))
     ;; call service
     (setq res (ros::service-call arm-planning-service-name gl))

     (when (and res wait-result)
       (ros::ros-info ";; plan service result -> ~A"
                      (let ((val (send res :error_code :val)))
                        (cond
                         ((> val 0) (setq ret nil) "Succeeded")
                         (t (elt *arm-navigation-error-code-list* (- val)))))))
     (if ret (return-from :motion-plan res))

     ;;filter
     (unless scene
       (setq scene (send (send self :get-planning-scene) :planning_scene)))
     (let ((req (instance arm_navigation_msgs::FilterJointTrajectoryWithConstraintsRequest :init
                          :group_name group_name
                          :start_state (send scene :robot_state)
                          :trajectory (send res :trajectory :joint_trajectory)
                          :goal_constraints (send gl :motion_plan_request :goal_constraints)
                          :allowed_time (ros::Time filter_time))))
       (setq res (ros::service-call "/trajectory_filter_server/filter_trajectory_with_constraints" req))
       (when (and res wait-result)
         (ros::ros-info ";; filter service result -> ~A"
                        (let ((val (send res :error_code :val)))
                          (cond
                           ((> val 0) "Succeeded")
                           (t (elt *arm-navigation-error-code-list* (- val))))))))

     res))
  (:move-arm-to-goal
   (confkey &key (tolerance_below 0.08) (tolerance_above 0.08)
            (planning_time 5.0) (wait-result t))
   (let ((arm-action-client (cdr (assoc :action (cdr (assoc confkey config-list)))))
         (group_name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
         (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
         (gl (instance arm_navigation_msgs::MoveArmGoal :init)))
     (unless (and arm-action-client group_name joint-list)
       ;; error
       (ros::ros-error "configuration not found ~A"
                       (list confkey arm-action-client group_name joint-list))
       (return-from :move-arm-to-goal))

      ;; (send gl :planning_scene_diff :link_padding link_padding)
      ;; (send gl :operations collision_operations)
     (send gl :motion_plan_request :group_name group_name)
     (send gl :motion_plan_request :num_planning_attempts 1)
     (send gl :motion_plan_request :allowed_planning_time (ros::Time planning_time))
     (send gl :motion_plan_request :planner_id planner-id)
     (send gl :planner_service_name arm-planning-service-name)

     (send gl :motion_plan_request :goal_constraints :joint_constraints
           (mapcar #'(lambda (j)
                       (let* ((n (send j :name))
                              (nm (if (symbolp n) (symbol-name n) n)))
                         (instance arm_navigation_msgs::JointConstraint :init
                                   :joint_name nm
                                   :position (send j :ros-joint-angle)
                                   :tolerance_above tolerance_above
                                   :tolerance_below tolerance_below)
                         )) joint-list))
     ;; send goal
     (send arm-action-client :send-goal
           (instance arm_navigation_msgs::MoveArmActionGoal :init :goal gl))

     (when wait-result
       (send arm-action-client :wait-for-result)
       (ros::ros-info ";; action result -> ~A / ~A / ~A"
                      (ros::goal-status-to-string (send arm-action-client :get-state))
                      (let ((val (send (send arm-action-client :get-result) :error_code :val)))
                        (if (> val 0) "Succeeded"
                          (elt *arm-navigation-error-code-list* (- val))))
                      (send arm-action-client :get-goal-status-text)))
     arm-action-client
     ))
  ;;; wrapper
  (:get-ik-for-pose
   (cds confkey &key (use-actual-seed t) (retry t) (get-scene t)
        (end-coords) ;; (list :rarm :end-coords)
        (frame-id default-frame-id) (timeout 2.0))
   (when get-scene (send self :get-planning-scene))
   (when end-coords
     (let ((rcds (send self :get-robot-coords confkey)))
       (setq cds (send cds :copy-worldcoords))
       (send cds :transform
             (send (send (send* robot end-coords) :worldcoords)
                   :transformation rcds))))
   (let* ((msg (ros::coords->tf-pose-stamped cds frame-id))
          (ret (send self :get-ik-for-pose-single msg confkey
                     :use-actual-seed use-actual-seed
                     :timeout timeout)))
     (cond
      ((null ret) -255)
      ((= (send ret :error_code :val) 1)
       (ros::ros-info ";; success IK")
       (apply-joint_state (send ret :solution :joint_state) robot)
       nil)
      (t
       (ros::ros-info ";; IK error at ~A / ~A"
                      (send ret :error_code :val)
                      (elt *arm-navigation-error-code-list* (- (send ret :error_code :val))))
       (if retry
           (send self :get-ik-for-pose cds confkey
                 :retry nil :use-actual-seed (if use-actual-seed nil t)
                 :timeout timeout)
         (send ret :error_code :val)))
      )))
  (:planning-make-trajectory
   (confkey &key (angle-vector) (get-scene t) (use-scene t) (planning-time 5.0) (filter-time 4.0) (wait t))
   )
  (:planning-make-trajectory-to-coords
   (cds confkey &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (wait t) (get-scene t) (frame-id default-frame-id)
        (filter-time 4.0) (use-scene t))
   )
  (:planning-move-arm
   (confkey &key (angle-vector) (get-scene t) (planning-time 5.0) (wait t))
   (let (ret)
     (when get-scene (send self :get-planning-scene))
     (if angle-vector (send robot :angle-vector angle-vector))
     (setq ret
           (send self :move-arm-to-goal confkey :planning_time planning-time :wait-result wait))
     (if ret (setq ret (send ret :get-result)))
     (cond
      ((null ret)
       ;;(warn "~%");; error
       -255)
      ((= (send ret :error_code :val) 1) nil)
      (t
       (ros::ros-error "~A" (elt *arm-navigation-error-code-list* (- (send ret :error_code :val))))
       (send ret :error_code :val)
       ))))
  (:planning-move-arm-to-coords
   (cds confkey &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (wait t) (get-scene t) (frame-id default-frame-id))
   (when get-scene (send self :get-planning-scene))
   (when (send self :get-ik-for-pose cds confkey
               :use-actual-seed t :retry t :frame-id frame-id)
     (return-from :planning-move-arm-to-coords nil))
   (send self :planning-move-arm confkey :planning-time planning-time :wait wait) ;; get-scene ?
   )
  )

(defun worldcoords->link-relative (wcds &key ((:link lname) "ROOT") (robot *pr2*))
  (let ((base (send robot (intern (string-upcase lname) *keyword-package*))))
    (send (send base :worldcoords) :transformation
          (send wcds :worldcoords))))
