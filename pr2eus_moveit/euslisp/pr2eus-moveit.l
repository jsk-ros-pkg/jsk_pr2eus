(ros::load-ros-manifest "pr2eus_moveit")

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :collision-object-publisher "package://pr2eus_moveit/euslisp/collision-object-publisher.l")

(defvar *moveit-error-code-list*
  (list
   (cons 1 "SUCCESS")
   (cons 99999 "FAILURE")
   ;;
   (cons -1 "PLANNING_FAILED")
   (cons -2 "INVALID_MOTION_PLAN")
   (cons -3 "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE")
   (cons -4 "CONTROL_FAILED")
   (cons -5 "UNABLE_TO_AQUIRE_SENSOR_DATA")
   (cons -6 "TIMED_OUT")
   (cons -7 "PREEMPTED")
   ;; planning & kinematics request errors
   (cons -10 "START_STATE_IN_COLLISION")
   (cons -11 "START_STATE_VIOLATES_PATH_CONSTRAINTS")
   ;;
   (cons -12 "GOAL_IN_COLLISION")
   (cons -13 "GOAL_VIOLATES_PATH_CONSTRAINTS")
   (cons -14 "GOAL_CONSTRAINTS_VIOLATED")
   ;;
   (cons -15 "INVALID_GROUP_NAME")
   (cons -16 "INVALID_GOAL_CONSTRAINTS")
   (cons -17 "INVALID_ROBOT_STATE")
   (cons -18 "INVALID_LINK_NAME")
   (cons -19 "INVALID_OBJECT_NAME")
   ;; system errors
   (cons -21 "FRAME_TRANSFORM_FAILURE")
   (cons -22 "COLLISION_CHECKING_UNAVAILABLE")
   (cons -23 "ROBOT_STATE_STALE")
   (cons -24 "SENSOR_INFO_STALE")
   ;; kinematics errors
   (cons -31 "NO_IK_SOLUTION")
   ))

(defclass moveit-environment
  :super propertied-object
  :slots (config-list
          scene-service
          planning-service
          robot
          default-frame-id default-link
          multi-dof-name multi-dof-frame
          ))

;; frame-id
;; multi-dof-joint name/frame-id
;; group-name -> joint-list, target-link
(defmethod moveit-environment
  (:init
   (&key ((:scene-service sc-srv) "/get_planning_scene")
         ((:planning-service pl-srv) "/plan_kinematic_path")
         ((:robot rb) *pr2*) (frame-id "base_footprint") ;; frame-id needs to be contained in robot_model
         (multi-dof-joint-name "virtual_joint")
         (multi-dof-frame-id "odom_combined"))
   (setq scene-service sc-srv
         planning-service pl-srv
         robot rb
         default-frame-id frame-id
         multi-dof-name multi-dof-joint-name
         multi-dof-frame multi-dof-frame-id)
   (setq default-link (send self :search-link-from-name frame-id))
   (setq config-list (send self :default-configuration))
   self)
  (:robot (&rest args) (forward-message-to robot args))
  (:search-link-from-name
   (link-name)
   (cond
    ((find-method robot (intern (string-upcase (format nil "~A_lk" link-name)) *keyword-package*))
     (send robot (intern (string-upcase (format nil "~A_lk" link-name)) *keyword-package*)))
    ((find-method robot (intern (string-upcase link-name) *keyword-package*))
     (send robot (intern (string-upcase link-name) *keyword-package*)))
    (t
     (find-if #'(lambda (l) (cond ((symbolp (send l :name) (string= (symbol-string (send l :name)))))
                                  ((stirngp (send l :name) (string= (send l :name))))
                                  (t nil))) (send robot :links))
     )))
  (:copy-robot-state (rb)
   (send robot :reset-coords)
   (send robot :transform (send rb :worldcoords))
   (send robot :angle-vector (send rb :angle-vector))
   robot)
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "r_wrist_roll_link"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "l_wrist_roll_link"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rarm-torso
               (cons :group-name "right_arm_and_torso")
               (cons :target-link
                     (send self :search-link-from-name "r_wrist_roll_link"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :rarm :joint-list)))
               )
         (list :larm-torso
               (cons :group-name "left_arm_and_torso")
               (cons :target-link
                     (send self :search-link-from-name "l_wrist_roll_link"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :larm :joint-list)))
               )
         (list :arms ;; not implemented yet
               (cons :group-name "arms")
               (cons :target-link
                     (list (send self :search-link-from-name "l_wrist_roll_link")
                           (send self :search-link-from-name "r_wrist_roll_link")))
               (cons :joint-list (append (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
#| ;; sample of pr2_moveit_config/config/pr2.srdf
    <group name="base">
        <joint name="world_joint" />
    </group>
    <group name="left_arm">
        <chain base_link="torso_lift_link" tip_link="l_wrist_roll_link" />
    </group>
    <group name="left_arm_and_torso">
        <chain base_link="base_link" tip_link="l_wrist_roll_link" />
    </group>
    <group name="right_arm">
        <chain base_link="torso_lift_link" tip_link="r_wrist_roll_link" />
    </group>
    <group name="right_arm_and_torso">
        <chain base_link="base_link" tip_link="r_wrist_roll_link" />
    </group>
    <group name="arms">
        <group name="left_arm" />
        <group name="right_arm" />
    </group>
    <group name="head">
        <joint name="head_pan_joint" />
        <joint name="head_tilt_joint" />
    </group>
    <group name="torso">
        <joint name="torso_lift_joint" />
    </group>
    <group name="whole_body">
        <group name="base" />
        <group name="arms" />
        <group name="torso"/>
    <group/>
|#
         ))
  (:worldcoords->default-frame-relative (wcds)
   (send (send default-link :worldcoords) :transformation
         (send wcds :worldcoords)))
  (:get-group-tip-coords (confkey) ;; get-robot-coords
   (let ((link-inst(cdr (assoc :target-link (cdr (assoc confkey config-list))))))
     (send link-inst :copy-worldcoords)))
  (:get-planning-scene (&optional (components 1023))
   (get-planning-scene :scene-service scene-service :components components))
  (:get-ik-for-pose
   (cds confkey &key (use-actual-seed t) (retry t)
        (end-coords) ;; (list :rarm :end-coords)
        (frame-id default-frame-id) (timeout 0.05)
        (attempts) (avoid-collision t) &allow-other-keys)
   (let ((tgt-cds (send cds :copy-worldcoords))
         (group-name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
         (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
         scene rstate constraints)
     (when use-actual-seed
       (setq scene (send self :get-planning-scene))
       (when scene (setq rstate (send scene :robot_state))))

     (when end-coords ;; cds is target coords of end-coords
       (let ((rcds (send self :get-group-tip-coords confkey)))
         (send tgt-cds :transform
               (send (send (send* robot end-coords) :worldcoords)
                     :transformation rcds))))

     (setq tgt-cds (send self :worldcoords->default-frame-relative tgt-cds))

     (let* ((msg (ros::coords->tf-pose-stamped tgt-cds frame-id))
            (req (instance moveit_msgs::GetPositionIKRequest :init
                           :ik_request
                           (instance moveit_msgs::PositionIKRequest :init
                                     :group_name group-name
                                     :avoid_collisions avoid-collision
                                     :constraints (if constraints constraints
                                                    (instance moveit_msgs::constraints :init))
                                     :robot_state (if rstate rstate
                                                    (instance moveit_msgs::RobotState :init))
                                     :attempts (if attempts attempts 0)
                                     :timeout (ros::time timeout)
                                     :pose_stamped msg)))
            (res (ros::service-call "/compute_ik" req)))
       (cond
        ((= (send res :error_code :val) 1) ;; success
         ;; have to do apply multi-dof-joint ...
         (apply-joint_state (send res :solution :joint_state) robot))
        (t
         (warn ";; ik error at ~A~%"
               (assoc (send res :error_code :val) *moveit-error-code-list*))
         (return-from :get-ik-for-pose nil)))
       (send robot :angle-vector)
       )))
  (:motion-plan
   (confkey &key (scene)
            (group-name (cdr (assoc :group-name (cdr (assoc confkey config-list)))))
            (joint-list (cdr (assoc :joint-list (cdr (assoc confkey config-list)))))
            (planning-attempts 1) (planning-time 5.0) (tolerance-below 0.001) (tolerance-above 0.001)
            )
   (let ((mpr (instance moveit_msgs::motionplanrequest :init))
         res)
     ;;
     (unless scene
       (setq scene (send self :get-planning-scene)))

     (send mpr :workspace_parameters :header :stamp (ros::time-now))
     (send mpr :workspace_parameters :header :frame_id multi-dof-frame);;
     ;;
     (send mpr :workspace_parameters :max_corner :x 1.0)
     (send mpr :workspace_parameters :max_corner :y 1.0)
     (send mpr :workspace_parameters :max_corner :z 1.0)
     (send mpr :workspace_parameters :min_corner :x -1.0)
     (send mpr :workspace_parameters :min_corner :y -1.0)
     (send mpr :workspace_parameters :min_corner :z -1.0)
     ;;
     (send mpr :start_state (send scene :robot_state))

     (send mpr :goal_constraints
      (list
       (instance moveit_msgs::constraints :init :name ""
                 :joint_constraints
                 (mapcar #'(lambda (jn)
                             (instance moveit_msgs::jointconstraint :init
                                       :joint_name (send jn :name)
                                       :position (send jn :ros-joint-angle)
                                       :tolerance_above tolerance-above
                                       :tolerance_below tolerance-below
                                       :weight 1.0))
                         joint-list))))

     (send mpr :planner_id "") ;; ??
     (send mpr :group_name group-name)
     (send mpr :num_planning_attempts planning-attempts)
     (send mpr :allowed_planning_time planning-time)
     (setq res
           (ros::service-call planning-service
                              (instance moveit_msgs::GetMotionPlanRequest
                                        :init :motion_plan_request mpr)))
     (cond
      ((= (send res :motion_plan_response :error_code :val) 1) ;; success
       ;; have to do apply multi-dof-joint ...
       (send res :motion_plan_response))
      (t
       (warn ";; motion plan error at ~A~%"
             (assoc (send res :motion_plan_response :error_code :val)
                    *moveit-error-code-list*))
       (return-from :motion-plan nil)))
     ))
#|
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
  (:planning-make-trajectory
   (confkey &key (set-angle-vector) (get-scene t) (use-scene t) (planning-time 5.0)
            (filter-time 4.0) (wait t) &allow-other-keys)
   (let (ret scene)
     (if set-angle-vector (send robot :angle-vector set-angle-vector))
     (when get-scene (setq scene (send self :get-planning-scene)))
     (setq ret
           (send self :motion-plan confkey
                 :planning_time planning-time :filter_time filter-time
                 :wait-result wait :scene (if use-scene scene)))
     (cond
      ((derivedp ret arm_navigation_msgs::FilterJointTrajectoryWithConstraintsResponse)
       (cond
        ((> (send ret :error_code :val) 0)
         (send ret :trajectory))
        ((= (send ret :error_code :val) 0)
         (get ret :original-trajectory))))
      (t
       nil))
     ))
  (:planning-make-trajectory-to-coords
   (cds confkey &key (end-coords) ;; (list :rarm :end-coords)
        (planning-time 5.0) (wait t) (get-scene t) (frame-id default-frame-id)
        (filter-time 4.0) (use-scene t) &allow-other-keys)
   (let (ret scene)
     (when get-scene (setq scene (send self :get-planning-scene)))
     (when (send self :get-ik-for-pose cds confkey :end-coords end-coords
                 :use-actual-seed t :retry t :frame-id frame-id)
       (return-from :planning-make-trajectory-to-coords nil))
     (send self :planning-make-trajectory confkey
           :planning-time planning-time :filter-time filter-time
           :use-scene use-scene :wait wait :get-scene get-scene)
     ))
  (:planning-move-arm
   (confkey &key (set-angle-vector) (get-scene t) (planning-time 5.0) (wait t)
            &allow-other-keys)
   (let (ret)
     (if set-angle-vector (send robot :angle-vector set-angle-vector))
     (when get-scene (send self :get-planning-scene))
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
        (planning-time 5.0) (wait t) (get-scene t) (frame-id default-frame-id)
        &allow-other-keys)
   (let (ret)
     (when get-scene (send self :get-planning-scene))
     (when (steq ret (send self :get-ik-for-pose cds confkey :end-coords end-coords
                           :use-actual-seed t :retry t :frame-id frame-id))
       (return-from :planning-move-arm-to-coords ret))
     (send self :planning-move-arm confkey :planning-time planning-time :wait wait) ;; get-scene ?
     ))
|#
  )

(defun worldcoords->link-relative (wcds &key ((:link lname) "ROOT") (robot *pr2*))
  (let ((base (send robot (intern (string-upcase lname) *keyword-package*))))
    (send (send base :worldcoords) :transformation
          (send wcds :worldcoords))))