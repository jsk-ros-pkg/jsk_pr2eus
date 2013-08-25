(ros::load-ros-manifest "pr2eus_moveit")

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :collision-object-publisher "package://pr2_moveit/euslisp/collision-object-publisher.l")

(defclass moveit_environment
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
(defmethod moveit_environment
  (:init
   (&key ((:scene-service sc-srv) "/environment_server/set_planning_scene_diff")
         ((:planning-service pl-srv) "ompl_planning/plan_kinematic_path")
         ((:robot rb) *pr2*) (frame-id "base_footprint") ;; frame-id needs to be contained in robot_model
         (multi-dof-joint-name "virtual_joint")
         (multi-dof-frame-id "odom_combined"))
   (setq scene-service sc-srv
         planning-service pl-srv
         robot rb
         default-frame-id frame-id)
   (setq default-link (send self :search-link-from-name frame-id))
   (setq config-list (send self :default-configuration))
   self)
  (:robot (&rest args) (forward-message-to robot args))
  (:search-link-from-name
   (link-name)
   (cond
    ((find-method robot (intern (string-upcase (format nil "~A_lk")) *keyword-package*))
     (send robot (intern (string-upcase (format nil "~A_lk")) *keyword-package*)))
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
               )))
  (:worldcoords->default-frame-relative (wcds)
   (send (send default-link :worldcoords) :transformation
         (send wcds :worldcoords)))
  (:get-group-tip-coords (confkey) ;; get-robot-coords
   (let ((link-inst(cdr (assoc :target-link (cdr (assoc confkey config-list))))))
     (send link-inst :copy-worldcoords)))
#|
  (:get-planning-scene
   (&key (collision-operations (instance arm_navigation_msgs::OrderedCollisionOperations :init))
         (link-padding))
   (let ((planning_scene_req
          (instance arm_navigation_msgs::SetPlanningSceneDiffRequest :init)))
     (send planning_scene_req :planning_scene_diff :link_padding link-padding)
     (send planning_scene_req :operations collision-operations)
     (ros::service-call planning-scene-service planning_scene_req))
   )
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
                         ((= val 0) "service Aborted ??")
                         (t (elt *arm-navigation-error-code-list* (- val)))))))
     (if ret (return-from :motion-plan res))

     ;;filter
     (unless scene
       (setq scene (send self :get-planning-scene)))
     ;; Are blew lines needed ???
     (let ((hdr_traj (send res :trajectory :joint_trajectory :header))
           (hdr_scene (send scene :planning_scene :robot_state :joint_state :header)))
       (send hdr_traj :stamp (send hdr_scene :stamp)))

     (let ((req (instance arm_navigation_msgs::FilterJointTrajectoryWithConstraintsRequest :init
                          :group_name group_name
                          :start_state (send scene :planning_scene :robot_state)
                          :trajectory (send res :trajectory :joint_trajectory)
                          :goal_constraints (send gl :motion_plan_request :goal_constraints)
                          :allowed_time (ros::Time filter_time))))
       (setq res (ros::service-call "/trajectory_filter_server/filter_trajectory_with_constraints" req))
       (when (and res wait-result)
         (ros::ros-info ";; filter service result -> ~A"
                        (let ((val (send res :error_code :val)))
                          (cond
                           ((> val 0) "Succeeded")
                           ((= val 0) (setf (get res :original-trajectory)
                                            (send req :trajectory))
                            "service Aborted ??")
                           (t (elt *arm-navigation-error-code-list* (- val))))
                        ))))
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
        (frame-id default-frame-id) (timeout 2.0) &allow-other-keys)
   (let ((tgt-cds (send cds :copy-worldcoords)))
     (when get-scene (send self :get-planning-scene))

     (when end-coords ;; cds is target coords of end-coords
       (let ((rcds (send self :get-robot-coords confkey)))
         (send tgt-cds :transform
               (send (send (send* robot end-coords) :worldcoords)
                     :transformation rcds))))

   (setq tgt-cds (send self :worldcoords->default-frame-relative tgt-cds))

   (let* ((msg (ros::coords->tf-pose-stamped tgt-cds frame-id))
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
                 :end-coords end-coords :frame-id frame-id :get-scene get-scene
                 :retry nil :use-actual-seed (if use-actual-seed nil t)
                 :timeout timeout)
         (send ret :error_code :val))
       ));;/cond
     )))
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