#!/usr/bin/env roseus
(ros::load-ros-manifest "pr2eus_openrave")
(ros::load-ros-manifest "orrosplanning")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2eus_openrave")

(ros::advertise "openrave_marker_array" visualization_msgs::MarkerArray 100)

;; We set dae-list below because package://pr2eus/pr2.l do not include collada file path, waiting for its support. This is very temporary.
(setq *arm-dae-list*
      (list
       "package://pr2_description/meshes/gripper_v0/gripper_palm.dae"
       "package://pr2_description/meshes/gripper_v0/l_finger.dae"
       "package://pr2_description/meshes/gripper_v0/l_finger.dae"
       "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"
       "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"
       "package://pr2_description/meshes/forearm_v0/wrist_flex.dae"
       "package://pr2_description/meshes/forearm_v0/forearm.dae"
       "package://pr2_description/meshes/upper_arm_v0/upper_arm.dae"
       "package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae"))

(defun mesh->marker-msg
  (cds mesh_resource header
       &key ((:color col) (float-vector 1 1 1)) ((:scale sc) 1000)
       ((:id idx) 0) ((:mesh_use_embedded_materials use_embedded) t)
       (alpha 1.0) ns lifetime)
    (let* ((msg (instance visualization_msgs::Marker :init
			  :type visualization_msgs::Marker::*MESH_RESOURCE*
			  :header header :id idx)))
      (send msg :mesh_resource mesh_resource)
      (send msg :mesh_use_embedded_materials use_embedded)
      (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
      (send msg :color (vector->rgba col alpha))
      (send msg :frame_locked t)
      (send msg :pose (ros::coords->tf-pose cds))
      (if ns (send msg :ns ns))
      (if lifetime (send msg :lifetime (ros::time lifetime)))
      msg))

(defmethod pr2-interface
  (:service-call-joint-state
   ()
   (let ((req (instance orrosplanning::SetJointStateRequest :init)) res)
     (send self :update-robot-state)
     (send req :jointstate (send self :publish-joint-state))
     (if (and
	  (ros::service-exists "SetJointState")
	  (ros::wait-for-service "SetJointState" 30))
	 (setq res (ros::service-call "SetJointState" req))
       (ros::ros-warn "SetJointState did not respond"))
     res))
  (:angle-vector-motion-plan
   (av &key (move-target :larm) (use-torso t))
   (let (coords)
     (send robot :angle-vector av)
     (setq coords (send robot move-target :end-coords :copy-worldcoords))
     (send self :move-end-coords-plan coords :move-target move-target :use-torso use-torso)))
  (:move-end-coords-plan
   (coords &key (move-target :larm) (use-torso t))
   (let (joint-trajectory joint-trajectory-points joints avs tms (tm (ros::time -1)) robot-list (diff-sum 0) (diff-thre 100))
     ;; call openrave
     (send self :show-goal-hand-coords coords move-target)

     (setq joint-trajectory
	   (send self :call-openrave-move-to-hand-position
		 coords
		 (format nil "~A~A" (case move-target (:larm "leftarm") (:rarm "rightarm")) (if use-torso "_torso" ""))
		 (case move-target (:larm "l_gripper_tool_frame") (:rarm "r_gripper_tool_frame"))))
     ;; JointTrajectory to angle-vector
     (if joint-trajectory
	 (setq joint-trajectory-points (send joint-trajectory :points)
	       joints (mapcar #'(lambda (x) (send robot (intern (string-upcase x) *keyword-package*))) (send joint-trajectory :joint_names))))
     ;;
     (if (= (length joint-trajectory-points) 0) (return-from :move-end-coords-plan nil))
     (dolist (point joint-trajectory-points)
       (mapcar '(lambda (ajoint aposition)
		  (cond
		   ((derivedp ajoint rotational-joint)
		    (send ajoint :joint-angle (rad2deg aposition)))
		   (t
		    (send ajoint :joint-angle (* 1000 aposition)))))
	       joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (when (cadr avs)
	 (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
	 (if (> diff-sum diff-thre)
	     (progn (setq diff-sum 0)(push (send self :get-robot-list move-target) robot-list))))
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000) tms)
       (setq tm (send point :time_from_start))
       )
     (push (send self :get-robot-list move-target) robot-list)
     ;; send visualization_msgs to rviz ;;TODO adjust time to *ri* ?
     (send self :show-mesh-traj-with-color *arm-dae-list* robot-list)



     (send robot :angle-vector (car (last avs)))
     ;; send to ri
     (send self :angle-vector-sequence (nreverse avs) (nreverse tms))

     (if avs
	 (list avs tms))
     ))
  (:call-openrave-move-to-hand-position
   (coords manip-name hand-frame-id)
   (unless coords
     (ros::ros-warn "armplanning-srvcall: no target coords is specified")
     (return-from :call-openrave-move-to-hand-position nil))
   (unless manip-name
     (ros::ros-warn "armplanning-srvcall: no manip-name is specified")
     (return-from :call-openrave-move-to-hand-position nil))
   (let ((req (instance orrosplanning::MoveToHandPositionRequest :init))
	 (hand-goal (ros::coords->tf-pose-stamped coords "base_footprint"))
	 res (mux-req (instance topic_tools::MuxSelectRequest :init)))
     (ros::ros-info "pos ~A ~A ~A"
		    (send hand-goal :pose :position :x)
		    (send hand-goal :pose :position :y)
		    (send hand-goal :pose :position :z))
     (ros::ros-info "ori ~A ~A ~A ~A"
		    (send hand-goal :pose :orientation :x)
		    (send hand-goal :pose :orientation :y)
		    (send hand-goal :pose :orientation :z)
		    (send hand-goal :pose :orientation :w))
     (send req :hand_frame_id hand-frame-id)
     (send req :manip_name manip-name)
     (send req :hand_goal hand-goal)

     (send self :service-call-joint-state)

     (when (ros::service-exists "collision_map_mux/select")
       (send mux-req :topic "collision_map_occ")
       (ros::service-call "collision_map_mux/select" mux-req))
     (if (and
	  (ros::service-exists "MoveToHandPosition")
	  (ros::wait-for-service "MoveToHandPosition" 30))
	 (setq res (ros::service-call "MoveToHandPosition" req))
       (ros::ros-warn "MoveToHandPosition did not respond"))
     (when (ros::service-exists "collision_map_mux/select")
       (send mux-req :topic "collision_map_none")
       (ros::service-call "collision_map_mux/select" mux-req))

     (if res (send res :traj))))

  (:show-goal-hand-coords ;;very temporary
   (coords move-target)
   (let (robot-list (hand-state (make-list 4)) (goal-hand-parts-list (make-list 5)))
     (setq robot-list (send self :get-robot-list move-target))
     (setf (elt hand-state 0) (send (elt robot-list 0) :transformation (elt robot-list 1)))
     (setf (elt hand-state 1) (send (elt robot-list 0) :transformation (elt robot-list 2)))
     (setf (elt hand-state 2) (send (elt robot-list 0) :transformation (elt robot-list 3)))
     (setf (elt hand-state 3) (send (elt robot-list 0) :transformation (elt robot-list 4)))
     (setf (elt goal-hand-parts-list 0) (send (send coords :copy-worldcoords) :translate (float-vector -180 0 0) :local))
     (setf (elt goal-hand-parts-list 1) (send (send (elt goal-hand-parts-list 0) :copy-worldcoords) :transform (elt hand-state 0)))
     (setf (elt goal-hand-parts-list 2) (send (send (elt goal-hand-parts-list 0) :copy-worldcoords) :transform (elt hand-state 1)))
     (setf (elt goal-hand-parts-list 3) (send (send (elt goal-hand-parts-list 0) :copy-worldcoords) :transform (elt hand-state 2)))
     (setf (elt goal-hand-parts-list 4) (send (send (elt goal-hand-parts-list 0) :copy-worldcoords) :transform (elt hand-state 3)))
     (send self :show-mesh-traj-with-color (subseq *arm-dae-list* 0 5) (list goal-hand-parts-list) :lifetime 0 :color #f(1 0 0))))

  (:get-robot-list ;;very temporary
   (move-target)
   (let ((robot-list (make-list (length *arm-dae-list*))))
     (if (eq move-target :rarm)
	 (progn
	   (setf (elt robot-list 0) (send robot :r_gripper_palm_link :copy-worldcoords))
	   (setf (elt robot-list 1) (send robot :r_gripper_l_finger_link :copy-worldcoords))
	   (setf (elt robot-list 2) (send (send robot :r_gripper_r_finger_link :copy-worldcoords) :rotate pi :x))
	   (setf (elt robot-list 3) (send robot :r_gripper_l_finger_tip_link :copy-worldcoords))
	   (setf (elt robot-list 4) (send (send robot :r_gripper_r_finger_tip_link :copy-worldcoords) :rotate pi :x))
	   (setf (elt robot-list 5) (send robot :r_wrist_flex_link :copy-worldcoords))
	   (setf (elt robot-list 6) (send robot :r_forearm_link :copy-worldcoords))
	   (setf (elt robot-list 7) (send robot :r_upper_arm_link :copy-worldcoords))
	   (setf (elt robot-list 8) (send robot :r_shoulder_lift_link :copy-worldcoords)))
       (progn
	 (setf (elt robot-list 0) (send robot :l_gripper_palm_link :copy-worldcoords))
	 (setf (elt robot-list 1) (send robot :l_gripper_l_finger_link :copy-worldcoords))
	 (setf (elt robot-list 2) (send (send robot :l_gripper_r_finger_link :copy-worldcoords) :rotate pi :x))
	 (setf (elt robot-list 3) (send robot :l_gripper_l_finger_tip_link :copy-worldcoords))
	 (setf (elt robot-list 4) (send (send robot :l_gripper_r_finger_tip_link :copy-worldcoords) :rotate pi :x))
	 (setf (elt robot-list 5) (send robot :l_wrist_flex_link :copy-worldcoords))
	 (setf (elt robot-list 6) (send robot :l_forearm_link :copy-worldcoords))
	 (setf (elt robot-list 7) (send robot :l_upper_arm_link :copy-worldcoords))
	 (setf (elt robot-list 8) (send robot :l_shoulder_lift_link :copy-worldcoords))))
    robot-list))
  (:show-mesh-traj-with-color
   (arm-dae-list robot-list &key ((:lifetime lf) 20)
		 (ns "mesh_traj") ((:color col) #f(0.5 0.5 0.5)))
   (let ((msg (instance visualization_msgs::MarkerArray :init))
	 (header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
	 (l (length robot-list)) markers-list alpha-list)
     (dotimes (i l)
       (push (+ (/ (/ (1+ i) 2.0) l) 0.5) alpha-list))
     (dotimes (i l)
       (let (markers)
	 (mapcar #'(lambda(x y) (push (mesh->marker-msg x y header :mesh_use_embedded_materials nil :color col :alpha (elt alpha-list i)) markers))
		 (elt robot-list i) arm-dae-list)
	 (mapcar #'(lambda(x) (send x :lifetime (ros::time lf)) (send x :ns ns)) markers)
	 (push markers markers-list)))
     (setq markers-list (flatten markers-list))
     (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
     (send msg :markers markers-list)
     (ros::publish "openrave_marker_array" msg)
     ))
  )


;;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1200)) :move-target :rarm :use-torso t)
;;; (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) 5000 :move-target :rarm)
