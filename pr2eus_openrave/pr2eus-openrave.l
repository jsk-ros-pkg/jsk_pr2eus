#!/usr/bin/env roseus
(ros::load-ros-manifest "pr2eus_openrave")
(ros::load-ros-manifest "orrosplanning")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2eus_openrave")

(ros::advertise "openrave_marker_array" visualization_msgs::MarkerArray 100)

;; We set dae-list below because package://pr2eus/pr2.l do not include collada file path, waiting for its support. This is very temporary.

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
   (coords &key (move-arm :larm) (use-torso t))
   (let (joint-trajectory joint-trajectory-points joints avs tms (tm (ros::time -1)) link-coords-list (diff-sum 0) (diff-thre 100)
			  (link-list (send robot :link-list (send robot move-arm :end-coords :parent) (car (send robot move-arm)))))
     ;; call openrave
     (send self :show-goal-hand-coords coords move-arm)

     (setq joint-trajectory
	   (send self :call-openrave-move-to-hand-position
		 coords
		 (format nil "~A~A" (case move-arm (:larm "leftarm") (:rarm "rightarm")) (if use-torso "_torso" ""))
		 (case move-arm (:larm "l_gripper_tool_frame") (:rarm "r_gripper_tool_frame"))))
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
       ;;
       (when (cadr avs)
	 (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
	 (if (> diff-sum diff-thre)
	     (progn (setq diff-sum 0) (push (send-all link-list :copy-worldcoords) link-coords-list))))
       ;;
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000) tms)
       (setq tm (send point :time_from_start))
       )
     (push (send-all link-list :copy-worldcoords) link-coords-list)
     ;; send visualization_msgs to rviz
     (send self :show-mesh-traj-with-color (send-all (flatten (send-all link-list :bodies)) :name) link-coords-list)

     (send robot :angle-vector (car (last avs)))
     ;; send to ri
     (send self :angle-vector-sequence (nreverse avs) (nreverse tms))

     (if joint-trajectory
	 (list avs tms) nil)
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

  (:show-goal-hand-coords
   (coords move-arm)
   (let ()
     (send self :show-mesh-traj-with-color
	   (send-all (flatten (send-all (send robot :gripper move-arm :links) :bodies)) :name)
	   (send-all (send-all  (send robot :gripper move-arm :links) :copy-worldcoords) :transform (send robot move-arm :end-coords :transformation coords) :local)
	   :lifetime 0 :color #f(1 0 0))
     ))

  (:show-mesh-traj-with-color
   (link-dae-list link-coords-list &key ((:lifetime lf) 20)
		 (ns "mesh_traj") ((:color col) #f(0.5 0.5 0.5)))
   (print link-dae-list)
   (print link-coords-list)
   (let ((msg (instance visualization_msgs::MarkerArray :init))
	 (header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
	 (l (length link-coords-list)) markers-list alpha-list)
     (dotimes (i l)
       (push (+ (/ (/ (1+ i) 2.0) l) 0.5) alpha-list))
     (dotimes (i l)
       (let (markers)
	 (mapcar #'(lambda(x y) (push (mesh->marker-msg x y header :mesh_use_embedded_materials nil :color col :alpha (elt alpha-list i)) markers))
		 (elt link-coords-list i) link-dae-list)
	 (print markers)
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
