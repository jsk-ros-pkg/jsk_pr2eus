#!/usr/bin/env roseus
(ros::load-ros-manifest "topic_tools")
(ros::load-ros-manifest "pr2eus_openrave")
(ros::load-ros-manifest "orrosplanning")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2eus_openrave")

(ros::advertise "openrave_marker_array" visualization_msgs::MarkerArray 100)


(defun remove-marker (id)
  (let*	((header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "base_footprint")) (ns "robot_traj")
	 (msg (instance visualization_msgs::Marker :init :header header :ns ns))
	 (msgarray (instance visualization_msgs::MarkerArray :init)))
    (send msg :id id)
    (send msg :action visualization_msgs::Marker::*DELETE*)
    (send msgarray :markers (list msg))
    (ros::publish "openrave_marker_array" msgarray)
    ))

(defmethod robot-interface
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
   (av &key (move-arm :larm) (use-torso t)
       ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50) ((:speed-scale speed-scale) 1.0))
   (let (coords joint-trajectory (av-rad av))
     (send robot :angle-vector av)
     (setq coords (send robot move-arm :end-coords :copy-worldcoords))
     (send self :show-goal-hand-coords (send coords :copy-worldcoords) move-arm)
     (dotimes (i (length av-rad)) (setf (elt av-rad i) (deg2rad (elt av-rad i))))
     (setq joint-trajectory
	   (send self :call-openrave-move-manipulator
		 av-rad
		 (format nil "~A~A" (case move-arm (:larm "leftarm") (:rarm "rightarm")) (if use-torso "_torso" ""))
		 ))
     (send self :joint-trajectory-to-angle-vector-list move-arm joint-trajectory :diff-sum diff-sum :diff-thre diff-thre :speed-scale speed-scale)
     ))
  (:move-end-coords-plan
   (coords &key (move-arm :larm) (use-torso t) ((:lifetime lf) 20) (send-trajectory t)
	   ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50) ((:speed-scale speed-scale) 1.0)
	   (manip-name (if (derivedp (*ri* . robot) pr2-robot) (case move-arm (:larm "l_gripper_tool_frame") (:rarm "r_gripper_tool_frame")) "")))
   ;; for fast result view
   (send self :show-goal-hand-coords (send coords :copy-worldcoords) move-arm)

   (let* (joint-trajectory)
     (setq joint-trajectory
	   (send self :call-openrave-move-to-hand-position
		 coords
		 (format nil "~A~A" (case move-arm (:larm "leftarm") (:rarm "rightarm")) (if use-torso "_torso" ""))
		 manip-name
;;		 (case move-arm (:larm "l_gripper_tool_frame") (:rarm "r_gripper_tool_frame"))
		 ))
     (send self :joint-trajectory-to-angle-vector-list move-arm joint-trajectory :diff-sum diff-sum :diff-thre diff-thre :speed-scale speed-scale :send-trajectory send-trajectory)
     ))
  (:joint-trajectory-to-angle-vector-list
   (move-arm joint-trajectory
             &key ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50)
             (show-trajectory t) (send-trajectory t)
             ((:speed-scale speed-scale) 1.0))
   (let* (joint-trajectory-points
          joints avs tms  (tm (ros::time -1)) link-coords-list
          (org-av (send robot :angle-vector))
          (dae-link-list
           (send self :find-descendants-dae-links (send robot move-arm :root-link))))
     ;; JointTrajectory to angle-vector
     (if joint-trajectory
         (setq joint-trajectory-points (send joint-trajectory :points)
               joints (mapcar #'(lambda (x)
                                  (send robot (intern (string-upcase x) *keyword-package*)))
                              (send joint-trajectory :joint_names))))
     ;;
     (if (= (length joint-trajectory-points) 0)
         (return-from :joint-trajectory-to-angle-vector-list nil))
     (dolist (point joint-trajectory-points)
       (mapc '(lambda (ajoint aposition)
                (send ajoint :ros-joint-angle aposition))
             joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (when (cadr avs)
         (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
         (when (> diff-sum diff-thre)
           (setq diff-sum 0)
           (when show-trajectory
             (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
                   link-coords-list))))
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000 speed-scale) tms)
       (setq tm (send point :time_from_start))
       )
     (when show-trajectory
       (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
             link-coords-list)
       ;; send visualization_msgs to rviz
       ;; (send self :show-mesh-traj-with-color (send-all (flatten (send-all link-list :bodies)) :name) link-coords-list)
       (send self :show-mesh-traj-with-color
             (flatten (send-all dae-link-list :bodies))
             link-coords-list :lifetime (+ (/ (apply #'+ tms) 1000.0) 10)))
     (cond
      (send-trajectory
       (send robot :angle-vector (car (last avs)))
       ;; send to *ri*
       (send self :angle-vector-sequence (setq avs (reverse avs)) (setq tms (reverse tms))))
      (t
       (send robot :angle-vector org-av)
       ))
     (if joint-trajectory (list (reverse avs) (reverse tms)))
     ))
  (:call-openrave-move-to-hand-position
   (coords manip-name hand-frame-id)
   (let ((req (instance orrosplanning::MoveToHandPositionRequest :init))
	 (hand-goal (ros::coords->tf-pose-stamped coords "base_footprint"))
	 res (mux-req (instance topic_tools::MuxSelectRequest :init))
	 )
     (ros::ros-info "pos ~A ~A ~A"
		    (send hand-goal :pose :position :x)
		    (send hand-goal :pose :position :y)
		    (send hand-goal :pose :position :z))
     (ros::ros-info "ori ~A ~A ~A ~A"
		    (send hand-goal :pose :orientation :x)
		    (send hand-goal :pose :orientation :y)
		    (send hand-goal :pose :orientation :z)
		    (send hand-goal :pose :orientation :w))
     (send req :manip_name manip-name)
     (send req :hand_goal hand-goal)
     (send req :hand_frame_id hand-frame-id)

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

  (:call-openrave-move-manipulator
   (av manip-name)
   (let ((req (instance orrosplanning::MoveManipulatorRequest :init)) res)
     (ros::ros-info "angle-vector ~A" av)
     (ros::ros-info "manip-name   ~A" manip-name)
     (send req :manip_name manip-name)
     (send req :manip_goal av)
     (send self :service-call-joint-state)
     (if (and
	  (ros::service-exists "MoveManipulator")
	  (ros::wait-for-service "MoveManipulator" 30))
	 (setq res (ros::service-call "MoveManipulator" req))
       (ros::ros-warn "MoveManipulator did not respond"))
     (if res (send res :traj))))

  (:show-goal-hand-coords
   (coords move-arm)
   (let* ((gripper-bodies
	   (flatten (send-all (send robot move-arm :gripper :links) :bodies)))
	  (gripper-coords
	   (mapcar #'(lambda (gripper-link)
		       (send (send coords :copy-worldcoords) :transform
			     (send (send robot move-arm :end-coords) :transformation gripper-link)
			     :local))
		   gripper-bodies)))
     (send self :show-mesh-traj-with-color
     	   gripper-bodies (list gripper-coords)
     	   :lifetime 0 :color #f(1 0 1) :ns "hand_traj")
     ))
  (:find-descendants-dae-links
   (l)
   (unless l (return-from :find-descendants-dae-links nil))
   (append (list l)
           (mapcan #'(lambda (x) (send self :find-descendants-dae-links x)) (send l :child-links))))
  (:show-mesh-traj-with-color
   (link-body-list link-coords-list &key ((:lifetime lf) 20)
                   (ns "robot_traj") ((:color col) #f(0.5 0.5 0.5)))
   (let ((msg (instance visualization_msgs::MarkerArray :init))
         (header (instance std_msgs::header :init
                           :stamp (ros::time-now)
                           :frame_id (send (car (send robot :links)) :name)))
         (l (length link-coords-list)) markers-list alpha-list
         (base-cds (send (car (send robot :links)) :copy-worldcoords)))
     (setq base-cds (send base-cds :inverse-transformation))
     (dotimes (i l)
       (push (+ (/ (/ (1+ i) 2.0) l) 0.5) alpha-list))
     (dotimes (i l)
       (let (mrk markers)
         (mapcar #'(lambda (abody acoords)
                     (setq acoords (send (send acoords :copy-worldcoords) :transform base-cds :world))
		     (cond ((send abody :name)
			    (setq mrk (mesh->marker-msg
				       acoords
				       (send abody :name)
				       header
				       :mesh_use_embedded_materials nil :color col :alpha (elt alpha-list i))))
			   (t
			    (setq mrk (object->marker-msg
				       abody
				       header
				       :coords acoords
				       :color col
				       :alpha (elt alpha-list i)
				       ))))
		     (send mrk :lifetime (ros::time lf))
		     (send mrk :ns ns)
		     (push mrk markers))
		 link-body-list
		 (elt link-coords-list i))
	 (push markers markers-list)))
     (setq markers-list (flatten markers-list))
     (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
     (send msg :markers markers-list)
     (ros::publish "openrave_marker_array" msg)
     ))
  )


;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1200) :rpy (float-vector pi/2 0 0)) :move-arm :rarm :use-torso t) 
;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1000) :rpy (float-vector 0 0 pi/2)) :move-arm :rarm :use-torso t)
