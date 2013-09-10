#!/usr/bin/env roseus
(ros::load-ros-manifest "topic_tools")
(ros::load-ros-manifest "pr2eus_openrave")
(ros::load-ros-manifest "orrosplanning")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2eus_openrave")

(defun remove-marker (id &key (ns "") (topic-name "marker_array"))
  (let*	((header (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "base_footprint"))
	 (msg (instance visualization_msgs::Marker :init :header header :ns ns))
	 (msgarray (instance visualization_msgs::MarkerArray :init)))
    (send msg :id id)
    (send msg :action visualization_msgs::Marker::*DELETE*)
    (send msgarray :markers (list msg))
    (ros::publish topic-name msgarray)
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
   (av &key (move-arm :larm) (use-torso t) (send-trajectory t)
       ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50) ((:speed-scale speed-scale) 1.0)
       &allow-other-keys)
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
     (send self :joint-trajectory-to-angle-vector-list move-arm joint-trajectory
           :diff-sum diff-sum :diff-thre diff-thre :speed-scale speed-scale
           :send-trajectory send-trajectory)
     ))
  (:move-end-coords-plan
   (coords &key (move-arm :larm) (use-torso t) ((:lifetime lf) 20) (send-trajectory t)
           ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50) ((:speed-scale speed-scale) 1.0)
           (manip-name (if (derivedp (*ri* . robot) pr2-robot)
                           (case move-arm
                             (:larm "l_gripper_tool_frame")
                             (:rarm "r_gripper_tool_frame")) ""))
           &allow-other-keys)
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
     (send self :joint-trajectory-to-angle-vector-list move-arm joint-trajectory
           :diff-sum diff-sum :diff-thre diff-thre :speed-scale speed-scale
           :send-trajectory send-trajectory)
     ))
  ;;
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
  ;;
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
  )


;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1200) :rpy (float-vector pi/2 0 0)) :move-arm :rarm :use-torso t) 
;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1000) :rpy (float-vector 0 0 pi/2)) :move-arm :rarm :use-torso t)
