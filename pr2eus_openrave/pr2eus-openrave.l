#!/usr/bin/env roseus
(ros::load-ros-manifest "pr2eus_openrave")
(ros::load-ros-manifest "orrosplanning")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2eus_openrave")

(defmethod pr2-interface
  (:angle-vector-motion-plan
   (av &key (move-target :larm) (use-torso t))
   (let (coords mname)
     (send robot :angle-vector av)
     (setq coords (send robot move-target :end-coords :copy-worldcoords))
     (send self :move-end-coords-plan coords :move-target move-target :use-torso use-torso)))
  (:move-end-coords-plan
   (coords &key (move-target :larm) (use-torso t))
   (let (joint-trajectory joint-trajectory-points joints avs tms (tm (ros::time -1)))
     ;; call openrave
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
     (dolist (point joint-trajectory-points)
       (mapcar '(lambda (ajoint aposition)
		  (cond
		   ((derivedp ajoint rotational-joint)
		    (send ajoint :joint-angle (rad2deg aposition)))
		   (t
		    (send ajoint :joint-angle (* 1000 aposition)))))
	       joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000) tms)
       (setq tm (send point :time_from_start))
       )
     ;; send to ri
     (send self :angle-vector-sequence (nreverse avs) (nreverse tms))
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
	 res)
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
     (if (and
	  (ros::service-exists "MoveToHandPosition")
	  (ros::wait-for-service "MoveToHandPosition" 30))
	 (setq res (ros::service-call "MoveToHandPosition" req))
       (ros::ros-warn "MoveToHandPosition did not respond"))
     (if res (send res :traj))))
  )


;;; (send *ri* :move-end-coords-plan (make-coords :pos #f(600 -100 1200)) :move-target :rarm :use-torso t)
;;; (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) 5000 :move-target :rarm)
