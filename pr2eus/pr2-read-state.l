#!/usr/bin/env roseus
;;;;
;;;; read robot state from pr2 robot and print jacobian
;;;;

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_read_state")
(ros::advertise "/Left_endeffector_force" geometry_msgs::WrenchStamped 1)
(ros::advertise "/Right_endeffector_force" geometry_msgs::WrenchStamped 1)

(pr2)
(make-irtviewer)
(objects (list *pr2*))

(setq *ri* (instance pr2-interface :init))
(ros::rate 10)
(do-until-key
 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
 (when (boundp '*irtviewer*)
   (send *irtviewer* :draw-objects :flush nil)
   (send *pr2* :draw-torque *viewer* :torque-vector (send *ri* :state :torque-vector) :flush t))
 ;; larm jacobian
 (let ((target (send *pr2* :larm :end-coords)))
   (ros::ros-info "larm jacobian ~a"
     (format-array
      (send *pr2* :calc-jacobian-from-link-list
 	    (send *pr2* :link-list (send target :parent))
 	    :move-target target
 	    :rotation-axis t
 	    :translation-axis t)
      "" 7 3 nil)))

 ;; dual arm jacobian
 (with-move-target-link-list
  (mt ll *pr2* '(:rarm :larm))
  (ros::ros-info "dual-arm jacobian ~%~a"
    (format-array
     (send *pr2* :calc-jacobian-from-link-list
 	   ll :move-target mt
 	   :rotation-axis '(t t)
 	   :translation-axis '(t t))
     "" 7 3 nil)))

 ;; torque
 (ros::ros-info "torque ~%~a"
   (format-array
    (send *ri* :state :torque-vector)
    "" 7 3 nil))

 ;; fingertip pressure
 (let ((larm-val (send *ri* :finger-pressure :larm :zero nil))
       (rarm-val (send *ri* :finger-pressure :rarm :zero nil)))
   (when larm-val
   (ros::ros-info "larm-fingertip ~%l:~a~%r:~a"
 		  (car larm-val) (cadr larm-val)))
   (when rarm-val
   (ros::ros-info "rarm-fingertip ~%l:~a~%r:~a"
 		  (car rarm-val) (cadr rarm-val)))
   )

 ;;caculate forces applied to arms
 (dotimes (i 2)
   (let ((msg (instance geometry_msgs::WrenchStamped :init))
	 force target)
     (if (= i 0) (setq arm :larm)
       (setq arm :rarm))
     (setq target (send *pr2* arm :end-coords))     
     (setq force (scale -1 (send *pr2* :calc-force-from-joint-torque arm (send *ri* :state :torque-vector) :use-torso nil)))
     (print force)
     (send msg :header :stamp (send *ri* :state :stamp))
     (send msg :header :frame_id (send (send target :parent) :name))
     (send msg :wrench :force :x (elt force 0))
     (send msg :wrench :force :y (elt force 1))
     (send msg :wrench :force :z (elt force 2))
     (send msg :wrench :torque :x (elt force 3))
     (send msg :wrench :torque :y (elt force 4))
     (send msg :wrench :torque :z (elt force 5))
     (if (= i 0) (ros::publish "/Left_endeffector_force" msg)
       (ros::publish "/Right_endeffector_force" msg))
     ))

 (ros::spin-once)
 ;;(ros::sleep)
 (if (boundp '*irtviewer*)
   (x::window-main-one))
 )
