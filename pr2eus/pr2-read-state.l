#!/usr/bin/env runeus
;;;;
;;;; read robot state from pr2 robot and print jacobian
;;;;

(pushnew (format nil "~A/euslisp/"
                 (read-line (setq *strm* (piped-fork "rospack find roseus"))))
         *load-path* :test #'equal)

(load "roseus.l")
(load (format nil "~A/pr2-interface.l" (ros::rospack-find "pr2eus")))
(roseus "pr2_read_state")

(pr2)
(make-irtviewer)
(objects (list *pr2*))

(setq *ri* (instance pr2-interface :init))
(ros::rate 10)
(do-until-key
 (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
 (send *irtviewer* :draw-objects)
 (let ((target (send *pr2* :larm :end-coords)))
   (ros::ros-info "larm jacobian ~a"
		  (send *pr2* :calc-jacobian-from-link-list
			(send *pr2* :link-list (send target :parent))
			:move-target target
			:rotation-axis t
			:translation-axis t)))
 (ros::spin-once)
 (ros::sleep))
