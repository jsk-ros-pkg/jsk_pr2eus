#!/usr/bin/env roseus
(load "package://pr2eus_openrave/pr2eus-openrave.l")

(defun init-settings ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*)
      (setq *ri* (instance pr2-interface :init)))
  (objects (list *pr2*))
  (ros::subscribe "ray_coords" geometry_msgs::PoseStamped #'pose-cb)
  (setq *begin* (ros::time-now))
  )

(defun pose-cb (msg)
  (let ((box (make-cube 100 100 100))
	(htime (send msg :header :stamp))
	(p (ros::tf-pose->coords (send msg :pose))))
    (cond ((> 0 (send (ros::time- htime *begin*) :to-sec))
	   (return-fron pose-cb nil)))
    (ros::ros-info "subscribed~A" p)
    (send box :transform p)
    (objects (list *pr2* box))
    ;; (send *ri* :move-end-coords-plan
    ;; 	  (send (send p :copy-worldcoords) :translate #f(-10 0 10) :world)
    ;; 	  :move-target :rarm :use-torso t)
    (send *ri* :move-end-coords-plan
    	  (make-coords :pos (send (send (send p :copy-worldcoords) :translate #f(-10 0 10) :world) :pos)) :move-target :rarm :use-torso t)
    (send *ri* :wait-interpolation)
    ))

(init-settings)
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  )

