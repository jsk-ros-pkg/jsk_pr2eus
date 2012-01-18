#!/usr/bin/env roseus
(load "package://pr2eus_openrave/pr2eus-openrave.l")
(setq *simulation* nil)
(setq *exit-on-fail* nil)

(defun init-settings ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*)
      (setq *ri* (instance pr2-interface :init)))
  (unless *simulation*
      (objects (list *pr2*)))
  (ros::subscribe "ray_coords" geometry_msgs::PoseStamped #'pose-cb)
  (setq *begin* (ros::time-now))
  )

(defun pose-cb (msg)
  (let ((box (make-cube 100 50 50))
	(htime (send msg :header :stamp))
	(p (ros::tf-pose->coords (send msg :pose)))
	ret)
    (cond ((> 0 (send (ros::time- htime *begin*) :to-sec))
	   (return-from pose-cb nil)))
    (if *simulation*
	(send *ri* :publish-joint-state))
    (ros::ros-info "subscribed~A" p)
    (send box :transform p)
    (unless *simulation*
      (objects (list *pr2* box)))
    (setq ret (send *ri* :move-end-coords-plan
		    (send (send p :copy-worldcoords) :translate #f(-10 0 0) :local)
		    :move-arm :rarm :use-torso t))
    (ros::ros-info "move-end-coords-plan -> ~A~%" ret)
    (if (and *exit-on-fail* (null ret)) (exit 1))
    (send *ri* :wait-interpolation)
    ))

(defun execute-main ()
  (init-settings)
  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (if (and x::*display* (> x::*display* 0)) (x::window-main-one))
    (ros::sleep)
    ))

