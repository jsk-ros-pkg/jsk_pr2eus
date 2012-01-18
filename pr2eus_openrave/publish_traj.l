#!/usr/bin/env roseus
(load "package://pr2eus_openrave/pr2eus-openrave.l")

(defun init-settings ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (objects (list *pr2*))
  (ros::advertise "openrave_marker_array" visualization_msgs::MarkerArray 100)
  (setq *arm-dae-list* ;; this is not good at all
	(list
	 "package://pr2_description/meshes/gripper_v0/gripper_palm.dae"
	 "package://pr2_description/meshes/gripper_v0/l_finger.dae"
	 "package://pr2_description/meshes/gripper_v0/l_finger.dae"
	 "package://pr2_description/meshes/forearm_v0/wrist_flex.dae"
	 "package://pr2_description/meshes/forearm_v0/forearm.dae"
	 "package://pr2_description/meshes/upper_arm_v0/upper_arm.dae"
	 "package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae"))
  (setq *point-list* nil *line-list* nil *robot-list* nil))

(defun record-arm (&key (arm :rarm))
  (setq *point-list* nil *line-list* nil *robot-list* nil)
  (ros::ros-info "start recording... (push any key to stop)")
  (do-until-key
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (push (GetPointList (send *pr2* arm :end-coords)) *point-list*)
   (push (GetLineList (send *pr2* arm :end-coords)) *line-list*)
   (push (GetRobotList :arm arm) *robot-list*)
   (send *irtviewer* :draw-objects) (x::window-main-one)
   (unix::usleep (* 1000 1500)) (warn "."))
  (nreverse *point-list*) (nreverse *line-list*) (nreverse *robot-list*)
  (format t "
(show-lines-lists-with-color-and-spheres *line-list* *point-list*)
(show-mesh-traj *arm-dae-list* *robot-list*)
(show-mesh-traj-with-color *arm-dae-list* *robot-list*)
(show-mesh-traj-with-color-with-line)
"))

(defun GetPointList (arg)
  (let ((child (copy-object arg)) point-list)
    (while (send (send (send (send child :parent) :parent) :parent) :parent)
      (push (send (send child :copy-worldcoords) :worldpos) point-list)
      (setq child (send child :parent)))
    point-list))

(defun GetLineList (arg)
  (let ((child (copy-object arg)) pair-list)
    (while (send (send (send (send child :parent) :parent) :parent) :parent)
      (push (list (send (send child :copy-worldcoords) :worldpos)
		  (send (send (send child :parent) :copy-worldcoords) :worldpos)) pair-list)
      (setq child (send child :parent)))
    pair-list))

(defun GetRobotList (&key (arm :rarm)) ;; very temporary
  (let ((robot-list (make-list (length *arm-dae-list*))))
    (if (eq arm :rarm)
	(progn
	  (setf (elt robot-list 0) (send *pr2* :r_gripper_palm_link :copy-worldcoords))
	  (setf (elt robot-list 1) (send *pr2* :r_gripper_r_finger_link :copy-worldcoords))
	  (setf (elt robot-list 2) (send (send *pr2* :r_gripper_l_finger_link :copy-worldcoords) :rotate pi :x))
	  (setf (elt robot-list 3) (send *pr2* :r_wrist_flex_link :copy-worldcoords))
	  (setf (elt robot-list 4) (send *pr2* :r_forearm_link :copy-worldcoords))
	  (setf (elt robot-list 5) (send *pr2* :r_upper_arm_link :copy-worldcoords))
	  (setf (elt robot-list 6) (send *pr2* :r_shoulder_lift_link :copy-worldcoords)))
	(progn
	  (setf (elt robot-list 0) (send *pr2* :l_gripper_palm_link :copy-worldcoords))
	  (setf (elt robot-list 1) (send *pr2* :l_gripper_r_finger_link :copy-worldcoords))
	  (setf (elt robot-list 2) (send (send *pr2* :l_gripper_l_finger_link :copy-worldcoords) :rotate pi :x))
	  (setf (elt robot-list 3) (send *pr2* :l_wrist_flex_link :copy-worldcoords))
	  (setf (elt robot-list 4) (send *pr2* :l_forearm_link :copy-worldcoords))
	  (setf (elt robot-list 5) (send *pr2* :l_upper_arm_link :copy-worldcoords))
	  (setf (elt robot-list 6) (send *pr2* :l_shoulder_lift_link :copy-worldcoords))))
    robot-list))

(defun show-sphere-sample
  (p &key ((:lifetime lf) 20) (ns "sphere")
     ((:color col) #f(1 0 0)) ((:scale sc) 15) (alpha 0.8))
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
         (sp-msg (sphere->marker-msg (make-sphere sc :pos p) header
				     :color col :alpha alpha))
	 (msg (instance visualization_msgs::MarkerArray :init)))
    (send sp-msg :ns ns)
    (send sp-msg :lifetime (ros::time lf))
    (send msg :markers (list sp-msg))
    (ros::publish "openrave_marker_array" msg)
    ))

(defun show-line-sample
  (p1 p2 &key ((:lifetime lf) 20) (ns "line")
      ((:color col) #f(1 0 0)) ((:scale sc) 15) (alpha 0.8))
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
         (li-msg (line->marker-msg (list p1 p2) header
				   :scale sc :color col :alpha alpha))
	 (msg (instance visualization_msgs::MarkerArray :init)))
    (send li-msg :ns ns)
    (send li-msg :lifetime (ros::time lf))
    (send msg :markers (list li-msg))
    (ros::publish "openrave_marker_array" msg)
    ))


(defun show-lines-lists-with-color-and-spheres
  (point-list sphere-list &key ((:lifetime lf) 10) (ns "lines_and_spheres")
	      ((:color col) #f(0.5 0.5 1)) ((:scale sc) 20) (alpha 0.5))
  (let ((msg (instance visualization_msgs::MarkerArray :init))
	(header (instance std_msgs::header :init
			  :stamp (ros::time-now) :frame_id "base_footprint"))
	(l (length point-list)) markers-list color-list alpha-list)
    ;; lines
    (dotimes (i l)
      ;;      (push (+ (/ (/ i 2.0) l) 0.5) alpha-list)
      (push (/ (/ i 2.0) l) alpha-list)
      (if (< i (/ l 2.0)) ;; graduation from blue to red
	  (push (float-vector (* (/ (* i 1.0) l) 2.0) 0 1) color-list)
	(push (float-vector 1 0 (* (/ (- l (* i 1.0)) l) 2.0)) color-list)))
    (nreverse color-list) (nreverse alpha-list)
    (dotimes (i l)
      (let (markers)
	(mapcar #'(lambda(x) (push (line->marker-msg (list (elt x 0) (elt x 1)) header :scale sc :color (elt color-list i) :alpha (elt alpha-list i)) markers)) (elt point-list i))
	(mapcar #'(lambda(x) (send x :lifetime (ros::time lf)) (send x :ns ns)) markers)
	(push markers markers-list)))
    ;; spheres
    (when sphere-list
      (dolist (i sphere-list)
	(let (markers)
	  (mapcar #'(lambda(x) (push (sphere->marker-msg (make-sphere sc :pos x) header :color col :alpha alpha) markers)) i)
	  (mapcar #'(lambda(x) (send x :lifetime (ros::time lf)) (send x :ns ns)) markers)
	  (push markers markers-list))))
    (setq markers-list (flatten markers-list))
    (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
    (send msg :markers markers-list)
    (ros::publish "openrave_marker_array" msg)
    ))

(defun show-mesh-sample
  (mesh_resource cds &key ((:lifetime lf) 20) (ns "mesh"))
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_footprint"))
         (ro-msg (mesh->marker-msg cds mesh_resource header))
	 (msg (instance visualization_msgs::MarkerArray :init)))
    (send ro-msg :ns ns)
    (send ro-msg :lifetime (ros::time lf))
    (send msg :markers (list ro-msg))
    (ros::publish "openrave_marker_array" msg)
    ))

(defun show-mesh-traj
  (arm-dae-list robot-list &key ((:lifetime lf) 20) (ns "mesh_traj"))
  (let ((msg (instance visualization_msgs::MarkerArray :init))
	(header (instance std_msgs::header :init
			  :stamp (ros::time-now) :frame_id "base_footprint"))
	markers-list)
    (dolist (l robot-list)
      (let (markers)
	(mapcar #'(lambda(x y) (push (mesh->marker-msg x y header) markers)) l arm-dae-list)
	(mapcar #'(lambda(x) (send x :lifetime (ros::time lf)) (send x :ns ns)) markers)
	(push markers markers-list)
	))
    (setq markers-list (flatten markers-list))
    (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
    (send msg :markers markers-list)
    (ros::publish "openrave_marker_array" msg)
    ))

(defun show-mesh-traj-with-color
  (arm-dae-list robot-list &key ((:lifetime lf) 10)
		(ns "mesh_traj") ((:color col) (float-vector 0.1 0.1 0.1)))
  (let ((msg (instance visualization_msgs::MarkerArray :init))
	(header (instance std_msgs::header :init
			  :stamp (ros::time-now) :frame_id "base_footprint"))
	(l (length robot-list)) markers-list alpha-list)
    (dotimes (i l)
      (push (/ (/ i 1.5) l) alpha-list))
    ;;      (push (+ (/ (/ i 2.0) l) 0.5) alpha-list))
    (nreverse alpha-list)
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

(defun mesh->marker-msg
  (cds mesh_resource header &key
       ((:color col) (float-vector 1 1 1)) ((:scale sc) 1000)
       ((:id idx) 0) ((:mesh_use_embedded_materials mesh_color) t)
       (alpha 1.0) ns lifetime)
    (let* ((msg (instance visualization_msgs::Marker :init
			  :type visualization_msgs::Marker::*MESH_RESOURCE*
			  :header header :id idx)))
      (send msg :mesh_resource mesh_resource)
      (send msg :mesh_use_embedded_materials mesh_color)
      (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
      (send msg :color (vector->rgba col alpha))
      (send msg :frame_locked t)
      (send msg :pose (ros::coords->tf-pose cds))
      (if ns (send msg :ns ns))
      (if lifetime (send msg :lifetime (ros::time lifetime)))
      msg))


(defun show-mesh-traj-with-color-with-line ()
  (show-lines-lists-with-color-and-spheres *line-list* nil)
  (show-mesh-traj-with-color *arm-dae-list* *robot-list*)
  )

(format t "
(init-settings)
(record-arm)
(show-mesh-traj-with-color-with-line)
")

(defun test1 ()
  (setq *a* (copy-object *line-list*))
  (setq *b* (copy-object *robot-list*))
  )

(defun test2 ()
  (setq *c* (copy-object *line-list*))
  (setq *d* (copy-object *robot-list*))
  )

(defun test3 ()
  (show-lines-lists-with-color-and-spheres *a* nil)
  (show-mesh-traj-with-color *arm-dae-list* *b*)
  )

(defun test4 ()
  (show-lines-lists-with-color-and-spheres *c* nil)
  (show-mesh-traj-with-color *arm-dae-list* *d*)
  )
