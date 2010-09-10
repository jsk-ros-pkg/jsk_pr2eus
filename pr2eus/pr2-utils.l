;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")

(defun load-pr2-file ()
  (let ((robot "pr2") s
        fname_urdf fname_fix_urdf fname_collada fname_yaml fname_lisp)
    (unless (and (ros::is-initialized)
                 (setq s (ros::get-param "/robot_description")))
      (warning-message 2 "load model file from pr2eus/pr2.l~%")
      (return-from load-pr2-file (require :pr2 "package://pr2eus/pr2.l")))

    (setq fname_urdf (format nil "/tmp/~a_~d.urdf" robot (unix::getpid)))
    (setq fname_fix_urdf (format nil "/tmp/~a_~d_fix.urdf" robot (unix::getpid)))
    (setq fname_collada (format nil "/tmp/~a_~d.dae" robot (unix::getpid)))
    (setq fname_yaml (ros::resolve-ros-path (format nil "package://euscollada/~A.yaml" robot)))
    (setq fname_lisp (format nil "/tmp/~a_~d.l" robot (unix::getpid)))

    (unless (probe-file fname_lisp)
      (with-open-file
       (f fname_urdf :direction :output)
       (format f s))

      (unix::system (format nil "sed s/\.dae/\.stl/ < ~A > ~A" fname_urdf fname_fix_urdf))
      (unix::system (format nil "rosrun collada_urdf_jsk_patch urdf_to_collada ~A ~A" fname_fix_urdf fname_collada))
      (unix::system (format nil "rosrun euscollada collada2eus ~A ~A ~A" fname_collada fname_yaml fname_lisp))

      (warning-message 2 "load model file from parameter server /robot_description")
      (load fname_lisp)) ;; force read file when created
    (require :pr2 fname_lisp)))

(defmethod pr2-robot
  (:inverse-kinematics
   (target-coords &rest args &key (link-list) &allow-other-keys)
   (let* ((weight (instantiate float-vector (send self :calc-target-joint-dimension link-list)))
	  (union-link-list (send self :calc-union-link-list link-list))
	  (torso-id (position torso_lift_joint (send-all union-link-list :joint))))
     (fill weight 1)
     (if torso-id (setf (elt weight torso-id) 0.1))
     (send-super* :inverse-kinematics target-coords
		  :rthre (deg2rad 5)
		  :thre 10
		  :stop 300
		  :weight weight
		  :link-list link-list
		  args)))
  (:limb
   (limb method &rest args)
   (case method
     (:gripper
      (send* self :gripper limb args))
     (t
      (send-super* :limb limb method args))))
  (:gripper
   (limb &rest args)
   (cond
    ((memq :joint-angle args)
     (if (null (cdr args))
         (case limb
           (:larm
            (* (send self :l_gripper_l_finger_joint :joint-angle) 2))
           (:rarm
            (* (send self :r_gripper_l_finger_joint :joint-angle) 2)))
       (let ((a/2 (/ (cadr args) 2)))
         (case limb
           (:larm
            (send self :l_gripper_l_finger_joint :joint-angle a/2)
            (send self :l_gripper_r_finger_joint :joint-angle a/2)
            (send self :l_gripper_l_finger_tip_joint :joint-angle a/2)
            (send self :l_gripper_r_finger_tip_joint :joint-angle a/2))
           (:rarm
            (send self :r_gripper_l_finger_joint :joint-angle a/2)
            (send self :r_gripper_r_finger_joint :joint-angle a/2)
            (send self :r_gripper_l_finger_tip_joint :joint-angle a/2)
            (send self :r_gripper_r_finger_tip_joint :joint-angle a/2)))
         (* a/2 2))))))
  )



