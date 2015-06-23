;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")

(defmethod pr2-robot
  (:select-target-arm
   (c)
   (cond ((and (consp c) (= 2 (length c))) ;; dual arm
	  (let ((v0 (send self :inverse-transform-vector (send (car c) :worldpos)))
		(v1 (send self :inverse-transform-vector (send (cadr c) :worldpos))))
	    (if (> (elt v0 1) (elt v1 1)) (list :larm :rarm) (list :rarm :larm))))
         ((consp c) ;; single arm
	  (let ((v (send self :inverse-transform-vector (send (car c) :worldpos))))
	    (if (> (elt v 1) 0) :larm :rarm)) ;; check y-coords
          )
	 (t         ;; single arm
	  (let ((v (send self :inverse-transform-vector (send c :worldpos))))
	    (if (> (elt v 1) 0) :larm :rarm)) ;; check y-coords
	  )))
  (:inverse-kinematics
   (target-coords &rest args &key (link-list) (move-arm)
                  (use-torso t) (move-target) (stop 300)
                  (use-base nil) (start-coords (send self :copy-worldcoords))
                  (thre (cond
                         ((atom target-coords) 10)
                         (t (make-list (length target-coords) :initial-element 10))))
                  (rthre (cond
                         ((atom target-coords) (deg2rad 5))
                         (t (make-list (length target-coords) :initial-element (deg2rad 5)))))
		  (base-range (list :min #f(-30 -30 -30)
				   :max #f( 30  30  30)))
                  (additional-weight-list)
                  &allow-other-keys)
   (let (diff-pos-rot)
     ;;
     ;; move-arm     x o x x o o x o ;; only in this function
     ;; link-list    x x o x o x o o ;; defined in upper class
     ;; move-target  x x x o x o o o ;; defined in upper class
     ;;              1 3 x x x 4 2 x
     ;;
     ;; 1:set move-arm from target-coords, set move-target from move-arm
     ;; 2;do nothing, since move-arm is used to set link-list and move-target
     ;; 3;set move-coords and link-list from move-arm
     ;; 4;set link-list from move-arm and move-target
     (unless move-arm
       (setq move-arm (send self :select-target-arm target-coords)))
     (unless move-target
       (if (consp move-arm)
	   (setq move-target (mapcar #'(lambda (arm) (send self arm :end-coords)) move-arm))
	 (setq move-target (send self move-arm :end-coords))))
     (unless link-list
       (setq link-list
             (if (consp target-coords) ;; dual arm
                 (mapcar #'(lambda (target)
                             ;; check if target's parent within limbs
                             (let ((l target) move-arm)
                               (while l
                                 (cond ((memq l (send self :larm))
                                        (setq move-arm :larm))
                                       ((memq l (send self :rarm))
                                        (setq move-arm :rarm)))
                                 (setq l (send l :parent)))
                               (send self :link-list (send target :parent)
                                     (unless use-torso (car (send self move-arm))))))
                             move-target)
               (send self :link-list (send move-target :parent)
                     (unless use-torso (car (send self move-arm)))))))

     ;; use base
     (cond
      (use-base
       (setq diff-pos-rot
             (concatenate float-vector
                          (send start-coords :difference-position self)
                          (send start-coords :difference-rotation self)))
       (send self :move-to start-coords :world)
       (with-append-root-joint
        (ll self link-list
            :joint-class omniwheel-joint
            :joint-args base-range)
        (send (caar ll) :joint :joint-angle
              (float-vector (elt diff-pos-rot 0)
                            (elt diff-pos-rot 1)
                            (rad2deg (elt diff-pos-rot 5))))
        (send-super* :inverse-kinematics target-coords
                     :rthre rthre
                     :thre thre
                     :stop stop
                     :additional-weight-list
                     (append
                      (list
                       (list (send self :torso_lift_joint :child-link)
                             (if (numberp use-torso) use-torso 0.005))
                       (list (car (send self :links))
                             (if (eq use-base t) 0.1 use-base))
                       )
                      additional-weight-list
                      )
                     :link-list (car ll) ;; link-list
                     :move-target move-target
                    args)))
      (t
       (send-super* :inverse-kinematics target-coords
                    :rthre rthre
                    :thre thre
                    :stop stop
                    :additional-weight-list
                    (list
                     (list (send self :torso_lift_joint :child-link)
                           (if (numberp use-torso) use-torso 0.005))
                     )
                    :link-list link-list
                    :move-target move-target
                    args))
       )))
  (:gripper
   (limb &rest args)
   (cond
    ((memq :links args)
     (case limb
       (:larm (list (send self :links :l_gripper_palm_link)
		    (send self :links :l_gripper_l_finger_link) (send self :links :l_gripper_r_finger_link)
		    (send self :links :l_gripper_l_finger_tip_link) (send self :links :l_gripper_r_finger_tip_link)))
       (:rarm
	(list (send self :links :r_gripper_palm_link)
	      (send self :links :r_gripper_l_finger_link) (send self :links :r_gripper_r_finger_link)
	      (send self :links :r_gripper_l_finger_tip_link) (send self :links :r_gripper_r_finger_tip_link)))))
    ((memq :joint-list args)
     (case limb
       (:larm (list (send self :l_gripper_l_finger_joint)
		    (send self :l_gripper_l_finger_tip_joint)
		    (send self :l_gripper_r_finger_joint)
		    (send self :l_gripper_r_finger_tip_joint)))
       (:rarm (list (send self :r_gripper_l_finger_joint)
		    (send self :r_gripper_l_finger_tip_joint)
		    (send self :r_gripper_r_finger_joint)
		    (send self :r_gripper_r_finger_tip_joint)))))
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
         (* a/2 2))))
    (t (send-super* :gripper limb args))
    ))
  (:init-ending ()
   ;; from pr2 manual(http://pr2support.willowgarage.com/wiki/PR2%20Manual/Chapter8#Forces_and_Torques)
   ;; set velocity and torque max
   (dolist (p '(;;(joint-name max-veloctity max-torque)
		(:torso_lift_joint 0.013 10000)
		(:laser_tilt_joint 10.00 0.65)
		(:head_pan_joint 6.00 2.65)
		(:head_tilt_joint 5.00 15.00)
		(:l_shoulder_pan_joint 2.10 30.00)
		(:r_shoulder_pan_joint 2.10 30.00)
		(:l_shoulder_lift_joint 2.10 30.00)
		(:r_shoulder_lift_joint 2.10 30.00)
		(:l_upper_arm_roll_joint 3.27 30.00)
		(:r_upper_arm_roll_joint 3.27 30.00)
		(:l_elbow_flex_joint 3.30 30.00)
		(:r_elbow_flex_joint 3.30 30.00)
		(:l_forearm_roll_joint 3.60 30.00)
		(:r_forearm_roll_joint 3.60 30.00)
		(:l_wrist_flex_joint 3.10 10.00)
		(:r_wrist_flex_joint 3.10 10.00)
		(:l_wrist_roll_joint 3.60 10.00)
		(:r_wrist_roll_joint 3.60 10.00)
		(:l_gripper_joint 0.20 1000)
		(:r_gripper_joint 0.20 1000)))
     (let ((j (elt p 0)) (v (elt p 1)) (r (elt p 2)))
       ;(warn "set ~A max-joint-velocity ~A(rad/s or m/s)~%" j v)
       ;(warn "set ~A max-joint-torque ~A(Nm or N)~%" j r)
       (send self j :max-joint-velocity v)
       (send self j :max-joint-torque r))
     ) ;; dolist
   (send-super :init-ending)) ;;
  )



