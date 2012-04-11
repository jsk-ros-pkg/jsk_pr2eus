;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")

(defmethod pr2-robot
  (:select-target-arm
   (c)
   (let ((v (send self :inverse-transform-vector (send c :worldpos))))
     (if (> (elt v 1) 0) :larm :rarm)))
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
                  &allow-other-keys)
   (let (weight union-link-list torso-id base-weight diff-pos-rot)
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
     (unless (or move-arm (consp target-coords)) ;; move-arm cannot be used if target-coords is cons
       (setq move-arm (send self :select-target-arm target-coords)))
     (unless move-target
       (setq move-target (send self move-arm :end-coords)))
     (unless link-list
       (setq link-list (send self :link-list (send move-target :parent)
                             (unless use-torso (car (send self move-arm))))))

     ;; use torso
     (setq weight (instantiate float-vector (send self :calc-target-joint-dimension link-list))
           union-link-list (send self :calc-union-link-list link-list)
           torso-id (position (send self :torso_lift_joint) (send-all union-link-list :joint)))
     (fill weight 1)
     (when (and use-torso torso-id)
       (if (not (numberp use-torso)) (setq use-torso 0.005))
       (if torso-id (setf (elt weight torso-id) use-torso)))

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
            :joint-args
            (list :min #f(-30 -30 -30)
                  :max #f( 30  30  30)))
        (send (caar ll) :joint :joint-angle
              (float-vector (elt diff-pos-rot 0)
                            (elt diff-pos-rot 1)
                            (rad2deg (elt diff-pos-rot 5))))
        (if (eq use-base t) (setq use-base 0.1))
        (send-super* :inverse-kinematics target-coords
                     :rthre rthre
                     :thre thre
                     :stop stop
                     :weight (concatenate float-vector (float-vector use-base use-base use-base) weight)
                     :link-list (car ll) ;; link-list
                     :move-target move-target
                    args)))
      (t
       (send-super* :inverse-kinematics target-coords
                    :rthre rthre
                    :thre thre
                    :stop stop
                    :weight weight
                    :link-list link-list
                    :move-target move-target
                    args))
       )))
  (:gripper
   (limb &rest args)
   (cond
    ((memq :links args)
     (case limb
       (:larm (list (send self :l_gripper_palm_link)
		    (send self :l_gripper_l_finger_link) (send self :l_gripper_r_finger_link)
		    (send self :l_gripper_l_finger_tip_link) (send self :l_gripper_r_finger_tip_link)))
       (:rarm
	(list (send self :r_gripper_palm_link)
	      (send self :r_gripper_l_finger_link) (send self :r_gripper_r_finger_link)
	      (send self :r_gripper_l_finger_tip_link) (send self :r_gripper_r_finger_tip_link)))))
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



