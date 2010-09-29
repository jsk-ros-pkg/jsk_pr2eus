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
   (target-coords &rest args &key (link-list) (move-target) &allow-other-keys)
   (unless move-target
     (setq move-target (send self (send self :select-target-arm target-coords) :end-coords)))
   (unless link-list
     (setq link-list (send self :link-list (send move-target :parent))))
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
		  :move-target move-target
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



