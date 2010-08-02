;;
;;
;;
(require :pr2 "package://euscollada/pr2.l")

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
  )


