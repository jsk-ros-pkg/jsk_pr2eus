(load (format nil "~A/src/urdf2eus.l" (ros::rospack-find "eusurdf")))

(defclass pr2-robot
  :super urdf-robot
  :slots ())

(defmethod pr2-robot
  (:init (&rest args)
         (prog1 (send-super* :init args)
           (send self :reset-pose)))
  (:inverse-kinematics
   (&key (larm-target) (rarm-target)
         ;; *arm-target = {coordinates / :fix / :relative }
         (translation-axis t) (rotation-axis t) (debug-view :no-message)
         ((:loop max-iter) 100))
   (let* ((limbs (remove nil (list (if larm-target :larm) (if rarm-target :rarm))))
          (move-targets (mapcar #'(lambda (x) (send self x :end-coords)) limbs))
          (larm-end-coords (send self :larm :end-coords :copy-worldcoords))
          (rarm-end-coords (send self :rarm :end-coords :copy-worldcoords))
          (relative-coords (cond ((eq larm-target :relative)
                                  (send larm-end-coords :transformation rarm-target))
                                 ((eq rarm-target :relative)
                                  (send rarm-end-coords :transformation larm-target))))
          (link-lists (mapcar #'(lambda (x) (send self :link-list (send x :parent))) move-targets))
          (translation-axis (make-list (length limbs) :initial-element translation-axis))
          (rotation-axis
           (remove nil (list (cond ((derivedp larm-target coordinates) rotation-axis)
                                   ((or (eq larm-target :fix) (eq larm-target :relative)) t)
                                   (t nil))
                             (cond ((derivedp rarm-target coordinates) rotation-axis)
                                   ((or (eq rarm-target :fix) (eq rarm-target :relative)) t)
                                   (t nil)))))
	  (weight (instantiate float-vector (send self :calc-target-joint-dimension link-lists)))
          ret)
     (fill weight 1)
     (setf (elt weight 0) 0.1)
     (dotimes (i max-iter ret)
       (let* ((target-coords
               (let ((mid-end-coords))
                 (cond ((eq larm-target :fix)
                        (list larm-end-coords rarm-target))
                       ((eq rarm-target :fix)
                        (list larm-target rarm-end-coords))
                       ((eq larm-target :relative)
                        (setq mid-end-coords
                              (make-coords :pos (apply #'midpoint 0.5 (send self :arms :end-coords :worldpos))
                                           :rot (send larm-end-coords :worldrot)))
                        (let* ((larm-target
                                (send (send mid-end-coords :copy-worldcoords) :transform
                                      (make-coords :pos (scale 0.5 (v- (send relative-coords :worldpos)))
                                                   :rot (send larm-end-coords :worldrot))))
                               (rarm-target
                                (send (send larm-target :copy-worldcoords) :transform relative-coords)))
                          (list larm-target rarm-target)))
                       ((eq rarm-target :relative)
                        (list (send (send mid-end-coords :copy-worldcoords) :transform (send relative-coords :inverse-transformation))
                              (send (send mid-end-coords :copy-worldcoords) :transform relative-coords)))
                       (t
                        (remove nil (list larm-target rarm-target))))))
              (dif-pos
               (mapcar #'(lambda (ec tc)
                           (send ec :difference-position tc))
                       move-targets target-coords))
              (dif-rot
               (mapcar #'(lambda (ec tc)
                           (send ec :difference-rotation tc))
                       move-targets target-coords)))
	 
         (setq ret
               (send self :inverse-kinematics-loop
                     dif-pos dif-rot
                     :target-coords target-coords
                     :translation-axis translation-axis
                     :rotation-axis rotation-axis
                     :move-target move-targets :link-list link-lists
                     :debug-view debug-view
		     :weight weight
                     ))
         (if (eq ret :ik-succeed) (return))
         (let* ((link-lists (remove nil (list (if (eq larm-target :relative) (elt link-lists 0))
                                              (if (eq rarm-target :relative) (elt link-lists 1)))))
                (union-link-list (send self :calc-union-link-list link-lists))
                (union-joint-list (send-all union-link-list :joint))
                (joint-angle-limit-nspace (joint-angle-limit-nspace union-joint-list)))
           (mapcar #'(lambda (j a)
                       (send j :joint-angle (deg2rad a) :relative t))
                   union-joint-list (coerce joint-angle-limit-nspace cons)))
         ))
     (eq ret :ik-succeed)
     ))
  (:reset-pose
   ()
   (send self :head :angle-vector #f(0 0))
   (send self :larm :angle-vector #f(60 80 70 20 -120 -30 180))
   (send self :rarm :angle-vector #f(-60 80 -70 -20 -120 -30 180))
   (send self :torso :angle-vector #f(50))
   (send self :angle-vector)
   )
  (:reset-manip-pose
   ()
   (send self :head :angle-vector #f(0 50))
   (send self :rarm :angle-vector #f(-75 50 -110 20 -110 -10 -10))
   (send self :larm :angle-vector #f(75 50 110 -20 -110 -10 -10))
   (send self :torso :angle-vector #f(300))
   (send self :angle-vector)
   )
  (:middle-body-joint-angle-list
   ()
   (append (send-all (send self :larm) :joint)
	                     (send-all (send self :rarm) :joint)
                             (send-all (send self :torso) :joint)))
  (:full-body-joint-angle-list
   ()
   (append (send-all (send self :head) :joint)
		     (send-all (send self :larm) :joint)
		     (send-all (send self :rarm) :joint)
		     (send-all (send self :torso) :joint)))
  )
(defun pr2 ()
  (setq *pr2* (instance pr2-robot :init "full_pr2.urdf")))
