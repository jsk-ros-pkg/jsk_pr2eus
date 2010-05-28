(load (format nil "~A/src/urdf2eus.l" (read (piped-fork "rospack find eusurdf"))))

(defclass pr2-robot
  :super urdf-robot
  :slots ())

(defmethod pr2-robot
  (:init (&rest args)
         (prog1 (send-super* :init args)
           (send self :reset-pose)))
  (:inverse-kinematics
   (&key (larm-target) (rarm-target)
         (translation-axis t) (rotation-axis t) (debug-view :no-message))
   (let* ((limbs (remove nil (list (if larm-target :larm) (if rarm-target :rarm))))
          (move-targets (mapcar #'(lambda (x) (send self x :end-coords)) limbs))
          (link-lists (mapcar #'(lambda (x) (send self :link-list (send x :parent))) move-targets))
          (target-coords (remove nil (list larm-target rarm-target)))
          (translation-axis (make-list (length limbs) :initial-element translation-axis))
          (rotation-axis (make-list (length limbs) :initial-element rotation-axis)) ret)
     (dotimes (i 100 ret)
       (let ((dif-pos
              (mapcar #'(lambda (ec tc) (send ec :difference-position tc))
                      move-targets target-coords))
             (dif-rot
              (mapcar #'(lambda (ec tc) (send ec :difference-rotation tc))
                      move-targets target-coords)))
         (setq ret
               (send self :inverse-kinematics-loop
                     dif-pos dif-rot
                     :target-coords target-coords
                     :translation-axis translation-axis
                     :rotation-axis rotation-axis
                     :move-target move-targets :link-list link-lists
                     :debug-view debug-view))
         ))))
  (:reset-pose
   ()
   (send self :larm :angle-vector #f(60 70 80 30 -120 30 0))
   (send self :rarm :angle-vector #f(-60 70 -80 -30 -120 30 0))
   (send self :torso :angle-vector #f(50))
   (send self :angle-vector)
   )
  (:middle-body-joint-angle-list
   ()
   (append (send-all (send self :larm) :joint)
                             (send-all (send self :rarm) :joint)
                             (send-all (send self :torso) :joint)))
  )

(defun pr2 ()
  (setq *pr2* (instance pr2-robot :init "full_pr2.urdf")))
