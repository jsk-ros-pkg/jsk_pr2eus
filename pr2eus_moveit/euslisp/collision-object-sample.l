(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
(load "models/foldable-desk-object.l")
(load "models/petbottle-object.l")

(defun init-demo ()
  (samplerobot-init) ;; create *ri* / *sr*
  (setq *co* (instance collision-object-publisher :init))

  (setq *table* (instance foldable-desk-object :init))
  (setq *bottle* (instance petbottle-object :init))

  (send *bottle* :translate #f(-100.0 140.0 705.0))

  (send *sr* :fix-leg-to-coords (make-coords :pos #f(-450 0 0)))
  (send *sr* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -10.0 0.0 -60.0 9.0 -6.5 25.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 10.0 0.0 -60.0 -9.0 -6.5 -25.0 0.0 0.0 0.0))
  ;; (send *ri* :angle-vector (send *sr* :angle-vector) 6000)
  ;;(objects (list *table* *bottle* *sr*))
  (pub-obj *table*)
  (pub-obj *bottle*)
  )

(defun pub-obj (obj) ;; need link
  (let ((cds
         (send (send *sr* :waist_link0_lk) :transformation (send obj :worldcoords)))
        (org-cds (send obj :copy-worldcoords)))
    (send *co* :add-object obj :frame_id "WAIST_LINK0" :relative-pose cds)
    obj))
