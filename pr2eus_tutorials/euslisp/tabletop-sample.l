#!/usr/bin/env roseus

(ros::roseus-add-msgs "posedetection_msgs")

(load "package://pr2eus_tutorials/euslisp/pr2main.l")
;; (load "package://pr2eus_tutorials/arm-navigation-sample.l")

(setq *tfl* (instance ros::transform-listener :init))

(defun detect-callback ( msg )
  (setq *msg* msg))

(setq *msg* nil)
(ros::roseus "tabletop_object_sample")
(ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection
                #'detect-callback)

;; goto initial pose
(goto-init-pose)

(defun detect-one (&optional (timeout 500))
  "subscribe just one detection result and return list of worldcoords of object's centroid"
  (let (lst
        (cntr 0)
        trans-base->table)
    (setq *msg* nil)

    (ros::rate 10)
    (while (null *msg*)
      (ros::spin-once)
      (ros::sleep)
      (if (> (incf cntr) timeout)
          (return-from detect-one null)))

    (setq trans-base->table
          (send *tfl* :lookup-transform
                "/base_footprint"
                (send *msg* :header :frame_id)
                (send *msg* :header :stamp)))

    (dolist (obj (send *msg* :objects))
      (let ((trs
             (send (send trans-base->table :copy-worldcoords)
                   :transform (ros::tf-pose->coords (send obj :pose)))))
        (setf (get trs :type) (send obj :type))
        (push trs lst)
        ))
    lst))

(unix::sleep 5) ;; waiting detection

(defun pickup-pick (&key (arm :rarm))
  (let ((av-st (send *pr2* :angle-vector))
        av-ed av-mid)
  (setq *res* (detect-one))
  ;; sort nearest first
  (sort *res*
        #'(lambda (x y)
            (<= (norm (send (send (send *pr2* arm :end-coords :worldcoords) :transformation x) :pos))
                (norm (send (send (send *pr2* arm :end-coords :worldcoords) :transformation y) :pos)))))
  (cond
   ((send *pr2* arm :inverse-kinematics (car *res*) :rotation-axis :z)
    ;;(setq av-ed (send *pr2* :angle-vector))
    (let ((cds (send *pr2* arm :end-coords :copy-worldcoords)))
      (send cds :translate (float-vector 0 0 35)) ;; offset of centroid
      (if (send *pr2* arm :inverse-kinematics cds :rotation-axis :z)
          (setq av-ed (send *pr2* :angle-vector))))

    (let ((cds (send *pr2* arm :end-coords :copy-worldcoords)))
      (send cds :translate (float-vector -75 0 0)) ;; linear reach to object
      (if (send *pr2* arm :inverse-kinematics cds)
          (setq av-mid (send *pr2* :angle-vector))))

    (when av-mid
      (send *ri* :stop-grasp arm)
      (send *ri* :wait-interpolation) ;; ??

      (send *ri* :angle-vector-sequence
            (list av-mid av-ed) (list 3800 1200))
      (send *ri* :wait-interpolation)

      (> (send *ri* :start-grasp arm) 8) ;; hand width is more than 8mm
      )
    )
   (t nil))
  ))

(defun pickup-up (&key (arm :rarm))
  (send *pr2* arm :move-end-pos (float-vector 0 0 55) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2500)
  (send *ri* :wait-interpolation)
  (goto-init-pose)
  )

(warn ";;
;; (pickup-pick :arm :rarm)
;; (pickup-up :arm :rarm)
;; (pickup-pick :arm :larm)
;; (pickup-up :arm :larm)
;;
")
