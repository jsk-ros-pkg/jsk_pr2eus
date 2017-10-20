(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(pr2-init)

(deftest test-angle-vector
  (let (av0 av1 av2)
    (setq *ri* (instance pr2-interface :init))
    (assert (setq av0 (send *pr2* :reset-pose)))
    (send *pr2* :larm :shoulder-p :joint-angle 0)
    (assert (setq av1 (send *pr2* :angle-vector)))
    (send *pr2* :rarm :shoulder-p :joint-angle 0)
    (assert (setq av2 (send *pr2* :angle-vector)))
    (assert (send *ri* :angle-vector av1 2000))

    ;; (length avs) > (length tms)
    (assert (send *ri* :angle-vector-sequence (list av2 av0) 2000))
    ;; (length avs) = (length tms)
    (assert (send *ri* :angle-vector-sequence (list av0 av1) (list 2000 3000)))
    ;; (length avs) < (length tms)
    (assert (send *ri* :angle-vector-sequence (list av0 av1) (list 1000 2000 3000)))
    ))

(deftest test-angle-vector-limb
  (let (av0 av1 av2)
    (setq *ri* (instance pr2-interface :init))
    (assert (setq av0 (send *pr2* :reset-pose)))
    (send *pr2* :larm :shoulder-p :joint-angle 0)
    (assert (setq av1 (send *pr2* :angle-vector)))
    (send *pr2* :rarm :shoulder-p :joint-angle 0)
    (assert (setq av2 (send *pr2* :angle-vector)))
    (assert (send *ri* :angle-vector av1 2000 :larm-controller))
    (assert (send *ri* :angle-vector-sequence (list av2 av0) 2000 :larm-controller))
    ))

(deftest test-angle-vector-duration
  (let (tm)
    (setq *ri* (instance pr2-interface :init))
    (assert (send *pr2* :reset-pose))
    (assert (send *ri* :angle-vector (send *pr2* :angle-vector)))
    (assert (= (send *ri* :angle-vector-duration (send *ri* :state :potentio-vector) (send *pr2* :angle-vector) 5 1.0) 1))
    (send *pr2* :larm :shoulder-p :joint-angle 0)
    (setq tm (send *ri* :angle-vector-duration (send *ri* :state :potentio-vector) (send *pr2* :angle-vector) 5 1.0 :larm-controller))
    (assert (> tm 1))
    (setq tm (send *ri* :angle-vector-duration (send *ri* :state :potentio-vector) (send *pr2* :angle-vector) 5 1.0 :rarm-controller))
    (assert (eps= tm 1.0))
    ))

;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165#discussion_r37421484
(deftest test-go-pos
  (let ()
    (setq *ri* (instance pr2-interface :init))
    (assert (send *ri* :go-pos 1 0 0) "(send *ri* :go-pos 1 0 0)") ;; go-pos is relative to current position
    (assert (send *ri* :go-pos 0 1 90) "(send *ri* :go-pos 0 1 90)")
    (assert (send *ri* :go-pos-no-wait -1 1 -90) "(send *ri* :go-pos-no-wait -1 1 -90)")
    (assert (send *ri* :go-waitp) "(send *ri* :go-waitp)")
    (assert (send *ri* :go-wait) "(send *ri* :go-wait)")
    (assert (eps-v= (send (send *ri* :worldcoords ) :worldpos) #f(0 0 0)))
    ))

(deftest test-move-to
  (let ()
    (setq *ri* (instance pr2-interface :init))
    (assert (send *ri* :move-to (make-coords :pos #f(1000 0 0))) "(send *ri* :move-to (make-coords :pos #f(1000 0 0)))") ;; default is world and wait
    (send *ri* :move-to (make-coords :pos #f(1000 1000 0) :rpy (float-vector pi/2 0 0)))
    (assert (send *ri* :move-to (make-coords) :no-wait t) "(send *ri* :move-to (make-coords) :no-wait t)") ;; no-wait t means not wait so need to call wait
    (assert (send *ri* :move-to-wait) "(send *ri* :move-to-wait)") ;; wait move-to
    (assert (eps-v= (send (send *ri* :worldcoords ) :worldpos) #f(0 0 0)))
    ))


;; https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/143
(defclass pr2-interface-wrong
  :super pr2-interface)
(defmethod pr2-interface-wrong
  (:torso-controller
   ()
   (list
	(list
	 (cons :controller-action "torso_controller/follow_joint_trajectory")
	 (cons :controller-state "torso_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "torso_lift_joint-NOT-FOUND"))))))


(deftest test-wrong-controller
  (let ()
    (setq *ri* (instance pr2-interface-wrong :init))
    (send *pr2* :reset-manip-pose)
    (assert (send *ri* :angle-vector (send *pr2* :angle-vector)))
    (send *pr2* :reset-pose)
    (assert (send *ri* :angle-vector-sequence (list (send *pr2* :angle-vector))))))

;;
;; https://github.com/jsk-ros-pkg/jsk_robot/pull/849#issuecomment-334906516
;;
;; add test to check sub-angle-vector over 620
(deftest test-sub-angle-vector-1000
  (let (angle diff msg)
    ;;(setq *ri* (instance pr2-interface :init))
    (send *pr2* :reset-manip-pose)
    (send *ri* :robot :reset-manip-pose)
    (dolist (inc (list 10 -10))
      (send *ri* :robot :larm :elbow-r :joint-angle 0)
      (setq angle 0.0)
      (setq diff 0.0)
      (do ((i 0 (incf i inc)))
          ((if (> inc 0) (<= 1000 i) (>= -1000 i)))
          (send *pr2* :larm :elbow-r :joint-angle i)
          (setq diff (elt (send *ri* :sub-angle-vector (send *pr2* :angle-vector) (send *ri* :robot :angle-vector)) 5))
          (setq msg (format nil ":sub-angle-vector : original angle-vector from ~A to ~A, expected ~A ~A~%" i diff angle (eps= angle diff)))
          (warning-message 2 msg)
          (assert (eps= angle diff) msg)
          (incf angle inc)
          (if (>= angle 185.0) (setq angle -170.0))
          (if (<= angle -185.0) (setq angle 170.0))
          ))
    ))

(deftest test-angle-vector-over-640
  (let (msg)
    (send *pr2* :reset-manip-pose)
    (send *ri* :robot :reset-manip-pose)
    (dolist (angle (list 60 420 780))
      (send *pr2* :larm :elbow-r :joint-angle angle)
      (send-message *ri* robot-interface :angle-vector (send *pr2* :angle-vector) 500)
      (send *ri* :wait-interpolation)
      (setq msg
            (format nil "*ri* ~A, *pr2* ~A ~A~%"
                    (elt (send *ri* :state :potentio-vector) 5)
                    (elt (send *pr2* :angle-vector) 5)
                    (eps= (elt (send *ri* :state :potentio-vector) 5) 60.0)))
      (warning-message 2 msg)
      (assert (eps= (elt (send *ri* :state :potentio-vector) 5) 60.0) msg)
      )
    ))

(deftest test-angle-vector-under-640
  (let (msg)
    (send *pr2* :reset-manip-pose)
    (send *ri* :robot :reset-manip-pose)
    (dolist (angle (list -60 -420 -780))
      (send *pr2* :larm :elbow-r :joint-angle angle)
      (send-message *ri* robot-interface :angle-vector (send *pr2* :angle-vector) 500)
      (send *ri* :wait-interpolation)
      (setq msg
            (format nil "*ri* ~A, *pr2* ~A ~A~%"
                    (elt (send *ri* :state :potentio-vector) 5)
                    (elt (send *pr2* :angle-vector) 5)
                    (eps= (elt (send *ri* :state :potentio-vector) 5) -60.0)))
      (warning-message 2 msg)
      (assert (eps= (elt (send *ri* :state :potentio-vector) 5) -60.0) msg)
      )
    ))

;; test to check unintentional 180 rotation during end-coords interpolation
;; c.f. https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/325
(deftest test-angle-vector-sequence-end-coords-interpolation
  (let ((vec #f(50.0 47.7222 12.2897 117.176 -106.014 -77.3995 -76.8709 -105.904 -47.7222 12.2897 -117.176 -106.014 77.3995 -76.8709 105.904 0.0 0.0))
        (v0 #f(50.0 53.1894 14.6871 123.249 -121.524 -74.0435 -76.2839 266.776 -53.1894 14.6871 -123.249 -121.524 74.0435 -76.2839 93.2238 0.0 0.0))
        (i 0) avs av-pre)
    (setq avs (send *ri* :angle-vector-sequence (list vec (send *pr2* :reset-pose)) (list 1000 1000) :default-controller 0 :end-coords-interpolation t))
    (setq av-pre (car avs))
    (dolist (av (cdr avs))
      (format t "~A/~A diff ~A, max ~A~%" (incf i) (length avs) (v- av av-pre) (abs (geo::find-extream (coerce (v- av av-pre) cons) #'abs #'>=)))
      (assert (< (abs (geo::find-extream (coerce (v- av av-pre) cons) #'abs #'>=)) 180)
              (format nil "found unintentional +180 joint rotation ~A" (v- av av-pre)))
      (setq av-pre av))
    ;;
    ))

;; test to check unintentional 180 rotation during end-coords interpolation
;; c.f. https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/325
(deftest test-angle-vector-sequence-end-coords-interpolation-head
  (let ((v1 #f(50.0 -16.4077 18.0876 93.6936 -54.8829 77.4245 -30.5008 123.25 -50.4711 19.1116 -59.6967 -13.4894 275.94 -109.525 -35.7734 -47.3098 49.3376))
        (v2 #f(191.303 50.0682 17.9701 102.195 -10.7369 43.6391 -103.396 221.198 32.3493 45.9097 -15.2106 -55.9981 227.517 -63.9359 38.9722 0.0 0.0))
        avs)
    (send *ri* :angle-vector v1)
    (send *ri* :wait-interpolation)
    (setq avs (send *ri* :angle-vector v2 2000 :default-controller 0 :end-coords-interpolation t))
    (format t "abs = ~A~%" avs)
    (assert avs)
    ))


(run-all-tests)
(exit)


