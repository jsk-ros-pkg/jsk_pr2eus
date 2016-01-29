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
    (send *ri* :go-pos 1 0 0) ;; go-pos is relative to current position
    (send *ri* :go-pos 0 1 90)
    (send *ri* :go-pos-no-wait -1 1 -90)
    (send *ri* :go-waitp)
    (send *ri* :go-wait)
    (assert (eps-v= (send (send *ri* :worldcoords ) :worldpos) #f(0 0 0)))
    ))

(deftest test-move-to
  (let ()
    (setq *ri* (instance pr2-interface :init))
    (send *ri* :move-to (make-coords :pos #f(1000 0 0))) ;; default is world and wait
    (send *ri* :move-to (make-coords :pos #f(1000 1000 0) :rpy (float-vector pi/2 0 0)))
    (send *ri* :move-to (make-coords) :no-wait t) ;; no-wait t means not wait so need to call wait
    (send *ri* :move-to-wait) ;; wait move-to
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

(run-all-tests)
(exit)


