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
    (assert (send *ri* :angle-vector-sequence (list av2 av0) 2000))
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


