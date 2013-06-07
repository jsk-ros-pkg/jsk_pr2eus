(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(deftest test-wait-interpolation
  (setq *pr2* (pr2))

  (while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
    (setq *ri* (instance pr2-interface :init)))

  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (makunbound '*ri*)
  )

(deftest test-start-grasp
  (setq *pr2* (pr2))

  (while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
    (setq *ri* (instance pr2-interface :init)))

  (dotimes (i 10)
    (send *ri* :move-gripper :arms 100.0 :effort 20 :wait t)
    (unix:sleep 1)
    (assert (< (setq r (send *ri* :start-grasp :rarm)) 10)
            (format nil "~A check :start-grasp :rarm -> ~A" i r))
    (assert (< (setq r (send *ri* :start-grasp :larm)) 10)
            (format nil "~A check :start-grasp :larm -> ~A" i r))
    (unix:sleep 1))

  (makunbound '*ri*)
  )

(deftest test-tuckle-arm
  (dotimes (i 3)
    (while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
      (setq *ri* (instance pr2-interface :init)))
    (pr2-tuckarm-pose :larm)
    (pr2-tuckarm-pose :rarm)
    (makunbound '*ri*)
    ))

(deftest test-wait-interpolation-after-stop-grasp
  (setq *pr2* (pr2))

  (while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
    (setq *ri* (instance pr2-interface :init)))
  (send *ri* :stop-grasp :rarm :wait t)
  (send *ri* :wait-interpolation)
  (makunbound '*ri*)
  )

(run-all-tests)
(exit)

