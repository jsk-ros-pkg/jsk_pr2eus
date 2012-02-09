#!/usr/bin/env roseus
(ros::roseus "test_pr2eus_openrave")

(load "unittest.l")
(init-unit-test)

(load "package://pr2eus_openrave/pr2eus-openrave.l")
(setq *exit-on-fail* t)

(defun init-settings ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*)
    (setq *ri* (instance pr2-interface :init)))
  )

(defun remove-all-marker ()
  (dotimes (i 500)
    (remove-marker i)))

(setq *test-coords*
      (list
       (make-coords :pos (float-vector 700 100 1100) :rpy (float-vector pi/2 0 0))
       (make-coords :pos (float-vector 750 -100 1400) :rpy (float-vector 0 0 pi/2))
       ))

(deftest execute-main ()
  (init-settings)
  (ros::wait-for-service "/MoveToHandPosition" -1)

  (pr2-tuckarm-pose :larm)
  ;;test
  (dolist (p *test-coords*)
    (let (ret)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :look-at (send (send p :copy-worldcoords) :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (setq ret (send *ri* :move-end-coords-plan
		      (send p :copy-worldcoords) :move-arm :rarm :use-torso t :lifetime 10))
      (ros::ros-info "move-end-coords-plan -> ~A~%" ret)
      (if (and *exit-on-fail* (null ret)) (exit 1))

      (unix::sleep 1)
      (remove-all-marker)
      (unix::sleep 1)
      (pr2-tuckarm-pose :larm)
      ))
  )

(run-all-tests)
(exit)

;;(execute-main)