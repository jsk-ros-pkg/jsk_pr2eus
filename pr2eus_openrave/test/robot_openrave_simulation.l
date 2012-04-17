#!/usr/bin/env roseus
(ros::roseus "test_pr2eus_openrave")

(load "unittest.l")
(init-unit-test)

(load "package://pr2eus_openrave/pr2eus-openrave.l")
(setq *exit-on-fail* t)

(ros::defrosparam *model-fname* "~eusmodel_fname" "package://pr2eus/pr2.l")
(ros::defrosparam *robot-name* "~robot_name" "pr2")

(defun init-settings ()
  (unless (boundp '*robot*)
    (load *model-fname*)
    (setq *robot* (eval (read-from-string (format nil "(~A)" *robot-name*)))))
  (unless (boundp '*ri*)
    (setq *ri*
          (cond
           ((string= *robot-name* "pr2") (instance pr2-interface :init))
           (t (instance robot-interface :init :robot (eval (read-from-string (format nil "~A-robot" *robot-name*)))))))))

(defun remove-all-marker ()
  (dotimes (i 500)
    (remove-marker i)))

(setq *test-coords*
      (cond
       ((string= *robot-name* "hironx")
        (list (make-coords :pos (float-vector 500 -300 1300))))
       ((string= *robot-name* "hrp4c")
        (list (make-coords :pos (float-vector 500 -300 900))))
       ((string= *robot-name* "pr2")
        (list
         (make-coords :pos (float-vector 700 100 1100) :rpy (float-vector pi/2 0 0))
         (make-coords :pos (float-vector 750 -100 1400) :rpy (float-vector 0 0 pi/2))))))

(deftest execute-main ()
  (init-settings)
  (ros::wait-for-service "/MoveToHandPosition" -1)

  (cond
   ((string= *robot-name* "pr2") (pr2-tuckarm-pose :larm))
   ((find-method *robot* :reset-pose) (send *robot* :reset-pose)))
  ;;test
  (dolist (p *test-coords*)
    (let (ret)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector))
      (send *robot* :head :look-at (send (send p :copy-worldcoords) :worldpos))
      (send *ri* :angle-vector (send *robot* :angle-vector))
      (setq ret (send *ri* :move-end-coords-plan
                      (send p :copy-worldcoords) :move-arm :rarm :use-torso (if (string= *robot-name* "hironx") nil t) :lifetime 10))
      (ros::ros-info "move-end-coords-plan -> ~A~%" ret)
      (if (and *exit-on-fail* (null ret)) (exit 1))

      (unix::sleep 1)
      (remove-all-marker)
      (unix::sleep 1)
      (cond
       ((string= *robot-name* "pr2") (pr2-tuckarm-pose :larm))
       (t (send *robot* :reset-pose)))
      ))
  )

(run-all-tests)
(exit)

;;(execute-main)