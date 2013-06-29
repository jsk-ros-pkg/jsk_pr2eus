(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(setq *pr2* (pr2))

(while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
  (setq *ri* (instance pr2-interface :init)))

(deftest test-wait-interpolation
  (assert (send *pr2* :reset-pose))
  (assert (send *ri* :angle-vector (send *pr2* :angle-vector) 2000))
  (assert (send *ri* :wait-interpolation))
  )

(deftest test-start-grasp
  (dotimes (i 3)
    (send *ri* :move-gripper :arms 100.0 :effort 20 :wait t)
    (unix:sleep 1)
    (assert (< (setq r (send *ri* :start-grasp :rarm)) 10)
            (format nil "~A check :start-grasp :rarm -> ~A" i r))
    (assert (< (setq r (send *ri* :start-grasp :larm)) 10)
            (format nil "~A check :start-grasp :larm -> ~A" i r))
    (unix:sleep 1))
  )

(deftest test-tuckle-arm
  (assert (pr2-tuckarm-pose :larm))
  (assert (pr2-tuckarm-pose :rarm))
  )

(deftest test-wait-interpolation-after-stop-grasp
  (assert (send *ri* :stop-grasp :rarm :wait t))
  (assert (send *ri* :wait-interpolation))
  )

;; test code for https://sourceforge.net/p/jsk-ros-pkg/tickets/91

(defun check-arm-rotation (arm av0 av1)
  (let ((frame_id (case arm
                        (:larm "/l_gripper_tool_frame")
                        (:rarm "/r_gripper_tool_frame")))
        c0 c1 rotate-flag ret)
    (ros::ros-info "check-arm-rotation ~A" av0)
    (ros::ros-info "                   ~A" av1)
    (when (and x::*display* (> x::*display* 0))
      (setq *b* (make-cube 30 30 100))
      (setf (get *b* :face-color) :red)
      (send *b* :move-to (send (send *pr2* arm :end-coords) :copy-worldcoords) :world)
      (send *b* :translate #f(0 0 50))
      (send (send *pr2* arm :end-coords) :assoc *b*)
      (objects (list *pr2* *b*)))
    (send *pr2* :angle-vector av1)
    (setq c1 (send *pr2* arm :end-coords :copy-worldcoords))
    (send *pr2* :angle-vector av0)
    (setq c0 (send *pr2* arm :end-coords :copy-worldcoords))
    (assert (< (norm (send c0 :difference-rotation c1 :rotation-axis :z)) pi/2) "~A -> ~A" c0 c1) ;; check if initial coords and last coords has same direction
    (dotimes (i 10)
      (send *pr2* :angle-vector (midpoint (/ i 10.0) av0 av1))
      (when (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
      (setq c1 (send *pr2* arm :end-coords :copy-worldcoords))
      (if (> (norm (send c0 :difference-rotation c1)) pi/2)
          (setq rotate-flag t)))
    (when rotate-flag
      (ros::ros-warn "                   : this is rotation motion, check with angle-vector-with-constraint")
      (send *ri* :angle-vector av0 500)
      (send *ri* :wait-interpolation)
      (assert (send *ri* :angle-vector-with-constraint av1 2000 arm :rotation-axis :z) ":anlge-vector-with-constraint ~A" av1)
      (send *ri* :wait-interpolation)
      (return-from check-arm-rotation nil))

    (ros::ros-warn "                   : this is NOT rotation motion, check with simulator")

    (send *ri* :angle-vector av0 500)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector av1 2000)
    (setq c0 (send *tfl* :lookup-transform "/base_footprint" frame_id (ros::time 0)))
    (ros::ros-info "        end-coords : ~A" c0)
    (while (null (some #'identity (send-all (*ri* . controller-actions) :wait-for-result :timeout 0.01)))
      (setq c1 (send *tfl* :lookup-transform "/base_footprint" frame_id (ros::time 0)))
      (ros::ros-info "        end-coords : ~A, diff ~A" c1 (norm (send c0 :difference-rotation c1)))
      (assert (< (norm (send c0 :difference-rotation c1)) pi/2) "~A -> ~A" c0 c1))
    ))


(deftest test-arm-rotation

  (check-arm-rotation :larm
                      #f(167.707 29.4094 -11.5979 30.2555 -21.9039 170.135 -32.6374 158.616 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 34.683 15.5661)
                      #f(167.707 25.5906 28.6932 5.78789 -16.8607 -32.6516 -13.3686 26.8255 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 34.6855 39.3216))


  (check-arm-rotation :larm
                      #f(78.892 122.143 16.5724 128.784 -118.564 -86.9813 -53.4699 32.4022 -30.746 10.2413 -59.3886 -105.093 -89.2534 -28.9254 174.516 70.999 54.714)
                      #f(100.0 10.6748 2.30296 30.0196 -97.4113 107.346 -62.9762 171.355 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 -3.07712 54.4601))

  (check-arm-rotation :larm
                      #f(92.7342 37.2226 12.3315 46.6177 -44.4835 187.553 -30.9676 128.145 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 34.6816 27.3987)
                      #f(55.2304 122.301 14.1074 126.756 -121.311 -90.6499 -52.6751 34.2688 -30.7465 10.2422 -59.3904 -105.095 -89.2577 -28.9264 174.513 71.0005 52.6066)
                     )

  (check-arm-rotation :larm
                      #f(49.9542 47.7996 25.9635 50.0418 -121.521 1.54531 -41.8865 0.0 -60.0012 74.0017 -70.0009 -120.0 -19.9994 -29.9995 -0.0 20.1722 54.5159)
                      #f(49.9491 26.7269 -13.1397 32.8162 -37.719 73.9471 -61.6853 40 -60.0014 73.9984 -69.999 -120.0 -20.0002 -29.9998 -0.0 18.5228 15.5653))

  (check-arm-rotation :larm
                      #f(67.7477 42.8826 4.27682 59.283 -33.9308 71.4826 -18.4835 225.439 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0)
                      #f(67.7477 106.511 -17.9604 97.431 -118.365 225.343 -17.2478 30.768 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0))

  ;; :angle-vector-with-constraint :arms
  (dolist (arm '(:larm :rarm :arms))
    (send *ri* :angle-vector #f(67.7477 42.8826 4.27682 59.283 -33.9308 71.4826 -18.4835 225.439 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0) 500)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector-with-constraint #f(67.7477 106.511 -17.9604 97.431 -118.365 225.343 -17.2478 30.768 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0) 5000 arm)
    (send *ri* :wait-interpolation))
  )


(run-all-tests)
(exit)

