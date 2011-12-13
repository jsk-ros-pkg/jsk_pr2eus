(load "unittest.l")
(init-unit-test)
(load "package://pr2eus/pr2-interface.l")

;; test for pr2-interface
;;  . read topics which is subscribed by pr2-interface

(ros::roseus "pr2-read-state-test")
(pr2)
(defvar *ri* (instance pr2-interface :init))
(defvar *tfl* (instance ros::transform-listener :init))
(unix::sleep 1)

(deftest pr2-read-joint-state-test ()
  (ros::spin-once)
  (assert (send *ri* :state :stamp)
	  "can not read :stamp in robot-state")
  (assert (send *ri* :state :potentio-vector)
	  "can not read :potentio-vector")
  (assert (send *ri* :state :torque-vector)
	  "can not read torque-vector")
  (let ((lend (send *tfl* :lookup-transform "/base_footprint" "/l_gripper_tool_frame"
		    (ros::time 0)))
	(rend (send *tfl* :lookup-transform "/base_footprint" "/r_gripper_tool_frame"
		    (ros::time 0))))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (assert (> 10 (distance (send lend :worldpos)
			    (send *pr2* :larm :end-coords :worldpos)))
	    "larm-end-coords of model and one from tf are not same position")
    (assert (> 10 (distance (send rend :worldpos)
			    (send *pr2* :rarm :end-coords :worldpos)))
	    "rarm-end-coords of model and one from tf are not same position")
    )
  )

(deftest pr2-read-finger-pressure-test ()
  (ros::spin-once)
  (let ((larm-p (send *ri* :state :larm-pressure))
	(rarm-p (send *ri* :state :rarm-pressure)))
    (assert (= 22 (length (car  larm-p))))
    (assert (= 22 (length (cadr larm-p))))
    (assert (= 22 (length (car  rarm-p))))
    (assert (= 22 (length (cadr rarm-p))))
  ))

(deftest pr2-read-odom-test ()
  (ros::spin-once)
  (assert (send *ri* :state :odom :stamp))
  (assert (send *ri* :state :odom :pose))
  (assert (send *ri* :state :odom :velocity))
  )

(deftest pr2-calc-worldcoords-test ()
  (ros::spin-once)
  (assert (send *ri* :state :worldcoords))
  )

(run-all-tests)
(exit)
