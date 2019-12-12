(require :unittest "lib/llib/unittest.l")
(load "package://pr2eus/pr2-interface.l")

;; avoid print violate max/min-angle that exceeds 4M log limit
(setf (symbol-function 'warning-message-org)
      (symbol-function 'warning-message))
(defun warning-message (color format &rest mesg)
  (unless (or (substringp "violate min-angle" format)
              (substringp "violate max-angle" format))
    (apply #'warning-message-org color format mesg)))

(defun shortest-angle (d0 d1)
  (atan2 (sin (- d0 d1)) (cos (- d0 d1))))

;; initialize *pr2*

(setq *pr2* (pr2))

(while (or (not (boundp '*ri*)) (send *ri* :simulation-modep))
  (setq *ri* (instance pr2-interface :init)))

(when (send *ri* :simulation-modep)
  (ros::ros-warn "*ri* is running with simulation mode, something goes wrong ....")
  (sys::exit 1))

(init-unit-test)

(deftest test-wait-interpolation
  (ros::ros-info "send reset-pose and wait-interpolation")
  (assert (send *pr2* :reset-pose))
  (assert (send *ri* :angle-vector (send *pr2* :angle-vector) 1000))
  (assert (every #'null (send *ri* :wait-interpolation))
          ":wait-interpolation must be list of nil")
  (assert (null (send *ri* :interpolatingp)))

  (ros::ros-info "send reset-pose, cancel goal and wait-interpolation")
  (send *pr2* :arms :shoulder-p :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (unix:sleep 2)
  ;; emulate someone stops controller from outside of *ri**
  (let* ((ca (car (*ri* . controller-actions)))
         (jn (cdr (assoc :joint-names (car (send *ri* :default-controller)))))
         (goal (send ca :make-goal-instance)))
    (send goal :goal :trajectory :joint_names jn)
    (ros::publish (format nil "~A/goal" (send ca :name)) goal))
  (assert (every #'null (send *ri* :wait-interpolation)))
  (assert (null (send *ri* :interpolatingp)))
  ;;
  (ros::ros-info "send reset-pose, cancel goal and wait-interpolation-smooth")
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (unix:sleep 2)
  ;; emulate someone stops controller from outside of *ri**
  (let* ((ca (car (*ri* . controller-actions)))
         (jn (cdr (assoc :joint-names (car (send *ri* :default-controller)))))
         (goal (send ca :make-goal-instance)))
    (send goal :goal :trajectory :joint_names jn)
    (send goal :goal :trajectory :points
          (list (instance trajectory_msgs::JointTrajectoryPoint
                          :init
                          :positions (instantiate float-vector (length jn))
                          :time_from_start (ros::time 0.1))))
    (ros::publish (format nil "~A/goal" (send ca :name)) goal))
  (assert (null (send *ri* :wait-interpolation-smooth 500))) ;; see https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/187#issuecomment-303074351
  (assert (send *ri* :interpolating-smoothp 500)) ;; see https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/187#issuecomment-303074351
  )

(deftest test-start-grasp
  (dotimes (i 3)
    (send *ri* :move-gripper :arms 100.0 :effort 5000 :wait t)
    (unix:sleep 1)
    (assert (< (setq r (send *ri* :start-grasp :rarm :gain 5)) 20)
            (format nil "~A check :start-grasp :rarm -> ~A" i r))
    (assert (< (setq r (send *ri* :start-grasp :rarm :gain 5)) 10)
            (format nil "~A check :start-grasp :rarm -> ~A" i r))
    (assert (< (setq r (send *ri* :start-grasp :larm :gain 5)) 20)
            (format nil "~A check :start-grasp :larm -> ~A" i r))
    (assert (< (setq r (send *ri* :start-grasp :larm :gain 5)) 10)
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
      (assert (send *ri* :angle-vector-with-constraint av1 500 arm :rotation-axis :z) ":anlge-vector-with-constraint ~A" av1)
      (send *ri* :wait-interpolation)
      (return-from check-arm-rotation nil))

    (ros::ros-warn "                   : this is NOT rotation motion, check with simulator")

    (send *ri* :angle-vector av0 500)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector av1 500)
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
    (send *ri* :angle-vector-with-constraint #f(67.7477 106.511 -17.9604 97.431 -118.365 225.343 -17.2478 30.768 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0) 1000 arm)
    (send *ri* :wait-interpolation))
  )

(deftest test-wait-interpolation-timeout
  (let ((ret))
    (ros::ros-warn "send angle-vector with 1 sec")
    (warning-message 3 "send angle-vector with 1 sec~%")
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

    (setq ret (send *ri* :wait-interpolation))
    (ros::ros-warn ":wait-interpolation returns ~A (= nil)" ret)
    (warning-message 3 ":wait-interpolation returns ~A (= nil)~%" ret)
    (assert (every #'null ret) "interpolation must be finished")
    (setq ret (send *ri* :interpolatingp))
    (ros::ros-warn ":interpolatingp returns ~A (= nil)" ret)
    (warning-message 3 ":interpolatingp returns ~A (= nil)~%" ret)
    (assert (null ret) "interpolation must be finished")

    (ros::ros-warn "send angle-vector with 5 sec")
    (warning-message 3 "send angle-vector with 5 sec~%")
    (send *pr2* :reset-manip-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (setq ret (send *ri* :wait-interpolation nil 0.1))
    (ros::ros-warn ":wait-interpolation nil 0.1, returns ~A (= t)" ret)
    (warning-message 3 ":wait-interpolation nil 0.1 returns ~A (= t)~%" ret)
    (assert (some #'identity ret) "robot should be interpolating")
    (ros::rate 5)
    (dotimes (i (* 3 5)) ;; check if :interpolatingp does not returns nil for 3.0 sec
      (setq ret (send *ri* :interpolatingp))
      (ros::ros-warn ":interpolatingp returns ~A (= t)" ret)
      (warning-message 3 ":interpolatingp returns ~A (= t)~%" ret)
      (assert ret "robot should be interpolating")
      (ros::sleep))
    ;;
    (setq ret (send *ri* :wait-interpolation))
    (ros::ros-warn ":wait-interpolation, returns ~A (= nil)" ret)
    (warning-message 3 ":wait-interpolation returns ~A (= nil)~%" ret)
    (assert (every #'null ret) "interpolation must be finished")
    (setq ret (send *ri* :interpolatingp))
    (ros::ros-warn ":interpolatingp returns ~A (= nil)" ret)
    (warning-message 3 ":interpolatingp returns ~A (= nil)~%" ret)
    (assert (null ret) "interpolation must be finished")
    ))

(deftest test-end-coords-interpolation
  (let (tm-0 tm-1 tm-diff)
    (send *pr2* :reset-manip-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-pos #f(50 0 0) :world)
    (send *pr2* :rarm :move-end-pos #f(50 0 0) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 nil 0
          :min-time 1.0
          :end-coords-interpolation t
          :end-coords-interpolation-steps 10)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (< tm-diff 2) (format nil "end-coords-interpolation takes too long time ~A" tm-diff))
    ))

(deftest test-go-stop
  (let ((x 0.3) (y 0.3) (d 90)
        pos0 pos1)
    (pr2-tuckarm-pose)
    (ros::ros-info "send :go-pos ~A ~A ~A, assuming it takse longer than 2 sec" x y d)
    (send *ri* :go-pos-no-wait x y d)
    (send (instance ros::duration :init 2) :sleep)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "get current locateion ~A and send :go-stop" pos0)
    (send *ri* :go-stop)
    (unix:sleep 1)
    (assert (equal (send (*ri* . move-base-action) :get-state) actionlib_msgs::GoalStatus::*preempted*)) ;; make sure go-pos is preempted
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (setq diff-pos (norm (send pos0 :difference-position pos1)))
    (ros::ros-info "get current locateion ~A, and diff position ~A" pos1 diff-pos)
    ;; diff in 3cm
    (assert (< diff-pos 30) (format nil "go-stop does not stop: ~A > 30" diff-pos))))

(defun go-pos-func (&key (x 0.3) (y 0.3) (d 90) (func :go-pos)) ;; [m] [m] [degree]
  (let (pos0 pos1 pos2 diff-pos diff-rot tm0 tm1)
    (pr2-tuckarm-pose)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "send ~A ~A ~A ~A" func x y d)
    (setq tm0 (ros::time-now))
    (send *ri* func x y d)
    ;; wait for 2 sec
    (send (instance ros::duration :init 2) :sleep)
    (setq tm1 (ros::time-now))
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (setq diff-pos (send pos0 :difference-position pos1))
    (setq diff-rot (send pos0 :difference-rotation pos1))
    ;; if func is -no-wait, make sure that :go-pos-* returns immediately but move forward
    (when (substringp "NO-WAIT" (string func))
      (ros::ros-info "~A returns immediately ~A" func (send (ros::time- tm1 tm0) :to-sec))
      (ros::ros-info " difference-position: ~A -> ~A" diff-pos (norm diff-pos))
      (assert (< (abs (- (send (ros::time- tm1 tm0) :to-sec) 2.0)) 0.5))
      (assert (> (norm diff-pos) 25))
      (ros::ros-info "wait for result..")
      ;; wait for result
      (cond ((substringp "GO-POS-UNSAFE" (string func)) ;; FIXME
             (send *ri* :go-pos-unsafe-wait))
            (t
             (send *ri* :move-to-wait :retry 1 :frame-id (*ri* . base-frame-id))))
      (unix:sleep 1)
      (setq pos1 (send *ri* :state :worldcoords "map"))
      (setq diff-pos (send pos0 :difference-position pos1))
      (setq diff-rot (send pos0 :difference-rotation pos1))
      )
    (ros::ros-info "reached to the goal")
    (ros::ros-info " difference-position: ~A -> ~A ~A" diff-pos (- (elt diff-pos 0) (* 1000 x)) (- (elt diff-pos 1) (* 1000 y)))
    (ros::ros-info " difference-rotation: ~A -> ~A" diff-rot (shortest-angle (deg2rad d) (elt diff-rot 2)))
    ;; diff in 10cm
    (assert (< (abs (- (elt diff-pos 0) (* 1000 x))) 100)
            (format nil "go-pos moves incorrectly in x axis: ~A != ~A"
                    (* 1000 x) (elt diff-pos 0)))
    (assert (< (abs (- (elt diff-pos 1) (* 1000 y))) 100)
            (format nil "go-pos moves incorrectly in y axis: ~A != ~A"
                    (* 1000 y) (elt diff-pos 1)))
    (assert (< (abs (shortest-angle (deg2rad d) (elt diff-rot 2))) 0.8) ;; go-pos-unsafe-no-wait need 0.55... other cases fit within 0.1
            (format nil "go-pos moves incorrectly in d axis: ~A != ~A"
                    (deg2rad d) (elt diff-rot 2)))
    ))

(deftest test-go-pos
  (go-pos-func :func :go-pos))

(deftest test-go-pos-no-wait
  (go-pos-func :func :go-pos-no-wait))

(deftest test-go-pos-unsafe
  (go-pos-func :func :go-pos-unsafe))

(deftest test-go-pos-unsafe-no-wait
  (if (*ri* . move-base-trajectory-action)
      (go-pos-func :func :go-pos-unsafe-no-wait)
    (ros::ros-warn "SKIP TEST :go-pos-unsafe-no-wait requires move-base-trajectory-action")))


(defun go-velocity-func (&key wait)
  (let ((vel-x 1) (vel-y 1) (vel-d 1.57)  ;; [m/s] [m/s] [rad/s]
        pos0 pos1 pos2 pos3 tm0 tm1 diff-pos vel-diff-pos)
    (pr2-tuckarm-pose)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (setq tm0 (ros::time-now))
    (ros::ros-info "(before go-velocity) current location ~A" pos0)
    (ros::ros-info "send :go-velocity ~A ~A ~A :wait ~A"  vel-x vel-y vel-d wait)
    (send *ri* :go-velocity vel-x vel-y vel-d 3000 :wait wait) ;; go-velocity blocks for 3 sec if wait is t
    (unless wait
      (send (instance ros::duration :init 2) :sleep)
      (setq tm1 (ros::time-now))
      (setq pos1 (send *ri* :state :worldcoords "map"))
      (setq diff-pos (send pos0 :difference-position pos1))
      (setq diff-rot (send pos0 :difference-rotation pos1))
      (ros::ros-info ":go-velocity :wait ~A returns immediately ~A" wait (send (ros::time- tm1 tm0) :to-sec))
      (ros::ros-info " difference-position: ~A -> ~A" diff-pos (norm diff-pos))
      (assert (< (abs (- (send (ros::time- tm1 tm0) :to-sec) 2.0)) 0.5))
      (assert (> (norm diff-pos) 25))
      (send (*ri* . move-base-trajectory-action) :wait-for-result)
      )
    (setq tm1 (ros::time-now))
    (unix:sleep 1) ;; make sure robot actually stops
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "(after 1 sec )      current location ~A" pos1)
    (setq diff-pos (norm (send pos0 :difference-position pos1)))
    (setq vel-diff-pos
          (* 1000 (norm (float-vector vel-x vel-y))
             (send (ros::time- tm1 tm0) :to-sec)))
    (ros::ros-info " difference-position from position ~A" diff-pos)
    (ros::ros-info " difference-position from velocity ~A (~A sec)" vel-diff-pos (send (ros::time- tm1 tm0) :to-sec))
    (assert (>  vel-diff-pos diff-pos)
            (format nil "go-velocity moves too much: ~A" diff-pos))
    ))

(deftest test-go-velocity
  (if (*ri* . move-base-trajectory-action)
      (go-velocity-func)
    (ros::ros-warn "SKIP TEST: default behavior of :go-velocity is :wait nil, and this requires move-base-trajectory-action")))

(deftest test-go-velocity-wait
  (go-velocity-func :wait t))

(deftest test-move-to
  (let ((x 0.3) (y 0.3) (d 1.57) ;; [m] [m] [rad]
        pos0)
    (pr2-tuckarm-pose)
    (ros::ros-info "send :move-to ~A ~A ~A" x y d)
    (send *ri* :move-to
          (make-coords :pos (float-vector (* 1000 x) (* 1000 y) 0)
                       :rpy (float-vector d 0 0))
          :frame-id "map")
    (unix:sleep 1)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "reached to the goal")
    (ros::ros-info " difference-position: ~A -> ~A ~A" (send pos0 :worldpos) (- (elt (send pos0 :pos) 0) (* 1000 x)) (- (elt (send pos0 :pos) 1) (* 1000 y)))
    (ros::ros-info " difference-rotation: ~A -> ~A" (send pos0 :worldrot) (shortest-angle d (elt (car (send pos0 :rpy-angle)) 0)))
    ;; diff in 10cm
    (assert (< (abs (- (elt (send pos0 :pos) 0) (* 1000 x))) 100)
            (format nil "move-to moves incorrectly in x axis: ~A != ~A"
                    (* 1000 x) (elt (send pos0 :pos) 0)))
    (assert (< (abs (- (elt (send pos0 :pos) 1) (* 1000 y))) 100)
            (format nil "move-to moves incorrectly in y axis: ~A != ~A"
                    (* 1000 y) (elt (send pos0 :pos) 1)))
    (assert (< (abs (shortest-angle d (elt (car (send pos0 :rpy-angle)) 0))) 0.1)
            (format nil "go-pos-unsafe moves incorrectly in d axis: ~A != ~A"
                    d (elt (car (send pos0 :rpy-angle)) 0)))
    ))

(run-all-tests)
(exit)

