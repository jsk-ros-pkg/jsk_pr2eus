;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")
(require :robot-interface "package://pr2eus/robot-interface.l")
(ros::load-ros-manifest "topic_tools")

;;;
;;; pr2 robot interface
;;;

(ros::load-ros-manifest "pr2eus")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "nav_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "control_msgs")
(load "package://pr2eus/speak.l")

(defmethod pr2-robot
  (:torque-vector
   (&rest args)
   (if args (ros::ros-warn "pr2 torque-vector does not have parameters"))
   (coerce (send-all (send self :joint-list) :joint-torque) float-vector)))

(defclass pr2-interface
  :super robot-interface
  :slots (r-gripper-action l-gripper-action
                           move-base-action move-base-trajectory-action
                           finger-pressure-origin))
(defmethod pr2-interface
  (:init
   (&rest args &key (type :default-controller)
          (move-base-action-name "move_base") &allow-other-keys)
   (send-super* :init :robot pr2-robot :type type
                :groupname "pr2_interface"
                args)
   ;; add controllers
   (dolist (l (list
               (cons :larm-controller "l_arm_controller/follow_joint_trajectory")
               (cons :rarm-controller "r_arm_controller/follow_joint_trajectory")
               (cons :head-controller "head_traj_controller/follow_joint_trajectory")
               (cons :torso-controller "torso_controller/follow_joint_trajectory")))
     (let ((type (car l))
           (name (cdr l))
           action)
       (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                             controller-actions))
       (setf (gethash type controller-table) (list action))
       ))
   ;;
   (ros::advertise "j_robotsound" sound_play::SoundRequest 5)
   ;;
   (ros::subscribe "/base_odometry/odom" nav_msgs::Odometry
                   #'send self :pr2-odom-callback :groupname groupname)
   (ros::subscribe "/pressure/r_gripper_motor" pr2_msgs::PressureState
                   #'send self :pr2-fingertip-callback :rarm-pressure :groupname groupname)
   (ros::subscribe "/pressure/l_gripper_motor" pr2_msgs::PressureState
                   #'send self :pr2-fingertip-callback :larm-pressure :groupname groupname)
   ;;
   (setq r-gripper-action (instance ros::simple-action-client :init
                                    "/r_gripper_controller/gripper_action"
                                    pr2_controllers_msgs::Pr2GripperCommandAction
                                    :groupname groupname))
   (setq l-gripper-action (instance ros::simple-action-client :init
                                    "/l_gripper_controller/gripper_action"
                                    pr2_controllers_msgs::Pr2GripperCommandAction
                                    :groupname groupname))
   (setq move-base-action (instance ros::simple-action-client :init
                                    move-base-action-name move_base_msgs::MoveBaseAction
                                    :groupname groupname))
   (setq move-base-trajectory-action
         (instance ros::simple-action-client :init
                   "/base_controller/joint_trajectory_action"
                   pr2_controllers_msgs::JointTrajectoryAction
                   :groupname groupname))

   ;; wait for pr2-action server (except move_base)
   (setq joint-action-enable t)
   (dolist (action (list r-gripper-action l-gripper-action))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, pr2-interface is disabled" action)
       (return)))
   t)
  ;;
  (:pr2-odom-callback
   (msg)
   (let ((parsed
	  (list
	   (cons :stamp (send msg :header :stamp))
	   (cons :pose (ros::tf-pose->coords (send msg :pose :pose)))
	   (cons :velocity (float-vector
			    (* 1000 (send msg :twist :twist :linear :x))
			    (* 1000 (send msg :twist :twist :linear :y))
			    (send msg :twist :twist :angular :z))))))
     (send self :set-robot-state1 :odom parsed)))
  ;;
  (:state (&rest args) ;; overwrite for jsk_maps/pr2
   (case (car args)
     (:worldcoords
      (unless joint-action-enable
	(return-from :state (send self :worldcoords)))
      (send-super :state :worldcoords (or (cadr args) "/world")))
     (t
      (send-super* :state args))
     ))
  (:publish-joint-state () ;; overwrite for pr2
   (send-super :publish-joint-state (append (send robot :joint-list) (send robot :larm :gripper :joint-list) (send robot :rarm :gripper :joint-list))))
  ;;
  (:wait-interpolation (&rest args);; overwrite for pr2, due to some joint is stll moving after joint-trajectry-action stops
   (unless joint-action-enable (return-from :wait-interpolation nil))
   (let (result)
     (ros::ros-info "wait-interpolation debug: start")
   ;;  (setq result (send-all controller-actions :wait-for-result))
     ;;(setq result (send-super* :wait-interpolation args))
     (dolist (ca controller-actions)
       (push (send ca :wait-for-result) result))
     (ros::ros-info "wait-interpolation debug: end")
     (setq result (reverse result))
	 (while (ros::ok)
	   (send self :update-robot-state)
	   (when (every #'(lambda(x)(< (abs (send x :joint-velocity))
				       (if (derivedp x rotational-joint) 0.05 0.001)))
			  (send robot :joint-list))
		 (return)))
	 result))

  ;;
  ;;
  (:larm-controller
   ()
   (list
	(list
	 (cons :controller-action "l_arm_controller/follow_joint_trajectory")
	 (cons :controller-state "l_arm_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "l_shoulder_pan_joint"
							  "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
							  "l_elbow_flex_joint" "l_forearm_roll_joint"
							  "l_wrist_flex_joint" "l_wrist_roll_joint")))))
  (:rarm-controller
   ()
   (list
	(list
	 (cons :controller-action "r_arm_controller/follow_joint_trajectory")
	 (cons :controller-state "r_arm_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "r_shoulder_pan_joint"
							  "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
							  "r_elbow_flex_joint" "r_forearm_roll_joint"
							  "r_wrist_flex_joint" "r_wrist_roll_joint")))))
  (:head-controller
   ()
   (list
	(list
	 (cons :controller-action "head_traj_controller/follow_joint_trajectory")
	 (cons :controller-state "head_traj_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:torso-controller
   ()
   (list
	(list
	 (cons :controller-action "torso_controller/follow_joint_trajectory")
	 (cons :controller-state "torso_controller/state")
	 (cons :action-type control_msgs::FollowJointTrajectoryAction)
	 (cons :joint-names (list "torso_lift_joint")))))
  (:default-controller
   ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :head-controller)
    (send self :torso-controller)))
  (:midbody-controller
   ()
   (list
    (list
     (cons :controller-action "midbody_controller/joint_trajectory_action")
     (cons :controller-state "midbody_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_pan_joint"
			      "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
			      "l_elbow_flex_joint" "l_forearm_roll_joint"
			      "l_wrist_flex_joint" "l_wrist_roll_joint"
			      "r_shoulder_pan_joint"
			      "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
			      "r_elbow_flex_joint" "r_forearm_roll_joint"
			      "r_wrist_flex_joint" "r_wrist_roll_joint"
			      "torso_lift_joint")))
    (send self :head-controller)))
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/joint_trajectory_action")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_pan_joint"
			      "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
			      "l_elbow_flex_joint" "l_forearm_roll_joint"
			      "l_wrist_flex_joint" "l_wrist_roll_joint"
			      "r_shoulder_pan_joint"
			      "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
			      "r_elbow_flex_joint" "r_forearm_roll_joint"
			      "r_wrist_flex_joint" "r_wrist_roll_joint"
			      "torso_lift_joint"
			      "head_pan_joint" "head_tilt_joint")))))
  ;;
  (:controller-angle-vector (av tm type) ;; obsolate
   (send self :angle-vector av tm type))
  (:larm-angle-vector (av tm)
   (send self :angle-vector av tm :larm-controller))
  (:rarm-angle-vector (av tm)
   (send self :angle-vector av tm :rarm-controller))
  (:head-angle-vector (av tm)
   (send self :angle-vector av tm :head-controller))
  ;;
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
   (unless joint-action-enable
     (send robot arm :gripper :joint-angle (* pos 1000))
     (send self :publish-joint-state)
     (if viewer (send self :draw-objects))
     (return-from :move-gripper nil))
   (let* (goal
	  (clients (case arm
		     (:rarm (list r-gripper-action))
		     (:larm (list l-gripper-action))
		     (:arms (list r-gripper-action l-gripper-action))
		     (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm)))))
     (dolist (client clients)
       (setq goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal))
     (if wait (send-all clients :wait-for-result))
     ))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 0.01) ((:objects objs) objects))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait t)
   (unless joint-action-enable
     (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
       (let ((grasp-convex
	      (convex-hull-3d
	       (flatten
		(mapcar
		 #'(lambda (l)
		     (send-all (send l :bodies) :worldcoords)
		     (send-all (send l :bodies) :vertices))
		 (send robot a :gripper :links))))))
	 (dolist (obj objs)
	   (when (and (find-method obj :faces)
		      (not (= (pqp-collision-check grasp-convex obj) 0)))
	     (if (send obj :parent) (send (send obj :parent) :dissoc obj))
	     (send robot a :end-coords :assoc obj)))))
     ;; (send self :update-robot-state) ;; update state of 'robot' for real robot
     (return-from :start-grasp
       (case arm
         (:larm (send robot :l_gripper_joint :joint-angle))
         (:rarm (send robot :r_gripper_joint :joint-angle))
         (t (list
             (send robot :l_gripper_joint :joint-angle)
             (send robot :r_gripper_joint :joint-angle))))))
   ;; for real robot
   (let ((clients (case arm
                    (:rarm (list (cons :r_gripper_joint r-gripper-action)))
                    (:larm (list (cons :r_gripper_joint l-gripper-action)))
                    (:arms (list (cons :r_gripper_joint r-gripper-action)
				 (cons :l_gripper_joint l-gripper-action)))
                    (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm))))
         aresult ajoint (resend-goal t) (resend-count 0) ret)
     (while (and resend-goal (< resend-count 2))
       (setq resend-goal nil)
       (send self :update-robot-state) ;; update state of 'robot' for real robot
       (dolist (client clients)
         (setq aresult (send (cdr client) :get-result)
               ajoint (send robot (car client)))
         (ros::ros-debug ";; :move-gripper check reached_goal ~A, position ~A, robot-body angle ~A" (send aresult :reached_goal) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
         (unless (send aresult :reached_goal) ;; in case that reached_gal is fail, try one more time
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A, unreached" (send ajoint :name) (* 1000 (send aresult :position)))
           (setq resend-goal t))
       ;;
         (unless (eps= (* 1000 (send aresult :position)) (send ajoint :joint-angle) 2)
           (ros::ros-debug ";; :move-gripper ~A need to resend goal, position = ~A/~A, result and udpate-body differs" (send ajoint :name) (* 1000 (send aresult :position)) (send ajoint :joint-angle))
           (setq resend-goal t)))
       (incf resend-count)
       (if resend-goal (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait t)))
     (setq ret
           (case arm
             (:rarm (* 1000 (send (send r-gripper-action :get-result) :position)))
             (:larm (* 1000 (send (send l-gripper-action :get-result) :position)))
             (:arms (mapcar #'(lambda (c) (* 1000 (send (send c :get-result) :position))) (list r-gripper-action l-gripper-action)))))
     (ros::ros-debug ";; :move-gripper ~A returns ~A" (send ajoint :name) ret)
     ret))
  (:stop-grasp
   (&optional (arm :arms) &key (wait nil))
   (send self :move-gripper arm 0.09 :wait wait)
   (unless joint-action-enable
     (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
       (dolist (obj (send robot a :end-coords :descendants))
	 (send robot arm :end-coords :dissoc obj))))
   t)
  ;;
  (:pr2-fingertip-callback
   (arm msg) ;; arm = :(r|l)arm-pressure
   (let ((pressure (list (send msg :l_finger_tip) (send msg :r_finger_tip))))
     (send self :set-robot-state1 arm pressure)))
  (:reset-fingertip
   ()
   (send self :spin-once)
   (setq finger-pressure-origin
	 (mapcar #'(lambda(k)(cons k (copy-seq (send self :state k))))
		 '(:rarm-pressure :larm-pressure))))
  (:finger-pressure
   (arm &key (zero nil))
   (setq arm (case arm (:rarm :rarm-pressure) (:larm :larm-pressure)))
   (let ((current (send self :state arm))
	 (origin (cdr (assoc arm finger-pressure-origin))))
     (if zero
	 (when (and current origin)
	   (mapcar #'v- current origin))
       current)))
  ;;
  (:go-stop (&optional (force-stop t))
   (when joint-action-enable
     (send move-base-action :cancel-all-goals)
     (prog1
         (send move-base-trajectory-action :cancel-all-goals)
       (if force-stop (send self :go-velocity 0 0 0)))
     ))
  (:move-to
   (coords &key (retry 10) (frame-id "/world") (wait-for-server-timeout 5))
   (let (ret (count 0) (tm (ros::time-now))
	     (map-to-frame (send *tfl* :lookup-transform "/map" frame-id (ros::time 0)))
             (goal (instance move_base_msgs::MoveBaseActionGoal :init)))
     (unless joint-action-enable
       (let ((orig-coords (send robot :copy-worldcoords)))
	 (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	     ((> curr-tm 1000))
	   (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
	   (if viewer (send self :draw-objects))))
       (return-from :move-to t))
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from :move-to))
     ;;
     (send goal :header :stamp tm)
     (send goal :goal :target_pose :header :stamp tm)
     (if map-to-frame
	 (progn
	   (send goal :goal :target_pose :header :frame_id "/map")
	   (send goal :goal :target_pose :pose
		 (ros::coords->tf-pose (send (send coords :copy-worldcoords) :transform map-to-frame :world))))
       (progn ;; fail to find "/map" to frame_id
	   (send goal :goal :target_pose :header :frame_id frame-id)
	   (send goal :goal :target_pose :pose (ros::coords->tf-pose coords))
	 ))
     (while (and (null ret) (<= (inc count) retry))
       (if (> count 0) ;; retry
	   (clear-costmap))
       (send goal :header :seq count)
       (send goal :goal :target_pose :header :seq count)
       (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send goal :goal :target_pose :pose :position)) (send goal :goal :target_pose :header :frame_id) count)
       (send move-base-action :send-goal goal)
       (send move-base-action :wait-for-result)
       (when (eq (send move-base-action :get-state)
                 actionlib_msgs::GoalStatus::*preempted*)
         (setq ret nil)
         (return))
       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
           (setq ret t)))
     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
     ;;
     (when ret
       (let (diff diff-len current-coords lret map-goal-coords)
         ;;
         (setq map-goal-coords
               (if (string= frame-id "/base_footprint")
                   (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                 (send (send *tfl* :lookup-transform "/map" frame-id (ros::time 0))
                       :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
         (setq lret (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 5))
         (ros::ros-warn ":move-to wait-for transform /map to /base_footprint -> ~A" lret)
         (when (null lret)
           (ros::ros-error ":move-to wait-for transform /map to /base_footprint failed")
           (return-from :move-to nil))
         (setq current-coords (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
         (setq diff (send current-coords :transformation map-goal-coords))
         (ros::ros-warn ":move-to current-coords  ~A" current-coords)
         (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
         (ros::ros-warn "          error-coords   ~A" diff)
         (ros::ros-warn "         target-coords   ~A" coords)
         ;;
         (dotimes (i 2)
           (if (< (setq diff-len (norm (subseq (send diff :worldpos) 0 2))) 200) ;; move_base thre = 200mm
               (let* ((msec (* diff-len 10))
                      (x (/ (elt (send diff :worldpos) 0) msec))
                      (y (/ (elt (send diff :worldpos) 1) msec))
                      (d (/ (elt (car (rpy-angle (send diff :worldrot))) 0) (/ msec 1000))))
                 (ros::ros-warn ":move-to -> :go-velocity x:~A y:~A d:~A msec:~A" x y d msec)
                 (unix:usleep (* 400 1000)) ;; 400ms ???
                 (let ((acret (send self :go-velocity x y d msec :wait t)))
                   (unless acret
                     (return-from :move-to nil)))
                 ;;(unix::usleep (* (round msec) 1000)) ;; why time wait
                 )
             (progn
               (ros::ros-error "too far from goal position ~A mm (> 200mm)" diff-len)
	       ;; move-to suceeded buf away from 200 mm
	       (ros::ros-error ":move-to try to send /move_base_simple/goal")
	       (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
	       (send goal :goal :target_pose :header :seq (1+ count))
	       (ros::publish "/move_base_simple/goal" (send goal :goal :target_pose))
	       (unix:sleep 3)
               (return-from :move-to nil)
               ))
           ;;
           (setq map-goal-coords
                 (if (string= frame-id "/base_footprint")
                     (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                   (send (send *tfl* :lookup-transform "/map" frame-id (ros::time 0))
                         :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
           (setq lret (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 5))
           (ros::ros-warn ":move-to wait-for transform /map to /base_footprint -> ~A" lret)
           (when (null lret)
             (ros::ros-error ":move-to wait-for transform /map to /base_footprint failed")
             (return-from :move-to nil))
           (setq current-coords (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
           (setq diff (send current-coords :transformation map-goal-coords))
           (ros::ros-warn ":move-to current-coords  ~A" current-coords)
           (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
           (ros::ros-warn "          error-coords   ~A" diff)
           (ros::ros-warn "         target-coords   ~A" coords)
           ) ;;  (do (i 2)
         ))
     ret))
  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id "/base_footprint")
     ))
  (:go-velocity
   (x y d ;; [m/sec] [m/sec] [rad/sec]
    &optional (msec 1000) ;; msec is total animation time [msec]
    &key (stop t) (wait))
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
	   (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x msec) (* y msec) 0)) ;; [m/sec] * [msec] = [mm]
       (send coords :rotate (deg2rad (/ (* d msec) 1000.0)) :z) ;; [rad/sec] * [msec] / 1000 = [rad]->(deg2rad)->[deg]
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	   ((> curr-tm msec))
	 (send robot :newcoords (midcoords (/ curr-tm (float msec)) orig-coords coords))
	 (if viewer (send self :draw-objects))))
     (return-from :go-velocity t))
   (let ((goal (send self :move-trajectory x y d msec :stop stop)))
     (prog1
         (send move-base-trajectory-action :send-goal goal)
       (if wait (send move-base-trajectory-action :wait-for-result)))
     ))
  (:go-pos-unsafe
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (ros::ros-info "go-pos-unsafe (x y d) = (~A ~A ~A)" x y d)
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
           (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x 1000) (* y 1000) 0))
       (send coords :rotate (deg2rad d) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
           ((> curr-tm 1000))
         (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
         (if viewer (send self :draw-objects))))
     (return-from :go-pos-unsafe t))
   (let (msec step goal (maxvel 0.295) (maxrad 0.495) (counter 0))
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
                             (/ (abs (deg2rad d)) (* maxrad 0.8)))))
     (setq msec (max msec 1000))
     (setq step (/ 1000.0 msec))
     (while (and (or (> (sqrt (+ (* x x) (* y y)))  0.025)
                     (> (abs d) 2.5) ;; 3 cm and 3deg
                     (= counter 0))  ;; try at least 1 time
                 (< counter 3)) ;; magic number 3 times
       (setq goal (send self :move-trajectory (* x step) (* y step)
                        (* (deg2rad d) step) msec :stop t))
       (send move-base-trajectory-action :send-goal goal)
       (let ((acret
              (send move-base-trajectory-action :wait-for-result)))
         (unless acret
           (return-from :go-pos-unsafe nil))
         (send move-base-trajectory-action :spin-once))
       (send self :spin-once)
       (send self :spin-once)
       (let ((goal-position (send (elt (send goal :goal :trajectory :points) 1) :positions)) ;; goal
             (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
             (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
             diff-position v)
         (setq diff-position (v- goal-position (v+ odom-pos (float-vector 0 0 odom-angle))))
         (setq v (v- (rotate-vector (float-vector (elt diff-position 0) (elt diff-position 1) 0)
                                    (- odom-angle) :z)
                     (float-vector 0 0 odom-angle)))
         (setq x (elt v 0)
               y (elt v 1)
               d (rad2deg (elt diff-position 2)))
         (if (> d (* 360 0.8)) (setq d (- d 360)))
         (if (< d (* -360 0.8)) (setq d (+ d 360)))
         (ros::ros-info "                diff-pos ~A ~A, diff-angle ~A" x y d)
         )
       (incf counter)
       ) ;; while
     t))
  (:move-trajectory
   (x y d &optional (msec 1000) &key (stop t)) ;; [m/sec] [m/sec] [rad/sec]
   (send self :spin-once)
   (let ((sec (/ msec 1000.0))
	 (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
	 (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
	 (msg (instance trajectory_msgs::JointTrajectory :init))
	 (goal (instance pr2_controllers_msgs::JointTrajectoryActionGoal :init))
	 (pt1 (instance trajectory_msgs::JointTrajectoryPoint :init))
	 (pt2 (instance trajectory_msgs::JointTrajectoryPoint :init)))
     (send pt1 :time_from_start (ros::time))
     (send pt2 :time_from_start (ros::time sec))
     (send pt1 :positions (v+ odom-pos
			      (float-vector 0 0 odom-angle)))
     (send pt2 :positions (v+ (v+ odom-pos
				  (float-vector 0 0 (+ odom-angle (* sec d))))
			     (rotate-vector (scale sec (float-vector x y 0))
					    odom-angle :z)))
     (send pt1 :velocities (rotate-vector (float-vector x y d) odom-angle :z))
     (if stop
	 (send pt2 :velocities (float-vector 0 0 0)) ;; To stop just
       (send pt2 :velocities (rotate-vector (float-vector x y d) odom-angle :z)))

     (ros::ros-debug "move-trajectory (x y d msec) = (~A ~A ~A ~A)" (* sec x) (* sec y) (* sec d) msec)
     (ros::ros-debug "                odom-pos ~A, odom-angle ~A" odom-pos odom-angle)
     (ros::ros-debug "                pt1 ~A" (send pt1 :positions))
     (ros::ros-debug "                pt2 ~A" (send pt2 :positions))
     ;;
     (send msg :header :stamp (ros::time-now))
     (send msg :joint_names (list "base_link_x" "base_link_y" "base_link_pan"))
	 (send msg :points (list pt1 pt2))
     (send goal :goal :trajectory msg)
     ;;
     goal))
  ;;
  ;;
  ;;
  (:wait-torso (&optional (timeout 0))
   (let ((act (find-if #'(lambda (x) (string= (send x :name) "torso_controller/follow_joint_trajectory"))
                       controller-actions)))
     (when act
       (send act :wait-for-result :timeout timeout))))
  )

;;
;;
;; workaround for unintentional 360 joint rotation problem [#91]
(defmethod pr2-interface
  (:check-continuous-joint-move-over-180
   (diff-av)
   (let ((i 0) add-new-trajectory-point)
     (dolist (j (send robot :joint-list))
       ;; for continuous rotational joint
       (when (and (> (- (send j :max-angle) (send j :min-angle)) 360)
                  (> (abs (elt diff-av i)) 180))
         (ros::ros-warn "continuous joint (~A) moves ~A degree, commanded joint differs from original trajectory to avoid unintentional 360 rotation" (send j :name) (elt diff-av i))
         (setq add-new-trajectory-point t))
       (incf i (send j :joint-dof)))
     add-new-trajectory-point))
  (:angle-vector
   (av &optional (tm 3000) &rest args)
   (let (diff-av)
     ;; use reference-vector to get last commanded joint and use :angle-vector to toruncate the joint limit to eus lisp style
     (setq diff-av (v- av (send robot :angle-vector (send self :state :reference-vector))))
     ;; use shortest path for contiuous joint
     ;;
     (when (send self :check-continuous-joint-move-over-180 diff-av)
       (return-from :angle-vector
         (send* self :angle-vector-sequence (list av) (list tm) args))) ;; when
     (send-super* :angle-vector av tm args)
     ))
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)) &rest args)
   (if (atom tms) (setq tms (make-list (length avs) :initial-element tms)))
   (if (< (length tms) (length avs))
       (setq tms (append tms (make-list (- (length avs) (length tms)) :initial-element (car (last tms))))))
   (unless (send self :simulation-modep)
     (let* ((prev-av (send robot :angle-vector (send self :state :reference-vector)))
            (len-av (length prev-av))
            (max-av (fill (instantiate float-vector len-av)  180))
            (min-av (fill (instantiate float-vector len-av) -180))
            diff-av (l 0) dist div)
       (dolist (av avs)
         ;; check if joint move more than 180 degree which has -inf/inf limit
         (setq diff-av (v- av prev-av))
         (when (send self :check-continuous-joint-move-over-180 diff-av)
           (setq dist (abs (geo::find-extream (coerce diff-av cons) #'abs #'>=)))
           (setq div (round (/ dist 120.0)))
           (ros::ros-warn "original trajectory command :")
           (ros::ros-warn "                       : ~A ~A" avs tms)
           (ros::ros-warn "  current angle vector : ~A" prev-av)
           (ros::ros-warn "new trajectory command : dvi = ~A" div)
           (dotimes (i div)
             (ros::ros-warn "                       : ~A ~A" (midpoint (/ (+ i 1.0) div) prev-av av) (/ (elt tms l) div)))
           (dotimes (i (1- div))
             (setq avs (list-insert (midpoint (/ (+ i 1.0) div) prev-av av) (+ l i) avs)))
           (setf (elt tms l) (/ (elt tms l) div))
           (dotimes (i (1- div))
             (setq tms (list-insert (elt tms l) l tms)))
           (incf l (1- div))
           (ros::ros-warn "new trajectory command :")
           (ros::ros-warn "                       : ~A ~A" avs tms)
           )
         ;;
         (setq prev-av av)
         (incf l))
       )) ;; when simulation-modep
     (send-super* :angle-vector-sequence avs tms args))
  ) ;; pr2-interface

(defmethod pr2-interface
  (:angle-vector-with-constraint
   (av1 &optional (tm 3000) (arm :arms) &key (rotation-axis t) (translation-axis t) &rest args)
   (let (av0 c0 c1 avs tms ret arm-av pav)
     (setq pav (send robot :angle-vector))
     (unless (memq arm '(:larm :rarm :arms))
       (warning-message 1 ":angle-vector-with-constraint arm=~A is not supported, use :larm or :rarm~%" arm)
       (return-from :angle-vector-with-constraint nil))
     (send robot :angle-vector av1)
     (cond ((memq arm '(:larm :rarm))
            (setq c1 (send robot arm :end-coords :copy-worldcoords)))
           (t
            (setq c1 (cons (send robot :larm :end-coords :copy-worldcoords)
                           (send robot :rarm :end-coords :copy-worldcoords)))))
     (setq av0 (send robot :angle-vector (send self :state :reference-vector)))
     (cond ((memq arm '(:larm :rarm))
            (setq c0 (send robot arm :end-coords :copy-worldcoords))
            (setq arm-av (send robot arm :angle-vector)))
           (t
            (setq c0 (cons (send robot :larm :end-coords :copy-worldcoords)
                           (send robot :rarm :end-coords :copy-worldcoords)))
            (setq arm-av (cons (send robot :larm :angle-vector)
                               (send robot :rarm :angle-vector)))))
     (dotimes (i 10)
       (send robot :angle-vector (midpoint (/ (1+ i) 10.0) av0 av1))
       (cond ((memq arm '(:larm :rarm))
              (setq arm-av (send robot arm :angle-vector arm-av)))
             (t
              (setq arm-av (list (send robot :larm :angle-vector (car arm-av))
                                 (send robot :rarm :angle-vector (cdr arm-av))))))
       (cond ((memq arm '(:larm :rarm))
              (setq ret (send robot arm :inverse-kinematics (midcoords (/ (1+ i) 10.0) c0 c1)
                              :rotation-axis rotation-axis :translation-axis translation-axis)))
             (t
              (setq ret
                    (and (send robot :larm :inverse-kinematics (midcoords (/ (1+ i) 10.0) (car c0) (car c1))
                               :rotation-axis rotation-axis :translation-axis translation-axis)
                         (send robot :rarm :inverse-kinematics (midcoords (/ (1+ i) 10.0) (cdr c0) (cdr c1))
                               :rotation-axis rotation-axis :translation-axis translation-axis)))))
       (unless ret
         (warning-message 1 ":angle-vector-with-constraint ik failed~%")
         (send robot :angle-vector pav)
         (return-from :angle-vector-with-constraint nil))
       (cond ((memq arm '(:larm :rarm))
              (setq arm-av (send robot arm :angle-vector)))
             (t
              (setq arm-av (cons (send robot :larm :angle-vector)
                                 (send robot :rarm :angle-vector)))))
       (push (send robot :angle-vector) avs)
       (push (/ tm 10.0) tms))
     (setq avs (reverse avs))
     (send-super :angle-vector-sequence avs tms) ;; avoid :check-continuous-joint-move-over-180
     )) ; :angle-vector-with-constraint
  ) ;; defmethod pr2-interface

   

;;;;;
;;;;; utility functions pr2 robot
;;;;;
(defun pr2-init (&optional (create-viewer))
  ;; env
  (unless (boundp '*pr2*) (pr2))
  (unless (ros::ok) (ros::roseus "pr2_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;(send *pr2* :move-to (send *ri* :state :worldcoords) :world)

  (when create-viewer (objects (list *pr2*)))
  )

;; (list larm-v rarm-v)
(defconstant *pr2-tuckarm-pose-rarm-free* (list #f(5 70 105 -90 -70 -6 -20)
                                                #f(0 60 -80 -121 -15 -90 20)))
(defconstant *pr2-tuckarm-pose-larm-free* (list #f(0 60 80 -121 15 -90 -20)
                                                #f(-5 70 -105 -90 70 -6 20)))

;; check if current pose is tuckarm pose or not
;;       which arm is free
;; args = () (:rarm) (:larm)
;; return = nil :rarm :larm
(defun check-tuckarm-pose (&key (thre 20) &rest args)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((l-angle (map float-vector #'(lambda(d)(- d (* 360 (round (/ d 360.0)))))
                      (send *pr2* :larm :angle-vector)))
        (r-angle (map float-vector #'(lambda(d)(- d (* 360 (round (/ d 360.0)))))
                      (send *pr2* :rarm :angle-vector)))
	(weight #f(5 2 1 0.5 0.1 0 0))
        is-rarm is-larm)
    (setq is-larm (and (< (norm (map float-vector #'* (v- l-angle (car *pr2-tuckarm-pose-larm-free*)) weight)) thre)
                       (< (norm (map float-vector #'* (v- r-angle (cadr *pr2-tuckarm-pose-larm-free*)) weight)) thre)))
    (setq is-rarm (and (< (norm (map float-vector #'* (v- l-angle (car *pr2-tuckarm-pose-rarm-free*)) weight)) thre)
                       (< (norm (map float-vector #'* (v- r-angle (cadr *pr2-tuckarm-pose-rarm-free*)) weight)) thre)))
    (cond ((and (memq :rarm args) is-rarm) :rarm)
          ((and (memq :larm args) is-larm) :larm)
          (is-rarm :rarm)
          (is-larm :larm))
    ))

;; send pr2 to move to tuckarm pose if not the pose now
;; args is set the arm to move freely
(defun pr2-tuckarm-pose (&rest args)
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (free-arm (or (car args) current-arm :larm))
         (msec 500))
    (when (not (eq current-arm free-arm))
      (progn
        (setq msec 2000)
        (send *pr2* :larm :angle-vector #f( 25 0 0 -121 0 -6 0))
        (send *pr2* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        ))
    (if (eq free-arm :larm)
        (progn
          (send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-larm-free*))
          (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
        (send *pr2* :larm :angle-vector (car *pr2-tuckarm-pose-rarm-free*))
        (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec)))
    (send *ri* :wait-interpolation)
    (if (eq free-arm :larm)
        (progn
          (send *pr2* :larm :angle-vector (car *pr2-tuckarm-pose-larm-free*))
          (send *ri* :larm-angle-vector (send *pr2* :angle-vector) msec))
      (progn
        (send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-rarm-free*))
        (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) msec)))
    (send *ri* :wait-interpolation)
    t
    ))

;; send pr2 to move to reset pose
(defun pr2-reset-pose ()
  (let ()
    (send *pr2* :reset-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ))

;;
;; navigation-client.l
;;

;; reset local costmap and clear unknown grid around robot
(defun clear-costmap ()
  (call-empty-service "/move_base_node/clear_costmaps")
  (call-empty-service "/move_base_node/clear_unknown_space"))

;; change inflation range of local costmap
(defun change-inflation-range (&optional (range 0.55))
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config :doubles
	  (list (instance dynamic_reconfigure::DoubleParameter :init
			  :name "inflation_radius" :value range)))
    (ros::service-call "/move_base_node/local_costmap/set_parameters" req)
    ))

;; do not use tilt laser
(defun use-tilt-laser-obstacle-cloud (enable)
  ;; check if we have service call
  (let ((req (instance topic_tools::MuxSelectRequest :init
                       :topic (if enable "ground_object_cloud" "empty_cloud"))))
    (ros::service-call "/tilt_laser_mux/select" req)))
