;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")
(require :robot-interface "package://pr2eus/robot-interface.l")

;;;
;;; pr2 robot interface
;;;

(ros::load-ros-manifest "pr2eus")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "nav_msgs")
(ros::roseus-add-msgs "sound_play")
(load "speak.l")

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
   (send-super :init :robot pr2-robot :type type
               :groupname "pr2_interface")
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
      (send-super :state :worldcoords (or (cadr args) "/world")))
     (t
      (send-super* :state args))
     ))
  (:publish-joint-state () ;; overwrite for pr2
   (send-super :publish-joint-state (append (send robot :joint-list) (send robot :larm :gripper :joint-list) (send robot :rarm :gripper :joint-list))))
  ;;
  (:wait-interpolation ;; overwrite for pr2
   ()
   (unless joint-action-enable (return-from :wait-interpolation nil))
   (let (result)
	 (setq result (send-all controller-actions :wait-for-result))
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
	 (cons :controller-action "l_arm_controller/joint_trajectory_action")
	 (cons :controller-state "l_arm_controller/state")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "l_shoulder_pan_joint"
							  "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
							  "l_elbow_flex_joint" "l_forearm_roll_joint"
							  "l_wrist_flex_joint" "l_wrist_roll_joint")))))
  (:rarm-controller
   ()
   (list
	(list
	 (cons :controller-action "r_arm_controller/joint_trajectory_action")
	 (cons :controller-state "r_arm_controller/state")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "r_shoulder_pan_joint"
							  "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
							  "r_elbow_flex_joint" "r_forearm_roll_joint"
							  "r_wrist_flex_joint" "r_wrist_roll_joint")))))
  (:head-controller
   ()
   (list
	(list
	 (cons :controller-action "head_traj_controller/joint_trajectory_action")
	 (cons :controller-state "head_traj_controller/state")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:torso-controller
   ()
   (list
	(list
	 (cons :controller-action "torso_controller/joint_trajectory_action")
	 (cons :controller-state "torso_controller/state")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
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
  (:controller-angle-vector
   (av tm type)
   (send self :angle-vector av tm
		 :controller-actions (mapcar #'(lambda (param) (find (cdr (assoc :controller-action param)) controller-actions :key #'(lambda (x) (send x :name)) :test #'string=)) (send self type))
		 :controller-type type))
  (:larm-angle-vector (av tm)
   (send self :controller-angle-vector av tm :larm-controller))
  (:rarm-angle-vector (av tm)
   (send self :controller-angle-vector av tm :rarm-controller))
  (:head-angle-vector (av tm)
   (send self :controller-angle-vector av tm :head-controller))
  ;;
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
   (unless joint-action-enable
     (send robot arm :gripper :joint-angle (* pos 1000))
     (send self :publish-joint-state)
     (if viewer (send viewer :draw-objects))
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
   (&optional (arm :arms) &key ((:gain g) 0.01))
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
	 (dolist (obj objects)
	   (when (and (find-method obj :faces)
		      (not (= (pqp-collision-check grasp-convex obj) 0)))
	     (if (send obj :parent) (send (send obj :parent) :dissoc obj))
	     (send robot a :end-coords :assoc obj)))))
     (return-from :start-grasp nil))
   (send self :update-robot-state) ;; update state of 'robot' 
   (case arm
	 (:larm (send robot :l_gripper_joint :joint-angle))
	 (:rarm (send robot :r_gripper_joint :joint-angle))
	 (t (list 
	     (send robot :l_gripper_joint :joint-angle)
	     (send robot :r_gripper_joint :joint-angle)))))
  (:stop-grasp
   (&optional (arm :arms))
   (send self :move-gripper arm 0.09 :wait nil)
   (unless joint-action-enable
     (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
       (dolist (obj (send robot a :end-coords :descendants))
	 (send robot arm :end-coords :dissoc obj)))))
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
  (:move-to
   (coords &key (retry 10) (frame-id "/world") (wait-for-server-timeout 5))
   (let (ret (count 0) (tm (ros::time-now))
	     (pose (ros::coords->tf-pose coords))
             (goal (instance move_base_msgs::MoveBaseActionGoal :init))
	     (map-goal-coords))
     (unless joint-action-enable
       (let ((orig-coords (send robot :copy-worldcoords)))
	 (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	     ((> curr-tm 1000))
	   (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
	   (if viewer (send viewer :draw-objects))))
       (return-from :move-to t))
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from :move-to))
     ;;
     (send goal :header :stamp tm)
     (send goal :goal :target_pose :header :stamp tm)
     (send goal :goal :target_pose :header :frame_id frame-id)
     (send goal :goal :target_pose :pose pose)
     (while (and (null ret) (<= (inc count) retry))
       (if (> count 0) ;; retry
	   (clear-costmap))
       (ros::ros-info "move-to : send-goal to ~A (~d)" (send coords :worldpos) count)
       (send move-base-action :send-goal goal)
       (send move-base-action :wait-for-result)
       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
           (setq ret t)))
     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
     ;;
     (setq map-goal-coords (send (send *tfl* :lookup-transform "/map" frame-id (ros::time 0)) :transform (send coords :copy-worldcoords))) ;; goal-coords in /map coordinates
     (when ret
       (let (diff tmp msec current-coords ret x y d)
	 (dotimes (i 2)
	   (setq ret (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 5))
	   (ros::ros-warn ":move-to wait-for transform /map to /base_footprint -> ~A" ret)
	   (when (null ret)
	     (ros::ros-error ":move-to wait-for transform /map to /base_footprint failed")
	     (return-from :move-to nil))
	   (setq current-coords (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
	   (ros::ros-warn ":move-to current-coords  ~A" current-coords)
	   (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
	   (ros::ros-warn "         coords          ~A" coords)
	   (setq diff (send current-coords :transformation map-goal-coords))
	   (if (< (norm (subseq (send diff :worldpos) 0 2)) 200) ;; move_base thre = 200mm
	       (progn
		 (setq msec (* (norm (subseq (send diff :worldpos) 0 2)) 10)
		       x (/ (elt (send diff :worldpos) 0) msec)
		       y (/ (elt (send diff :worldpos) 1) msec)
		       d (/ (elt (car (rpy-angle (send diff :worldrot))) 0) (/ msec 1000)))
		 (ros::ros-warn ":move-to -> :go-velocity x:~A y:~A d:~A msec:~A" x y d msec)
		 (unix:sleep 1) ;; ???
		 (send self :go-velocity x y d msec)
		 (unix::usleep (* (round msec) 1000)))
	     (ros::ros-error "too far from goal position (> 200mm)"))
	   (send *tfl* :wait-for-transform "/map" "/base_footprint" (ros::time-now) 5)
	   (setq current-coords (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
	   (setq diff (send current-coords :transformation map-goal-coords))
	   (ros::ros-warn ":move-to goal difference ~A" diff)
	   )
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
   (x y d &optional (msec 1000) &key (stop t)) ;; [m/sec] [m/sec] [rad/sec]
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
	   (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x msec) (* y msec) 0))
       (send coords :rotate (deg2rad (/ (* d msec) 1000.0)) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	   ((> curr-tm 1000))
	 (send robot :newcoords (midcoords (/ curr-tm 1000.0) orig-coords coords))
	 (if viewer (send viewer :draw-objects))))
     (return-from :go-velocity t))
   (let ((goal (send self :move-trajectory x y d msec :stop stop)))
     (send move-base-trajectory-action :send-goal goal)
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
	 (if viewer (send viewer :draw-objects))))
     (return-from :go-pos-unsafe t))
   (let (msec step goal (maxvel 0.3) (maxrad 0.5))
     ;; package://pr2_base_trajectory_action/config/pr2_base_link.yaml
     ;; 80% of maxvel = 0.3[m/sec]
     ;; 80% of maxrad = 0.5[rad/sec]
     (setq msec (* 1000 (max (/ (norm (float-vector x y)) (* maxvel 0.8))
			     (/ (abs (deg2rad d)) (* maxrad 0.8)))))
     (setq msec (max msec 1000))
     (setq step (/ 1000.0 msec))
     (while (or (> (sqrt (+ (* x x) (* y y)))  0.03) (> d 3)) ;; 3 cm and 3deg
       (setq goal (send self :move-trajectory (* x step) (* y step) (* (deg2rad d) step) msec :stop t))
       (send move-base-trajectory-action :send-goal goal)
       (send move-base-trajectory-action :wait-for-result)
       (send self :spin-once)
       (send self :spin-once)
       (let ((goal-position (send (elt (send goal :goal :trajectory :points) 1) :positions)) ;; goal
             (odom-pos (scale 0.001 (send (send self :state :odom :pose) :pos)))
             (odom-angle (elt (car (send (send self :state :odom :pose) :rpy-angle)) 0))
             diff-position v)
         (setq diff-position (v- goal-position (v+ odom-pos (float-vector 0 0 odom-angle))))
         (setq v (v- (rotate-vector (float-vector (elt diff-position 0) (elt diff-position 1) 0) (- odom-angle) :z)
                     (float-vector 0 0 odom-angle)))
         (setq x (elt v 0) y (elt v 1) d (rad2deg (elt diff-position 2)))
         (ros::ros-info "                diff-pos ~A ~A, diff-angle ~A" x y d)
         ) ;; let
       ) ;; dotimes
     ))
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

     (ros::ros-info "move-trajectory (x y d msec) = (~A ~A ~A ~A)" (* sec x) (* sec y) (* sec d) msec)
     (ros::ros-info "                odom-pos ~A, odom-angle ~A" odom-pos odom-angle)
     (ros::ros-info "                pt1 ~A" (send pt1 :positions))
     (ros::ros-info "                pt2 ~A" (send pt2 :positions))
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
  )

;;;;;
;;;;; utility functions pr2 robot
;;;;;
(defun setup-for-pr2 (&optional (create-viewer))
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
(defconstant *pr2-tuckarm-pose-rarm-free* (list #f( 5 70  105  -90 -70   -6  -20)
                                                #f( 0 60  -80 -121 -50 -114 -160)))
(defconstant *pr2-tuckarm-pose-larm-free* (list #f( 0 60   80 -121  50 -114  160)
                                                #f(-5 70 -105  -90  70   -6   20)))

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
		(send *ri* :wait-interpolation)))
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

(defun clear-costmap ()
  (call-empty-service "/move_base_node/clear_costmaps")
  (call-empty-service "/move_base_node/clear_unknown_space"))

(defun change-inflation-range (&optional (range 0.55))
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config :doubles
	  (list (instance dynamic_reconfigure::DoubleParameter :init
			  :name "inflation_radius" :value range)))
    (ros::service-call "/move_base_node/local_costmap/set_parameters" req)
    ))
