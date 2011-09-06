;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")

;;;
;;; interface to ros based pr2 system
;;;

(require :eustf "package://roseus/euslisp/eustf.l")
(require :actionlib "package://roseus/euslisp/actionlib.l")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "pr2_msgs")
(ros::roseus-add-msgs "nav_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")
(ros::roseus-add-msgs "sound_play")

(defclass ros-interface
  :super propertied-object
  :slots (robot robot-state joint-action-enable
		controller-type controller-actions
		viewer))

(defmethod ros-interface
  (:init
   (&rest args &key ((:robot r)) (type :default-controller) &allow-other-keys)
   (setq joint-action-enable t)
   (setq robot (instance r :init))
   ;;
   (ros::subscribe "/joint_states" sensor_msgs::JointState
		   #'send self :ros-state-callback)

   (setq controller-type type)
   (mapcar
    #'(lambda (param)
	(let* ((controller (cdr (assoc :controller param)))
	       (action-type (cdr (assoc :action-type param)))
	       (action (instance ros::simple-action-client :init controller action-type)))
	  (push action controller-actions)))
    (send self controller-type))
   (nreverse controller-actions)

   (dolist (action controller-actions)
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name) )
       (return)))
   ;;
   (unless joint-action-enable
     (let ((old-viewer user::*viewer*))
       (when (and x::*display* (> x::*display* 0))
	 (setq viewer (instance x::irtviewer :create :title (format nil "~A-interface" (send robot :name))))
	 (send viewer :objects (list robot))
	 (send viewer :look-all)
	 (if old-viewer (setq user::*viewer* old-viewer)))

       (ros::advertise "/joint_states" sensor_msgs::JointState 1)
       ))
   self)
  ;;
  (:publish-joint-state ;; for simulatio mode (joint-action-enable is nil)
   ()
   (let ((msg (instance sensor_msgs::JointState :init)) names positions velocities efforts)
     (send msg :header :stamp (ros::time-now))
     (dolist (j (send robot :joint-list))
       (push (string-downcase (symbol-name (send j :name))) names)
       (push (cond ((derivedp j rotational-joint)
		    (deg2rad (send j :joint-angle)))
		   (t
		    (* 0.001 (send j :joint-angle))))
	     positions)
       (push 0 velocities)
       (push 0 efforts))
     (send msg :name names)
     (send msg :position positions)
     (send msg :velocity velocities)
     (send msg :effort efforts)
     (ros::publish "/joint_states" msg)))
  (:angle-vector
   (av &optional (tm 3000) &key ((:controller-actions ca) controller-actions) ((:controller-type ct) controller-type))
   ;; for simulation mode
   (unless joint-action-enable
     (if av
       (let* ((prev-av (send robot :angle-vector))
	      (scale-av (send self :sub-angle-vector av prev-av)))
	 (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	     ((>= curr-tm tm))
	   (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	   (send self :publish-joint-state)
	   (if viewer (send viewer :draw-objects))))))

   (send robot :angle-vector av)
   (mapcar
    #'(lambda (action param)
        (send self :send-ros-controller
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              0					      ;; start time
              (list
	       (list av				      ;; positions
		     (instantiate float-vector (length av))  ;; velocities
		     (/ tm 1000.0)))))			      ;; duration
	ca (send self ct))
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)))
   (ros::spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
         (offset (instantiate float-vector (length (send robot :angle-vector))))
         tm tm-next
         (vel (instantiate float-vector (length (car avs)))))
       (prog1 ;; angle-vector-sequence returns avs
	   avs
	 (while avs
	   (setq av (pop avs))
	   (if (car tms) (setq tm (pop tms)))
	   (if (setq av-next (car avs))
	       (let ((v0 (send self :sub-angle-vector av av-prev))
		     (v1 (send self :sub-angle-vector av-next av)))
		 (setq tm-next (if (car tms) (car tms) tm))
		 (dotimes (i (length vel))
		   (setf (elt vel i)
			 (if (>= (* (elt v0 i) (elt v1 i)) 0)
			     (* 0.5 (+ (* (/ 1000.0 tm) (elt v0 i))
				       (* (/ 1000.0 tm-next) (elt v1 i))))
			   0.0)))
		 )
	     (fill vel 0))
	   ;; for simulation mode
	   (unless joint-action-enable
	     (let* ((prev-av (send robot :angle-vector))
		    (scale-av (send self :sub-angle-vector av prev-av)))
	       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
		   ((>= curr-tm tm))
		 (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
		 (send self :publish-joint-state)
		 (if viewer (send viewer :draw-objects)))))
	   ;;
	   (send robot :angle-vector av)

	   ;; add offset for rotational joint
	   ;; eus: -180 - 180
	   ;; joit trajectory points : -inf - + inf
	   (when av-prev
	     (let ((joint-list (send robot :joint-list)) (av-diff (v- av av-prev)) (i 0) j)
	       (while (setq j (pop joint-list))
		 (when (and (= (send j :min-angle) *-inf*) (= (send j :max-angle) *inf*))
		   (cond ((> (elt av-diff i)  180.0)
			  (incf (elt offset i) -360.0))
			 ((< (elt av-diff i) -180.0)
			  (incf (elt offset i)  360.0)
			  )))
		 (incf i)
		 )))
	   (push (list (v+ av offset)  ;; positions
		       (copy-seq vel)  ;; velocities
		       (+ (/ (+ st tm) 1000.0) 0.2)) ;; tm + duration
		 traj-points)
	   (setq av-prev av)
	   (incf st tm))
	 ;;
	 (mapcar
	  #'(lambda (action param)
	      (send self :send-ros-controller
		    action (cdr (assoc :joint-names param)) ;; action server and joint-names
		    0.1   ;; start time
		    traj-points))
	  controller-actions (send self controller-type))
	 )))
  (:wait-interpolation
   ()
   (unless joint-action-enable (return-from :wait-interpolation nil))
   (send-all controller-actions :wait-for-result))
  ;;
  ;;
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (unless joint-action-enable (return-from :send-ros-controller nil))
   (let* ((goal (send action :make-goal-instance))
	  (goal-points nil)
	  (st (ros::time+ (ros::time-now) (ros::time starttime)))
	  (joints (mapcar #'(lambda (x)
			      (send robot (intern (string-upcase x) *keyword-package*)))
			  joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp st)
     (dolist (trajpt trajpoints)
       (let* ((all-positions (elt trajpt 0))
	      (all-velocities (elt trajpt 1))
	      (duration (elt trajpt 2))
	      (positions (instantiate float-vector (length joint-names)))
	      (velocities (instantiate float-vector (length joint-names))))
	 (dotimes (i (length joints))
	   (let* ((joint (elt joints i))
		  (id (position joint (send robot :joint-list)))
		  p v)
	     (setq p (elt all-positions id)
		   v (elt all-velocities id))
	     (cond
	      ((derivedp joint rotational-joint)
	       (setq p (deg2rad p))
	       (setq v (deg2rad v)))
	      (t
	       (setq p (* 0.001 p))
	       (setq v (* 0.001 v))))
	     (setf (elt positions i) p)
	     (setf (elt velocities i) v)))
	 (push (instance trajectory_msgs::JointTrajectoryPoint
			 :init
			 :positions positions
			 :velocities velocities
			 :time_from_start (ros::time duration))
	       goal-points)
	 ))
     (ros::spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send action :send-goal goal)
     ))
  ;;
  (:ros-state-callback
   (msg)
   (dolist (key '(:name :position :velocity :effort))
     (if (assoc key robot-state)
	 (setf (cdr (assoc key robot-state)) (send msg key))
       (push (cons key (send msg key)) robot-state))))
  (:update-robot-state
   ()
   (let (joint-names positions efforts)
     (unless joint-action-enable
       (return-from :update-robot-state (send robot :angle-vector)))
     (unless robot-state (return-from :update-robot-state))
     (ros::spin-once)
     (setq joint-names (cdr (assoc :name robot-state))
	   positions (cdr (assoc :position robot-state))
	   velocities (cdr (assoc :velocity robot-state))
	   efforts (cdr (assoc :effort robot-state)))
     (mapcar #'(lambda (n p v e)
		 (let ((j (send robot (intern (string-upcase n) *keyword-package*))))
		   (when j
		     (cond
		      ;; position
		      ((derivedp j rotational-joint)
		       (send j :joint-angle (rad2deg p)))
		      (t
		       (send j :joint-angle (* 1000 p))))
			 ;; velocity
		     (send j :joint-velocity v)
		     ;; effort
		     (send j :joint-torque e))))
	     (coerce joint-names cons)
	     (coerce positions cons)
		 (coerce velocities cons)
	     (coerce efforts cons))))
  (:state
   (&rest args)
   (case (car args)
     ((:angle-vector nil)
      (ros::ros-warn "please use :state :potentio-vector~%")
      (send self :state :potentio-vector))
     (:potentio-vector
      (send self :update-robot-state)
      (send robot :angle-vector))
     (:torque-vector
      (send self :update-robot-state)
      (send robot :torque-vector))
     (t
      (let ((cur robot-state))
	(dolist (key args cur)
	  (setq cur (cdr (assoc key cur))))))
     ))
  ;;
  (:default-controller
   ()
   (list
    (list
     (cons :controller "fullbody_controller/joint_trajectory_action")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (symbol-name n)) (send-all (send robot :joint-list) :name))))))
  ;;
  (:sub-angle-vector (v0 v1)
   (let ((ret (v- v0 v1))
	 (joint-list (send robot :joint-list))
	 (i 0) j)
     (while (setq j (pop joint-list))
       (if (and (= (send j :min-angle) *-inf*) (= (send j :max-angle) *inf*))
	   (cond ((> (elt ret i) 180.0)
		  (setf (elt ret i) (- (elt ret i) 360.0)))
		 ((< (elt ret i) -180.0)
		  (setf (elt ret i) (+ (elt ret i) 360.0)))))
       (incf i))
     ret))
  ;;
  (:robot  (&rest args) (forward-message-to robot args))
  (:viewer (&rest args) (forward-message-to viewer args))
  ) ;; ros-interface

(defclass pr2-interface
  :super ros-interface
  :slots (r-gripper-action l-gripper-action
			   move-base-action finger-pressure-origin))
(defmethod pr2-interface
  (:init
   (&rest args &key (type :default-controller)
          (move-base-action-name "move_base")
		  &allow-other-keys)
   ;;
   (ros::subscribe "/base_odometry/odom" nav_msgs::Odometry
		   #'send self :pr2-odom-callback)
   (ros::subscribe "/pressure/r_gripper_motor" pr2_msgs::PressureState
		   #'send self :pr2-fingertip-callback :rarm-pressure)
   (ros::subscribe "/pressure/l_gripper_motor" pr2_msgs::PressureState
		   #'send self :pr2-fingertip-callback :larm-pressure)
   ;;
   (setq r-gripper-action (instance ros::simple-action-client :init
				    "/r_gripper_controller/gripper_action"
                    pr2_controllers_msgs::Pr2GripperCommandAction))
   (setq l-gripper-action (instance ros::simple-action-client :init
				    "/l_gripper_controller/gripper_action"
                    pr2_controllers_msgs::Pr2GripperCommandAction))
   (setq move-base-action (instance ros::simple-action-client :init
                    move-base-action-name move_base_msgs::MoveBaseAction))
   (ros::advertise "/base_controller/joint_trajectory_action/goal"
                    pr2_controllers_msgs::JointTrajectoryActionGoal 10)

   ;; wait for pr2-action server (except move_base)
   (setq joint-action-enable t)
   (dolist (action (list r-gripper-action l-gripper-action))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, pr2-interface is disabled" action)
       (return)))

   (send-super :init :robot pr2-robot :type type)
   )
  ;;
  (:pr2-odom-callback
   (msg)
   (let ((parsed
	  (list
	   (list :stamp (send msg :header :stamp))
	   (list :pose (ros::tf-pose->coords (send msg :pose :pose)))
	   (list :velocity (float-vector
			    (* 1000 (send msg :twist :twist :linear :x))
			    (* 1000 (send msg :twist :twist :linear :y))
			    (send msg :twist :twist :angular :z))))))
     (if (assoc :odom robot-state)
	 (dolist (k parsed)
	   (setf (cdr (assoc (car k) (cdr (assoc :odom robot-state)))) (cadr k)))
       (push (push :odom parsed) robot-state))))
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
	 (cons :controller "l_arm_controller/joint_trajectory_action")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "l_shoulder_pan_joint"
							  "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
							  "l_elbow_flex_joint" "l_forearm_roll_joint"
							  "l_wrist_flex_joint" "l_wrist_roll_joint")))))
  (:rarm-controller
   ()
   (list
	(list
	 (cons :controller "r_arm_controller/joint_trajectory_action")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "r_shoulder_pan_joint"
							  "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
							  "r_elbow_flex_joint" "r_forearm_roll_joint"
							  "r_wrist_flex_joint" "r_wrist_roll_joint")))))
  (:head-controller
   ()
   (list
	(list
	 (cons :controller "head_traj_controller/joint_trajectory_action")
	 (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
	 (cons :joint-names (list "head_pan_joint" "head_tilt_joint")))))
  (:torso-controller
   ()
   (list
	(list
	 (cons :controller "torso_controller/joint_trajectory_action")
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
     (cons :controller "midbody_controller/joint_trajectory_action")
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
     (cons :controller "fullbody_controller/joint_trajectory_action")
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
		 :controller-actions (mapcar #'(lambda (param) (find (cdr (assoc :controller param)) controller-actions :key #'(lambda (x) (send x :name)) :test #'string=)) (send self type))
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
		     (:arms (list r-gripper-action l-gripper-action)))))
     (dolist (client clients)
       (setq goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal))
     (if wait (mapcar #'(lambda(c)(send c :wait-for-result)) clients))
     ))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 0.01))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait t)
   (send self :state)
   (case arm
	 (:larm (send robot :l_gripper_joint :joint-angle))
	 (:rarm (send robot :r_gripper_joint :joint-angle))
	 (t (list 
	     (send robot :l_gripper_joint :joint-angle)
	     (send robot :r_gripper_joint :joint-angle)))))
  (:stop-grasp
   (&optional (arm :arms))
   (send self :move-gripper arm 0.09 :wait nil))
  ;;
  (:pr2-fingertip-callback
   (arm msg) ;; arm = :(r|l)arm-pressure
   (let ((pressure (list (send msg :l_finger_tip) (send msg :r_finger_tip))))
     (if (assoc arm robot-state)
	 (setf (cdr (assoc arm robot-state)) pressure)
       (push (cons arm pressure) robot-state))))
  (:reset-fingertip
   ()
   (ros::spin-once)
   (setq finger-pressure-origin
	 (mapcar #'(lambda(k)(copy-seq (assoc k robot-state)))
		 '(:rarm-pressure :larm-pressure))))
  (:finger-pressure
   (arm &key (zero nil))
   (setq arm (case arm (:rarm :rarm-pressure) (:larm :larm-pressure)))
   (let ((current (cdr (assoc arm robot-state)))
	 (origin (cdr (assoc arm finger-pressure-origin))))
     (if zero
	 (when (and current origin)
	   (mapcar #'v- current origin))
       current)))
  ;;
  (:move-to
   (coords &key (retry 10) (frame-id "/map") (wait-for-server-timeout 5))
   (let (ret (count 0) (pose (ros::coords->tf-pose coords))
             (goal (instance move_base_msgs::MoveBaseActionGoal :init)))
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (return-from :move-to))
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
     ret))
  (:go-pos
   (x y &optional (d 0)) ;; [m] [m] [degree]
   (let (c)
     (setq c (make-coords :pos (float-vector (* 1000 x) (* y 1000) 0)
			  :rpy (float-vector (deg2rad d) 0 0)))
     (send self :move-to c :retry 1 :frame-id "/base_link")
     ))
  (:go-velocity
   (x y d &optional (msec 1000) &key (stop t)) ;; [m/sec] [m/sec] [rad/sec]
   (ros::spin-once)
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
     ;;
     (send msg :header :stamp (ros::time-now))
     (send msg :joint_names (list "base_link_x" "base_link_y" "base_link_pan"))
	 (send msg :points (list pt1 pt2))
     (send goal :goal :trajectory msg)
     ;;
     (ros::publish "/base_controller/joint_trajectory_action/goal" goal) ;; this is actually actionlib
     ))
  ;;
  ;;
  ;;
  )

;;;;;
;;;;; utility functions pr2 robot
;;;;;

;; (list larm-v rarm-v)
(defconstant *pr2-tuckarm-pose-rarm-free* (list #f(5 75 105 -90 -70 0 20)
                                                #f(0 65 -80 -125 -50 -120 160)))
(defconstant *pr2-tuckarm-pose-larm-free* (list #f(0 65 70 -125 50 -120 160)
                                                #f(-5 75 -105 -90 70 0 20)))

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
  (let* ((current-arm (check-tuckarm-pose :thre 50)) ;; nil rarm larm
         (free-arm (or (car args) current-arm :larm))
		 (msec 500))
    (when (not (eq current-arm free-arm))
	  (progn
		(setq msec 2000)
		(send *pr2* :larm :angle-vector #f(25 0 0 -130 0 0 0))
		(send *pr2* :rarm :angle-vector #f(-25 0 0 -130 0 0 0))
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
  (call-empty-service "/move_base_node/clear_local_costmap")
  (call-empty-service "/move_base_node/clear_unknown_space"))

(defun initialize-costmap ()
  (call-empty-service "move_base_node/global_costmap/initialize_costmap")
  (call-empty-service "move_base_node/local_costmap/initialize_costmap")
  (clear-costmap))

(defun change-inflation-radius (range param-name) ;;  this is private function
  (unless (eps= (ros::get-param param-name) range 0.1)
    (ros::set-param param-name range)))

(defun change-inflation-range (&optional (range 0.55) &key (type :all))
  (let ((local-costmap "/move_base_node/local_costmap/inflation_radius")
	(global-costmap "/move_base_node/global_costmap/inflation_radius"))
    (case type
      (:global (change-inflation-radius range global-costmap))
      (:local  (change-inflation-radius range local-costmap))
      (t
       (change-inflation-radius range global-costmap)
       (change-inflation-radius range local-costmap)))
    ))
