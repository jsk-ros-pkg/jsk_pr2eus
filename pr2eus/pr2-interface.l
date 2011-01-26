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
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")

(defclass pr2-interface
  :super propertied-object
  :slots (robot robot-state joint-action-enable
		controller-type controller-actions
		r-gripper-action l-gripper-action
                move-base-action finger-pressure-origin
                viewer))
(defmethod pr2-interface
  (:init
   (&rest args &key (type :default-controller) &allow-other-keys)
   (setq joint-action-enable t)
   (setq robot (instance pr2-robot :init))
   ;;
   (ros::subscribe "/joint_states" sensor_msgs::JointState
		   #'send self :pr2-state-callback)
   (ros::subscribe "/pressure/r_gripper_motor" pr2_msgs::PressureState
		   #'send self :pr2-fingertip-callback :rarm-pressure)
   (ros::subscribe "/pressure/l_gripper_motor" pr2_msgs::PressureState
		   #'send self :pr2-fingertip-callback :larm-pressure)
   ;;
   (setq controller-type type)
   (mapcar
    #'(lambda (param)
	(let* ((controller (cdr (assoc :controller param)))
	       (action-type (cdr (assoc :action-type param)))
	       (action (instance ros::simple-action-client :init controller action-type)))
	  (push action controller-actions)))
    (send self controller-type))
   (nreverse controller-actions)
   (setq r-gripper-action (instance ros::simple-action-client :init
				    "/r_gripper_controller/gripper_action"
				    pr2_controllers_msgs::Pr2GripperCommandAction))
   (setq l-gripper-action (instance ros::simple-action-client :init
				    "/l_gripper_controller/gripper_action"
				    pr2_controllers_msgs::Pr2GripperCommandAction))
   (setq move-base-action (instance ros::simple-action-client :init
                                    "move_base" move_base_msgs::MoveBaseAction))
   ;; wait for all action server (except move_base)
   (dolist (action (append controller-actions (list r-gripper-action l-gripper-action)))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, pr2-interface is disabled" action)
       (return)))

   (unless joint-action-enable
     (let ((old-viewer user::*viewer*))
       (setq viewer (instance x::irtviewer :create :title "pr2-interface"))
       (send viewer :objects (list robot))
       (send viewer :look-all)
       (setq user::*viewer* old-viewer)))

   self)
  (:angle-vector
   (av &optional (tm 3000))
   (send robot :angle-vector av)
   (unless joint-action-enable
     (if av (send viewer :draw-objects)))

   (mapcar
    #'(lambda (action param)
        (send self :send-pr2-controller
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              av				      ;; positions
              (instantiate float-vector (length av))  ;; velocities
              0					      ;; start time
              (/ tm 1000.0)))			      ;; duration
    controller-actions (send self controller-type))
   (ros::spin-once)
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)))
   (ros::spin-once) ;; for :state :potentio-vector
   (let ((st 0)
         (av-prev (send self :state :potentio-vector)) av av-next
         tm tm-next
         (vel (instantiate float-vector (length (car avs)))))
     (while avs
       (setq av (pop avs))
       (if (car tms) (setq tm (pop tms)))
       (if (setq av-next (car avs))
           (let ((v0 (v- av av-prev))
                 (v1 (v- av-next av)))
             (setq tm-next (if (car tms) (car tms) tm))
             (dotimes (i (length vel))
               (setf (elt vel i)
                     (if (>= (* (elt v0 i) (elt v1 i)) 0)
                         (* 0.5 (+ (* (/ 1000.0 tm) (elt v0 i))
                                   (* (/ 1000.0 tm-next) (elt v1 i))))
                       0.0)))
             )
         (fill vel 0))
       ;;
       (send robot :angle-vector av)
       (mapcar
        #'(lambda (action param)
	    ;; if trajectry actoin other than JointTrajectoryAction
	    ;; could not deal with start time, they only accept duration
	    (when (or
		   (null avs) ;; only last avs is sent to SingleJointPosition and PointHead 
		   (eq (cdr (assoc :action-type param))
		       pr2_controllers_msgs::jointtrajectoryaction))
	      (send self :send-pr2-controller
		    action (cdr (assoc :joint-names param)) ;; action server and joint-names
		    av   ;; positions
		    vel  ;; velocities
		    (+ (/ st 1000.0) 0.1)   ;; start time
		    (+ (/ tm 1000.0) 0.2)))) ;; duration
	    controller-actions (send self controller-type))
       (setq av-prev av)
       (incf st tm)
       )
     (ros::spin-once)
     ))
  (:wait-interpolation
   ()
   (unless joint-action-enable (return-from :wait-interpolation nil))
   (send-all controller-actions :wait-for-result))
  (:send-pr2-controller
   (action joint-names all-positions all-velocities starttime duration)
   (unless joint-action-enable (return-from :send-pr2-controller nil))
   (let* ((st (ros::time+ (ros::time-now) (ros::time starttime)))
	  (goal (send action :make-goal-instance))
          (positions (instantiate float-vector (length joint-names)))
          (velocities (instantiate float-vector (length joint-names)))
          (joints (mapcar #'(lambda (x)
                              (send robot (intern (string-upcase x) *keyword-package*)))
                          joint-names)))
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
     (send goal :header :seq 1)
     (send goal :header :stamp st)
     (cond
      ((eq (class goal) pr2_controllers_msgs::JointTrajectoryActionGoal)
       (send goal :goal :trajectory :joint_names joint-names)
       (send goal :goal :trajectory :header :stamp st)
       (send goal :goal :trajectory :points
	     (list (instance trajectory_msgs::JointTrajectoryPoint
			     :init
			     :positions positions
			     :velocities velocities
			     :time_from_start (ros::time duration)))))
      ((eq (class goal) pr2_controllers_msgs::PointHeadActionGoal)
       (send goal :goal :target :point
	     (ros::pos->tf-point
	      (send (send (send robot :base_footprint) :transformation
			  (send (send (send robot :head_tilt_link) :copy-worldcoords) :translate #f(100000 0 0) :local))
		    :worldpos)))
       (send goal :goal :target :header :frame_id "/base_footprint")
       ;;(send goal :goal :pointing_frame "/high_def_optical_frame")
       (send goal :goal :min_duration (ros::time (+ starttime duration))))
      ((eq (class goal) pr2_controllers_msgs::SingleJointPositionActionGoal)
       (send goal :goal :position (elt positions 0))
       (send goal :goal :min_duration (ros::time (+ starttime duration))))
      (t (warning-message 1 ";; ~a is not found" (class goal))))
     (ros::spin-once)
     (send action :send-goal goal)
     ))
  ;;
  (:pr2-state-callback
   (msg)
   (dolist (key '(:name :position))
     (if (assoc key robot-state)
	 (setf (cdr (assoc key robot-state)) (send msg key))
       (push (cons key (send msg key)) robot-state))))
  (:state
   (&rest args)
   (let (joint-names positions)
     (unless joint-action-enable (return-from :state (send robot :angle-vector)))
     (unless robot-state (return-from :state))
     (ros::spin-once)
     (setq joint-names (cdr (assoc :name robot-state))
	   positions (cdr (assoc :position robot-state)))
     (mapcar #'(lambda (n p)
                (let ((j (send robot (intern (string-upcase n) *keyword-package*))))
                  (when j
		    (cond
		     ((derivedp j rotational-joint)
		      (send j :joint-angle (mod (rad2deg p) 360)))
		     (t
		      (send j :joint-angle (* 1000 p)))))))
	    (coerce joint-names cons)
            (coerce positions cons))
     (send robot :angle-vector)))
  ;;
  ;;
  ;;
  (:larm-controller
   ()
   (list
    (cons :controller "l_arm_controller/joint_trajectory_action")
    (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
    (cons :joint-names (list "l_shoulder_pan_joint"
			     "l_shoulder_lift_joint" "l_upper_arm_roll_joint"
			     "l_elbow_flex_joint" "l_forearm_roll_joint"
			     "l_wrist_flex_joint" "l_wrist_roll_joint"))))
  (:rarm-controller
   ()
   (list
    (cons :controller "r_arm_controller/joint_trajectory_action")
    (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
    (cons :joint-names (list "r_shoulder_pan_joint"
			     "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
			     "r_elbow_flex_joint" "r_forearm_roll_joint"
			     "r_wrist_flex_joint" "r_wrist_roll_joint"))))
  (:head-controller
   ()
   (list
    (cons :controller "head_traj_controller/point_head_action")
    (cons :action-type pr2_controllers_msgs::PointHeadAction)
    (cons :joint-names (list "head_pan_joint" "head_tilt_joint"))))
  (:torso-controller
   ()
   (list
    (cons :controller "torso_controller/position_joint_action")
    (cons :action-type pr2_controllers_msgs::SingleJointPositionAction)
    (cons :joint-names (list "torso_lift_joint"))))
  (:default-controller
   ()
   (list
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
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
   (unless joint-action-enable (return-from :move-gripper nil))
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
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait nil))
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
	 (mapcar #'(lambda(k)(assoc k robot-state))
		 '(:rarm-pressure :larm-pressure))))
  (:finger-pressure
   (arm)
   (setq arm (case arm (:rarm :rarm-pressure) (:larm :larm-pressure)))
   (let ((current (cdr (assoc arm robot-state)))
	 (origin (cdr (assoc arm finger-pressure-origin))))
     (when (and current origin)
       (mapcar #'v- current origin))))
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
       (ros::ros-info "move-to : send-goal to ~A (~d)" (send coords :worldpos) count)
       (send move-base-action :send-goal goal)
       (send move-base-action :wait-for-result)
       (if (eq (send move-base-action :get-state) actionlib_msgs::succeeded)
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
   (x y d) ;; [m/sec] [m/sec] [rad/sec]
   (let ((goal (instance geometry_msgs::Twist :init)))
     (send goal :linear :x x)
     (send goal :linear :y y)
     (send goal :angular :z r)
     (ros::publish "cmd_vel" goal)))
  )


