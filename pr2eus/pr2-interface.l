;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")

;;;
;;; interface to ros based pr2 system
;;;

(require :eustf "package://roseus/euslisp/eustf.l")
(require :actionlib "package://roseus/euslisp/actionlib.l")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")

(defclass pr2-interface
  :super propertied-object
  :slots (robot robot-state controller-type controller-actions
		r-gripper-action l-gripper-action))
(defmethod pr2-interface
  (:init
   (&rest args &key (type :default-controller) &allow-other-keys)
   (setq robot (instance pr2-robot :init))
   ;;
   (ros::subscribe "/joint_states" sensor_msgs::JointState
		   #'send self :pr2-state-callback)
   ;;
   (setq controller-type type)
   (mapcar
    #'(lambda (param)
	(let* ((controller (cdr (assoc :controller param)))
	       (action-type (cdr (assoc :action-type param)))
	       (action (instance ros::simple-action-client :init controller action-type)))
	  (ros::ros-info  "waiting ~A~%" controller)
	  (push action controller-actions)
	  (unless (send action :wait-for-server 10)
	    (ros::ros-error "~A is not respond, please restart pr2, check run-stop" controller))))
    (send self controller-type))
   (nreverse controller-actions)
   (setq r-gripper-action (instance ros::simple-action-client :init
				    "/r_gripper_controller/gripper_action"
				    pr2_controllers_msgs::Pr2GripperCommandAction))
   (setq l-gripper-action (instance ros::simple-action-client :init
				    "/l_gripper_controller/gripper_action"
				    pr2_controllers_msgs::Pr2GripperCommandAction))
   (send r-gripper-action :wait-for-server)
   (send l-gripper-action :wait-for-server)
   self)
  (:angle-vector
   (av &optional (tm 3000))
   (send robot :angle-vector av)
   (mapcar
    #'(lambda (action param)
	(send self :send-pr2-controller
	      action (cdr (assoc :joint-names param))
	      (/ tm 1000.0)))
    controller-actions (send self controller-type))
   (ros::spin-once)
   av)
  (:wait-interpolation
   ()
   (send-all controller-actions :wait-for-result))
  (:send-pr2-controller
   (action joint-names duration)
   (let* ((tm (ros::time duration))
	  (joints
	   (mapcar #'(lambda (x)
		       (send robot (intern (string-upcase x) *keyword-package*)))
		   joint-names))
	  (positions
	   (mapcar #'(lambda (x)
		       (cond
			((derivedp x rotational-joint)
			 (deg2rad (send x :joint-angle)))
			(t
			 (* 0.001 (send x :joint-angle)))))
		   joints))
	  (goal (send action :make-goal-instance)))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (cond
      ((eq (class goal) pr2_controllers_msgs::JointTrajectoryActionGoal)
       (send goal :goal :trajectory :joint_names joint-names)
       (send goal :goal :trajectory :header :stamp (ros::time-now))
       (send goal :goal :trajectory :points
	     (list (instance trajectory_msgs::JointTrajectoryPoint
			     :init
			     :positions positions
			     :velocities (make-list (length positions) :initial-element 0)
			     :time_from_start tm))))
      ((eq (class goal) pr2_controllers_msgs::PointHeadActionGoal)
       (send goal :goal :target :point
	     (ros::pos->tf-point
	      (send (send (send robot :base_footprint) :transformation
			  (send (send robot :head :end-coords) :translate #f(0 0 200) :local))
		    :worldpos)))
       (send goal :goal :target :header :frame_id "base_footprint")
       (send goal :goal :min_duration tm))
      ((eq (class goal) pr2_controllers_msgs::SingleJointPositionActionGoal)
       (send goal :goal :position (elt positions 0))
       (send goal :goal :min_duration tm))
      (t (warning-message 1 ";; ~a is not found" (class goal))))
     (ros::spin-once)
     (send action :send-goal goal)
     ))
  ;;
  (:pr2-state-callback
   (msg)
   (setq robot-state (list (cons :name (send msg :name))
			   (cons :position (send msg :position)))))
  (:state
   (&rest args)
   (let (joint-names positions)
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
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
   (let* ((goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
	  (c (send (send goal :goal) :command))
	  (clients (case arm
		     (:rarm (list r-gripper-action))
		     (:larm (list l-gripper-action))
		     (:arms (list r-gripper-action l-gripper-action)))))
     (send c :position pos)
     (send c :max_effort effort)
     (mapcar #'(lambda(c)(send c :send-goal goal)) clients)
     (ros::spin-once)
     (if wait
	 (mapcar #'(lambda(c)(send c :wait-for-result)) clients))
     ))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 0.01))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait nil))
  (:stop-grasp
   (&optional (arm :arms))
   (send self :move-gripper arm 0.09 :wait nil))
  )

