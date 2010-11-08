;;
;;
;;
(require :pr2 "package://pr2eus/pr2.l")

;;;
;;; interface to ros based pr2 system
;;;

(require :eustf "package://roseus/euslisp/eustf.l")
(require :actionlib "package://roseus/euslisp/actionlib.l")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")

(defclass pr2-interface
  :super propertied-object
  :slots (robot robot-state controller-type controller-actions
		r-gripper-action l-gripper-action                move-base-action))
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
   (send r-gripper-action :wait-for-server 10)
   (send l-gripper-action :wait-for-server 10)

   (setq move-base-action (instance ros::simple-action-client :init
                                    "move_base" move_base_msgs::MoveBaseAction))
   self)
  (:angle-vector
   (av &optional (tm 3000))
   (send robot :angle-vector av)
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
            (send self :send-pr2-controller
                  action (cdr (assoc :joint-names param)) ;; action server and joint-names
                  av   ;; positions
		  vel  ;; velocities
                  (+ (/ st 1000.0) 0.2)   ;; start time
                  (+ (/ tm 1000.0) 0.2))) ;; duration
        controller-actions (send self controller-type))
       (setq av-prev av)
       (incf st tm)
       )
     (ros::spin-once)
     ))
  (:wait-interpolation
   ()
   (send-all controller-actions :wait-for-result))
  (:send-pr2-controller
   (action joint-names all-positions all-velocities starttime duration)
   (let* ((st (ros::time+ (ros::time-now) (ros::time starttime)))
          (tm (ros::time duration))
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
			     :time_from_start tm))))
      ((eq (class goal) pr2_controllers_msgs::PointHeadActionGoal)
       (send goal :goal :target :point
	     (ros::pos->tf-point
	      (send (send (send robot :base_footprint) :transformation
			  (send (send (send robot :high_def_frame) :copy-worldcoords) :translate #f(1000 0 0) :local))
		    :worldpos)))
       (send goal :goal :target :header :frame_id "base_footprint")
       (send goal :goal :pointing_frame "high_def_frame")
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
  ;;
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
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
     (if wait  (mapcar #'(lambda(c)(send c :wait-for-result)) clients))
     ))
  (:start-grasp
   (&optional (arm :arms) &key ((:gain g) 0.01))
   (send self :move-gripper arm 0.0 :effort (* 2000 g) :wait nil))
  (:stop-grasp
   (&optional (arm :arms))
   (send self :move-gripper arm 0.09 :wait nil))
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
  )


