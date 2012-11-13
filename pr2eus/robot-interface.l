;;;
;;; robot interface to ROS based pr2 system
;;;

(require :eustf "package://roseus/euslisp/eustf.l")
(require :actionlib "package://roseus/euslisp/actionlib.l")

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "pr2_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")

;; add ros-joint-angle method using meter/radian
(defmethod rotational-joint
  (:ros-joint-angle
   (&optional v &rest args)
   (if v (setq v (rad2deg v)))
   (setq v (send* self :joint-angle v args))
   (deg2rad v))
  )
(defmethod linear-joint
  (:ros-joint-angle
   (&optional v &rest args)
   (if v (setq v (* 1000.0 v)))
   (setq v (send* self :joint-angle v args))
   (* 0.001 v))
  )

(defclass robot-interface
  :super propertied-object
  :slots (robot objects robot-state joint-action-enable warningp
                nodehandlename controller-type controller-actions
		viewer groupname))

(defmethod robot-interface
  (:init
   (&rest args &key ((:robot r)) ((:objects objs)) (type :default-controller)
          ((:groupname nh) "robot_multi_queue") &allow-other-keys)
   (setq joint-action-enable t)
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
		     (t r)))
   (setq groupname nh)
   (unless (ros::ok)
     (ros::roseus "default_robot_interface"))
   (ros::create-nodehandle groupname)
   ;;
   (defvar *tfl* (instance ros::transform-listener :init))
   (ros::subscribe "/joint_states" sensor_msgs::JointState
                   #'send self :ros-state-callback :groupname groupname)

   (setq controller-type type)
   (mapcar
    #'(lambda (param)
	(let* ((controller-action (cdr (assoc :controller-action param)))
	       (action-type (cdr (assoc :action-type param)))
	       (action (instance ros::simple-action-client :init controller-action action-type
                                 :groupname groupname)))
	  (push action controller-actions)))
    (send self controller-type))
   (nreverse controller-actions)

   (dolist (action controller-actions)
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name) )
       (return)))
   ;;
   (dolist (param (send self controller-type))
     (let* ((controller-state (cdr (assoc :controller-state param)))
	    (key (intern (string-upcase controller-state) *keyword-package*)))
       (ros::subscribe controller-state
                       pr2_controllers_msgs::JointTrajectoryControllerState
                       #'send self :set-robot-state1 key :groupname groupname)
     ))
   ;;
   (unless joint-action-enable
     (let ((old-viewer user::*viewer*))
       (when (and x::*display* (> x::*display* 0))
	 (setq viewer (instance x::irtviewer :create :title (format nil "~A-interface" (send robot :name))))
	 (send viewer :objects (list robot))
	 (send viewer :look-all)
	 (send self :objects objs)
	 (if old-viewer (setq user::*viewer* old-viewer)))

       (ros::advertise "/joint_states" sensor_msgs::JointState 1)
       ))
   self)
  ;;
  (:publish-joint-state ;; for simulation mode (joint-action-enable is nil)
   (&optional (joint-list (send robot :joint-list)))
   (let (msg names positions velocities efforts)
     (setq msg (joint-list->joint_state joint-list))
     (send msg :header :stamp (ros::time-now))
     (unless joint-action-enable
       (ros::publish "/joint_states" msg))
     msg))
  (:angle-vector
   (av &optional (tm 3000) &key ((:controller-actions ca) controller-actions) ((:controller-type ct) controller-type))
   ;; check max-joint-velocity
   (let ((jlist (send robot :joint-list)) j
	 (diff-av (v- av (or (send self :state :potentio-vector) (send robot :angle-vector)))))
     (dotimes (i (length jlist))
       (setq j (elt jlist i))
       (cond ((derivedp j linear-joint) ;; msec = mm / [m/sec]
	      (setq tm (max tm (/ (elt diff-av i) (send j :max-joint-velocity)))))
	     ((derivedp j rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
	      (setq tm (max tm (/ (deg2rad (elt diff-av i)) (send j :max-joint-velocity) 0.001)))))))
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
   (send self :spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
         (offset (instantiate float-vector (length (send robot :angle-vector))))
         tm tm-next
         (vel (instantiate float-vector (length (car avs)))))
       (prog1 ;; angle-vector-sequence returns avs
	   avs
	 (while avs
	   (setq av (pop avs))
	   (when (= (car tms) 0.0)
	     (ros::ros-error "0.0 in time list @ :angle-vector-sequence"))
	   (if (car tms) (setq tm (max (pop tms) 1.0))) ;; minimum is 1[msec]
	   (if (setq av-next (car avs))
	       (let ((v0 (send self :sub-angle-vector av av-prev))
		     (v1 (send self :sub-angle-vector av-next av)))
		 (setq tm-next (max (if (car tms) (car tms) tm) 1.0))
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
  (:stop-motion (&key (stop-time 0))
   (let ((av (send self :state :potentio-vector)))
     (send self :angle-vector av stop-time)
     (send self :wait-interpolation)))
  (:cancel-angle-vector
   (&key ((:controller-actions ca) controller-actions)
         ((:controller-type ct) controller-type))
   (send-all ca :cancel-all-goals)
   (when wait
     (send self :wait-interpolation)
     (send self :spin-once))
   t)
  ;;
  (:reference-vector
   ()
   (send self :state-vector :desired))
  (:potentio-vector
   () ;; not use this method, -> use update-robot-state and :state :potentio-vector
   (send self :state-vector :actual))
  (:error-vector
   ()
   (send self :state-vector :error))
  (:state-vector
   (type &key ((:controller-actions ca) controller-actions)
	 ((:controller-type ct) controller-type))
   (let* ((joint-list (send robot :joint-list))
	  (av (instantiate float-vector (length joint-list))))
     (dolist (param (send self controller-type))
       (let* ((ctrl (cdr (assoc :controller-state param)))
	      (key (intern (string-upcase ctrl) *keyword-package*))
	      (msg (send self :state key))
	      jn jangle jpos j)
	 (print msg)
	 (if (not msg)
	     (return-from :state-vector nil))
	 (dotimes (idx (length (send msg :joint_names)))
	   (setq jname (elt (send msg :joint_names) idx))
	   (setq jangle (elt (send msg type :positions) idx))
	   (setq jpos (position jname (send-all joint-list :name) :test #'string=))
	   (setq j (elt joint-list jpos))
	   (setf (elt av jpos)
		 (cond ((derivedp j linear-joint)     (* jangle 1000))
		       ((derivedp j rotational-joint) (rad2deg jangle))))
	   )))
     av ))
  ;;
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (unless joint-action-enable (return-from :send-ros-controller nil))
   (if (and warningp
	    (yes-or-no-p (format nil "~C[3~CmAre you sure to move the real robot? (~A) ~C[0m" #x1b 49 (send action :name) #x1b)))
       (return-from :send-ros-controller nil))
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
     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send action :send-goal goal)
     ))
  ;;
  (:set-robot-state1
   (key msg)
     (if (assoc key robot-state)
	 (setf (cdr (assoc key robot-state)) msg)
       (push (cons key msg) robot-state)))   
  (:ros-state-callback
   (msg)
   (dolist (key '(:name :position :velocity :effort))
     (send self :set-robot-state1 key (send msg key)))
   (send self :set-robot-state1 :stamp (send msg :header :stamp)))
  (:update-robot-state
   ()
   (let (joint-names positions velocities efforts)
     (send self :spin-once)
     ;; (unless joint-action-enable
     ;;   (return-from :update-robot-state (send robot :angle-vector)))
     (unless robot-state (return-from :update-robot-state))
     (setq joint-names (cdr (assoc :name robot-state))
	   positions (cdr (assoc :position robot-state))
	   velocities (cdr (assoc :velocity robot-state))
	   efforts (cdr (assoc :effort robot-state)))
     (let ((joint-num (length joint-names)))
       (when (not (eq joint-num (length velocities)))
	 (setq velocities (instantiate float-vector joint-num)))
       (when (not (eq joint-num (length efforts)))
	 (setq efforts (instantiate float-vector joint-num))))
     (mapcar #'(lambda (n p v e)
		 (let (j (kn (intern (string-upcase n) *keyword-package*)))
		   (when (and (find-method robot kn) (setq j (send robot kn)))
                     (send j :ros-joint-angle p)
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
     (:worldcoords
      (send *tfl* :lookup-transform (or (cadr args) "/map") "/base_footprint" (ros::time)))
     (t
      (let ((mm (find (car args) (remove-if-not #'(lambda (x) (substringp "-VECTOR" (string x))) (send self :methods)))))
        (if mm ;; if xx-vector method exists
            (send self mm)
          (let ((cur robot-state))
            (dolist (key args cur)
              (setq cur (cdr (assoc key cur))))))
        ))))
  ;;
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/joint_trajectory_action")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
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
  (:objects (&optional objs)
	    (when objs
	      (setq objects (mapcar #'(lambda (o) (let ((p (send o :parent))) (if p (send p :dissoc o))(copy-object o)(if p (send p :assoc o)))) objs))
	      (mapcar #'(lambda (o) (setf (get o :GL-DISPLAYLIST-ID) nil)) (x::draw-things objects))
	      (if viewer (send viewer :objects (append (list robot) objects))))
	    objects)
  (:joint-action-enable (&optional (e :dummy)) (if (not (eq e :dummy)) (setq joint-action-enable e)) joint-action-enable)
  (:warningp (&optional (w :dummy)) (if (not (eq w :dummy)) (setq warningp w)) warningp)
  (:spin-once () (ros::spin-once groupname))
  (:send-trajectory (joint-trajectory-msg
                     &key ((:controller-actions ca) controller-actions) ((:controller-type ct) controller-type)
                     (starttime 1))
   (mapcar
    #'(lambda (action param)
        (send self :send-trajectory-each
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              joint-trajectory-msg
              starttime))
    ca (send self ct)))
  (:send-trajectory-each
   (action joint-names traj &optional (starttime 0))
   (let* ((jnames (send traj :joint_names))
          (st (ros::time+ (ros::time-now) (ros::time starttime)))
          (ilst (mapcar #'(lambda (jn) (position jn jnames :test #'string=)) joint-names))
          points-lst)
     (when (some #'identity ilst)
       (setq ilst (mapcar #'(lambda (jn)
                              (let ((p (position jn jnames :test #'string=)))
                                (unless p
                                  (setq p (send robot (intern (string-upcase jn) *keyword-package*))))
                                p))
                          joint-names))
       (dolist (p (send traj :points))
         (let ((poss (send p :positions))
               (vels (send p :velocities))
               (effs (send p :accelerations))
               plst vlst elst)
           (dolist (i ilst)
             (cond
              ((numberp i)
               (push (elt poss i) plst)
               (if vels (push (elt vels i) vlst))
               (if effs (push (elt effs i) elst)))
              (t
               (push (send i :ros-joint-angle) plst)
               (if vels (push 0 vlst))
               (if effs (push 0 elst))
               (ros::ros-warn ";; trajectory contains lacking joint names")
               )))
           (push
            (instance trajectory_msgs::JointTrajectoryPoint :init
                      :positions (coerce (nreverse plst) float-vector)
                      :velocities (if vels (coerce (nreverse vlst) float-vector))
                      :accelerations (if effs (coerce (nreverse elst) float-vector))
                      :time_from_start (send p :time_from_start)) points-lst)
           ))
       (let ((goal (send action :make-goal-instance)))
         (send goal :header :stamp st)
         (send goal :header :seq 1)
         (send goal :goal :trajectory :header :stamp st)
         (send goal :goal :trajectory :header :seq 1)
         (send goal :goal :trajectory :joint_names joint-names)
         (send goal :goal :trajectory :points (nreverse points-lst))
         (send self :spin-once)
         (send action :send-goal goal))
       (apply-trajectory_point joint-names (car (last points-lst)) robot)
       ) ;;; /when ilst
     ))
  ) ;; ros-interface

;;
(defclass ros-interface
  :super robot-interface
  :slots ())
(defmethod ros-interface
  (:init
   (&rest args)
   (ros::ros-error "please use robot-interface class, ros-interface class will be removed")
   (send-super* :init args))
  )

(defun joint-list->joint_state (jlist &key (position) (effort 0) (velocity 0))
  (let (nlist plist vlist elist)
    (cond
     ((numberp velocity)
      (setq vlist (instantiate float-vector (length jlist)))
      (fill vlist velocity))
     ((float-vector-p velocity)
      (setq vlist (copy-object velocity)))
     )
    (cond
     ((numberp effort)
      (setq elist (instantiate float-vector (length jlist)))
      (fill elist effort))
     ((float-vector-p effort)
      (setq elist (copy-object effort)))
     )
    (cond
     ((numberp position)
      (setq plist (instantiate float-vector (length jlist)))
      (fill plist position))
     ((float-vector-p position)
      (setq plist (copy-object position)))
     )
    (dolist (j jlist)
      (let* ((n (send j :name))
             (nm (if (symbolp n) (symbol-name n) n)))
        (push nm nlist)
        (unless position
          (push (send j :ros-joint-angle) plist))
        ))
    (instance sensor_msgs::JointState :init
              :name (nreverse nlist)
              :position (if position plist
                          (coerce (nreverse plist) float-vector))
              :velocity vlist
              :effort elist)
    ))

(defun apply-joint_state (jointstate robot)
  (let ((cntr 0)
        (ps (send jointstate :position))
        (vl (send jointstate :velocity))
        (ef (send jointstate :effort)))
    (if (/= (length vl) (length (send jointstate :name))) (setq vl nil)) ;; vl is not set
    (if (/= (length ef) (length (send jointstate :name))) (setq ef nil)) ;; ef is not set
    (dolist (jn (send jointstate :name))
      (let ((j (send robot (intern (string-upcase jn) *keyword-package*))))
        (send j :ros-joint-angle (elt ps cntr))
        (if vl (send j :joint-velocity (elt vl cntr)))
        (if ef (send j :joint-torque (elt ef cntr)))
        (incf cntr)))
    ))

(defun apply-trajectory_point (names trajpoint robot)
  (let ((cntr 0)
        (ps (send trajpoint :positions))
        ;;(vl (send trajpoint :velocities))
        ;;(ef (send trajpoint :accelerations))
        )
    (dolist (jn names)
      (let ((j (send robot (intern (string-upcase jn) *keyword-package*))))
        (send j :ros-joint-angle (elt ps cntr))
        (incf cntr)))
    ))

(defun apply-joint_trajectory (joint-trajectory robot &optional (offset 200.0))
  (let ((names (send joint-trajectory :joint_names))
        (points (send joint-trajectory :points))
        avs tms ptm)
    (dolist (p points)
      (apply-trajectory_point names p robot)
      (push (send robot :angle-vector) avs)
      (cond
       ((eq p (car points))
        (setq ptm (* 1000.0 (send (send p :time_from_start) :to-sec)))
        (push (+ ptm offset) tms))
       (t
        (let ((tm (* 1000.0 (send (send p :time_from_start) :to-sec))))
          (push (- tm ptm) tms)
          (setq ptm tm)))
        ))
    (list (nreverse avs)
          (nreverse tms))
    ))

(provide :robot-interface "robot-interface.l")
