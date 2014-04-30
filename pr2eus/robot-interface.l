;;;
;;; robot interface to ROS based pr2 system
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "pr2_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")
;;(ros::roseus-add-msgs "sensor_msgs") ;; roseus depends on sensor_msgs
;;(ros::roseus-add-msgs "visualization_msgs") ;; roseus depends on visualization_msgs

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
                controller-type controller-actions controller-timeout
                namespace controller-table ;; hashtable :type -> (action)
                visualization-topic
                joint-states-topic
                viewer groupname))

(defmethod robot-interface
  (:init
   (&rest args &key ((:robot r)) ((:objects objs)) (type :default-controller)
          (use-tf2) ((:groupname nh) "robot_multi_queue") ((:namespace ns))
          ((:joint-states-topic jst) "joint_states")
          ((:controller-timeout ct) 3)
          ((:visuzlization-marker-topic vmt) "robot_interface_marker_array")
          &allow-other-keys)
   (setq joint-states-topic jst)
   (setq joint-action-enable t)
   (setq controller-timeout ct)
   (setq namespace ns)
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
                     (t r)))
   (setq groupname nh)
   (unless (ros::ok)
     (ros::roseus "default_robot_interface"))
   (ros::create-nodehandle groupname)
   ;;
   (setq visualization-topic vmt)
   (ros::advertise visualization-topic visualization_msgs::MarkerArray 100)
   ;;
   (when (ros::get-param "use_sim_time" nil)
     (ros::subscribe "/clock" rosgraph_msgs::Clock #'(lambda (msg)))
     (ros::ros-warn "/use_sim_time is TRUE, check if /clock is pusblished or not")
     (unix:sleep 1)
     (when (= (ros::get-num-publishers "/clock") 0)
       (ros::ros-error "/use_sim_time is TRUE and /clock is NOT PUBLISHED")
       (exit 1)))

   (cond
    (use-tf2
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::buffer-client :init))))
    (t
     (unless (boundp '*tfl*)
       (defvar *tfl* (instance ros::transform-listener :init)))))

   (ros::subscribe (if namespace (format nil "~A/~A" namespace joint-states-topic)
                     joint-states-topic) sensor_msgs::JointState
                   #'send self :ros-state-callback :groupname groupname)
   ;;
   (setq controller-table (make-hash-table :size 14 :test #'eq :rehash-size 1.2))
   (setq controller-type type)
   (setq controller-actions
         (send self :add-controller controller-type :joint-enable-check t))
   ;;
   (when (send self :simulation-modep)
     (let ((old-viewer user::*viewer*))
       (when (and x::*display* (> x::*display* 0))
         (setq viewer (get (geo::find-viewer (send robot :name)) :pickviewer))
         (unless viewer
           (setq viewer (instance x::irtviewer :create :title (format nil "~A Kinematics Simulator" (send robot :name)) :view-name (send robot :name) :draw-floor t)))
         (send viewer :objects (list robot))
         (send self :draw-objects)
         (send self :objects objs)
         (if old-viewer (setq user::*viewer* old-viewer)))
       (ros::advertise (if namespace (format nil "~A/~A" namespace joint-states-topic)
                         joint-states-topic) sensor_msgs::JointState 1)
       ))
   self)
  ;;
  (:add-controller (ctype &key (joint-enable-check))
   (let (tmp-actions)
     (mapcar
      #'(lambda (param)
          (let* ((controller-action (cdr (assoc :controller-action param)))
                 (action-type (cdr (assoc :action-type param)))
                 (action (instance ros::simple-action-client :init
                                   (if namespace (format nil "~A/~A" namespace controller-action)
                                     controller-action) action-type
                                   :groupname groupname)))
            (push action tmp-actions)))
      (send self ctype))
     (setq tmp-actions (nreverse tmp-actions))
     ;;
     (dolist (action tmp-actions)
       (unless (and joint-action-enable (send action :wait-for-server controller-timeout))
         (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name))
         (when joint-enable-check
           (setq joint-action-enable nil)
           (return))))
     ;;
     (dolist (param (send self ctype))
       (let* ((controller-state (cdr (assoc :controller-state param)))
              (key (intern (string-upcase controller-state) *keyword-package*)))
         (ros::subscribe (if namespace (format nil "~A/~A" namespace controller-state)
                           controller-state)
                         pr2_controllers_msgs::JointTrajectoryControllerState
                         #'send self :set-robot-state1 key :groupname groupname)))
     ;;
     (setf (gethash ctype controller-table) tmp-actions)
     tmp-actions
     ))
  ;;
  (:publish-joint-state ;; for simulation mode (joint-action-enable is nil)
   (&optional (joint-list (send robot :joint-list)))
   (let (msg names positions velocities efforts)
     (setq msg (joint-list->joint_state joint-list))
     (send msg :header :stamp (ros::time-now))
     (when (send self :simulation-modep)
       (ros::publish (if namespace (format nil "~A/~A" namespace joint-states-topic)
                       joint-states-topic) msg))
     msg))
  (:angle-vector
   (av &optional (tm 3000) (ctype controller-type))
   ;; check max-joint-velocity
   (let ((idx 0)
         (diff-av (v- av (or (send self :state :potentio-vector) (send robot :angle-vector)))))
     (dolist (jt (send robot :joint-list))
       (cond ((derivedp jt linear-joint) ;; msec = mm / [m/sec]
              (setq tm (max tm (abs (/ (elt diff-av idx) (send jt :max-joint-velocity))))))
             ((derivedp jt rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
              (setq tm (max tm (abs (/ (deg2rad (elt diff-av idx))
                                       (send jt :max-joint-velocity) 0.001))))))
       (incf idx)
       ))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av
       (let* ((prev-av (send robot :angle-vector))
	      (scale-av (send self :sub-angle-vector av prev-av)))
	 (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	     ((>= curr-tm tm))
	   (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	   (send self :publish-joint-state)
	   (if viewer (send self :draw-objects))))))

   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table)))
     (unless cacts
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector))
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                0  ;; start time
                (list
                 (list av                                     ;; positions
                       (instantiate float-vector (length av)) ;; velocities
                       (/ tm 1000.0)))))                      ;; duration
      cacts (send self ctype)))
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)) (ctype controller-type))
   (send self :spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
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
           (when (send self :simulation-modep)
	     (let* ((prev-av (send robot :angle-vector))
		    (scale-av (send self :sub-angle-vector av prev-av)))
	       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
		   ((>= curr-tm tm))
		 (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
		 (send self :publish-joint-state)
		 (if viewer (send self :draw-objects)))))
	   ;;
	   (send robot :angle-vector av)
	   (push (list av
		       (copy-seq vel)  ;; velocities
		       (/ (+ st tm) 1000.0)) ;; tm + duration
		 traj-points)
	   (setq av-prev av)
	   (incf st tm))
	 ;;
         (let ((cacts (gethash ctype controller-table)))
           (unless cacts
             (warn ";; controller-type: ~A not found" ctype)
             (return-from :angle-vector-sequence))
           (mapcar
            #'(lambda (action param)
                (send self :send-ros-controller
                      action (cdr (assoc :joint-names param)) ;; action server and joint-names
                      0.1   ;; start time
                      traj-points))
            cacts (send self ctype)))
         )))
  (:wait-interpolation (&optional (ctype) (timeout 0)) ;; controller-type
   (when (send self :simulation-modep)
     (return-from :wait-interpolation nil))
   (cond
    (ctype
     (let ((cacts (gethash ctype controller-table)))
       (send-all cacts :wait-for-result :timeout timeout)))
    (t (send-all controller-actions :wait-for-result :timeout timeout))))
  (:stop-motion (&key (stop-time 0))
   (let ((av (send self :state :potentio-vector)))
     (send self :angle-vector av stop-time)
     (send self :wait-interpolation)))
  (:cancel-angle-vector
   (&key ((:controller-actions ca) controller-actions)
         ((:controller-type ct) controller-type)
         (wait))
   (send-all ca :cancel-all-goals)
   (when wait
     (send self :wait-interpolation)
     (send self :spin-once))
   t)
  ;;
  (:worldcoords () (send robot :copy-worldcoords))
  (:torque-vector () (send robot :torque-vector))
  (:potentio-vector () (send robot :angle-vector))
  (:reference-vector () (send self :state-vector :desired))
  (:actual-vector () (send self :state-vector :actual))
  (:error-vector () (send self :state-vector :error))
  (:state-vector
   (type &key ((:controller-actions ca) controller-actions)
         ((:controller-type ct) controller-type))
   (let* ((joint-list (send robot :joint-list))
          (av (instantiate float-vector (length joint-list))))
     (dolist (param (send self controller-type))
       (let* ((ctrl (cdr (assoc :controller-state param)))
              (key (intern (string-upcase ctrl) *keyword-package*))
              (msg (send self :state key))
              (idx 0))
         ;;(print (list ctrl key msg))
         (unless msg (return-from :state-vector nil))
         (dolist (jname (send msg :joint_names))
           (let ((jangle (elt (send msg type :positions) idx))
                 (j (find-if #'(lambda (jn) (string= (send jn :name) jname))
                             joint-list)))
             (incf idx)
             (when j
               (setf (elt av (position j joint-list))
                     (cond ((derivedp j linear-joint)     (* jangle 1000))
                           ((derivedp j rotational-joint) (rad2deg jangle))))))
           )))
     av))
  ;;
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (when (send self :simulation-modep)
     (return-from :send-ros-controller nil))
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
   (send self :update-robot-state)
   (unless args (return-from :state))
   (case (car args)
     ((:potentio-vector :angle-vector)
      (send robot :angle-vector))
     (:torque-vector
      (send robot :torque-vector))
     (:worldcoords
      (send *tfl* :lookup-transform (or (cadr args) "/map") "/base_footprint" (ros::time)))
     (t
      (let ((mm (find (car args) (remove-if-not #'(lambda (x) (substringp "-VECTOR" (string x))) (send self :methods)))))
        (if mm ;; if xx-vector method exists
            (send* self mm (cdr args))
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
	      (setq objects (mapcar #'(lambda (o) (let ((p (send o :parent))) (if p (send p :dissoc o)) (setq o (copy-object o)) (if p (send p :assoc o)) o)) objs))
	      (mapcar #'(lambda (o) (setf (get o :GL-DISPLAYLIST-ID) nil)) (x::draw-things objects))
	      (when viewer
                (send viewer :objects (append (list robot) objects))
                (send self :draw-objects)))
	    objects)
  (:draw-objects ()
   (when viewer
     (send viewer :look-all (send (geo::make-bounding-box (flatten (send-all (x::draw-things robot) :vertices))) :grow 0.3))
     (send viewer :draw-objects)
     (x::window-main-one)))
  ;;
  (:joint-action-enable (&optional (e :dummy)) (if (not (eq e :dummy)) (setq joint-action-enable e)) joint-action-enable)
  (:simulation-modep () (null joint-action-enable))
  (:warningp (&optional (w :dummy)) (if (not (eq w :dummy)) (setq warningp w)) warningp)
  (:spin-once () (ros::spin-once groupname))
  (:send-trajectory (joint-trajectory-msg
                     &key ((:controller-actions ca) controller-actions) ((:controller-type ct) controller-type)
                     (starttime 1) &allow-other-keys)
   (mapcar
    #'(lambda (action param)
        (send self :send-trajectory-each
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              joint-trajectory-msg
              starttime))
    ca (send self ct)))
  (:send-trajectory-each
   (action joint-names traj &optional (starttime 0.2))
   (let* ((jnames (send traj :joint_names))
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
       (let ((goal (send action :make-goal-instance))
             (st (ros::time+ (ros::time-now) (ros::time starttime))))
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
  (:ros-wait (tm &key (spin) (spin-self) (finish-check) &allow-other-keys) ;; just wait in ros-time
   (ros::rate 100) ;;
   (let ((st (ros::time-now)))
     (if spin (ros::spin-once))
     (if spin-self (send self :spin-once))
     (while t
       (when finish-check
         (if (funcall finish-check self)
           (return)))
       (let ((tdiff (ros::time- (ros::time-now) st)))
         (if (> (send tdiff :to-sec) tm)
             (return)))
       (if spin (ros::spin-once))
       (if spin-self (send self :spin-once))
       (ros::sleep)))
   t)
  ) ;; robot-interface
;; ros visualization methods
(defmethod robot-interface
  (:joint-trajectory-to-angle-vector-list
   (move-arm joint-trajectory
             &key ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50)
             (show-trajectory t) (send-trajectory t)
             ((:speed-scale speed-scale) 1.0) &allow-other-keys)
   (let* (joint-trajectory-points
          joints avs tms  (tm (ros::time -1)) link-coords-list
          (org-av (send robot :angle-vector))
          (dae-link-list
           (send self :find-descendants-dae-links (send robot move-arm :root-link))))
     ;; JointTrajectory to angle-vector
     (if joint-trajectory
         (setq joint-trajectory-points (send joint-trajectory :points)
               joints (mapcar #'(lambda (x)
                                  (send robot (intern (string-upcase x) *keyword-package*)))
                              (send joint-trajectory :joint_names))))
     ;;
     (if (= (length joint-trajectory-points) 0)
         (return-from :joint-trajectory-to-angle-vector-list nil))
     (dolist (point joint-trajectory-points)
       (mapc '(lambda (ajoint aposition)
                (send ajoint :ros-joint-angle aposition))
             joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (when (cadr avs)
         (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
         (when (> diff-sum diff-thre)
           (setq diff-sum 0)
           (when show-trajectory
             (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
                   link-coords-list))))
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000 speed-scale) tms)
       (setq tm (send point :time_from_start))
       )
     (when show-trajectory
       (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
             link-coords-list)
       ;; send visualization_msgs to rviz
       ;; (send self :show-mesh-traj-with-color (send-all (flatten (send-all link-list :bodies)) :name) link-coords-list)
       (send self :show-mesh-traj-with-color
             (flatten (send-all dae-link-list :bodies))
             link-coords-list :lifetime (+ (/ (apply #'+ tms) 1000.0) 10)))
     (cond
      (send-trajectory
       (send robot :angle-vector (car (last avs)))
       ;; send to *ri*
       (send self :angle-vector-sequence (setq avs (reverse avs)) (setq tms (reverse tms))))
      (t
       (send robot :angle-vector org-av)
       ))
     (if joint-trajectory (list (reverse avs) (reverse tms)))
     ))
  (:show-goal-hand-coords (coords move-arm)
   (let* ((gripper-bodies
           (flatten (send-all (send robot move-arm :gripper :links) :bodies)))
          (gripper-coords
           (mapcar #'(lambda (gripper-link)
                       (send (send coords :copy-worldcoords) :transform
                             (send (send robot move-arm :end-coords) :transformation gripper-link)
                             :local))
                   gripper-bodies)))
     (send self :show-mesh-traj-with-color
           gripper-bodies (list gripper-coords)
           :lifetime 0 :color #f(1 0 1) :ns "hand_traj")
     (list gripper-bodies gripper-coords)))
  (:find-descendants-dae-links (l)
   (unless l (return-from :find-descendants-dae-links nil))
   (append (list l)
           (mapcan #'(lambda (x) (send self :find-descendants-dae-links x)) (send l :child-links))))
  (:show-mesh-traj-with-color
   (link-body-list link-coords-list &key ((:lifetime lf) 20)
                   (ns "robot_traj") ((:color col) #f(0.5 0.5 0.5)))
   (let ((msg (instance visualization_msgs::MarkerArray :init))
         (header (instance std_msgs::header :init
                           :stamp (ros::time-now)
                           :frame_id (send (car (send robot :links)) :name)))
         (l (length link-coords-list)) markers-list alpha-list
         (base-cds (send (car (send robot :links)) :copy-worldcoords)))
     (setq base-cds (send base-cds :inverse-transformation))
     (dotimes (i l)
       (push (+ (/ (/ (1+ i) 2.0) l) 0.5) alpha-list))
     (dotimes (i l)
       (let (mrk markers)
         (mapcar #'(lambda (abody acoords)
                     (setq acoords (send (send acoords :copy-worldcoords) :transform base-cds :world))
                     (cond ((send abody :name)
                            (setq mrk (mesh->marker-msg
                                       acoords
                                       (send abody :name) ;; body would have :name as filename of original mesh
                                       header
                                       :mesh_use_embedded_materials nil :color col :alpha (elt alpha-list i))))
                           (t
                            (setq mrk (object->marker-msg
                                       abody
                                       header
                                       :coords acoords
                                       :color col
                                       :alpha (elt alpha-list i)
                                       ))))
                     (send mrk :lifetime (ros::time lf))
                     (send mrk :ns ns)
                     (push mrk markers))
                 link-body-list
                 (elt link-coords-list i))
         (push markers markers-list)))
     (setq markers-list (flatten markers-list))
     (dotimes (x (length markers-list)) (send (elt markers-list x) :id x))
     (send msg :markers markers-list)
     (ros::publish visualization-topic msg)
     ))
  )
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
      (let ((jk (intern (string-upcase jn) *keyword-package*)))
        (when (find-method robot jk)
          (let ((j (send robot jk)))
            (send j :ros-joint-angle (elt ps cntr))
            (if vl (send j :joint-velocity (elt vl cntr)))
            (if ef (send j :joint-torque (elt ef cntr))))))
      (incf cntr))
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
