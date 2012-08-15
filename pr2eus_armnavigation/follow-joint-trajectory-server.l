;;
;;
;;
(ros::load-ros-manifest "control_msgs")

(defclass follow-joint-trajectory-server
  :super propertied-object
  :slots ((action :forward (:worker))
          reference-robot robot-interface
          rgoal)
  )
(defmethod follow-joint-trajectory-server
  (:init (nm &key (robot) (interface))
   (setq action
         (instance ros::simple-action-server :init nm
                   control_msgs::FollowJointTrajectoryAction
                   :execute-cb `(lambda (s g) (send ,self :execute-cb s g))
                   :preempt-cb `(lambda (s g) (send ,self :preempt-cb s g))
                   :accept-cb  `(lambda (s g) (send ,self :accept-cb  s g))
                   ))
   (setq reference-robot robot)
   (setq robot-interface interface)
   self
   )
  (:execute-cb
   (s g)
   (warn ";; execute ~A ~A~%" s g)
   (let ((msg (send s :result))
         (traj (send g :goal :trajectory))
         (path_tor (send g :goal :path_tolerance))
         (goal_tor (send g :goal :goal_tolerance))
         (gtime_tor (send g :goal :goal_time_tolerance)))
     (when (and reference-robot robot-interface)
       (cond
        ((find-method robot-interface :send-trajectory)
         (send robot-interface :send-trajectory traj))
        (t
         (let ((ret (apply-joint_trajectory traj reference-robot)))
           (send robot-interface :angle-vector-sequence
                 (car ret) (cadr ret)))
         ))
       ;; FIXME: add feedback while waiting
       (send robot-interface :wait-interpolation))
     (send s :set-succeeded msg)
     ))
  (:preempt-cb
   (s g)
   (warn ";; preempt ~A ~A~%" s g))
  (:accept-cb
   (s g)
   (warn ";; accept ~A ~A~%" s g))
  (:spin ()
   (ros::rate 100)
   (do-until-key
    (send self :worker)
    (send action :spin-once)
    (ros::sleep)))
  )

#|
(load "package://pr2eus/pr2-interface.l")
;;(load "package://pr2eus_armnavigation/follow-joint-trajectory-server.l")

(ros::roseus "fake_follow_joint_trajectory")

(setq *ri* (instance pr2-interface :init))

(setq *server* (instance moge :init "/follow_joint_trajectory"
                         :robot (pr2) :interface *ri*))
(ros::ros-info ";; start follow-server")
(send *server* :spin)
|#
