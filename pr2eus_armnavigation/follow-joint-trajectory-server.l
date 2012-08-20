#!/usr/bin/env roseus
;;
;;
(ros::load-ros-manifest "control_msgs")

(defclass follow-joint-trajectory-server
  :super propertied-object
  :slots ((action :forward (:worker))
          reference-robot interface-instance
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
   (setq interface-instance interface)
   self
   )
  (:execute-cb (s g)
   (warn ";; execute ~A ~A~%" s g)
   (let ((msg (send s :result))
         (traj (send g :goal :trajectory))
         (path_tor (send g :goal :path_tolerance))
         (goal_tor (send g :goal :goal_tolerance))
         (gtime_tor (send g :goal :goal_time_tolerance)))

     (when (and reference-robot interface-instance)
       (cond
        ((find-method interface-instance :send-trajectory)
         (send interface-instance :send-trajectory traj))
        (t
         (let ((ret (apply-joint_trajectory traj reference-robot)))
           (send interface-instance :angle-vector-sequence
                 (car ret) (cadr ret)))
         ))
       ;; FIXME: add feedback while waiting
       (send interface-instance :wait-interpolation))
     (send s :set-succeeded msg)
     ))
  (:preempt-cb (s g)
   (warn ";; preempt ~A ~A~%" s g))
  (:accept-cb (s g)
   (warn ";; accept ~A ~A~%" s g))
  (:spin ()
   (ros::rate 100)
   (do-until-key
    (send self :worker)
    (send action :spin-once)
    (ros::sleep)))
  (:spin-once () (send self :worker) (send action :spin-once))
  )

(defun start-node (&optional (trajectory "follow_joint_trajectory"))
  (load "package://pr2eus/pr2-interface.l")
  ;;(load "package://pr2eus_armnavigation/follow-joint-trajectory-server.l")

  (ros::roseus "fake_follow_joint_trajectory")

  (setq *ri* (instance pr2-interface :init))

  (cond
   ((listp trajectory)
    (setq *server-list* nil)
    (let ((rb (instance pr2-robot :init)))
      (dolist (traj trajectory)
        (push
         (instance follow-joint-trajectory-server
                   :init (ros::resolve-name traj)
                   :robot rb :interface *ri*) *server-list*))
      (ros::ros-info ";; start follow-servers")
      (ros::rate 100)
      (do-until-key
       (dolist (s *server-list*)
         (send s :spin-once))
       (ros::sleep))
      ))
   (t
    (setq *server* (instance follow-joint-trajectory-server
                             :init (ros::resolve-name trajectory)
                             :robot (instance pr2-robot :init) :interface *ri*))
    (ros::ros-info ";; start follow-server")
    (send *server* :spin))
   ))

;; (start-node)
