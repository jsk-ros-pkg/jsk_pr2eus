(ros::load-ros-manifest "pr2_mechanism_msgs")
(ros::load-ros-manifest "ee_cart_imped_msgs")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; roslaunch ee_cart_imped_launch load_ee_cart_imped.launch
(defmethod robot-interface
  ;; FIX: this function should go to robot-inferface
  (:change-controller (stop-controller start-controller)
   (let (req res)
     (setq req (instance pr2_mechanism_msgs::SwitchControllerRequest :init))
     (send req :stop_controllers (list stop-controller))
     (send req :start_controllers (list start-controller))
     (send req :strictness 2)
     (setq res (ros::service-call "/pr2_controller_manager/switch_controller" req))
     (ros::ros-warn "changing controller from ~A to ~A" stop-controller start-controller)
     (unless (send res :ok)
       (ros::ros-warn "failed to change controller")
       )
     ))
  ) ;; defmethod robot-interface

;; for ee_cart_imped
(defmethod pr2-interface
  (:init-imped-action-client
   ()
   (let (r-arm-imped-action l-arm-imped-action)
     (unless (send self :get 'r-arm-imped-action)
       (setq r-arm-imped-action (instance ros::simple-action-client
                                          :init "/r_arm_cart_imped_controller/ee_cart_imped_action"
                                          ee_cart_imped_msgs::EECartImpedAction
                                          :groupname groupname))
       (send self :put 'r-arm-imped-action r-arm-imped-action))
     (unless (send self :get 'l-arm-imped-action)
       (setq l-arm-imped-action (instance ros::simple-action-client
                                          :init "/l_arm_cart_imped_controller/ee_cart_imped_action"
                                          ee_cart_imped_msgs::EECartImpedAction
                                          :groupname groupname))
       (send self :put 'l-arm-imped-action l-arm-imped-action))
     ))
  (:start-impedance
   (&optional (arm :rarm))
   (case arm
     (:rarm
      (send self :change-controller "r_arm_controller" "r_arm_cart_imped_controller")
      )
     (:larm
      (send self :change-controller "l_arm_controller" "l_arm_cart_imped_controller")
      )
     (:arms
      (send self :change-controller "r_arm_controller" "r_arm_cart_imped_controller")
      (send self :change-controller "l_arm_controller" "l_arm_cart_imped_controller")
      )))
  (:stop-impedance
   (&optional (arm :rarm))
   (case arm
     (:rarm
      (send self :change-controller "r_arm_cart_imped_controller" "r_arm_controller")
      )
     (:larm
      (send self :change-controller "l_arm_cart_imped_controller" "l_arm_controller")
      )
     (:arms
      (send self :change-controller "r_arm_controller" "r_arm_cart_imped_controller")
      (send self :change-controller "l_arm_controller" "l_arm_cart_imped_controller")
      )))

  (:wait-impedance-trajectory
   (&optional (arm :rarm) (timeout 0))
#|
   (send self :init-imped-action-client)
   (let ((r-arm-imped-action (send self :get 'r-arm-imped-action))
         (l-arm-imped-action (send self :get 'l-arm-imped-action)))
     (case arm
       (:rarm
        (send r-arm-imped-action :wait-for-result :timeout timeout))
       (:larm
        (send l-arm-imped-action :wait-for-result :timeout timeout))
       (:arms
        (send r-arm-imped-action :wait-for-result :timeout timeout)
        (send l-arm-imped-action :wait-for-result :timeout timeout))
       )
     )
|#
   ;; TODO: impedance controller immediately return
   (send self :ros-wait timeout)
   )

  (:cancel-all-impedance-trajectry
   (&optional (arm :rarm))
   (send self :init-imped-action-client)
   (let ((r-arm-imped-action (send self :get 'r-arm-imped-action))
         (l-arm-imped-action (send self :get 'l-arm-imped-action)))
     (case arm
       (:rarm
        (send r-arm-imped-action :cancel-all-goal))
       (:larm
        (send l-arm-imped-action :cancel-all-goal))
       (:arms
        (send r-arm-imped-action :cancel-all-goal)
        (send l-arm-imped-action :cancel-all-goal)))))

  (:start-impedance-trajectory
   (arm trs &optional (wait -1))
   (send self :init-imped-action-client)
   (let ((goal (instance ee_cart_imped_msgs::EECartImpedActionGoal :init))
         (r-arm-imped-action (send self :get 'r-arm-imped-action))
         (l-arm-imped-action (send self :get 'l-arm-imped-action)))
     (send goal :goal :header :frame_id "base_footprint")
     (send goal :goal :header :stamp (ros::time-now))
     (send goal :header :stamp (ros::time-now))
     (dolist (tr trs)
       (send* self :add-impedance-trajectory (cons goal tr)))
     (let ((arm-client
            (case arm
              (:rarm
               (setq arm-client r-arm-imped-action))
              (:larm
               (setq arm-client l-arm-imepd-action)))))
       (cond
        ((not (numberp wait))
         (send arm-client :send-goal goal)
         (let ((tm 0.0))
           (dolist (trj (send goal :goal :trajectory))
             (incf tm (send (send trj :time_from_start) :to-sec)))
           (ros::ros-info ";; wait ~A sec" tm)
           (send self :ros-wait tm))
         )
        ((<= wait 0)
         (send arm-client :send-goal goal))
        (t
         (send arm-client :send-goal goal)
         (send self :ros-wait wait))
        )
       )))

  (:add-impedance-trajectory
   (goal coords ts &key (frame-id "base_footprint")
         (target-force (float-vector 1000 1000 1000 20 20 20)) ;; target force or impedance gain
         (force-control (list nil nil nil nil nil nil)) ;;(t: force control, nil: impedance control)
         (control-rot (unit-matrix))) ;; force and impedance controll coords
   (let ((crot (matrix2quaternion control-rot))
         (pos (scale 0.001 (send coords :worldpos)))
         (rot (matrix2quaternion (send coords :worldrot)))
         (addPoint (instance ee_cart_imped_msgs::StiffPoint :init))
         (prev-goal (send goal :goal :trajectory))
         (stp (send goal :header :stamp)))
     (send addPoint :pose :position :x (elt pos 0))
     (send addPoint :pose :position :y (elt pos 1))
     (send addPoint :pose :position :z (elt pos 2))
     (send addPoint :pose :orientation :x (elt rot 1))
     (send addPoint :pose :orientation :y (elt rot 2))
     (send addPoint :pose :orientation :z (elt rot 3))
     (send addPoint :pose :orientation :w (elt rot 0))
     (send addPoint :wrench_or_stiffness :force :x (elt target-force 0))
     (send addPoint :wrench_or_stiffness :force :y (elt target-force 1))
     (send addPoint :wrench_or_stiffness :force :z (elt target-force 2))
     (send addPoint :wrench_or_stiffness :torque :x (elt target-force 3))
     (send addPoint :wrench_or_stiffness :torque :y (elt target-force 4))
     (send addPoint :wrench_or_stiffness :torque :z (elt target-force 5))
     (send addPoint :isForceX (elt force-control 0))
     (send addPoint :isForceY (elt force-control 1))
     (send addPoint :isForceZ (elt force-control 2))
     (send addPoint :isTorqueX (elt force-control 3))
     (send addPoint :isTorqueY (elt force-control 4))
     (send addPoint :isTorqueZ (elt force-control 5))
     (send addPoint :forceDirection :x (elt crot 1))
     (send addPoint :forceDirection :y (elt crot 2))
     (send addPoint :forceDirection :z (elt crot 3))
     (send addPoint :forceDirection :w (elt crot 0))
     (send addPoint :time_from_start (ros::time (/ ts 1000.0)))
     (send addPoint :header :stamp stp)
     (send addPoint :header :frame_id frame-id)

     (send goal :goal :trajectory (append prev-goal (list addPoint)))
     ))
  ) ;; defmethod pr2-interface
