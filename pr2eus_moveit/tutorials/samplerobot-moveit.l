(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(require :samplerobot-interface
         "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defclass samplerobot-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod samplerobot-moveit-environment
  (:init
   (&key ((:robot rb) (samplerobot)))
   (send-super :init :robot rb
               :frame-id "WAIST_LINK0"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/odom")
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "rarm")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "larm")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rarm-torso
               (cons :group-name "rarm_torso")
               (cons :target-link
                     (send self :search-link-from-name "RARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :rarm :joint-list)))
               )
         (list :larm-torso
               (cons :group-name "larm_torso")
               (cons :target-link
                     (send self :search-link-from-name "LARM_LINK6"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :larm :joint-list)))
               )
         )
   )
  )

(defun sync-larm (&optional (tm 500))
  (let ((av (send *ri* :state :reference-vector)))
    (send *ri* :robot :angle-vector av)
    (send *ri* :angle-vector av tm)
    (send *ri* :wait-interpolation)
    (send *ri* :remove-joint-group "larm")
    (unix::usleep (* 100 1000))
    (send *ri* :add-joint-group "larm"
          (list "LARM_SHOULDER_P" "LARM_SHOULDER_R" "LARM_SHOULDER_Y" "LARM_ELBOW" "LARM_WRIST_Y" "LARM_WRIST_P"))
    (unix::usleep (* 100 1000))
    ))

#|
;; pr2eus_moveit
(samplerobot-init)
(send *sr* :angle-vector (send *ri* :state :reference-vector))

(setq *me* (instance samplerobot-moveit-environment :init))
(send *me* :robot :angle-vector (send *ri* :state :reference-vector))

(setq cds (send *me* :robot :larm :end-coords :copy-worldcoords))
(send cds :translate #f(0 0 100) :world)
(send *me* :get-ik-for-pose cds :larm :end-coords (list :larm :end-coords))

(setq ret (send *me* :motion-plan :larm))
(send *me* :execute-trajectory (send ret :trajectory))
|#

#|
moveit_demo
(samplerobot-init)
(send *sr* :angle-vector (send *ri* :state :reference-vector))

;;
execute plan in rviz
;;
(sync-larm)

(send *sr* :larm :wrist-r :joint-angle -20) ;; grasp
(send *ri* :angle-vector (send *sr* :angle-vector) 2000)

(send *sr* :angle-vector (send *ri* :state :reference-vector))
(send *sr* :larm :inverse-kinematics
      (send (send *sr* :larm :end-coords :copy-worldcoords) :translate #f(0 0 100) :world))
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
|#