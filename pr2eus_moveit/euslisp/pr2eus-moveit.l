(ros::load-ros-manifest "pr2eus_moveit")

(require :robot-moveit "package://pr2eus_moveit/euslisp/robot-moveit.l")

(defclass pr2-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod pr2-moveit-environment
  (:init (&key args)
         (send-super* :init :robot (instance pr2-robot :init) :frame-id "base_footprint" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "r_wrist_roll_link"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "l_wrist_roll_link"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rarm-torso
               (cons :group-name "right_arm_and_torso")
               (cons :target-link
                     (send self :search-link-from-name "r_wrist_roll_link"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :rarm :joint-list)))
               )
         (list :larm-torso
               (cons :group-name "left_arm_and_torso")
               (cons :target-link
                     (send self :search-link-from-name "l_wrist_roll_link"))
               (cons :joint-list (append (send robot :torso :joint-list) (send robot :larm :joint-list)))
               )
         (list :arms ;; can not use inverse-kinematics
               (cons :group-name "arms")
               (cons :target-link
                     (list (send self :search-link-from-name "l_wrist_roll_link")
                           (send self :search-link-from-name "r_wrist_roll_link")))
               (cons :joint-list (append (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         (list :arms-torso ;; can not use inverse-kinematics
               (cons :group-name "arms_and_torso")
               (cons :target-link
                     (list (send self :search-link-from-name "l_wrist_roll_link")
                           (send self :search-link-from-name "r_wrist_roll_link")))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
#| ;; sample of pr2_moveit_config/config/pr2.srdf
    <group name="base">
        <joint name="world_joint" />
    </group>
    <group name="left_arm">
        <chain base_link="torso_lift_link" tip_link="l_wrist_roll_link" />
    </group>
    <group name="left_arm_and_torso">
        <chain base_link="base_link" tip_link="l_wrist_roll_link" />
    </group>
    <group name="right_arm">
        <chain base_link="torso_lift_link" tip_link="r_wrist_roll_link" />
    </group>
    <group name="right_arm_and_torso">
        <chain base_link="base_link" tip_link="r_wrist_roll_link" />
    </group>
    <group name="arms">
        <group name="left_arm" />
        <group name="right_arm" />
    </group>
    <group name="head">
        <joint name="head_pan_joint" />
        <joint name="head_tilt_joint" />
    </group>
    <group name="torso">
        <joint name="torso_lift_joint" />
    </group>
    <group name="whole_body">
        <group name="base" />
        <group name="arms" />
        <group name="torso"/>
    <group/>
|#
         ))

(provide :pr2eus-moveit "pr2eus-moveit.l")
