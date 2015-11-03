^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2eus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2015-11-03)
------------------
* Bug Fixes

  * [robot-interface.l] change-inflation-range to use new service name (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/169)
  * :interpolating-smoothp not working (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/158)

    * [pr2eus/robot-interface] fix to work :wait-interpolation-smooth  (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/159)
    * test/default-ri-test.l: add test code for :wait-interpolation-smooth,
    * mv default-ri-test.launch-> default-ri-test.test, and add to CMakeLists.txt


* Add :go-* prototype functions  (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/164, https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/171)

  * robot-interface.l: use error instead of warn for :go-* prototype  functions (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/163)
  * [pr2eus/pr2-interface.l] fix return value of `:go-pos-unsafe-wait` along with (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/164)
  * [pr2eus/robot-inferface.l] clarify return value policy (https://github.com/k-okada/jsk_pr2eus/pull/5)
  * [pr2eus] fix go-pos-unsafe
  * pr2-interface.l: add :go-pos-unsafe, :go-pos-unsafe-no-wait, :go-pos-unsafe-wait
  * robot-interface.l: add go-* function prototype
  * pr2-interface.l : addk go-pos-no-wait and go-wait

* Support go-pos-no-wait in simulation mode

  * Display objects in simulationp (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/168)

    *   [robot-interface.l]: (send self :objects objs) should call even in simulationp
    *   [test/default-ri-test.l] add test for :objects methods

  * Fix :move-to in sim mode (check frame-I'd) add test for :move-to (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/167)

    * [pr2-interface.l] move to relative to current position only if frame-id argument is /base_footprint
    * [test/pr2-ri-test-simple.l] add test for move-to

  * Support move-to-no-wait in simplationp (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165)

   * [pr2-interface.l] :move-to-send , for simulation mode, do not try to call :lookup-transform
   * [pr2-interface.l] fix typo : if -> when, return-from :move-to -> return-from :move-to-send, https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/165#discussion_r37421484
   * [test/pr2-ri-test-simple.l] add test for go-pos, go-pos-no-wait, go-wait
   * [pr2eus/pr2eus/pr2-interface.l] fix typo (short modify) @h-kamada
   * test/test-ri-test.l: :wait-interpolation retuns a list of :interpolationg
   * pr2-interface : support timer-based motion for :move-to
   * more realistic simulation mode

* use default pr2_description (https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/149)

  * [pr2eus] change pr2 camera frame namespace from /openni to  /kinect_head (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/153)

* Other New Features

  * [pr2eus/robot-interface.l] add method :find-object to  robot-interface and test code (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/180)

* Misc Updates

  * [pr2eus/CMakeLists.txt]: remove old groovy codes
  * [pr2eus/speak.l] refactor speak.l (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/176)
    - super easy to read code
    - support wait and timeout for every speaking
    - support multi language with google engine
  * pass additional-weight-list when calling super class method (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/148)
  * [pr2ues/robot-interface.l] check length of avs and tms in  :angle-vector-sequence; add test code (https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/151)

* Contributors: Kamada Hitoshi, Kei Okada, Masaki Murooka, Yuki Furuta, Yuto Inagaki

0.1.11 (2015-06-11)
-------------------
* [pr2eus] Print warning message if controller-timeout is nil in robot-interface
* [robot-interface.l] do not raise error when controller have wrong joint name
* [test/pr2-ri-test-simple.l] add test for wrong controller
* Revert "[pr2eus] Use get-topics in speak.l to check whether already advertised or not"
  This reverts commit 134353868b4e826a8a879bb3ac3b9dcbb500a7da.
* [robot-interface.l] update joint in (*ri* . robot) only in controller-type
* [robot-interface.l] update only cotroller joint for simulation mode
* [robot-interface.l] add documents for public methods
* [robot-interface.l] :angle-vector-sequence use default if nil ctype was passed
* [robot-interface.l] :angle-vector use default if nil ctype was passed
* [pr2eus] Use get-topics in speak.l to check whether already advertised or not
* [pr2eus/CMakeLists.txt] add eusdoc
* [pr2eus] remove old manifest.xml
* [pr2eus] Fix :interpolatingp by using ros::*simple-goal-state-active* instead of actoinlib_msgs::GoalStatus::*active*
* [pr2eus] Support ctype in :interpolatingp
* add publish-joint-state and update viewer for the last pose in angle-vector-sequence
* [robot-interface.l] add zero div check
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Yuto Inagaki, Shintaro Noda

0.1.10 (2015-04-03)
-------------------
* [robot-interface.l, pr2-interface.l] support :fast in :angle-vector-sequence
* Contributors: Yuto Inagaki

0.1.9 (2015-04-03)
------------------
* [robot-interface.l] :min-time=0.0 in :angle-vector-sequence because smooth  angle-vector may have short duration for each angle-vector
* [jsk_pr2eus] FIx :angle-vector-sequence by passing ctype argument to :angle-vector-duration
* [pr2-interface.l] remove unused service call '/move_base_node/clear_unknown_space'
* [robot-interface.l] change default 5 to 1 as :scale in angle-vector
* [robot-intetface.l] check if :controller-type is valid in :angle-vector and :angle-vector-sequence
* [robot-interface.l] Support ctype in :angle-vector-duration
* [robot-interface.l] add :angle-vector-safe for prototype robot
* [robot-interface.l] Add euslisp implementation mannequin mode. (:eus-mannequin-mode)
* [robot-interface.l] modify robot-interface.l to support control_msgs::SingleJointPositionGoal
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Yuki Furuta, Yuto Inagaki

0.1.8 (2015-02-25)
------------------
* Modify wrong maintainer and author name.
* [pr2eus/robot-interface.l] load rosgraph_msgs
* [pr2eus/catkin.cmake] need to call roseus at the end of find_package so that roseus.cmake can read all package files
* Contributors: Kei Okada, Yuto Inagaki

0.1.7 (2015-02-10)
------------------
* [pr2eus] Add sound_play and rosgraph_msgs to find_package to generate messages for roseus
* Updat definition of make-robot-interface-from-name and add
  robot-init-from-name function
* modify :angle-vector-sequence to use angle-vector-duration
* [pr2eus] Add make-robot-interface-from-name function to create
  robot-interface instance from name
* [pr2eus] Repair :angle-vector args document
* return list of t at :wait-interpolation on simulation mode
* fix actionlib error
* fix :wait-interpolation-smooth
* create controller-action-client to process feedback for :wait-interpolation-smooth
* use angle-vector-duration when time is not setted
* add make-plan method for move base
* change variables names.
* enable specification of wait-until-update time for joint-state
* fix: do not use limited buffer for publishing joint state at simulation mode
* add :publish-joint-states-topic keyword to robot-interface for publishing joint_states from the other name
* add :wait t option to speak-en
* add nod function for pr2
* add tuckarm outside
* add test code to check default-robot-interface.l
* add google sound option
* add :move-trajectory-sequence
* add codes in order to use move-trajectory
* avoid to create action and subscriber twice
* reduce assoc
* use let only once
* merge joint-states message which contain other joints. add option to wait until all joint data is updated
* (pr2.l) Generate pr2.l model again
* (`jsk-ros-pkg/jsk_model_tools#18 <https://github.com/jsk-ros-pkg/jsk_model_tools/issues/18>`_) pr2eus/make-pr2-model-file.l : remove :camera method which is already committed to irtrobot.l
* do not loop bag file, to privet output TF_OLD_DATA
* add unsubscribe /clock after checking /clock
* Contributors: Hitoshi Kamada, Yuki Furuta, Kei Okada, Yuto Inagaki, JSK Lab member, Chi Wun Au, Masaki Murooka, Ryohei Ueda, Yohei Kakiuchi, Shunichi Nozawa

0.1.6 (2014-05-11)
------------------
* Merge pull request #32 from k-okada/add_roseus_msgs
  remove roseus_msgs from run_depend
* remove roseus_msgs from run_depend

0.1.5 (2014-05-03)
------------------
* Merge pull request #26 from k-okada/22_fix_use_sim_time_check
  fix wrong commit on #22
* fix wrong commit on #22
* Contributors: Kei Okada

0.1.4 (2014-05-02)
------------------
* add roseus_msgs to run_depend
* Contributors: Kei Okada

0.1.3 (2014-05-02)
------------------
* install sample program with executable bit
* Contributors: Kei Okada

0.1.2 (2014-05-01)
------------------
* install only lisp and launch files
* Contributors: Kei Okada

0.1.1 (2014-05-01)
------------------
* add metapackage
* change roseus-svnrevision -> roseus-repo-version, due to https://github.com/jsk-ros-pkg/jsk_roseus/pull/34
* set time-limit 1800
* bugfix: change link name
* disable pr2-ri-test since this requires gazebo
* fix find_package components for groovy, generae missing package via generete-all-msg-srv.sh
* add :controller-timeout keyword to robot-interface to specify
  the timeout to wait controller
* add warn and exit the program for `jsk-ros-pkg/jsk_common#186 <https://github.com/jsk-ros-pkg/jsk_common/issues/186>`_
* Merge pull request `#8 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/8>`_ from YoheiKakiuchi/fix_joint_trajectory
  fix send-trajectory
* `#11 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/11>`_: back to gazebo from gzserver when testing pr2-ri-test.launch
* `#11 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/11>`_: use gzserver instead of gazebo on test
* Merge remote-tracking branch 'origin/master' into youhei-tip
* fix send-trajectory
* fix send-trajectory
* add keyword :joint-states-topic for changing jonit_states name
* install euslisp files in the package root directory: last catkinize commit was also done by murooka
* catkinize pr2eus
* fixed method to get links for new pr2 model
* update pr2 model, fix kinect geometry
* use joint_trajectory_action -> follow_joint_trajectory
* delete commit r5583
* add --no-link-suffix,--no-joint-suffix, concerning backword compatibility
* update pr2 model
* do not use 0.2 sec marge, now the mergin is only 0.1 sec, see https://code.google.com/p/rtm-ros-robotics/issues/detail?id=276 for more detail
* fix window name and draw floor for robot-interface's simulation mode, see Isseue 42, this requries r979(https://sourceforge.net/p/jskeus/code/979/) of jskeus
* add comments for go-velocity arguments and use msec in animation codes
* remove unused local variables
* ignore not existing joint
* add move base range in args of ik
* use :additional-weight-list to set weight without using index of weight vector explicitly ;; test pr2's ik by euscollada/pr2.sh and ik-test.l
* update ros-wait
* fix minor bug
* add :ros-wait method to robot-interface
* fix for using :move-to with /base_footprint as frame_id, [`#234 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/234>`_]
* update parameter for avoiding warning message, [`#233 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/233>`_]
* remove :wait-interpolation finish check on pr2-tuckarm-pose
* move code of visuazlizing trajectory to robot-inreface.l from pr2eus_openrave
* modified loading dependant programs, no longer needed require basic roseus codes
* modified time-limit for low power PC
* add checking correctly finished :wait-interpolation on pr2-tuckarm-pose
* add check code for result of move command, nil will be returned if failed or canceled
* add optional force-stop to :go-stop method
* add check of length c = 2 for dual arm manipulation
* use angle-vector-sequence in angle-vector-with-constraint when ri simulation
* `#216 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/216>`_, support select-target-arm for dual ik
* setup :header :seq, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* send with move_base_simplw if /move_base/goal failed, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* use /map frame to send move_base/goal, see [`#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_]
* add description for voice text command
* enable to add arguments for xx-vector methods, which is reported kuroiwa
* r4702 requires fix to make-pr2-model-file.l `#200 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/200>`_
* fix pr2-ri-test to pass the test
* fix :stop-grasp retunrs t
* add :namespace keyword to robot-interface, see [tickets:`#203 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/203>`_]
* remove / from /joint_states according to [tickets:`#202 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/202>`_]
* add -r option (headless) for fuerte
* until hydro, gazebo needs GPU to start, so use DISPLAY to :0.0 for test
* do not wrap around -180/180 degree [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* support :angle-vector over 360 degree, [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fix time-limit 300->600
* add test code for :angle-vector-with-constraint
* support :arms in :angle-vector-with-constraint, [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* retry twice if :move-gripper is not converged, see [`#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_]
* remove pause mode flag
* add :angle-vector-with-constraiont method, may be we can move to robot-interface?
* add tset code for `#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_
* expand pr2_empty_world.launch files to respawn gazebo
* add test code which show wait-interpolation get dead
* use package:// for loading speak.l
* groovy needs throttled true to launch head-less gazebo?
* add debug message for :start-grasp
* fix `#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_, use robot-update-state to double check the length between tips
* set time-limit to 300
* shorten test code
* return gripper with when simulation mode
* [`#159 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/159>`_] fix start-grasp, resend move-gripper when reached_goal is nil
* add test-start-grasp
* fix commit error [r4499]
* fix: relax camera position differs
* add keyword :use-tf2 and :joint-state-topic to robot-interface
* relax camera position differs
* update pr1012 bag/yaml file for new pr2 robot with sensor robot
* add comment to get bag files
* update pr2.l eus model with sensor head
* update robot_description dump for pr1040
* add PR2_NO argument to make-pr2-model-file-test.launch
* add urdf file which dumped robot_description in pr1040
* add pr2-ri-test.launch
* fix for joint name mismatch between ros and eus
* :move-to retunls nil if not reached to the goal (not closer than 200mm) `#160 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/160>`_
* relax test sequence
* do not use collada_urdf_jsk_patch, use collada_urdf
* (send *ri* :state :worldcoords) return worldcoords when *ri* simulation
* commit add :draw-objects methods, update robot-interface viewer while :move-to in simulation mode
* :move-to takes absolute coordinats as an arguments, currently it does not take into account frame-id, every coords must be relative to world
* add comment
* revert [`#1445 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/1445>`_], since min/max limit of infinite rotational joint has changed from 180 to 270 in https://sourceforge.net/p/jskeus/tickets/25/
* go-pos moves robot in relatively: fix code unless joint-action-enable, Fixed [`#146 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/146>`_]
* fix wreit-r of reset pose from 180->0 [`#145 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/145>`_]
* support :object key in :start-grasp [`#144 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/144>`_]
* support if link-list and move-target is not defined in dual-arm ik mode
* add pr2 ik test with both hands
* support when dual-arm-ik when link-list is not set
* use ros::service-call to change tilt_laser_mux/select [`#94 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/94>`_]
* use check-continuous-joint-move-over-180 for simulation-modep [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fixed tuckarm-pose angle-vector
* fix: using :{larm,rarm,head,torso}-controller and :{larm,rarm,head,torso}-angle-vector
* add use-tilt-laser-obstacle-cloud
* workaround for unintentional 360 joint rotation problem [`#91 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/91>`_]
* fix to work pr2-read-state with X-less environment [`#59 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/59>`_]
* change name cancel-all-goals -> go-stop and do not speak in the method, check joint-action-enable, [`#66 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/66>`_]
* add cancel-all-goals
* add test for start-grasp
* add :simulation-modep method to robot-interface
* do not launch viewer when robot-interface is already created [`#71 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/71>`_]
* add pr2-grasp-test
* support no display environment [`#59 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/59>`_]
* fix [`#49 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/49>`_] by mikita
* suport (send *ri* :init :objects (list (roomxxx))) style interface for simulation environment with objects [`#49 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/49>`_]
* fix: add keyword :timeout
* temporary remove :add-controller for pr2
* fix: larm-angle-vector and rarm-angle-vector
* update robot-interface.l for using joint group
* method for adding additional controllers
* fix: tuckarm pose
* add :wait-torso method to pr2-interface
* update for using (send *ri* :potentio-vector)
* fix `#50 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/50>`_, velocity limit for both plug/minus
* added wait option for stop-grasp
* use PLATFORM_FLOAT64 for daeFloat, collada-fom for groovy uses -DCOLLADA_DOM_DAEFLOAT_IS64, update pr2.l to use double precision value
* update: method :state .. use :update-robot-state
* remove debug message
* fix bug for continuous turning
* add a missing variable
* fix: initialization function name should be {robotname}-init
* fix: check absolute rotation angle
* using method :cancel-all-goals instead of :cancel-goal
* add :cancel-angle-vector and :stop-motion method for stopping motion
* add updated urdf file and corresponding bag files
* update pr2 model for fuerte
* autogenerating camera frame for fuerte
* fix calling ros::init if ros is not running
* add :ros-joint-angle for using meter/radian unit
* change: enable to pass robot instance
* fix minor bugs
* fix minor bugs
* fix for liner-joint
* add :send-trajectory to robot interface for using directly JointTrajectory.msg
* move pr2-arm-navigation from pr2eus to pr2eus_armnavigation
* add arm-navigation wrapper for PR2
* add pr2-arm-navigation.l for using arm_navigation stack
* fix go-pos-unsafe, cehck if reached to the original goal using odom and retly if needed, set minimum go-pos-unsafe time to 1000 add debug message
* move kinect_frame transform infrmatin to /opt/ros/electric/urdf/robot.xml
* remove description for static tf nodes
* find vector method from (send self :methods) if exists such as :reference-vector and :error-vector
* find vector method from (send self :methods) if exists such as :reference-vector and :error-vector
* add groupname to slots variables of robot-interface
* add ros node initialize check
* change variable name viewer -> create-viewer
* add pr2-interface setup function
* change for using private queue group in robot-interface in order to divide spin group
* use rosrun rosbag play instaed of rosrun rosbag rosbag
* use equal, not eq to check link name
* use string joint/link name rule, add pr2-senros-robot for camera model
* fix for r3056 (use string as link name too, see `#748 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/748>`_)
* support dual-arm ik which uses target-coords, move-target, and link-list as cons ;; fix move-arm, thre, and rthre definitions
* update tuckarm-pose for non-collision and min-max safe version
* support :joint-action-enable to change real/virtual robot environment. Ask users to really move robot? when :warningp is set, `#758 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/758>`_
* support :stop keyword to :inverse-kinematics
* use lib/llib/unittest.l
* use string-equal to check joint-name
* key of controller action name (:controller -> :controller-action)
* fixed to use string type joint names
* fix for jskeus r773 :gripper method in irtrobot class
* add reference/error vector method in robot-interface
* fix for joint with string name, euscollada/src/collada2eus.cpp@2969
* use string joint-name
* spin once before check robot state variables
* fix typo
* update for `#719 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/719>`_, add accessor to openni camera frames
* support loos checking of cmaera name, currently we are trying to move namer name from string style to keyword style
* use (pr2) to instantiate pr2 robot
* change parent of larm-end-coords from l/r_gripper_parm_link to l/r_gripper_tool_frame
* fix pr2.l compile rule
* use _roscore_failed for not run make-pr2-model-file without roscore and /robot_description environment
* eps=0.01 for camera projection check
* update pr2.l
* update pr2model to r2714 euscollada
* update pr2 model for r2693 or euscollada
* add a test for link weight, update pr2.l model file
* retake pr1012_sensors.bag
* update test bagfile for pr2 sensors and kinect/tf
* check link-coords, currently this is commented out
* fix openni camera link coordinates see jsk_pr2_startup/jsk_pr2_sensors/kinect_head.launch
* update test bagfile for pr2 sensors
* add debug message and add pr2-camera-coords-test
* add debug message
* update pr2eus-test to make robot model on the fly
* update l_finger_tip_link position
* fix syntax error on :publish-joint-state
* fix syntax error on :publish-joint-state
* update publish-joint-state for pr2, publish gripper joint_state
* remove dependency for pr2_* from roseus
* update pr2.l with safty controller limit
* add black color to kinect
* add test for link position
* rename j_robotsound -> robotsound_jp
* sleep 1 second after advertising
* add japanese speech topic for pr2-interface
* move robot-interface from roseus to pr2eus
* added sound_play function
* add kinect camera
* add strict check for camera number test
* fix make-pr2-model-file as urdf_to_collada supports dae file loading
* robot-interface :state with no argument is obsolated, and add warning messages
* :go-pos-unsafe updated, 1000 times msec
* removed initialize-costmap, this is obsolated
* I checked latest pr2.l works well by my program
* pr2-interface :state :odom :pose should return coordinates
* add test for sensor read methods of pr2-interface
* added :set-robot-state1 method to update robot-state variable, and store the time stamp of current joint_states
* changed global frame for (:move-to and :state :worldcoords), /map -> /world
* unchanged min-max angle is OK
* added prosilica and kinect camra to bag in test
* change count for wait slow camera info topic
* do not make error when expected difference between unstable and stable model
* fix assert message type
* add debug messages
* fix tpo in format string
* rename variable, use stable and unstable
* fix camera test code
* fix to work when camera_info is not found
* add make-pr2-model-file-test
* remove debug code
* fix make-pr2-model-file so that other package can use this
* default frame-id of pr2:move-to is /map
* pr2-robot does not calcurate joint-torque in torque-vector method
* changed to use robot-interface
* devide pr2-interface into robot common interface and pr2 specific methods
* check if velocity and efforts in /joint_states are same length as joint list
* added joint-action-enable check for :publish-joint-state
* instantiate transform-listener in ros-interface :init
* error handling when time list contains 0.0 in angle-vector-sequence
* miss understanding of pr2-robot origin coords, base_footprint
* add (if p) in pr2-interface :objects
* fix when frame_id is base_link
* fix compile warning -> velocities in :update-robot-state
* add :state :worldcoords, update :move-to, use :go-velocity after the robot reached gaol using move_base navigation controller
* dissoc before copy-object
* check viewer in :objects, because viewer only exists in simulation mode
* changed go-pos-unsafe to use 80% of max velocity
* remove x::draw-things
* fix :start-grasp, dissoc if already assoced, use x::draw-thing in :objects, etc
* fix segfault
* add :objects for simulation mode to display objects in pr2-interface viewer, also simulation mode is supported in :start-grasp and :stop-grasp
* add :gripper :links to return gripper links
* do not call dynamic reconfigure to static costmap, but it will repaired
* update navigation utility to electric
* add simulation mode to go-pos-unsafe and go-velocity
* add go-pos-unsafe
* update navigation parameter methods in pr2-interface
* change pr2-interface to update robot-model by joint_state msg which contains unknown joint names
* add joint-action-enable for :move-to
* add accessor to :robot and :viewer
* fix when x::*display* is 0
* fix type anlge -> angle
* change :start-grasp :wait nil -> t, and returns the space length of the gripper
* update :move-gripper, move gripper in simulation mode
* update pr2-tuckarm-pose smarter
* fix gripper joint manually
* update tuckarm pose method, and send angle-vector by each controller
* dump euscollada-robot definition to euscollada robot files and update pr2eus/pr2.l
* update pr2.l for latest euscollada/pr2.l ;; use euscollada-robot class instead of robot-model class ;; please refer to jsk-ros-pkg -r1822 commit
* fix previous commit : do not invoke viewer when no x:*display* found
* do not invoke viewer when no x:*display* found
* add pr2-ik-test.l and pr2eus-test.launch
* fix l_gripper_r_finger_tip_link -> l_wrist_roll_link
* add pr2-ik-test.l
* manually fix bug `#560 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/560>`_
* use palm link as parent of endcoords
* update with kinect model
* update pr2 model with safety_limit
* use :state :potentio-vector instead of old :state method call
* update pr2-read-state.l to draw torque
* add max velocity and torque in :init-ending
* set the name of base_trajectory action to same other actions
* fix typo pr2_base_trajectory_action
* update topic name for pr2_base_trajectory_action
* revert accidentally commit
* update namespace of pr2_base_trajectory_action
* add publish-joint-state method, which publish joint_states when joint-action-enable is nil
* set joint-action-enable t before wait-fore pr2-action-server
* wait for joint-velocity to zero, in wait-interpolation for pr2
* add defun make-camera-from-ros-camera-info-aux
* make-camera-from-ros-camera-info-aux is required for non-roseus users
* fix *hrp4* -> robot
* split pr2-interface to pr2-interface and ros-interface
* remove defun make-camera-from-ros-camera-info-aux, which is now defined in roseus-utils.l
* support :state :torque-vector, by mikita
* add effort to state in pr2-interface class
* use :torso_lift_joint method
* add dummy massproperty pr2.l
* add message name to constant in msg definition
* update pr2.l model 2010523
* add clear-costmap, initialize-costmap, change-inflation-range, call clear-costmap when the robot retry move-to function i n (send *ri* :move-to)
* fix contious rotational joint problems, pr2 controller use joint angle value directory, so we add offset before sending the trajectory
* add and fix sub-angle-vector method, fix simulation mode
* :angle-vector-sequence returns angle-vector-sequence
* send only one message in pr2-angle-vector-sequence method
* fix diff-angle-vector in :angle-vector-sequence
* add diff-angle-vector function in :anlge-vector-sequence for calculating velocity vector for interpolation
* cropping angle of infinite rotational joint supported in irtmodel.l
* set :min and :max for infinite rotational joint is *inf* and *-inf*
* add simulation mode code in :angle-vector-sequence
* draw interpolated postures unless joint-action-enable in :angle-vector
* remove typo
* remove spin-once in (:angle-vector-sequence
* remove spin-once in (:angle-vector
* fix :inverse-kinematics move-arm move-target link-list, `#493 <https://github.com/jsk-ros-pkg/jsk_pr2eus/issues/493>`_
* if no viewer is executed before pr2-interface viewer, set pr2-interface viewer as a defulat *viewer*, so that users are able to use them as a default view
* fix fingertip pressure zero-reset, update pr2-read-state sample
* add ** to msg constant type
* we can send JointTrajectoryActionGoal to torso and head in diamondback
* update grasp timing in tuckarm-pose, add pr2-reset-pose
* add pr2 tuckarm pose function
* remove useless number 1 in ros::ros-warn
* use ros::ros-warn instaed of warning-message
* support sending go-velocity countinously, and once
* support sending go-velocity countinously
* fix go-velocity function
* add go-velocity method using trajectoy and safe_teleop
* add go-velocity to pr2-interface.l
* torso and head did not accept time_from_start, it only accept duration
* update pr2.l with :camera and :cameras
* add to generate :cameras and :camera by chen and k-okada
* require pr2-utils, show viewer in NON-joint-action-enable mode
* if robot-joint-disabled, :state sends recieved angle-vector
* pr2-interface :init works unless it connected to pr2
* update ros-infro comment
* update pr2.l using r769
* update :*-cmaera method definitoin, support forward-message-to
* fix :inverse-kinematics with use-base
* update :inverse-kinematics with use-base
* update :inverse-kinematics support use-torso, use-base, move-arm
* In head point action, pointing_frame is not used, and change translate length
* add fingertip pressure subscriber, to use finger-pressure call reset-fingertip beforehand
* set time out for gripper action
* action start time should be future, i think
* use :wait-interpolation, remove sleep
* fix do not generate pr2.l if it already exists
* add move_base_msgs
* fix problem, when not add roseus to /home/k-okada/ros/cturtle/ros/bin:/usr/local/cuda/bin/:.:/home/k-okada/bin:/usr/local/bin:/usr/local/svs/bin:/usr/java/j2sdk1.4.1/bin/:/usr/bin:/bin/:/usr/sbin:/sbin:/usr/X11R6/bin:/usr/local/jsk/bin:/home/k-okada/ros/cturtle/jsk-ros-pkg/euslisp/jskeus/eus/Linux/bin:/bin:/usr/h8300-hitachi-hms/bin:/usr/local/ELDK4.1/usr/bin:/home/k-okada/prog/scripts:/usr/local/src/gxp
* rename cmaera->camera-model, viewing->vwing
* update pr2model with new make-camera-from-ros-info-aux
* update to new make-camera-from-ros-info-aux
* update pr2 model file
* add pr2 model file at 100929
* delete load-pr2-file.l
* load-pr2-file is removed, now we use make-pr2-modle-file
* generate pr2model from camera_info and /robot_description
* front of high_def_frame is +x
* set pointing_frame to look-at-point action goal
* fix to move head-end-coords in sending current pose
* update :angle-vector-sequence to work with real-pr2 robot
* add :angle-vector-sequence based on interpolator::push in rats/src/interpolator.cpp
* update :send-pr2-controller interface (:send-pr2-controller nil (action joint-names all-positions all-velocities starttiem duration)
* support send *pr2* :inverse-kinematics c
* add test code for load-pr2-file
* add load-pr2-file
* add dual arm jacobian, torque sample by s.nozawa
* fix pr2 gripper action sending
* add hrp2 compatible :go-pos [m] [m] [degree] method
* remove waiting for move-base action in pr2-interface :init
* change to startable pr2-interface when move_base not found
* add :move-to method and move-base-action slot variable
* add :gripper and :override :limb of irtrobot.l to suppoer send *pr2* :larm :gripper :angle-vector
* change to use roseus, whcih automatically load roseus.l eustf.l actionlib.l
* change to use pr2.l in pr2eus directory
* rosmake pr2eus to generate pr2.l
* fix to use require for eustf and actionlib
* revert to r527 float mod is supported in eus
* result of (r2deg p) should be integer for using mod
* crop joint-angle to +- 360 in :state :potentio-vector
* add depend package
* add gripper action to pr2-interface
* wait at most 10 seconds
* fix return-from, in :state method
* fix syntax error (require :keyword path) <- (require path)
* add pr2_controllers_msgs
* fix to use package:// load style
* rename roseus-add-{msgs,srvs}->ros::roseus->add-{msgs,srvs}
* pr2model is obsoluted
* add pr2 ros controlelr and euslisp interface
* add utility functions for pr2 euslisp model
* add sample program and launch file for PR2 users
* remove piped-fork and use ros::rospack-find
* modify pr2model.l to head joint
* add reset manip pose to pr2
* fix pr2model, support :fix and :relative mode in :inverse-kinematics, see hold-cup in 2010_05_pr2ws/sample-motion.l for example
* override :init, set reset-pose as initial pose
* fix many bags to move pr2 by joint angle actionlib interface
* change middle-body-joint-angle-list API: omit string-upcase for joitn name
* add pr2eus model, which depends on urdf2eus
* Contributors: Haseru Chen, Yuki Furuta, Kei Okada, Yuto Inagaki, Satoshi Iwaishi, Manabu Saito, Shunichi Nozawa, Kazuto Murase, Masaki Murooka, Ryohei Ueda, Yohei Kakiuchi, Yusuke Furuta, Hiroyuki Mikita, Otsubo Satoshi
