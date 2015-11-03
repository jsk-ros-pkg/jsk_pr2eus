^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2eus_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2015-11-03)
------------------

0.1.11 (2015-06-11)
-------------------

0.1.10 (2015-04-03 18:49)
-------------------------

0.1.9 (2015-04-03 16:52)
------------------------

0.1.8 (2015-02-25)
------------------
* [pr2eus_moveit] package.xml fix version number
* [pr2eus_moveit] Catkinize pr2eus_moveit
* Contributors: Kei Okada, aginika

0.1.7 (2015-02-10)
------------------
* fix typo
* add code for using action-server instead of service
* add check-state-validity service and fix minor bug
* fix bug in collision-object-publisher.l
* change moveit groupname
* add code for using arms
* added eus2scene.l
* add publish-eusscene-marker.l
* add publish-eusscene.l
* Contributors: YoheiKakiuchi, mmurooka, tarukosu

0.1.6 (2014-05-11)
------------------

0.1.5 (2014-05-03)
------------------

0.1.4 (2014-05-02 22:28)
------------------------

0.1.3 (2014-05-02 18:04)
------------------------

0.1.2 (2014-05-01 22:43)
------------------------

0.1.1 (2014-05-01 02:14)
------------------------
* add pr2-tabletop-demo, picking object up on table useing moveit
* comment out debug message
* update pr2eus_moveit for using constraints
* Merge pull request #9 from YoheiKakiuchi/add_use_directly_joint_trajectory
  use joint trajectory mode for moveit
* add clear-world-scene method to pr2eus-moveit
* use joint trajectory mode for moveit
* fix typo :frame_id -> :frame-id
* enable to set object-id with keyword
* update publish-eusobject.l
* add publish-eusobject.l for publishing eus model to moveit environment
* change loading order for pr2eus-moveit
* fix minor bug
* add pr2-moveit.l
* fix typo
* add publish-collision-object
* add make-virtual-joint-constraints
* add :motion-plan-raw method for testing planning
* delete method for attached-object
* add :add-attached-object to collision-object-publisher
* add :query-planner-interface to pr2eus-moveit
* update
* add keyword for adding constraints to motion-plan
* add making constraints functions
* update pr2eus-moveit.l
* add method for robot-interface on pr2eus-moveit
* update pr2eus-moveit
* update pr2eus_moveit tutorials
* add :sync-robot-model method to pr2eus-moveit
* move collision-object-sample.l to tutorials
* add tutorials to pr2eus_moveit
* add :execute-trajectory method to pr2eus-moveit
* update sample for pr2eus_moveit
* add updating faces coords
* add collision-object-sample
* add :relative-pose keyword to collision-object-publisher.l
* fix typo and minor bug
* implement :motion-plan method to pr2eus-moveit.l
* fix typo
* add using torso configuration to pr2eus-moveit.l
* rename pr2eus_moveit.l -> pr2eus-moveit.l
* implement :get-ik-for-pose to moveit-environment
* rename scene-topic -> scene-service
* add pr2eus_moveit.l for using moveit from roseus interface
* add package dependancy to pr2eus_moveit
* move :get-planning-scene method to get-planning-scene function
* add pr2eus_moveit for using moveit components from roseus
* Contributors: Yohei Kakiuchi, YoheiKakiuchi, youhei
