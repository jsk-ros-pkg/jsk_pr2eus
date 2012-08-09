pr2eus_openrave ROS Launch Files
================================

**Description:** pr2eus_openrave

  
  
       pr2eus_openrave
  
    

**License:** BSD

collision_map_publisher.launch
------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave collision_map_publisher.launch

hironx_openrave_armplan.launch
------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave hironx_openrave_armplan.launch

hrp4c_openrave_armplan.launch
-----------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave hrp4c_openrave_armplan.launch

openrave_armplan.launch
-----------------------

.. code-block:: bash

  roslaunch pr2eus_openrave openrave_armplan.launch

pr2_openrave_armplan.launch
---------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave pr2_openrave_armplan.launch



This launch file is expected to be used inside PR2.

- Nodes: laser_assembler, pr2_laser_snapshotter and collision_map_self_occ_node are used to publish Topic: 'collision_map_occ'.

- A Node: armplanning_openrave.py is used to bridge Openrave and ROS.

.. image:: launch/images/pr2eus_openrave_sample.png
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
    <node args="--collision_map=/collision_map_mux        --wait-for-collisionmap=30 --viewer=''        --request-for-joint_states='service'" name="pr2eus_openrave_armplan" output="screen" pkg="orrosplanning" type="armplanning_openrave.py" />
    <node args="collision_map_mux collision_map_none collision_map_occ" name="mux_for_colmap" output="screen" pkg="topic_tools" type="mux">
      <remap from="mux" to="collision_map_mux" />
    </node>
    
    <node name="arm_navigation_point_cloud_assembler" output="screen" pkg="laser_assembler" respawn="true" type="point_cloud_assembler">
      <remap from="cloud" to="tilt_scan_filtered" />
      <param name="tf_cache_time_secs" type="double" value="10.0" />
      <param name="tf_tolerance_secs" type="double" value="0.0" />
      <param name="max_clouds" type="int" value="400" />
      <param name="ignore_laser_skew" type="bool" value="true" />
      <param name="fixed_frame" type="string" value="base_link" />
    </node>
    
    <node name="snapshotter" output="screen" pkg="pr2_arm_navigation_perception" respawn="true" type="pr2_laser_snapshotter">
      <param name="num_skips" type="int" value="1" />
      <remap from="laser_scanner_signal" to="laser_tilt_controller/laser_scanner_signal" />
      <remap from="build_cloud" to="arm_navigation_point_cloud_assemble/build_cloud" />
      <remap from="full_cloud" to="full_cloud_filtered" />
    </node>
  
    
    <node name="collision_map_self_occ_node" output="screen" pkg="collision_map" respawn="true" type="collision_map_self_occ_node">
      
      <param name="self_see_default_padding" type="double" value="0.04" />
      
      <param name="self_see_default_scale" type="double" value="1.0" />
      
      <param name="publish_static_over_dynamic_map" type="bool" value="true" />
      <param name="fixed_frame" type="string" value="base_link" />
      
      <param name="robot_frame" type="string" value="base_link" />
      <param name="origin_x" type="double" value="1.1" />
      <param name="origin_y" type="double" value="0.0" />
      <param name="origin_z" type="double" value="0.0" />
  
      <param name="dimension_x" type="double" value="1.5" />
      <param name="dimension_y" type="double" value="2.0" />
      <param name="dimension_z" type="double" value="2.0" />
  
      
      <param name="resolution" type="double" value="0.01" />
      
      <rosparam command="load" file="$(find pr2_arm_navigation_tutorials)/config/collision_map_sources.yaml" />
    </node>
  </launch>

pr2_openrave_armplan_server.launch
----------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave pr2_openrave_armplan_server.launch


  This launch file is expected to be used inside Local PC.

  Unlike pr2_openrave_armplan.launch, machine tag including localhost and c1 is already set. Nodes: laser_assembler, pr2_laser_snapshotter
  and collision_map_self_occ_node will be launched inside PR2, a Node: armplanning_openrave.py will be launched inside Local PC.

  

Contents
########

.. code-block:: xml

  <launch>
    <machine address="localhost" name="localhost" ros-package-path="$(env ROS_PACKAGE_PATH)" ros-root="$(env ROS_ROOT)" />
    <machine address="pr1012" name="c1" ros-package-path="$(env ROS_PACKAGE_PATH)" ros-root="$(env ROS_ROOT)" />
    
    <node args="--collision_map=/collision_map_mux --mapframe=''        --wait-for-collisionmap=30 --request-for-joint_states='service'" machine="localhost" name="pr2eus_openrave_armplan" output="screen" pkg="orrosplanning" type="armplanning_openrave.py" />
    <node args="collision_map_mux collision_map_none collision_map_occ" machine="c1" name="mux_for_colmap" output="screen" pkg="topic_tools" type="mux">
      <remap from="mux" to="collision_map_mux" />
    </node>
    
    <node machine="c1" name="arm_navigation_point_cloud_assembler" output="screen" pkg="laser_assembler" respawn="true" type="point_cloud_assembler">
      <remap from="cloud" to="tilt_scan_filtered" />
      <param name="tf_cache_time_secs" type="double" value="10.0" />
      <param name="tf_tolerance_secs" type="double" value="0.0" />
      <param name="max_clouds" type="int" value="400" />
      <param name="ignore_laser_skew" type="bool" value="true" />
      <param name="fixed_frame" type="string" value="base_link" />
    </node>
    
    <node machine="c1" name="snapshotter" output="screen" pkg="pr2_arm_navigation_perception" respawn="true" type="pr2_laser_snapshotter">
      <param name="num_skips" type="int" value="1" />
      <remap from="laser_scanner_signal" to="laser_tilt_controller/laser_scanner_signal" />
      <remap from="build_cloud" to="arm_navigation_point_cloud_assemble/build_cloud" />
      <remap from="full_cloud" to="full_cloud_filtered" />
    </node>
  
    
    <node machine="c1" name="collision_map_self_occ_node" output="screen" pkg="collision_map" respawn="true" type="collision_map_self_occ_node">
      
      <param name="self_see_default_padding" type="double" value="0.04" />
      
      <param name="self_see_default_scale" type="double" value="1.0" />
      
      <param name="publish_static_over_dynamic_map" type="bool" value="true" />
      <param name="fixed_frame" type="string" value="base_link" />
      
      <param name="robot_frame" type="string" value="base_link" />
      <param name="origin_x" type="double" value="1.1" />
      <param name="origin_y" type="double" value="0.0" />
      <param name="origin_z" type="double" value="0.0" />
  
      <param name="dimension_x" type="double" value="1.5" />
      <param name="dimension_y" type="double" value="2.0" />
      <param name="dimension_z" type="double" value="2.0" />
  
      
      <param name="resolution" type="double" value="0.01" />
      
      <rosparam command="load" file="$(find pr2_arm_navigation_tutorials)/config/collision_map_sources.yaml" />
    </node>
  </launch>

pr2_openrave_simulation.launch
------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave pr2_openrave_simulation.launch


  This launch file is used for simulation.

  Please check test/test_pr2_openrave_simulation.launch
  

Contents
########

.. code-block:: xml

  <launch>
    <param name="robot_description" textfile="$(find pr2_mechanism_model)/pr2.urdf" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
    <arg default="1" name="COLLISION_MAP_WAIT_TIME" />
    <node args="--request-for-joint_states='topic'        --collision_map=/collision_map_occ_throttle        --wait-for-collisionmap=$(arg COLLISION_MAP_WAIT_TIME) --use-simulation='true'" name="armplanning_openrave" output="screen" pkg="orrosplanning" type="armplanning_openrave.py" />
    
  </launch>

test_hironx_openrave_simulation.launch
--------------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave test_hironx_openrave_simulation.launch


roseus + openrave example for Kawada Hiro Robot

.. video:: $JENKINS_URL/job/$JOB_NAME/lastSuccessfulBuild/artifact/doc/$JOB_NAME/html/_images//rviz_hironx_openrave
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find pr2eus_openrave)/launch/hironx_openrave_armplan.launch">
      <arg name="test" value="true" />
    </include>
  
    </launch>

test_hrp4c_openrave_simulation.launch
-------------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave test_hrp4c_openrave_simulation.launch

test_pr2_openrave_simulation.launch
-----------------------------------

.. code-block:: bash

  roslaunch pr2eus_openrave test_pr2_openrave_simulation.launch


roseus + openrave example.

.. video:: $JENKINS_URL/job/$JOB_NAME/lastSuccessfulBuild/artifact/doc/$JOB_NAME/html/_images//rviz_pr2_openrave
  :width: 400

  

Contents
########

.. code-block:: xml

  <launch>
    <include file="$(find pr2eus_openrave)/launch/pr2_openrave_simulation.launch">
      <arg name="COLLISION_MAP_WAIT_TIME" value="20" />
    </include>
    <node args="--sync -d $(find pr2eus_openrave)/test/test_pr2eus_openrave.vcg" launch-prefix="glc-capture --start --out=$(find pr2eus_openrave)/build/rviz_pr2_openrave.glc" name="rviz_pr2_openrave" pkg="rviz" type="rviz" />
  
    <test args="$(find pr2eus_openrave)/test/pr2_openrave_simulation.l" name="pr2eus_openrave_eus" pkg="roseus" test-name="pr2_openrave_simulation" time-limit="3000" type="roseus" />
    
    <test args="$(find pr2eus_openrave)/build/rviz_pr2_openrave.glc" pkg="jsk_tools" test-name="z_pr2_openrave_encode" time-limit="3000" type="glc_encode.sh" />
    <param name="use_sim_time" value="true" />
    <node args="$(find pr2eus_openrave)/test/collision_map.bag --clock -d 5" name="rosbag_play_collision_map" pkg="rosbag" type="play" />
    <node args="0.130 0.016 1.338 -1.977 0.000 -1.487 /base_link /openni_rgb_optical_frame 100" name="base_to_kinect" pkg="tf" type="static_transform_publisher" />
  
    
  </launch>

