# pr2eus_tutorials

This is a repository for tutorials of pr2eus

## Demos

### PR2 Tabletop Object Detection

#### Real robot

```bash
# on PR2 real robot
roslaunch pr2eus_tutorials pr2_tabletop.launch
```

#### Simulation

```bash
# on local machine
rosrun pr2eus_tutorials pr2_tabletop_sim.sh # PLEASE NOTE NOT `roslaunch` BUT `rosrun`!
```

In sample code, pr2 brings up detected object.

```bash
rosrun pr2eus_tutorials pr2-tabletop-sample.l
```

![pr2_tabletop_sim](https://gist.githubusercontent.com/furushchev/b3f3bb08953407966f80f4b0ac70c7dd/raw/pr2_tabletop_screen.png)

## Reach Object Demo
### Reach Object Demo with PR2

pr2_gazebo is required.

```
# launch gazebo
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2eus_tutorials spawn_objects.launch

# launch recognition
roslaunch jsk_pcl_ros hsi_color_filter.launch INPUT:=/wide_stereo/points2 h_min:=75 s_min:=50

# visualization
rviz -d `rospack find pr2eus_tutorials`/config/pr2_reach_object.rviz

# eus
roscd pr2eus_tutorials/euslisp
roseus reach-object.l
(pr2-setup)
(reach-object-demo)
```

### Reach Object Demo with HRP2JSK

hrpsys_gazebo_tutorials is required.
(HRP2 model is closed.)

```
# launch gazebo
roslaunch hrpsys_gazebo_tutorials gazebo_hrp2jsk_no_controllers.launch
roslaunch pr2eus_tutorials spawn_objects.launch

# lanch hrpsys
rtmlaunch hrpsys_gazebo_tutorials hrp2jsk_hrpsys_bringup.launch KINEMATICS_MODE:=true

# launch recognition
roslaunch jsk_pcl_ros hsi_color_filter.launch INPUT:=/xtion/depth/points h_min:=75 s_min:=50

# visualization
rviz -d `rospack find pr2eus_tutorials`/config/hrp2jsk_reach_object.rviz

# eus
roscd pr2eus_tutorials/euslisp
roseus reach-object.l
(hrp2jsk-setup)
(reach-object-demo)
```
