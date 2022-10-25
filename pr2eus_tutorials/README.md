# pr2eus_tutorials

This is a repository for tutorials of pr2eus

## Installation

To play with this package, you can choose from two installation methods.
We assume the ROS environment is `kinetic`.
If you use ROS `indigo` distribution, please replace the word `kinetic` with `indigo` (or appropriate distributions).

### Using pre-built package

#### Install ROS

Follow the [instruction of ROS installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)

#### Install the package

```bash
sudo apt install ros-kinetic-pr2eus-tutorials
```

#### Load ROS Environment

```bash
source /opt/ros/kinetic/setup.bash
```

### Using source package

#### Install ROS

Follow the [instruction of ROS installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)

#### Setup catkin workspace

```bash
source /opt/ros/kinetic/setup.bash
sudo apt install python-catkin-tools python-wstool python-rosdep git
sudo rosdep init
rosdep update
# Create catkin workspace and download source repository
mkdir -p ~/ros/kinetic/src && cd ~/ros/kinetic/src
wstool init
wstool set jsk-ros-pkg/jsk_pr2eus --git https://github.com/jsk-ros-pkg/jsk_pr2eus.git -v master
wstool update
# Install dependencies for building the package
rosdep install --from-paths . -i -r -n -y
# Build the package
cd ~/ros/kinetic
catkin init
catkin build
```

#### Load ROS Environment

```bash
source ~/ros/kinetic/devel/setup.bash
```

## Tabletop Object Grasping Demo

### Tabletop Object Grasping Demo with PR2

#### Startup nodes

First we need to start nodes used for this demo.

##### Using a real robot

```bash
# on PR2 real robot
ssh <robot address>
roslaunch pr2eus_tutorials pr2_tabletop.launch
```

If you try using compressed topic from the real PR2,

```bash
# on local machine
roslaunch pr2eus_tutorials pr2_tabletop.launch remote:=true
```

You can locate a desk in front of the robot and put any objects on it.

##### Using a simulator

You can set physics engine with roslaunch argument.

```bash
# on local machine
# It may take time to download materials for the first time
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
```

You can see the robot is spawned in a scene with a desk and some objects.

#### Run demo

Then we can now start the demo program for picking objects.

##### Start Euslisp program

```bash
# on local machine
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
```

##### Start Rviz

After running the demo program above, you can see object bounding boxes in the `RViZ` window.
It means the robot now recognizes each objects as individual objects from camera sensor inputs.

```bash
# on local machine
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
```

If you try `roslaunch pr2eus_tutorials pr2_tabletop.launch` with `remote:=true`, meaning using compressed topic from the real PR2, execute this instead.
```bash
# on local machine
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop_remote.rviz
```

##### Additional setup for Kinetic local machine with a real robot

If you want to know why we need these node, please see [here](https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/387#issuecomment-470505882).
You also need to switch `Tabletop Object` rviz panel topic from `/bounding_box_interactive_marker/update` to `/bounding_box_interactive_marker/kinetic/update` .

```bash
# on local machine
rosrun jsk_robot_utils marker_msg_from_indigo_to_kinetic.py
rosrun topic_tools relay /bounding_box_interactive_marker/kinetic/feedback  /bounding_box_interactive_marker/feedback
```

##### Click object bouding on Rviz

You can click any object that you want the robot to pick up.


![pr2_tabletop_sim](https://gist.githubusercontent.com/furushchev/b3f3bb08953407966f80f4b0ac70c7dd/raw/pr2_tabletop_screen.png)


##### Click object on image_view2

You can click any pixels that you want the robot to pick up.

```bash
rosrun image_view2 image_view2 image:=/kinect_head/rgb/throttled/image_rect_color camera_info:=/head_mount_kinect/rgb/camera_info
```

If you try `roslaunch pr2eus_tutorials pr2_tabletop.launch` with `remote:=true`, execute this instead.

```bash
rosrun image_view2 image_view2 image:=/kinect_head_remote/rgb/throttled/image_rect_color camera_info:=/head_mount_kinect/rgb/camera_info
```

![pr2_interactive](https://user-images.githubusercontent.com/19769486/81817136-3913f200-9567-11ea-800a-f95bd4057cb5.png)


### Step-by-step Description of Demo program

In the bottom of the demo program `pr2-tabletop-object-grasp.l`, you can see a main function `demo`.

```lisp
(defun demo ()
  (setq *grasping-object-p* nil)
  (setq *arm* :rarm)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (pr2-init)
  (pr2-pregrasp-pose)
  (wait-for-grasp-target))
```

The `(pr2-init)` method is just a initialization function for pr2 robot that instantiate two objects required for robot manipulation from euslisp:

- `*pr2*`: This object is a kinematic model for a PR2 robot. This object includes any fundamental functions for robot modeling such as inverse kinematics, dynamics, geometric constraints and so on. You can visualize this model by evaluating `(objects (list *pr2*))`.
- `*ri*`: This is an object that send a control signal to the actual robot from euslisp kinematics model and receive the result or actual states of the robot. `ri` is an abbreviation of `robot interface`.

Please note that `(pr2-init)` function is defined in `pr2-interface.l` in the `pr2eus` package.
In this demo program, the function is loaded in the top of the script:

```lisp
(require :pr2-interface "package://pr2eus/pr2-interface.l")
```

After `(pr2-init)` is executed, the kinematic model looks like below:

![pr2-reset-pose](https://user-images.githubusercontent.com/1901008/39504750-d44efa06-4e08-11e8-8aef-7c0f3ce0802b.png)


After initialized the robot in euslisp, `(pr2-pregrasp-pose)` method is executed.

```lisp
(defun pr2-pregrasp-pose ()
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation))
```

The first line `(send *pr2* :reset-manip-pose)` changes the pose of euslisp kinematic model to the predefined pose called `reset-manip-pose`.
With calling this method, the actual robot does **NOT** move because this method only changes the states of the kinematic model. Instead you can see the current states of the kinematic model by `(objects (list *pr2*))`.
The kinematic model now looks like below:

![pr2-reset-manip-pose](https://user-images.githubusercontent.com/1901008/39504749-d42a4ff8-4e08-11e8-8597-6ca54b5a97e7.png)

The second line then send the current state of kinematic model to the actual robot.
The second argument `5000` allows the robot to take 5000 milliseconds to move to the pose. If the second argument is omitted, the default argument `3000` will be used.

After the second line, it will take 5 seconds until the robot ends to move, but the method call itself returns immediately.
The last method called in the last line is just for waiting for the robot until he ends to move to the specified pose.


## Reach Object Demo
### Reach Object Demo with PR2

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

`hrpsys_gazebo_tutorials` is required.
(Currently, HRP2 model is not provided for open source projects.)

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
