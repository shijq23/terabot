# terabot

A 4 wheel autonomous robot visual navigation by D435i.

## reference

* [robotic simulation](https://www.generationrobots.com/blog/en/robotic-simulation-scenarios-with-gazebo-and-ros/)

## environment

* Ubuntu 18.04 LTS
* ROS Melodic

## setup workspace

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## get source code

```bash
cd ~/git
git clone https://github.com/shijq23/terabot.git
cd ~/catkin_ws/src
ln -s ~/git/terabot/tarabot_control
ln -s ~/git/terabot/terabot_description
ln -s ~/git/terabot/terabot_gazebo
cd ~/catkin_ws
catkin make
source devel/setup.bash
```

## launch gazebo

```bash
roslaunch terabot_gazebo terabot_world.launch
```

## move it by command

```bash
rostopic pub -1 /terabot/leftWheel_effort_controller/command std_msgs/Float64 "data: 11.5"
rostopic pub -1 /terabot/rightWheel_effort_controller/command std_msgs/Float64 "data: 11.0"
```

## monitor joint states

```bash
rostopic echo /terabot/joint_states
```

## move it by teleop

```bash
rosrun turtlebot3_teleop turtlebot3_teleop_key cmd_vel:=/terabot/cmd_vel
roslaunch terabot_gazebo teleop_key.launch

```

## view camera image

```bash
rosrun image_view image_view image:=/terabot/camera1/image_raw
```

## view tf frames

```bash
rosrun tf view_frames
rosrun tf view_monitor
roswtf
```
# visualize the TF Tree

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

## node graph

```bash
rqt_graph
```

## find a pi

```bash
sudo nmap -sP 192.168.2.1/24 | awk '/^Nmap/{ip=$NF}/B8:27:EB/{print ip}'
```

## check parameters configuration

```bash
rosrun rqt_gui rqt_gui
rosrun rqt_reconfigure rqt_reconfigure
```

## ros-i2cpwmboard error

sudo apt-get install python-smbus i2c-tools
sudo apt-get install libi2c-dev

## interactive marker

in rviz, add InteractiveMarkers, select Update topic: /twist_marker_server/update
