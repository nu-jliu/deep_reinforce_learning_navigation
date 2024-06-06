# EE473 Deep Reinforcement Learning

* Allen Liu
* Spring 2024
* [Portfolio Post](https://nu-jliu.github.io/slam/)

## Package List

This repository consists of several ROS packages

* [nuturtle_description](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuturtle_description) - Display the turtlebot in the rviz

* [turtlelib](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/turtlelib) - The library used for geometry calculations

* [nusim](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nusim) - Simulate the turtlebot on rviz

* [nuturtle_control](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuturtle_control) - Controls the motion of the turtlebot.

* [nuslam](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuslam) - Using `Extended Kalman Filter` to perform `SLAM` algorithm on a turtlebot3

* [nuturtle_interfaces](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuturtle_interfaces) - All message interfaces for communication between the nodes

* [nuturtle_slam](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuturtle_slam) - Intergrare `slam_toolbox` for setting up learning environment

* [nuturtle_deep_rl](https://github.com/nu-jliu/deep_reinforce_learning_navigation/tree/main/nuturtle_deep_rl) - Wrapped with `OpenAI` gym for `DeepRL` traing environment

## Software Setup

### System Requirement

Ubuntu 22.04

### Install ROS2 Iron

Refer official `ROS2` document: [Install ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

#### Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Add Required Repositories

Add `ROS2` GPG key

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add repositories to `sources.list`

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install Development Tools

```bash
sudo apt update
sudo apt install ros-dev-tools
```

#### Install `ROS2`

Fetch updated respoitory

```bash
sudo apt update
```

Upgrade the system

```bash
sudo apt upgrade
```

Install `ROS2 Iron`

```bash
sudo apt install ros-iron-desktop-full
```

### Install All packages

To install all required packages, first clone all source code

```bash
mkdir -p ~/ws/slam_ws/src
cd ~/ws/slam_ws
vcs import --input https://raw.githubusercontent.com/ME495-Navigation/slam-project-nu-jliu/main/turtle.repos src
```

Build and install

```bash
cd ~/ws/slam_ws
colcon build --symlink-install
```

<!-- ## EFK-SLAM

### Software Structure

The rqt_graph is shown in the figure below:
![rosgraph](nuslam/images/rosgraph_landmark.svg)

### Launch

To launch the slam with landmark detection in simulation run

```bash
ros2 launch nuslam landmarks.launch.xml cmd_src:=teleop
```

### Video Demo

<video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/1090f3eb-7a68-45b1-9b95-fd0f915f2d55" controls></video> -->

## Deep Reinforcement Learning

### Software Structure

The `rqt_graph` of the entire system is shown in the figure below:

![rqt_graph](rosgraph.svg)

### Video Demo
https://github.com/nu-jliu/deep_reinforce_learning_navigation/assets/49068329/805b5a28-166a-4720-b573-a29af80eb05f

#### Train
The Training process is shown in the video below:

https://github.com/nu-jliu/deep_reinforce_learning_navigation/assets/49068329/7440550b-90f7-4124-9d45-86449c826d3b



