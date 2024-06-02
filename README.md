# ME495 Sensing, Navigation and Machine Learning For Robotics

* Allen Liu
* Winter 2024
* [Portfolio Post](https://nu-jliu.github.io/slam/)

## Package List

This repository consists of several ROS packages

* [nuturtle_description](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuturtle_description) - Display the turtlebot in the rviz

* [turtlelib](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/turtlelib) - The library used for geometry calculations

* [nusim](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nusim) - Simulate the turtlebot on rviz

* [nuturtle_control](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuturtle_control) - Controls the motion of the turtlebot.

* [nuslam](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuslam) - Using `Extended Kalman Filter` to perform `SLAM` algorithm on a turtlebot3

## Software Setup

To install all required packages

```bash
mkdir -p ~/ws/slam_ws/src
cd ~/ws/slam_ws
vcs import --input https://raw.githubusercontent.com/ME495-Navigation/slam-project-nu-jliu/main/turtle.repos src
```

## EFK-SLAM

### Software Structure

The rqt_graph is shown in the figure below:
![rosgraph](nuslam/images/rosgraph_landmark.svg)

### Launch

To launch the slam with landmark detection in simulation run

```bash
ros2 launch nuslam landmarks.launch.xml cmd_src:=teleop
```

### Video Demo

<!-- <video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/1090f3eb-7a68-45b1-9b95-fd0f915f2d55" controls></video> -->

![demo](https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/1090f3eb-7a68-45b1-9b95-fd0f915f2d55)
