# ME495 Sensing, Navigation and Machine Learning For Robotics
* Allen Liu
* Winter 2024

## Package List
This repository consists of several ROS packages
- [nuturtle_description](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuturtle_description) - Display the turtlebot in the rviz
- [turtlelib](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/turtlelib) - The library used for geometry calculations
- [nusim](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nusim) - Simulate the turtlebot on rviz
- [nuturtle_control](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuturtle_control) - Controls the motion of the turtlebot.
- [nuslam](https://github.com/ME495-Navigation/slam-project-nu-jliu/tree/main/nuslam) - Using `Extended Kalman Filter` to perform `SLAM` algorithm on a turtlebot3

## Software Setup
To install all required packages
```
mkdir -p ~/ws/slam_ws/src
cd ~/ws/slam_ws
vcs import --input https://raw.githubusercontent.com/ME495-Navigation/slam-project-nu-jliu/main/turtle.repos?token=GHSAT0AAAAAACHGN4BBCRW7U6I35BKHJAREZPXLFBQ src
```