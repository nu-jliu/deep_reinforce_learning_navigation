# NUSLAM

**Author**: Allen Liu

This package is used for performing `Extended-Kalman-Filter` Based `SLAM` algorithm on a turtlebot

## Frames
The structure of the frames is shown in the figure below:

![](images/frames.svg)

## SLAM with Fake Sensor

### Software Structure
The rqt_graph is shown in the figure below:
![](images/rosgraph_slam.svg)

### Launch
To launch the slam in simulation, run
```
ros2 launch nuslam slam.launch.xml cmd_src:=teleop
```

### Video Demo

<video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/8e766ea4-24a4-46eb-9b77-a019af0710dd" controls></video>

## SLAM with Landmark Detection

### Software Structure
The rqt_graph is shown in the figure below:
![](images/rosgraph_landmark.svg)

### Launch 
To launch the slam with landmark detection in simulation run
```
ros2 launch nuslam landmarks.launch.xml cmd_src:=teleop
```

### Video Demo

<video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/1090f3eb-7a68-45b1-9b95-fd0f915f2d55" controls></video>