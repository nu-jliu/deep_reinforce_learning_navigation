# NUTurtle Control
Controls the motion of the turtlebot

## Running on turtlebot3
On turtlebot run 
```
ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=none robot:=teleop use_rviz:=false
```
On laptop run
```
ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=circle robot:=none use_rviz:=true
```
<video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/f2b4ca4d-b961-4905-a3fe-bb694ff66bb4" title="run on turtlebot3" controls></video>

The final odometry error is
```
position:
    x: 0.00286417
    y: -0.0158253
    z: 0.0
orientation:
    x: 0.0
    y: 0.0
    z: -0.336885
    w: 0.999432
```