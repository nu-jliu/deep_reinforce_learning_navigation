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
<video src="https://github.com/ME495-Navigation/slam-project-nu-jliu/assets/49068329/ebca3873-0a3f-4a1b-95e1-c5c30443ebac" controls></video>

The final odometry error is
```
position:
    x: 0.330677
    y: 0741627
    z: 0.0
orientation:
    x: 0.0
    y: 0.0
    z: 0.296966
    w: 0.954888
```