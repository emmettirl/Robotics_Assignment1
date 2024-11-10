### Terminal 0
```
ros2 run turtlesim turtlesim_node
```

### Terminal 1
```
rosdep install -i --from-path src --rosdistro jazzy -y
source install/setup.bash
colcon build --packages-select robotics_assignment1
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/SpawnClient.py
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/SpawnTurtlesServer.py
ros2 run robotics_assignment1 SpawnTurtlesServer.py
```
### Terminal 2

```
source install/setup.bash
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/SpawnTurtlesServerClient.py
ros2 run robotics_assignment1 SpawnTurtlesServerClient.py 5
```

### Terminal 3
```
source install/setup.bash
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/MoveTurtle1ActionServer.py
ros2 run robotics_assignment1 MoveTurtle1ActionServer.py
```

### Test action server
```
source install/setup.bash
ros2 action send_goal t1Follow robotics_assignment1/action/Turtle1Follow "{order_x: 5, order_y: 5, order_theta: 0, order_linear_velocity: 1, order_angular_velocity: 0}"
```


