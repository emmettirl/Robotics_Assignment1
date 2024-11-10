## Build project
- ```rosdep install -i --from-path src --rosdistro jazzy -y```
- ```colcon build --packages-select Robotics_Assignment1```


## Source project
- ```source install/setup.bash```


### Terminal 0
```
ros2 run turtlesim turtlesim_node
```

### Terminal 1
```
source install/setup.bash
colcon build --packages-select robotics_assignment1
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/SpawnTurtlesServer.py
ros2 run robotics_assignment1 SpawnTurtlesServer.py
```
### Terminal 2

```
source install/setup.bash
dos2unix /root/ros2_ws/install/robotics_assignment1/lib/robotics_assignment1/SpawnTurtlesServerClient.py
ros2 run robotics_assignment1 SpawnTurtlesServerClient.py 5
```

