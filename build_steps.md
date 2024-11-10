### Build Project and Launch
```
rosdep install -i --from-path src --rosdistro jazzy -y
source install/setup.bash
colcon build --packages-select robotics_assignment1
find /root/ros2_ws/install -type f -exec dos2unix {} \;

ros2 launch robotics_assignment1 robotics_assignment1.launch.py
```

### Test Turtle Spawning
```
source install/setup.bash
ros2 run robotics_assignment1 SpawnTurtlesServerClient.py 5
```

### Test Turtle Following Action Goal
```
source install/setup.bash
ros2 action send_goal t1Follow robotics_assignment1/action/Turtle1Follow "{goal_turtle_name: 'turtle_1'}"
```

