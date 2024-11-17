# Robotics Assignment 

Author: Emmett Fitzharris
Student Number: R00222357
Date: 2024/11/17

### Build Project and Launch
```
rosdep install -i --from-path src --rosdistro jazzy -y
source install/setup.bash
colcon build --packages-select robotics_assignment1
find /root/ros2_ws/install -type f -exec dos2unix {} \;

ros2 launch robotics_assignment1 robotics_assignment1.launch.py
```

The code for this project is found in the src folder. 

Task 1:
The code relevant to Task 1 is in the file SpawnClient.py 
In the robotics_assignment1 package

Task 2:
The code for task2 2 is found in SpawnTurtlesServer.py and called by SpawnTurtlesServerClient.py

Task 3:

The code for task 3 is found in the file MoveTurtle1ActionServer.py

Task 4:

The code for task 4 is found in the file RunSimulationActionServer

Task 5:

The code for task 5 is found in the file RunSimulationActionClient.py

Task 6:
The code for task 6 is found in robotics_assignment1.launch.py, within the launch folder. 
