ros2 run turtlesim turtlesim_node

rosdep install -i --from-path src --rosdistro jazzy -y

colcon build --packages-select Robotics_Assignment1

source install/setup.bash

ros2 run Robotics_Assignment1 service

ros2 run Robotics_Assignment1 client 2 3

ros2 run Robotics_Assignment1 spawnClient 5.0 5.0 0.0 turtle1
