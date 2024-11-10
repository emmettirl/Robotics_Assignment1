## Open TurtleSim
- ```ros2 run turtlesim turtlesim_node```

## Build project
- ```rosdep install -i --from-path src --rosdistro jazzy -y```

- ```colcon build --packages-select Robotics_Assignment1```


## Source project
- ```source install/setup.bash```


## Run project

[//]: # (- ```ros2 run Robotics_Assignment1 service```)

[//]: # (- ```ros2 run Robotics_Assignment1 client 2 3```)

[//]: # (- ```ros2 run Robotics_Assignment1 spawnClient 5.0 5.0 0.0 turtle1```)

- ```ros2 run Robotics_Assignment1 spawnService```
- ```ros2 run Robotics_Assignment1 spawnClient 5```