cmake_minimum_required(VERSION 3.5)
project(robotics_assignment1)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnTurtles.srv"
  "action/Turtle1Follow.action"
  "action/RunSimulation.action"
)

# Install entry points
install(PROGRAMS
  robotics_assignment1/SpawnClient.py
  robotics_assignment1/SpawnTurtlesServer.py
  robotics_assignment1/SpawnTurtlesServerClient.py
  robotics_assignment1/MoveTurtle1ActionServer.py
  robotics_assignment1/RunSimulationActionServer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()