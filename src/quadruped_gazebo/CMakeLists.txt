cmake_minimum_required(VERSION 3.8)
project(quadruped_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install world(s)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install configs (rviz etc.)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
