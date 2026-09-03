cmake_minimum_required(VERSION 3.8)
project(quadruped_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gazebo_ground_truth_odometry.py
  scripts/generate_benchmark_world.py
  DESTINATION lib/${PROJECT_NAME}
)

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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_benchmark_generator
    test/test_benchmark_generator.py
  )
endif()

ament_package()
