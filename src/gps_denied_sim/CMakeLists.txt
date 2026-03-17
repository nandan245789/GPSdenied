cmake_minimum_required(VERSION 3.8)
project(gps_denied_sim)

find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Install model files
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
