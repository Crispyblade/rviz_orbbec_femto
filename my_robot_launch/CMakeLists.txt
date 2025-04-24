cmake_minimum_required(VERSION 3.8)
project(my_robot_launch)

# Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(orbbec_camera REQUIRED)
find_package(orbbec_camera_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(nav2_common REQUIRED)

# Install launch files and configs
install(
  DIRECTORY 
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  orbbec_camera
  orbbec_camera_msgs
  depth_image_proc
  rclcpp_components
  nav2_common
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
