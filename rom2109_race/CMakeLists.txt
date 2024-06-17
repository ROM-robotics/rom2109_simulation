cmake_minimum_required(VERSION 3.8)
project(rom2109_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

add_executable(linear_run src/linear.cpp)
ament_target_dependencies(linear_run 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
)
# add_executable(aes_run src/aes.cpp)
# ament_target_dependencies(aes_run 
#   rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
# )


add_executable(test_code src/test.cpp)
ament_target_dependencies(test_code 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
)
# Install Cpp executables
install(TARGETS
  linear_run
  #aes_run
  test_code
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
