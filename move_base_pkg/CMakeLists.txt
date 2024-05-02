cmake_minimum_required(VERSION 3.8)
project(move_base_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# To get tf from the auros to the robot
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)


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

add_executable(move_base_train_node src/MoveBaseTrain.cpp)

ament_target_dependencies(move_base_train_node rclcpp std_msgs sensor_msgs tf2 tf2_ros nav_msgs geometry_msgs tf2_geometry_msgs message_filters visualization_msgs)
target_link_libraries(move_base_train_node Eigen3::Eigen)

install(TARGETS
  move_base_train_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("\\033[1;32m -- ALL LIBRARIES FOUND AT MOVE BASE TRAIN PKG \\033[0m")
message(=============================================================)

ament_package()
