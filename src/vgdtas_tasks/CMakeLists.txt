cmake_minimum_required(VERSION 3.8)
project(vgdtas_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fer_perception_node src/fer_perception_node.cpp)
target_include_directories(fer_perception_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(fer_perception_node
  cv_bridge
  geometry_msgs
  moveit_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(fer_perception_node ${OpenCV_LIBRARIES})

add_executable(ur_loop_node src/ur_loop_node.cpp)
ament_target_dependencies(ur_loop_node moveit_task_constructor_core)

add_executable(ur_pickup_node src/ur_pickup_node.cpp)
ament_target_dependencies(ur_pickup_node moveit_ros_planning_interface moveit_task_constructor_core)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS fer_perception_node ur_loop_node ur_pickup_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
