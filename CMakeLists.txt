cmake_minimum_required(VERSION 3.5)
# TODO give a name to your project
project(ball_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# TODO find necessary libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
include_directories(include)

# TODO build the node as a library
add_executable(circle_detector src/circle_detector.cpp include/ball_detector/color_detector.hpp src/color_detector.cpp)

add_executable(talker src/publisher_member_function.cpp include/ball_detector/color_detector.hpp src/color_detector.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs baxter_core_msgs OpenCV cv_bridge)
#add_executable(test_exec include/test_projet/color_detector.hpp src/color_detector.cpp src/circle_detector.cpp)

# TODO include non-ROS2 directories (OpenCV, Yaml-cpp, etc.)
target_include_directories(circle_detector PRIVATE ${OpenCV_INCLUDE_DIRS})

# TODO link your node to its ROS 2 dependencies
ament_target_dependencies(circle_detector
  rclcpp
  std_msgs 
  baxter_core_msgs
  OpenCV
  cv_bridge)
  
# TODO let ament create an executable from your node
#rclcpp_components_register_node(circle_detector
#  PLUGIN "<package name>::<node class name>"
#  EXECUTABLE <executable_name>)

# install things
install(TARGETS circle_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
# install other directories if needed
install(DIRECTORY launch urdf maps params
  DESTINATION share/${PROJECT_NAME})

# register node
#ament_export_libraries(circle_detector)
ament_package()
