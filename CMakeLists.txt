cmake_minimum_required(VERSION 3.5)
# TODO give a name to your project
project(ball_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# TODO find necessary libraries


# TODO build the node as a library
ament_auto_add_executable(circle_detector src/circle_detector.cpp src/color_detector.cpp)
target_include_directories(circle_detector PRIVATE ${OpenCV_INCLUDE_DIRS} include)

#add_executable(circle_detector src/circle_detector.cpp include/ball_detector/color_detector.hpp src/color_detector.cpp)

#add_executable(talker src/publisher_member_function.cpp include/ball_detector/color_detector.hpp src/color_detector.cpp)
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs baxter_core_msgs OpenCV cv_bridge)
#add_executable(test_exec include/test_projet/color_detector.hpp src/color_detector.cpp src/circle_detector.cpp)
  
# TODO let ament create an executable from your node
#rclcpp_components_register_node(circle_detector
#  PLUGIN "<package name>::<node class name>"
#  EXECUTABLE <executable_name>)

# install things
# install(TARGETS circle_detector
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
  
# install other directories if needed
#install(DIRECTORY launch urdf maps params
#  DESTINATION share/${PROJECT_NAME})

# register node
# ament_export_libraries(circle_detector)
ament_auto_package()
