cmake_minimum_required(VERSION 3.5)
project(ball_detector_p)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(ball_detecctor src/ball_detector.cpp src/color_detector.cpp)
target_include_directories(ball_detector PRIVATE ${OpenCV_INCLUDE_DIRS} include)

ament_auto_add_executable(color_tester src/color_tester.cpp src/color_detector.cpp)
target_include_directories(color_tester PRIVATE ${OpenCV_INCLUDE_DIRS} include)

ament_auto_package()
