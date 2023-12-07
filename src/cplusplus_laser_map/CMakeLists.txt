cmake_minimum_required(VERSION 3.8)
project(cplusplus_laser_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PythonLibs REQUIRED) # Find Python libraries

# Include the directory where matplotlibcpp.h is located
include_directories(
  include/matplotlib-cpp
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(reading_laser src/reading_laser.cpp)
ament_target_dependencies(reading_laser rclcpp sensor_msgs)
target_link_libraries(reading_laser ${PYTHON_LIBRARIES}) # Link Python libraries

install(TARGETS
  reading_laser
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
