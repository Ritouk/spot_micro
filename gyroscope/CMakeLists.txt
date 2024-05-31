cmake_minimum_required(VERSION 3.8)
project(gyroscope)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gyroscope src/gyroscope.cpp)
ament_target_dependencies(gyroscope rclcpp sensor_msgs)

install(TARGETS
  gyroscope
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
