cmake_minimum_required(VERSION 3.5)
project(tinkerforge_imu_ros)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
add_compile_options("-w")

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  sensor_msgs
#  tf
#  tf2
#)

find_package(rclcpp REQUIRED)
#find_package(tf REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS} 
)

add_executable(tinkerforge_imu_ros
  src/tinkerforge_imu_ros.cpp
  src/brick_imu_v2.cpp
  src/ip_connection.cpp
)

ament_target_dependencies(tinkerforge_imu_ros rclcpp tf2 sensor_msgs)

install(TARGETS
  tinkerforge_imu_ros
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()

#target_link_libraries(tinkerforge_imu_ros ${catkin_LIBRARIES})

#install(TARGETS tinkerforge_imu_ros
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
