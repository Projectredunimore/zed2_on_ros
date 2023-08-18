cmake_minimum_required(VERSION 3.0.2)
project(zed2_on_ros)

include_directories(include zed-open-capture/include/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/zed2_on_ros_node.cpp)
add_executable(zed2_rect src/zed2_rect.cpp)
add_executable(zed2_depth src/zed2_depth.cpp)
add_executable(zed2_depth_info src/zed2_depth_info.cpp)
add_executable(zed2_multi src/zed2_multi.cpp)
add_executable(zed2_sensors src/zed2_sensors.cpp)

find_library(ZED_LIB zed_open_capture)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)

target_link_libraries(zed2_rect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)

target_link_libraries(zed2_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)

target_link_libraries(zed2_depth_info
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)

target_link_libraries(zed2_multi
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)

target_link_libraries(zed2_sensors
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZED_LIB}
)