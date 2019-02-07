cmake_minimum_required(VERSION 2.8.3)
project(gnss_inertial_localizer)

## Find catkin and any catkin packages

add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  roscpp
  rospy
  genmsg
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  cmake_modules
  message_filters
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
#catkin_package()


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS Eigen)

set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/ekf.cpp
  ${PROJECT_SOURCE_DIR}/src/get_jaccobian_f.cpp
  ${PROJECT_SOURCE_DIR}/src/get_jaccobian_h.cpp
  ${PROJECT_SOURCE_DIR}/src/ekf_publisher.cpp
#  ${PROJECT_SOURCE_DIR}/src/ekf_offline.cpp
  ${PROJECT_SOURCE_DIR}/src/enu_to_xyz.cpp
  ${PROJECT_SOURCE_DIR}/src/xyz_to_enu.cpp
)

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/gnss_inertial_localizer/ ${OpenCV_INCLUDE_DIRS})

add_library(gnss_inertial_localizer  ${SOURCE_FILES})

## Build talker and listener

#add_executable(ekf_tester src/ekf_tester.cpp)
#target_link_libraries(ekf_tester gnss_inertial_localizer ${catkin_LIBRARIES})

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node gnss_inertial_localizer ${catkin_LIBRARIES})

#add_executable(ekf_tester_gnss src/ekf_tester_gnss.cpp)
#target_link_libraries(ekf_tester_gnss gnss_inertial_localizer ${catkin_LIBRARIES})

add_executable(DummyData_Publisher src/DummyData_Publisher.cpp)
target_link_libraries(DummyData_Publisher gnss_inertial_localizer ${catkin_LIBRARIES})

add_executable(position_drawer src/position_drawer.cpp)
target_link_libraries(position_drawer gnss_inertial_localizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

