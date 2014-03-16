cmake_minimum_required(VERSION 2.8.3)
project(ros-quadrotor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
  laser_assembler
  ##Linea aggiunta (spero vada bene)
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(include)

add_library(joint_state_publisher SHARED src/gazebo_ros_joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(snapshotter src/periodic_snapshotter.cpp)
target_link_libraries(snapshotter ${catkin_LIBRARIES})

add_library(joint_tilter_controller SHARED src/joint_tilter_controller.cpp)
target_link_libraries(joint_tilter_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
