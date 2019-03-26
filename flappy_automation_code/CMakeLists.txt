cmake_minimum_required(VERSION 2.8.3)
project(flappy_automation_code)

set (CMAKE_CXX_STANDARD 11)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  PCL
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/flappy_automation_code.cpp)

target_link_libraries(${PROJECT_NAME}_node
${PCL_LIBRARIES} ${catkin_LIBRARIES}
)
