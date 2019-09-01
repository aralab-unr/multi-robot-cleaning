cmake_minimum_required(VERSION 2.8.3)
project(imu_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_ros
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_ros src/main.cpp)

target_link_libraries(imu_ros
  ${catkin_LIBRARIES}
)
