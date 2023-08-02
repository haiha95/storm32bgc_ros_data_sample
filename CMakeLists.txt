cmake_minimum_required(VERSION 3.0.2)
project(storm32bgc_ros_data_sample)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)
catkin_package(
  CATKIN_DEPENDS roscpp serial
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)