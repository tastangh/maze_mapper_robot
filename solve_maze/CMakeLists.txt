cmake_minimum_required(VERSION 3.0.2)
project(solve_maze)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(my_mapper src/mapper.cpp)
target_link_libraries(my_mapper ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
