cmake_minimum_required(VERSION 3.0.2)
project(bmflight_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mav_msgs
  sensor_msgs
  rotors_control
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bmflight_simulation
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(udplink
    src/udplink.h
    src/udplink.c
)


target_link_libraries(udplink ${catkin_LIBRARIES})
add_dependencies(udplink ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(sitl_interface_node src/sitl_interface_node.cpp)
target_link_libraries(sitl_interface_node
   ${catkin_LIBRARIES}
)
target_link_libraries(sitl_interface_node udplink ${catkin_LIBRARIES})
