cmake_minimum_required(VERSION 2.8.3)
project(px4_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf
)

find_package(yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4_teleop_com src/px4_teleop_com.cpp)
add_executable(px4_teleop_joy src/px4_teleop_joy.cpp)
add_executable(joy_publisher src/joy_publisher.cpp)

target_link_libraries(px4_teleop_com ${catkin_LIBRARIES})
target_link_libraries(joy_publisher ${catkin_LIBRARIES})
target_link_libraries(px4_teleop_joy ${catkin_LIBRARIES} yaml-cpp)

# Doxygen document generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
add_custom_target(gen_doc ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM )
endif()