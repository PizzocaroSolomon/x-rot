cmake_minimum_required(VERSION 2.8.3)
project(xrot_pre_proces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_ros
  roscpp
  std_msgs
  message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(PCL 1.7 REQUIRED)

#find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES winnica_auto_scanmatch
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xrot_pre_proces
    include/xrot_pre_proces/xrot_pre_proces.hpp
    src/xrot_pre_proces.cpp
)

target_link_libraries(xrot_pre_proces
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_dependencies(xrot_pre_proces ${PROJECT_NAME}_gencfg)

