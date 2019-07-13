cmake_minimum_required(VERSION 2.8.3)
project(coast_line_removal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES coast_line_removal
  CATKIN_DEPENDS nodelet pcl_ros pluginlib roscpp sensor_msgs message_filters
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(coast_line_removal
  src/coast_line_removal.cpp
)
target_link_libraries(coast_line_removal ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(coast_line_removal)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS coast_line_removal
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)