cmake_minimum_required(VERSION 2.8.3)
project(points_downsampler)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_config_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  velodyne_pointcloud
)

add_message_files(
  FILES
  PointsDownsamplerInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  pcl_conversions
  pcl_ros
  sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/voxel_grid_filter/voxel_grid_filter.cpp)

add_dependencies(voxel_grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

install(
  TARGETS
    voxel_grid_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
