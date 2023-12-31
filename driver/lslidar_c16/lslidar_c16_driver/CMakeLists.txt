cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c16_driver)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    pcl_ros
    roscpp
    sensor_msgs
    tf
    dynamic_reconfigure
    lslidar_c16_msgs
    nodelet
)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
        pcl_conversions
        rospy
	std_msgs
        genmsg
        cv_bridge
        message_generation
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)


include_directories(
		include
		${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)


catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    CATKIN_DEPENDS message_runtime std_msgs
    )


add_library(lslidar_input src/input.cc)
target_link_libraries(lslidar_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES})

add_library(lslidar_c16_driver src/lslidar_c16_driver.cpp)

target_link_libraries(lslidar_c16_driver
		lslidar_input
		${catkin_LIBRARIES})

# build the nodelet version
add_library(lslidar_c16_driver_nodelet src/lslidar_c16_driver_nodelet.cc src/lslidar_c16_driver.cpp)
target_link_libraries(lslidar_c16_driver_nodelet
		lslidar_input
		${catkin_LIBRARIES}
		)

add_executable(lslidar_c16_driver_node src/lslidar_c16_driver_node.cpp)

if(catkin_EXPORTED_TARGETS)
	add_dependencies(lslidar_input ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(lslidar_c16_driver_node
		lslidar_c16_driver
		lslidar_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
		)


install(TARGETS lslidar_input lslidar_c16_driver lslidar_c16_driver_nodelet lslidar_c16_driver_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)


install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)
install(FILES
		nodelet_lslidar_c16.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)