cmake_minimum_required(VERSION 2.8.3)
project(actionserver)
find_package(catkin REQUIRED actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  suturo_perception_msgs
  robosherlock
  robosherlock_msgs
  rapidjson_ros
  visualization_msgs
  rs_hsrb_perception
  image_transport)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE actionserver)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/actionserver/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   CATKIN_DEPENDS robosherlock actionlib_msgs suturo_perception_msgs robosherlock_msgs rapidjson_ros rs_hsrb_perception
   )
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include/actionserver
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
        )
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
find_library(RS_SUTURO rs_hsrb_perception)

add_executable(perception_server src/perception_server.cpp)
add_executable(ExtractObjectInfoClient src/ExtractObjectInfoClient.cpp)
add_executable(ExtractPlaneInfoClient src/ExtractPlaneInfoClient.cpp)

add_library(${PROJECT_NAME}
        src/PerceptionServer.cpp
	include/actionserver/PerceptionServer.h
        src/ExtractObjectInfoServer.cpp
        src/ExtractPlaneInfoServer.cpp)

target_link_libraries(${PROJECT_NAME} 
	${RS_SUTURO}
	${catkin_LIBRARIES}
)

target_link_libraries(perception_server
	${RS_SUTURO}
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

target_link_libraries(ExtractObjectInfoClient
	${RS_SUTURO}
	${catkin_LIBRARIES}
)

target_link_libraries(ExtractPlaneInfoClient
	${RS_SUTURO}
	${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(perception_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
## Sub Projects                                                               ##
################################################################################
#If you want to divide your projects into subprojects include the subdirectories
#each containing a CMakeLists.txt here
#add_subdirectory(src/xxx)
