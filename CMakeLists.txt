cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception)
find_package(catkin REQUIRED actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  suturo_perception_msgs
  robosherlock
  robosherlock_msgs
  rapidjson_ros
  visualization_msgs
  image_transport)

# OpenCV 3.3+
find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE suturo_perception)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/suturo_perception/types)
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
   CATKIN_DEPENDS robosherlock actionlib_msgs suturo_perception_msgs robosherlock_msgs rapidjson_ros
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
include_directories(${RS_INCLUDE_DIRS_LIST} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
################################################################################
## Sub Projects                                                               ##
################################################################################
#If you want to divide your projects into subprojects include the subdirectories
#each containing a CMakeLists.txt here
#add_subdirectory(src/xxx)

# Utilities
add_library(${PROJECT_NAME}
		src/server/PerceptionServer.cpp
		include/suturo_perception/PerceptionServer.h
		src/server/ExtractObjectInfoServer.cpp
		src/server/ExtractPlaneInfoServer.cpp
		src/utils/SuturoProcessManager.cpp
		src/utils/suturo_conversion.cpp
		include/suturo_perception/suturo_conversion.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Perception Action Server
add_executable(perception_server src/server/perception_server.cpp)
target_link_libraries(perception_server ${RS_SUTURO} ${catkin_LIBRARIES} ${PROJECT_NAME})

# Object Action client (for testing purposes)
add_executable(ExtractObjectInfoClient src/server/ExtractObjectInfoClient.cpp)
target_link_libraries(ExtractObjectInfoClient ${RS_SUTURO} ${catkin_LIBRARIES})

# Plane Action client (for testing purposes)
add_executable(ExtractPlaneInfoClient src/server/ExtractPlaneInfoClient.cpp)
target_link_libraries(ExtractPlaneInfoClient ${RS_SUTURO} ${catkin_LIBRARIES})

# Cluster Labeler 
rs_add_library(rs_clusterLabeler src/annotators/clusterLabeler.cpp)
target_link_libraries(rs_clusterLabeler ${catkin_LIBRARIES} ${PROJECT_NAME})

# Classification Evaluation 
rs_add_library(rs_classificationEvaluationAnnotator src/annotators/classificationEvaluationAnnotator.cpp)
target_link_libraries(rs_classificationEvaluationAnnotator ${catkin_LIBRARIES})

# Suturo Shape Annotator
rs_add_library(rs_SuturoShapeAnnotator src/annotators/SuturoShapeAnnotator.cpp)
target_link_libraries(rs_SuturoShapeAnnotator ${catkin_LIBRARIES})

# Suturo Classifier
rs_add_library(rs_SuturoClassifier src/annotators/SuturoClassifier.cpp)
target_link_libraries(rs_SuturoClassifier ${PROJECT_NAME} ${OpenCV_LIBS})

# Save Cluster Clouds and Images
rs_add_library(rs_SaveClusterCloudsAndImages src/annotators/SaveClusterCloudsAndImages.cpp)
target_link_libraries(rs_SaveClusterCloudsAndImages ${catkin_LIBRARIES})

# Region Annotator
rs_add_library(rs_RegionAnnotator src/annotators/RegionAnnotator.cpp)
target_link_libraries(rs_RegionAnnotator ${catkin_LIBRARIES})

# Suturo Region Filter
rs_add_library(rs_SuturoRegionFilter src/annotators/SuturoRegionFilter.cpp)
target_link_libraries(rs_SuturoRegionFilter ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
