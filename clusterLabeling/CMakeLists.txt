cmake_minimum_required(VERSION 2.8.3)
project(clusterLabeling)
find_package(catkin REQUIRED robosherlock rapidjson_ros)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE clusterLabeling)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/clusterLabeling/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

catkin_package(
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
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
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )
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

add_library(guitools
        src/gui/mainwindow.cpp
        src/gui/guiWrapper.cpp
        src/gui/guiWrapper.h
        src/gui/mainwindow.h
        src/gui/ui_mainwindow.h)
target_link_libraries(guitools Qt5::Widgets ${OpenCV_LIBRARIES})

rs_add_library(rs_clusterLabeler src/annotators/clusterLabeler.cpp)
target_link_libraries(rs_clusterLabeler ${catkin_LIBRARIES} guitools)

rs_add_library(rs_classificationEvaluationAnnotator src/annotators/classificationEvaluationAnnotator.cpp)
target_link_libraries(rs_classificationEvaluationAnnotator ${catkin_LIBRARIES})
