cmake_minimum_required(VERSION 2.8.3)
project(rviz_3d_object_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Disable mongodb_store deprecated warnings:
add_definitions(-Wno-deprecated-declarations)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  mongodb_store
  geometry_msgs
  mas_perception_msgs 
)

generate_messages(
  DEPENDENCIES
    mas_perception_msgs
)

catkin_package(
)

find_package(yaml-cpp REQUIRED)

# Adding include dir here necessary to find header files:
include_directories(${catkin_INCLUDE_DIRS} include/)
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

#########################
# Compile the RViz plugin
#########################

# List down all required cpp files to compile the plugin
set(PLUGIN_SRC_FILES
    src/rviz_plugin/object_visualization_manager.cpp
    # For some reason, including the header file is necessary. Otherwise the plugin fails to load!
    include/rviz_plugin/object_visualization_manager.h
)

# Build the RViz plugin as a library
add_library(${PROJECT_NAME} ${PLUGIN_SRC_FILES}) 

# List down all the dependencies ( i.e. libraries) of the plugin
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${GLEW_LIBRARIES})

# Define the paths where the build artifacts should be installed
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install the plugin description file to the catkin workspace so that RViz can is aware of it.
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#######################
# Compile Data Loader
#######################
# List down all required cpp files to compile the dataloader library
set(DATALOADER_SRC_FILES
    src/dataloader/modelloader/model_loader.cpp
    src/dataloader/modelloader/yaml_loader.cpp
    src/dataloader/modelloader/mesh.cpp
    src/dataloader/mdr_dataloader.cpp
)

# Build the dataloader as a library
set(DATALOADER_LIB_NAME dataloader)
add_library(${DATALOADER_LIB_NAME} ${DATALOADER_SRC_FILES})
target_link_libraries(${DATALOADER_LIB_NAME} ${YAML_CPP_LIBRARIES})


###########################
# Create a demo executable
###########################
add_executable(demo src/demo.cpp)
target_link_libraries(demo ${DATALOADER_LIB_NAME} ${catkin_LIBRARIES})

###########################
# Create a demo executable for YCB classes
###########################
add_executable(demo_ycb src/demo_ycb.cpp)
target_link_libraries(demo_ycb ${DATALOADER_LIB_NAME} ${catkin_LIBRARIES})

###########################
# Create a dataloader executable
###########################
add_executable(rviz_dataloader src/rviz_dataloader.cpp)
target_link_libraries(rviz_dataloader ${DATALOADER_LIB_NAME} ${catkin_LIBRARIES})
