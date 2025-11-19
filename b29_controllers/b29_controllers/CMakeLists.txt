cmake_minimum_required(VERSION 3.0.2)
project(b29_controllers)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        angles
        control_msgs
        control_toolbox
        controller_interface
        controller_manager
        hardware_interface
        joint_limits_interface
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
        std_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        urdf
        geometry_msgs
        nav_msgs

        rm_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        angles
        control_msgs
        control_toolbox
        controller_interface
        controller_manager
        hardware_interface
        joint_limits_interface
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
        std_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        urdf
        geometry_msgs
        nav_msgs

        rm_common
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/b29_joint_controller.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugin description file
install(FILES b29_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
)
