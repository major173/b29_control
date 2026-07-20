cmake_minimum_required(VERSION 3.5)

project(gp11_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_geometry
  image_transport
  tf
  tf2
  tf2_ros
)
if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

find_path(EIGEN3_INCLUDE_DIR Eigen/Dense PATHS /usr/include/eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 headers were not found")
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_filters rospy std_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge image_geometry image_transport tf tf2 tf2_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(GP11_NATIVE_OPTIMIZER_TARGET gp11_perception_native_optimizer)
set(GP11_NATIVE_PYTHON_OUTPUT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

add_library(${GP11_NATIVE_OPTIMIZER_TARGET} SHARED
  src/cpp/native_curve_optimizer.cpp
)

set_target_properties(${GP11_NATIVE_OPTIMIZER_TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  LIBRARY_OUTPUT_DIRECTORY ${GP11_NATIVE_PYTHON_OUTPUT_DIR}
)

target_include_directories(${GP11_NATIVE_OPTIMIZER_TARGET} PRIVATE ${EIGEN3_INCLUDE_DIR})

add_library(gp11_perception_tracker_core
  src/cpp/cable_tracker_node.cpp
)

set_target_properties(gp11_perception_tracker_core PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

target_link_libraries(gp11_perception_tracker_core
  ${GP11_NATIVE_OPTIMIZER_TARGET}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(gp11_d435i_cable_tracker_node
  src/nodes/gp11_d435i_cable_tracker_node.cpp
)

set_target_properties(gp11_d435i_cable_tracker_node PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

target_link_libraries(gp11_d435i_cable_tracker_node
  gp11_perception_tracker_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/gp11_capture_depth_background.py
    scripts/gp11_single_camera_depth_line.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch rviz)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)

install(
  TARGETS ${GP11_NATIVE_OPTIMIZER_TARGET}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
  TARGETS gp11_perception_tracker_core gp11_d435i_cable_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
