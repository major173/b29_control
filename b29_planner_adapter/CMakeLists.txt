cmake_minimum_required(VERSION 3.0.2)
project(b29_planner_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  b29_smc_auto_controller
  control_msgs
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_trajectory ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib b29_smc_auto_controller control_msgs roscpp sensor_msgs std_srvs trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_trajectory
  src/trajectory_processor.cpp
)
add_dependencies(${PROJECT_NAME}_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_trajectory ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/planner_adapter.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_trajectory ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  src/b29_planner_adapter_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory_processor test/test_trajectory_processor.cpp)
  if(TARGET test_trajectory_processor)
    target_link_libraries(test_trajectory_processor ${PROJECT_NAME}_trajectory ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_trajectory ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
