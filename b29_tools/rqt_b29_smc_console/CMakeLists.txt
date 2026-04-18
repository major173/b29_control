cmake_minimum_required(VERSION 3.0.2)
project(rqt_b29_smc_console)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  python_qt_binding
  b29_smc_auto_controller
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS b29_smc_auto_controller
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_b29_smc_console
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
    catkin_add_nosetests(test)
  else()
    message(STATUS "Skipping nosetests: missing test directory")
  endif()
endif()
