cmake_minimum_required(VERSION 3.0.2)
project(b29_smc_auto_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        message_generation
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
        smclib
        std_msgs
        b29_control)

set(SMC_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/smc")
set(SMC_VENDOR_INCLUDE_DIR "${SMC_VENDOR_DIR}/include")

if (EXISTS "${SMC_VENDOR_INCLUDE_DIR}/statemap.h")
    set(SMCLIB_INCLUDE_DIR "${SMC_VENDOR_INCLUDE_DIR}" CACHE PATH "Directory containing statemap.h" FORCE)
else()
    unset(SMCLIB_INCLUDE_DIR CACHE)
    find_path(SMCLIB_INCLUDE_DIR
            statemap.h
            PATH_SUFFIXES smclib)
endif()

if (NOT SMCLIB_INCLUDE_DIR)
    message(FATAL_ERROR "statemap.h not found. Expected in third_party/smc/include or a smclib include directory.")
endif()

add_message_files(
        DIRECTORY msg
        FILES
        AutoSensorInput.msg
        AutoDebugOverride.msg
        AutoStateTrace.msg)

generate_messages(
        DEPENDENCIES
        std_msgs)

catkin_package(
        INCLUDE_DIRS include gen/include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        controller_interface
        hardware_interface
        message_runtime
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
        smclib
        std_msgs)

include_directories(
        include
        include/b29_smc_auto_controller
        gen/include
        ${SMCLIB_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/b29_smc_auto_controller.cpp
        src/auto_input_mux.cpp
        src/command_dispatcher.cpp
        src/robot_context.cpp
        gen/src/RobotFSM_sm.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
        function(add_optional_gtest target_name source_file)
                if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}")
                        catkin_add_gtest(${target_name}
                                        ${source_file})
                        if (TARGET ${target_name})
                                target_link_libraries(${target_name} ${PROJECT_NAME} ${catkin_LIBRARIES})
                                add_dependencies(${target_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
                        endif()
                else()
                        message(STATUS "Skipping ${target_name}: missing ${source_file}")
                endif()
        endfunction()

        add_optional_gtest(test_auto_types test/test_auto_types.cpp)
        add_optional_gtest(test_auto_input_mux test/test_auto_input_mux.cpp)
        add_optional_gtest(test_command_dispatcher test/test_command_dispatcher.cpp)
        add_optional_gtest(test_trace_mapping test/test_trace_mapping.cpp)
        add_optional_gtest(test_robot_context test/test_robot_context.cpp)
endif()

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
        DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(
        DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(
        DIRECTORY scenarios/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenarios)

install(
        FILES include/statemap.h
        DESTINATION include)

install(
        DIRECTORY gen/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

install(
        FILES b29_smc_auto_controller_plugin.xml README.md
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
        scripts/replay_scenario.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
