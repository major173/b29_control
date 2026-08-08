cmake_minimum_required(VERSION 3.5)

project(gp11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  moveit_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES ReachPoint.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_runtime
  moveit_msgs
  rospy
  sensor_msgs
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_dual_anchor_flip.py)
endif()

catkin_install_python(
  PROGRAMS
    scripts/gp11_moveit_anchor_runtime.py
    scripts/gp11_plan_direction_audit.py
    scripts/gp11_real_anchor_tf.py
    scripts/gp11_real_execution_monitor.py
    scripts/gp11_real_motion_blackbox.py
    scripts/gp11_real_reach_target.py
    scripts/gp11_b29_attach_bridge.py
    scripts/gp11_cartesian_goal_server.py
    scripts/gp11_generate_anchored_urdf.py
    scripts/gp11_moveit_scene_loader.py
    scripts/gp11_single_flip.py
    scripts/gp11_automatic_flip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes rviz urdf)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
