//
// Created by yuchen on 2023/1/29.
//

#include "steering_engine/common/hardware_interface.h"
#include <ros/spinner.h>

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "st_hardware");
  ros::NodeHandle nh("~");

  steering_engine_hw::StRobotHW hardware;
  // controller_manager::ControllerManager cm(&robot_hw);

  ros::Rate loop_rate(40);
  ros::NodeHandle nh_hw("~");
  hardware.init(nh, nh_hw);

  // Setup a separate thread that will be used to service ROS callbacks.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static ros::Time previous_time = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();
    ros::Duration dt = current_time - previous_time;

    hardware.read(ros::Time::now(), ros::Duration(1.0 / 40));
    // In order real time ,this function maybe move into command sender
    hardware.write(ros::Time::now(), ros::Duration(1.0 / 40));

    hardware.updateControllerManager(current_time, dt);
    hardware.updateTf(current_time);

    previous_time = current_time;
    loop_rate.sleep();
  }

  return 0;
}
