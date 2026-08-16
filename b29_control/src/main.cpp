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

  double control_loop_rate_hz = 200.0;
  nh.param("/steering_engine_hw/control_loop_rate_hz",
           control_loop_rate_hz, 200.0);
  if (control_loop_rate_hz <= 0.0) {
    ROS_ERROR("control_loop_rate_hz must be greater than zero");
    return 1;
  }
  ros::Rate loop_rate(control_loop_rate_hz);
  ros::NodeHandle nh_hw("~");
  if (!hardware.init(nh, nh_hw)) {
    ROS_ERROR("Failed to initialize B29 hardware");
    return 1;
  }

  // Setup a separate thread that will be used to service ROS callbacks.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time previous_time = ros::Time::now();
  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    ros::Duration dt = current_time - previous_time;

    hardware.read(current_time, dt);
    hardware.updateControllerManager(current_time, dt);
    hardware.write(current_time, dt);

    previous_time = current_time;
    loop_rate.sleep();
  }

  return 0;
}
