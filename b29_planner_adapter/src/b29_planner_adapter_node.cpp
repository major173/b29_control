// SPDX-License-Identifier: BSD-3-Clause

#include <b29_planner_adapter/planner_adapter.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "b29_planner_adapter");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  b29_planner_adapter::PlannerAdapter adapter(node_handle, private_node_handle);
  if (!adapter.init())
  {
    ROS_FATAL("Failed to initialize B29 Planner adapter.");
    return 1;
  }

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
