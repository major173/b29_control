#include "gp11_perception/cable_tracker_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gp11_d435i_cable_tracker");
  gp11_perception::CableTrackerNode node;
  ros::spin();
  return 0;
}
