#include "graph_based_slam/graph_based_slam_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // options.use_intra_process_comms(true);
  auto node = std::make_shared<graphslam::GraphBasedSlamComponent>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
