#include <chrono>
#include <csignal>
#include <exception>
#include <memory>

#include "nav2_map_server/costmap_filter_info_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  // Keep lifecycle and DDS destruction off the asynchronous signal thread.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  try {
    auto node = std::make_shared<nav2_map_server::CostmapFilterInfoServer>();
    {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      while (rclcpp::ok() && !stop_requested) {
        executor.spin_once(std::chrono::milliseconds(100));
      }
      executor.remove_node(node->get_node_base_interface());
    }
    // The ROS context stays valid until the node, bond and publisher are gone.
    node->on_rcl_preshutdown();
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("managed_filter_info_server"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
