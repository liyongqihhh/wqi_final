#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ugvcar_navigation2/dynamic_path_policy.hpp"

namespace ugvcar_navigation2
{

class SelectStableDynamicPath : public BT::SyncActionNode
{
public:
  SelectStableDynamicPath(const std::string & name, const BT::NodeConfiguration & configuration)
  : BT::SyncActionNode(name, configuration) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate"),
      BT::InputPort<nav_msgs::msg::Path>("current"),
      BT::InputPort<bool>("current_valid", false, "Whether the remaining accepted path is valid"),
      BT::OutputPort<nav_msgs::msg::Path>("selected")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path candidate;
    nav_msgs::msg::Path current;
    bool current_valid = false;
    if (!getInput("candidate", candidate) || candidate.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    getInput("current", current);
    getInput("current_valid", current_valid);
    const auto points = [](const nav_msgs::msg::Path & path) {
        PathPoints result;
        result.reserve(path.poses.size());
        for (const auto & pose : path.poses) {
          result.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        return result;
      };
    setOutput(
      "selected", shouldReplaceDynamicPath(
        points(current), points(candidate), current_valid) ? candidate : current);
    return BT::NodeStatus::SUCCESS;
  }
};

class DynamicObstacleChangedCondition : public BT::ConditionNode
{
public:
  DynamicObstacleChangedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & configuration)
  : BT::ConditionNode(condition_name, configuration),
    node_(configuration.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    std::string status_topic;
    if (!getInput("status_topic", status_topic)) {
      throw BT::RuntimeError("Missing required input [status_topic]");
    }

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      status_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {
        const int active_threats = parseActiveThreats(message->data);
        if (active_threats < 0) {
          return;
        }
        if (
          last_active_threats_ >= 0 &&
          active_threats != last_active_threats_)
        {
          state_transition_pending_.store(true);
          // Let the 2 Hz global costmap consume the prediction cloud before
          // forcing exactly one replacement path on the following BT tick.
          settle_ticks_remaining_.store(1);
        }
        last_active_threats_ = active_threats;
      },
      options);
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();
    if (!state_transition_pending_.load()) {
      return BT::NodeStatus::FAILURE;
    }
    const int remaining = settle_ticks_remaining_.load();
    if (remaining > 0) {
      settle_ticks_remaining_.store(remaining - 1);
      return BT::NodeStatus::FAILURE;
    }
    return state_transition_pending_.exchange(false) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "status_topic",
        "/ugv/dynamic_replanning/status",
        "Velocity-aware dynamic-obstacle status topic")
    };
  }

private:
  static int parseActiveThreats(const std::string & status)
  {
    const std::string key = "active_threats=";
    const auto start = status.find(key);
    if (start == std::string::npos) {
      return -1;
    }
    const auto value_start = start + key.size();
    const auto value_end = status.find(';', value_start);
    try {
      return std::stoi(status.substr(value_start, value_end - value_start));
    } catch (const std::exception &) {
      return -1;
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  std::atomic_bool state_transition_pending_{false};
  std::atomic_int settle_ticks_remaining_{0};
  int last_active_threats_{-1};
};

}  // namespace ugvcar_navigation2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ugvcar_navigation2::SelectStableDynamicPath>(
    "SelectStableDynamicPath");
  factory.registerNodeType<ugvcar_navigation2::DynamicObstacleChangedCondition>(
    "DynamicObstacleChanged");
  factory.registerNodeType<ugvcar_navigation2::DynamicObstacleChangedCondition>(
    "DynamicObstacleCleared");
}
