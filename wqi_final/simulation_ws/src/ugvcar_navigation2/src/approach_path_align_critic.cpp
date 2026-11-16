#include "ugvcar_navigation2/approach_path_align_critic.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ugvcar_navigation2
{
void ApproachPathAlignCritic::onInit()
{
  dwb_critics::PathAlignCritic::onInit();
  auto node = node_.lock();
  const auto parameter = dwb_plugin_name_ + "." + name_ + ".disable_distance";
  nav2_util::declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(1.2));
  node->get_parameter(parameter, disable_distance_);
  if (!std::isfinite(disable_distance_) || disable_distance_ <= forward_point_distance_) {
    throw std::runtime_error("Approach alignment cutoff must exceed its forward projection");
  }
}

bool ApproachPathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  const double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  if (!std::isfinite(distance)) {
    return false;
  }
  if (distance <= disable_distance_) {
    zero_scale_ = true;
    return true;
  }
  return dwb_critics::PathAlignCritic::prepare(pose, velocity, goal, global_plan);
}
}  // namespace ugvcar_navigation2

PLUGINLIB_EXPORT_CLASS(ugvcar_navigation2::ApproachPathAlignCritic, dwb_core::TrajectoryCritic)
