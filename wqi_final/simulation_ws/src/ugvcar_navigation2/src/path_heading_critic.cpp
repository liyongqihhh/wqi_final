#include "ugvcar_navigation2/path_heading_critic.hpp"

#include <cmath>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "ugvcar_navigation2/path_heading_model.hpp"

namespace ugvcar_navigation2
{

void PathHeadingCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Path heading critic parent node expired");
  }
  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "lookahead_distance", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "activation_angle", rclcpp::ParameterValue(0.7));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "activation_linear_velocity", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "translation_weight", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "idle_penalty", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "wrong_turn_penalty", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "lookahead_distance", lookahead_distance_);
  node->get_parameter(prefix + "activation_angle", activation_angle_);
  node->get_parameter(
    prefix + "activation_linear_velocity", activation_linear_velocity_);
  node->get_parameter(prefix + "translation_weight", translation_weight_);
  node->get_parameter(prefix + "idle_penalty", idle_penalty_);
  node->get_parameter(prefix + "wrong_turn_penalty", wrong_turn_penalty_);
  if (
    lookahead_distance_ <= 0.0 || activation_angle_ <= 0.0 ||
    activation_linear_velocity_ < 0.0 || translation_weight_ < 0.0 ||
    idle_penalty_ < 0.0 || wrong_turn_penalty_ < 0.0)
  {
    throw std::runtime_error("Path heading critic parameters are invalid");
  }
}

bool PathHeadingCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  current_pose_ = pose;
  current_linear_velocity_ = velocity.x;
  has_target_ = false;
  if (global_plan.poses.size() < 2) {
    return true;
  }

  const geometry_msgs::msg::Pose2D * target = &global_plan.poses.back();
  for (const auto & path_pose : global_plan.poses) {
    if (
      std::hypot(path_pose.x - pose.x, path_pose.y - pose.y) >=
      lookahead_distance_)
    {
      target = &path_pose;
      break;
    }
  }
  const double dx = target->x - pose.x;
  const double dy = target->y - pose.y;
  if (std::hypot(dx, dy) < 1.0e-3) {
    return true;
  }
  desired_heading_ = std::atan2(dy, dx);
  has_target_ = true;
  return true;
}

double PathHeadingCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!has_target_) {
    return 0.0;
  }
  const double candidate_heading = trajectory.poses.empty() ?
    current_pose_.theta : trajectory.poses.back().theta;
  return pathHeadingScore(
    current_pose_.theta,
    current_linear_velocity_,
    desired_heading_,
    candidate_heading,
    trajectory.velocity.x,
    activation_angle_,
    activation_linear_velocity_,
    translation_weight_,
    idle_penalty_,
    wrong_turn_penalty_);
}

}  // namespace ugvcar_navigation2

PLUGINLIB_EXPORT_CLASS(
  ugvcar_navigation2::PathHeadingCritic,
  dwb_core::TrajectoryCritic)
