#include "ugvcar_navigation2/predictive_collision_critic.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ugvcar_navigation2
{

void PredictiveCollisionCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Predictive collision critic parent node expired");
  }
  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "tracks_topic",
    rclcpp::ParameterValue(std::string("/ugv/tracked_dynamic_obstacles")));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "robot_radius", rclcpp::ParameterValue(0.22));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "safety_margin", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "track_timeout", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "prediction_horizon", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "preferred_clearance", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "clearance_weight", rclcpp::ParameterValue(0.0001));
  node->get_parameter(prefix + "tracks_topic", tracks_topic_);
  node->get_parameter(prefix + "robot_radius", robot_radius_);
  node->get_parameter(prefix + "safety_margin", safety_margin_);
  node->get_parameter(prefix + "track_timeout", track_timeout_);
  node->get_parameter(prefix + "prediction_horizon", prediction_horizon_);
  node->get_parameter(prefix + "preferred_clearance", preferred_clearance_);
  node->get_parameter(prefix + "clearance_weight", clearance_weight_);
  if (
    robot_radius_ <= 0.0 || safety_margin_ < 0.0 ||
    track_timeout_ <= 0.0 || prediction_horizon_ <= 0.0 ||
    preferred_clearance_ <= 0.0 || clearance_weight_ < 0.0)
  {
    throw std::runtime_error("Predictive collision critic parameters are invalid");
  }

  tracks_subscription_ = node->create_subscription<
    ugvcar_navigation2_interfaces::msg::DynamicObstacleArray>(
    tracks_topic_, rclcpp::SensorDataQoS(),
    std::bind(
      &PredictiveCollisionCritic::tracksCallback, this,
      std::placeholders::_1));
  auto parameters_interface = node->get_node_parameters_interface();
  auto topics_interface = node->get_node_topics_interface();
  status_publisher_ = rclcpp::create_publisher<std_msgs::msg::String>(
    parameters_interface,
    topics_interface,
    "/ugv/predictive_dwa/status",
    rclcpp::QoS(10));
  RCLCPP_INFO(
    node->get_logger(),
    "Predictive DWA critic listening on %s",
    tracks_topic_.c_str());
}

void PredictiveCollisionCritic::reset()
{
  prepared_obstacles_.clear();
  evaluations_.clear();
}

bool PredictiveCollisionCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  current_pose_ = pose;
  evaluations_.clear();
  ugvcar_navigation2_interfaces::msg::DynamicObstacleArray tracks;
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    if (!received_tracks_) {
      prepared_obstacles_.clear();
      return true;
    }
    tracks = latest_tracks_;
  }

  auto node = node_.lock();
  if (!node) {
    return false;
  }
  const rclcpp::Time stamp(tracks.header.stamp);
  double age = stamp.nanoseconds() > 0 ? (node->now() - stamp).seconds() : 0.0;
  age = std::max(0.0, age);
  if (age > track_timeout_) {
    prepared_obstacles_.clear();
    return true;
  }
  try {
    prepared_obstacles_ = transformTracks(tracks, age);
  } catch (const tf2::TransformException & exception) {
    prepared_obstacles_.clear();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "Predictive DWA track transform failed: %s", exception.what());
  }
  return true;
}

double PredictiveCollisionCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  std::vector<TimedPoint2D> samples;
  samples.reserve(trajectory.poses.size() + 1);
  samples.push_back(TimedPoint2D{current_pose_.x, current_pose_.y, 0.0});
  const std::size_t count = std::min(
    trajectory.poses.size(), trajectory.time_offsets.size());
  for (std::size_t index = 0; index < count; ++index) {
    const rclcpp::Duration offset(trajectory.time_offsets[index]);
    const double time = offset.seconds();
    if (time <= samples.back().time || time > prediction_horizon_ + 1.0e-6) {
      continue;
    }
    samples.push_back(
      TimedPoint2D{
        trajectory.poses[index].x,
        trajectory.poses[index].y,
        time});
  }

  const PredictiveCollisionResult result = evaluatePredictiveCollision(
    samples,
    prepared_obstacles_,
    robot_radius_,
    safety_margin_,
    prediction_horizon_);
  evaluations_.push_back(
    CandidateEvaluation{
      trajectory.velocity.x,
      trajectory.velocity.theta,
      result.earliest_collision_time,
      result.minimum_clearance});
  return predictiveTrajectoryScore(
    result, prediction_horizon_, preferred_clearance_, clearance_weight_);
}

void PredictiveCollisionCritic::debrief(
  const nav_2d_msgs::msg::Twist2D & command)
{
  if (!status_publisher_) {
    return;
  }
  const CandidateEvaluation * selected = nullptr;
  double best_error = std::numeric_limits<double>::infinity();
  for (const auto & evaluation : evaluations_) {
    const double error = std::hypot(
      evaluation.linear_velocity - command.x,
      evaluation.angular_velocity - command.theta);
    if (error < best_error) {
      best_error = error;
      selected = &evaluation;
    }
  }

  std_msgs::msg::String status;
  std::ostringstream stream;
  stream << "algorithm=predictive_dwa;dynamic_tracks="
         << prepared_obstacles_.size()
         << ";candidate_trajectories=" << evaluations_.size()
         << ";selected_v=" << command.x
         << ";selected_w=" << command.theta;
  if (selected != nullptr) {
    stream << ";selected_ttc=";
    if (std::isfinite(selected->collision_time)) {
      stream << selected->collision_time;
    } else {
      stream << "inf";
    }
    stream << ";minimum_clearance=" << selected->minimum_clearance
           << ";collision_free="
           << (std::isfinite(selected->collision_time) ? "false" : "true");
  }
  status.data = stream.str();
  status_publisher_->publish(status);
}

void PredictiveCollisionCritic::tracksCallback(
  const ugvcar_navigation2_interfaces::msg::DynamicObstacleArray::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(tracks_mutex_);
  latest_tracks_ = *message;
  received_tracks_ = true;
}

std::vector<MovingCircle2D> PredictiveCollisionCritic::transformTracks(
  const ugvcar_navigation2_interfaces::msg::DynamicObstacleArray & message,
  const double age) const
{
  std::vector<MovingCircle2D> result;
  result.reserve(message.obstacles.size());
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  tf2::Transform transform;
  transform.setIdentity();
  if (!message.header.frame_id.empty() && message.header.frame_id != target_frame) {
    const auto stamped = costmap_ros_->getTfBuffer()->lookupTransform(
      target_frame, message.header.frame_id, tf2::TimePointZero);
    tf2::fromMsg(stamped.transform, transform);
  }
  for (const auto & obstacle : message.obstacles) {
    if (obstacle.radius <= 0.0) {
      continue;
    }
    const tf2::Vector3 source_position(
      obstacle.position.x + obstacle.velocity.x * age,
      obstacle.position.y + obstacle.velocity.y * age,
      obstacle.position.z + obstacle.velocity.z * age);
    const tf2::Vector3 source_velocity(
      obstacle.velocity.x,
      obstacle.velocity.y,
      obstacle.velocity.z);
    const tf2::Vector3 position = transform * source_position;
    const tf2::Vector3 velocity = transform.getBasis() * source_velocity;
    result.push_back(
      MovingCircle2D{
        position.x(), position.y(), velocity.x(), velocity.y(), obstacle.radius});
  }
  return result;
}

}  // namespace ugvcar_navigation2

PLUGINLIB_EXPORT_CLASS(
  ugvcar_navigation2::PredictiveCollisionCritic,
  dwb_core::TrajectoryCritic)
