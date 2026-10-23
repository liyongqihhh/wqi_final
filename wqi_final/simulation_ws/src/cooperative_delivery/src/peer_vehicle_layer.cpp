#include "cooperative_delivery/peer_vehicle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cooperative_delivery
{
void PeerVehicleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("Peer vehicle layer has no lifecycle node");}
  declareParameter("pose_topic", rclcpp::ParameterValue("/ground_truth/odom"));
  declareParameter("radius", rclcpp::ParameterValue(.60));
  declareParameter("timeout", rclcpp::ParameterValue(1.0));
  std::string topic;
  node->get_parameter(name_ + ".pose_topic", topic);
  node->get_parameter(name_ + ".radius", radius_);
  node->get_parameter(name_ + ".timeout", timeout_);
  if (!std::isfinite(radius_) || !std::isfinite(timeout_) || radius_ <= 0 || timeout_ <= 0) {
    throw std::runtime_error("Peer vehicle radius and timeout must be positive and finite");
  }
  enabled_ = true;
  current_ = false;
  subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {receive(*message);});
}

void PeerVehicleLayer::receive(const nav_msgs::msg::Odometry & message)
{
  if (!std::isfinite(message.pose.pose.position.x) ||
    !std::isfinite(message.pose.pose.position.y) || message.header.frame_id.empty()) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = message;
  received_ns_ = clock_->now().nanoseconds();
  received_ = true;
}

void PeerVehicleLayer::reset()
{
  // A costmap-clear request must not erase a real parked vehicle.
  current_ = false;
}

void PeerVehicleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  auto include = [&](double x, double y) {
      *min_x = std::min(*min_x, x - radius_);
      *min_y = std::min(*min_y, y - radius_);
      *max_x = std::max(*max_x, x + radius_);
      *max_y = std::max(*max_y, y + radius_);
    };
  // LayeredCostmap resets this union before composing layers, removing the old disk.
  if (previous_) {include(old_x_, old_y_);}
  nav_msgs::msg::Odometry pose;
  bool received;
  int64_t stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose = pose_;
    received = received_;
    stamp = received_ns_;
  }
  const double age = (clock_->now().nanoseconds() - stamp) * 1e-9;
  current_ = received && age >= 0 && age <= timeout_;
  if (received) {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = pose.header.frame_id;
    // The existing simulation map origin coincides with Gazebo world.
    if (point.header.frame_id == "world") {point.header.frame_id = "map";}
    point.point = pose.pose.pose.position;
    try {
      if (point.header.frame_id != layered_costmap_->getGlobalFrameID()) {
        point = tf_->transform(point, layered_costmap_->getGlobalFrameID());
      }
      x_ = point.point.x;
      y_ = point.point.y;
      marked_ = true;
    } catch (const tf2::TransformException &) {
      current_ = false;
    }
  }
  // Keep the last envelope on data loss and mark the costmap non-current (fail closed).
  if (marked_) {
    include(x_, y_);
    old_x_ = x_;
    old_y_ = y_;
    previous_ = true;
  }
}

void PeerVehicleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!marked_) {return;}
  const double radius = radius_ + .5 * std::sqrt(2.0) * grid.getResolution();
  // The dirty union may cover the entire campus for distant peers. Only this
  // peer's disk can change costs; intersect it with the requested update region.
  int left, bottom, right, top;
  grid.worldToMapEnforceBounds(x_ - radius, y_ - radius, left, bottom);
  grid.worldToMapEnforceBounds(x_ + radius, y_ + radius, right, top);
  min_i = std::max(min_i, left);
  min_j = std::max(min_j, bottom);
  max_i = std::min(max_i, right + 1);
  max_j = std::min(max_j, top + 1);
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double x, y;
      grid.mapToWorld(i, j, x, y);
      if (std::hypot(x - x_, y - y_) <= radius) {
        grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}
}  // namespace cooperative_delivery

PLUGINLIB_EXPORT_CLASS(cooperative_delivery::PeerVehicleLayer, nav2_costmap_2d::Layer)
