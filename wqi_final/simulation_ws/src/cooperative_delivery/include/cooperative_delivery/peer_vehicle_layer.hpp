#ifndef COOPERATIVE_DELIVERY__PEER_VEHICLE_LAYER_HPP_
#define COOPERATIVE_DELIVERY__PEER_VEHICLE_LAYER_HPP_

#include <mutex>
#include <string>
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace cooperative_delivery
{
// Fleet-shared geometry, not a laser observation or a future-position prediction.
class PeerVehicleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;

protected:
  void receive(const nav_msgs::msg::Odometry & message);

private:
  std::mutex mutex_;
  nav_msgs::msg::Odometry pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  double radius_{.60}, timeout_{1.0}, x_{0}, y_{0}, old_x_{0}, old_y_{0};
  int64_t received_ns_{0};
  bool received_{false}, marked_{false}, previous_{false};
};
}  // namespace cooperative_delivery
#endif
