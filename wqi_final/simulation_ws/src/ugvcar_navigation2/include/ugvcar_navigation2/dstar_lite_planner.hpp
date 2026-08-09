#ifndef UGVCAR_NAVIGATION2__DSTAR_LITE_PLANNER_HPP_
#define UGVCAR_NAVIGATION2__DSTAR_LITE_PLANNER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

#include "ugvcar_navigation2/dstar_lite.hpp"

namespace ugvcar_navigation2
{

class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool costIsTraversable(std::uint8_t cost) const;
  std::optional<std::size_t> findTraversableCell(
    unsigned int map_x, unsigned int map_y, double tolerance) const;
  std::optional<std::size_t> findGoalCell(
    unsigned int goal_x, unsigned int goal_y) const;
  void resetGridIfGeometryChanged();
  nav_msgs::msg::Path makePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & requested_goal,
    bool use_exact_start,
    bool use_exact_goal,
    const std::vector<std::size_t> & cells) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<DStarLiteGrid> grid_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;
  std::string global_frame_;
  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double tolerance_{0.25};
  double start_tolerance_{0.6};
  int lethal_cost_{253};
  bool allow_unknown_{false};
  double cost_penalty_{3.0};
  int max_expansions_{1000000};
};

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__DSTAR_LITE_PLANNER_HPP_
