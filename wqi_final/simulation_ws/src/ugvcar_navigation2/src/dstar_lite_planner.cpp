#include "ugvcar_navigation2/dstar_lite_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ugvcar_navigation2
{

void DStarLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("D* Lite planner parent node expired");
  }

  const std::string prefix = name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "start_tolerance", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "lethal_cost", rclcpp::ParameterValue(253));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "cost_penalty", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "max_expansions", rclcpp::ParameterValue(1000000));
  node->get_parameter(prefix + "tolerance", tolerance_);
  node->get_parameter(prefix + "start_tolerance", start_tolerance_);
  node->get_parameter(prefix + "lethal_cost", lethal_cost_);
  node->get_parameter(prefix + "allow_unknown", allow_unknown_);
  node->get_parameter(prefix + "cost_penalty", cost_penalty_);
  node->get_parameter(prefix + "max_expansions", max_expansions_);
  if (
    tolerance_ < 0.0 || start_tolerance_ < 0.0 ||
    lethal_cost_ <= 0 || lethal_cost_ > 255 ||
    cost_penalty_ < 0.0 || max_expansions_ <= 0)
  {
    throw nav2_core::PlannerException("D* Lite planner parameters are invalid");
  }

  status_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/ugv/dstar_lite/status", rclcpp::QoS(10));
  resetGridIfGeometryChanged();
  RCLCPP_INFO(
    node->get_logger(),
    "Configured D* Lite incremental planner on %u x %u grid",
    width_, height_);
}

void DStarLitePlanner::cleanup()
{
  grid_.reset();
  status_publisher_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void DStarLitePlanner::activate()
{
  if (status_publisher_) {
    status_publisher_->on_activate();
  }
}

void DStarLitePlanner::deactivate()
{
  if (status_publisher_) {
    status_publisher_->on_deactivate();
  }
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();
  if (!node || costmap_ == nullptr) {
    throw nav2_core::PlannerException("D* Lite planner is not configured");
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "D* Lite start and goal must be in " + global_frame_);
  }

  const auto started_at = std::chrono::steady_clock::now();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *costmap_->getMutex());
  resetGridIfGeometryChanged();

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_x, start_y))
  {
    throw nav2_core::PlannerException("D* Lite start is outside the costmap");
  }
  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    throw nav2_core::PlannerException("D* Lite goal is outside the costmap");
  }

  const std::size_t requested_start_cell =
    static_cast<std::size_t>(start_y) * width_ + start_x;
  const auto effective_start = findTraversableCell(
    start_x, start_y, start_tolerance_);
  if (!effective_start.has_value()) {
    throw nav2_core::PlannerException(
            "D* Lite found no traversable cell within start tolerance");
  }
  const auto effective_goal = findGoalCell(goal_x, goal_y);
  if (!effective_goal.has_value()) {
    throw nav2_core::PlannerException(
            "D* Lite found no traversable cell within goal tolerance");
  }
  const auto * raw_costs = costmap_->getCharMap();
  std::vector<std::uint8_t> costs(
    raw_costs, raw_costs + static_cast<std::size_t>(width_) * height_);
  const DStarLiteResult result = grid_->plan(
    *effective_start, *effective_goal, costs);
  if (result.cells.empty()) {
    throw nav2_core::PlannerException("D* Lite found no finite-cost path");
  }
  const std::size_t requested_goal_cell =
    static_cast<std::size_t>(goal_y) * width_ + goal_x;
  nav_msgs::msg::Path path = makePath(
    start, goal, *effective_start == requested_start_cell,
    *effective_goal == requested_goal_cell, result.cells);
  lock.unlock();

  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started_at).count();
  if (status_publisher_ && status_publisher_->is_activated()) {
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "algorithm=dstar_lite;reused="
           << (result.reused_search ? "true" : "false")
           << ";changed_cells=" << result.changed_cells
           << ";expanded_vertices=" << result.expansions
           << ";path_cells=" << result.cells.size()
           << ";adjusted_start="
           << (*effective_start == requested_start_cell ? "false" : "true")
           << ";planning_ms=" << elapsed_ms;
    status.data = stream.str();
    status_publisher_->publish(status);
  }
  return path;
}

bool DStarLitePlanner::costIsTraversable(const std::uint8_t cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < static_cast<std::uint8_t>(lethal_cost_);
}

std::optional<std::size_t> DStarLitePlanner::findGoalCell(
  const unsigned int goal_x, const unsigned int goal_y) const
{
  return findTraversableCell(goal_x, goal_y, tolerance_);
}

std::optional<std::size_t> DStarLitePlanner::findTraversableCell(
  const unsigned int map_x, const unsigned int map_y,
  const double tolerance) const
{
  if (costIsTraversable(costmap_->getCost(map_x, map_y))) {
    return static_cast<std::size_t>(map_y) * width_ + map_x;
  }
  const int radius = static_cast<int>(std::ceil(tolerance / resolution_));
  double best_distance = std::numeric_limits<double>::infinity();
  std::optional<std::size_t> best;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = static_cast<int>(map_x) + dx;
      const int y = static_cast<int>(map_y) + dy;
      if (
        x < 0 || y < 0 || x >= static_cast<int>(width_) ||
        y >= static_cast<int>(height_))
      {
        continue;
      }
      const double distance = resolution_ * std::hypot(dx, dy);
      if (
        distance <= tolerance && distance < best_distance &&
        costIsTraversable(costmap_->getCost(x, y)))
      {
        best_distance = distance;
        best = static_cast<std::size_t>(y) * width_ +
          static_cast<std::size_t>(x);
      }
    }
  }
  return best;
}

void DStarLitePlanner::resetGridIfGeometryChanged()
{
  if (costmap_ == nullptr) {
    return;
  }
  const auto next_width = costmap_->getSizeInCellsX();
  const auto next_height = costmap_->getSizeInCellsY();
  const auto next_resolution = costmap_->getResolution();
  const auto next_origin_x = costmap_->getOriginX();
  const auto next_origin_y = costmap_->getOriginY();
  if (
    grid_ && width_ == next_width && height_ == next_height &&
    resolution_ == next_resolution && origin_x_ == next_origin_x &&
    origin_y_ == next_origin_y)
  {
    return;
  }
  width_ = next_width;
  height_ = next_height;
  resolution_ = next_resolution;
  origin_x_ = next_origin_x;
  origin_y_ = next_origin_y;
  grid_ = std::make_unique<DStarLiteGrid>(
    width_, height_, resolution_, static_cast<std::uint8_t>(lethal_cost_),
    allow_unknown_, cost_penalty_, static_cast<std::size_t>(max_expansions_));
}

nav_msgs::msg::Path DStarLitePlanner::makePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & requested_goal,
  const bool use_exact_start,
  const bool use_exact_goal,
  const std::vector<std::size_t> & cells) const
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  path.poses.reserve(cells.size() + 1);
  geometry_msgs::msg::PoseStamped first = start;
  first.header = path.header;
  path.poses.push_back(first);

  const std::size_t first_cell = use_exact_start ? 1 : 0;
  for (std::size_t index = first_cell; index < cells.size(); ++index) {
    const std::size_t cell = cells[index];
    const unsigned int map_x = static_cast<unsigned int>(cell % width_);
    const unsigned int map_y = static_cast<unsigned int>(cell / width_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    costmap_->mapToWorld(
      map_x, map_y, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  if (use_exact_goal) {
    geometry_msgs::msg::PoseStamped exact_goal = requested_goal;
    exact_goal.header = path.header;
    if (
      path.poses.empty() ||
      std::hypot(
        path.poses.back().pose.position.x - exact_goal.pose.position.x,
        path.poses.back().pose.position.y - exact_goal.pose.position.y) > 1.0e-4)
    {
      path.poses.push_back(exact_goal);
    } else {
      path.poses.back().pose.orientation = exact_goal.pose.orientation;
    }
  }

  for (std::size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(next.y - current.y, next.x - current.x));
    path.poses[index].pose.orientation.x = orientation.x();
    path.poses[index].pose.orientation.y = orientation.y();
    path.poses[index].pose.orientation.z = orientation.z();
    path.poses[index].pose.orientation.w = orientation.w();
  }
  return path;
}

}  // namespace ugvcar_navigation2

PLUGINLIB_EXPORT_CLASS(
  ugvcar_navigation2::DStarLitePlanner, nav2_core::GlobalPlanner)
