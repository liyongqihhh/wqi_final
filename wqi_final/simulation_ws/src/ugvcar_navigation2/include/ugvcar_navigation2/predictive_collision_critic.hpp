#ifndef UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_CRITIC_HPP_
#define UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_CRITIC_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/string.hpp"
#include "ugvcar_navigation2_interfaces/msg/dynamic_obstacle_array.hpp"

#include "ugvcar_navigation2/predictive_collision_model.hpp"

namespace ugvcar_navigation2
{

class PredictiveCollisionCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;
  void debrief(const nav_2d_msgs::msg::Twist2D & command) override;

private:
  struct CandidateEvaluation
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double collision_time{std::numeric_limits<double>::infinity()};
    double minimum_clearance{std::numeric_limits<double>::infinity()};
  };

  void tracksCallback(
    const ugvcar_navigation2_interfaces::msg::DynamicObstacleArray::SharedPtr message);
  std::vector<MovingCircle2D> transformTracks(
    const ugvcar_navigation2_interfaces::msg::DynamicObstacleArray & message,
    double age) const;

  std::mutex tracks_mutex_;
  ugvcar_navigation2_interfaces::msg::DynamicObstacleArray latest_tracks_;
  bool received_tracks_{false};
  std::vector<MovingCircle2D> prepared_obstacles_;
  std::vector<CandidateEvaluation> evaluations_;
  geometry_msgs::msg::Pose2D current_pose_;
  rclcpp::Subscription<
    ugvcar_navigation2_interfaces::msg::DynamicObstacleArray>::SharedPtr
    tracks_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  std::string tracks_topic_{"/ugv/tracked_dynamic_obstacles"};
  double robot_radius_{0.22};
  double safety_margin_{0.25};
  double track_timeout_{0.75};
  double prediction_horizon_{3.0};
  double preferred_clearance_{0.8};
  double clearance_weight_{0.0001};
};

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_CRITIC_HPP_
