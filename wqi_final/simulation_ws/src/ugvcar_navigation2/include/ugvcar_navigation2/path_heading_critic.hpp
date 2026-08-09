#ifndef UGVCAR_NAVIGATION2__PATH_HEADING_CRITIC_HPP_
#define UGVCAR_NAVIGATION2__PATH_HEADING_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace ugvcar_navigation2
{

class PathHeadingCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  geometry_msgs::msg::Pose2D current_pose_;
  double current_linear_velocity_{0.0};
  double desired_heading_{0.0};
  bool has_target_{false};
  double lookahead_distance_{0.8};
  double activation_angle_{0.7};
  double activation_linear_velocity_{0.08};
  double translation_weight_{3.0};
  double idle_penalty_{0.6};
  double wrong_turn_penalty_{1.0};
};

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__PATH_HEADING_CRITIC_HPP_
