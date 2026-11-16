#ifndef UGVCAR_NAVIGATION2__APPROACH_PATH_ALIGN_CRITIC_HPP_
#define UGVCAR_NAVIGATION2__APPROACH_PATH_ALIGN_CRITIC_HPP_

#include "dwb_critics/path_align.hpp"

namespace ugvcar_navigation2
{
class ApproachPathAlignCritic : public dwb_critics::PathAlignCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

private:
  double disable_distance_{1.2};
};
}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__APPROACH_PATH_ALIGN_CRITIC_HPP_
