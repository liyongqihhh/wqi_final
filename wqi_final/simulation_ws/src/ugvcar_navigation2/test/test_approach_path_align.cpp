#include <limits>

#include "gtest/gtest.h"
#include "ugvcar_navigation2/approach_path_align_critic.hpp"

TEST(ApproachPathAlign, DisablesProjectedNoseBeforeObservedStoppingDistance)
{
  ugvcar_navigation2::ApproachPathAlignCritic critic;
  geometry_msgs::msg::Pose2D pose, goal;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  goal.x = 0.79;
  EXPECT_TRUE(critic.prepare(pose, velocity, goal, path));
  EXPECT_DOUBLE_EQ(critic.getScale(), 0.0);
}

TEST(ApproachPathAlign, HandlesAnAlreadyReachedGoalWithoutMapAccess)
{
  ugvcar_navigation2::ApproachPathAlignCritic critic;
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  EXPECT_TRUE(critic.prepare(pose, velocity, pose, path));
  EXPECT_DOUBLE_EQ(critic.getScale(), 0.0);
}

TEST(ApproachPathAlign, NonfiniteGoalDoesNotBecomeAZeroAlignmentScore)
{
  ugvcar_navigation2::ApproachPathAlignCritic critic;
  geometry_msgs::msg::Pose2D pose, goal;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;
  goal.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(critic.prepare(pose, velocity, goal, path));
}
