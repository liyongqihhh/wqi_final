#include "gtest/gtest.h"
#include "ugvcar_navigation2/dynamic_path_policy.hpp"

using ugvcar_navigation2::PathPoints;
using ugvcar_navigation2::shouldReplaceDynamicPath;

TEST(DynamicPathPolicy, RejectsEmptyCandidatesAndAcceptsFirstRoute)
{
  EXPECT_FALSE(shouldReplaceDynamicPath({}, {}, false));
  EXPECT_TRUE(shouldReplaceDynamicPath({}, {{0.0, 0.0}, {10.0, 0.0}}, false));
}

TEST(DynamicPathPolicy, DoesNotReplaceSafeRouteForSmallJitter)
{
  const PathPoints current{{0.0, 0.0}, {5.0, 0.05}, {10.0, 0.0}};
  const PathPoints candidate{{2.0, 0.0}, {10.0, 0.0}};
  EXPECT_FALSE(shouldReplaceDynamicPath(current, candidate, true));
}

TEST(DynamicPathPolicy, MeasuresRemainingLengthInsteadOfAlreadyDrivenLength)
{
  const PathPoints current{{0.0, 0.0}, {100.0, 0.0}};
  const PathPoints candidate{{90.0, 0.0}, {95.0, 2.0}, {100.0, 0.0}};
  EXPECT_DOUBLE_EQ(ugvcar_navigation2::remainingPathLength(current, {90.0, 0.0}), 10.0);
  EXPECT_FALSE(shouldReplaceDynamicPath(current, candidate, true));
}

TEST(DynamicPathPolicy, ReplacesInvalidRouteEvenWhenDetourIsLonger)
{
  const PathPoints current{{0.0, 0.0}, {10.0, 0.0}};
  const PathPoints candidate{{0.0, 0.0}, {0.0, 20.0}, {10.0, 20.0}, {10.0, 0.0}};
  EXPECT_TRUE(shouldReplaceDynamicPath(current, candidate, false));
  EXPECT_FALSE(shouldReplaceDynamicPath(current, candidate, true));
}

TEST(DynamicPathPolicy, ReleasesCampusWideDetourWhenShortcutClears)
{
  const PathPoints current{{0.0, 0.0}, {0.0, 140.0}, {20.0, 140.0}, {20.0, 0.0}};
  const PathPoints candidate{{0.0, 0.0}, {20.0, 0.0}};
  EXPECT_TRUE(shouldReplaceDynamicPath(current, candidate, true));
}

TEST(DynamicPathPolicy, RequiresBothAbsoluteAndRelativeImprovement)
{
  const PathPoints current{{0.0, 0.0}, {0.0, 100.0}, {100.0, 100.0}, {100.0, 0.0}};
  const PathPoints candidate{{0.0, 0.0}, {0.0, 98.0}, {100.0, 98.0}, {100.0, 0.0}};
  EXPECT_FALSE(shouldReplaceDynamicPath(current, candidate, true));
}

TEST(DynamicPathPolicy, HeadingTargetNeverSelectsHistoricalPointsBehindRobot)
{
  const PathPoints path{{-2.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};
  const auto target = ugvcar_navigation2::forwardPathTarget(path, {0.0, 0.0}, 0.8);
  EXPECT_NEAR(target.first, 0.8, 1.0e-6);
  EXPECT_DOUBLE_EQ(target.second, 0.0);
  const auto near_goal = ugvcar_navigation2::forwardPathTarget(path, {3.8, 0.0}, 0.8);
  EXPECT_DOUBLE_EQ(near_goal.first, 4.0);
}
