#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "ugvcar_navigation2/predictive_collision_model.hpp"

using ugvcar_navigation2::MovingCircle2D;
using ugvcar_navigation2::TimedPoint2D;

TEST(PredictiveCollisionModel, DetectsFasterObstacleCatchingFromBehind)
{
  const std::vector<TimedPoint2D> trajectory{{0.0, 0.0, 0.0}, {2.0, 0.0, 5.0}};
  const std::vector<MovingCircle2D> obstacles{{-2.0, 0.0, 0.8, 0.0, 0.45}};
  const auto result = ugvcar_navigation2::evaluatePredictiveCollision(
    trajectory, obstacles, 0.4, 0.2, 5.0);

  ASSERT_TRUE(std::isfinite(result.earliest_collision_time));
  EXPECT_NEAR(result.earliest_collision_time, 2.375, 1.0e-6);
}

TEST(PredictiveCollisionModel, IgnoresParallelObstacleWithConstantGap)
{
  const std::vector<TimedPoint2D> trajectory{{0.0, 0.0, 0.0}, {2.0, 0.0, 5.0}};
  const std::vector<MovingCircle2D> obstacles{{-2.0, 0.0, 0.4, 0.0, 0.45}};
  const auto result = ugvcar_navigation2::evaluatePredictiveCollision(
    trajectory, obstacles, 0.4, 0.2, 5.0);

  EXPECT_FALSE(std::isfinite(result.earliest_collision_time));
  EXPECT_NEAR(result.minimum_clearance, 0.95, 1.0e-6);
}

TEST(PredictiveCollisionModel, DetectsDiagonalCrossingBetweenSamples)
{
  const std::vector<TimedPoint2D> trajectory{{0.0, 0.0, 0.0}, {2.0, 0.0, 5.0}};
  const std::vector<MovingCircle2D> obstacles{{1.0, -2.0, 0.0, 0.8, 0.3}};
  const auto result = ugvcar_navigation2::evaluatePredictiveCollision(
    trajectory, obstacles, 0.3, 0.2, 5.0);

  EXPECT_TRUE(std::isfinite(result.earliest_collision_time));
  EXPECT_LT(result.earliest_collision_time, 2.5);
}

TEST(PredictiveCollisionModel, CurvedDetourCanRemainCollisionFree)
{
  const std::vector<MovingCircle2D> obstacles{{1.0, 0.0, 0.0, 0.0, 0.25}};
  const std::vector<TimedPoint2D> straight{{0.0, 0.0, 0.0}, {2.0, 0.0, 4.0}};
  const std::vector<TimedPoint2D> detour{
    {0.0, 0.0, 0.0}, {0.5, 0.8, 1.5}, {1.5, 0.8, 2.5}, {2.0, 0.0, 4.0}};

  const auto straight_result = ugvcar_navigation2::evaluatePredictiveCollision(
    straight, obstacles, 0.2, 0.1, 4.0);
  const auto detour_result = ugvcar_navigation2::evaluatePredictiveCollision(
    detour, obstacles, 0.2, 0.1, 4.0);
  EXPECT_TRUE(std::isfinite(straight_result.earliest_collision_time));
  EXPECT_FALSE(std::isfinite(detour_result.earliest_collision_time));
  EXPECT_GT(detour_result.minimum_clearance, 0.0);
}

TEST(PredictiveCollisionModel, RejectsInvalidHorizon)
{
  const std::vector<TimedPoint2D> trajectory{{0.0, 0.0, 0.0}};
  EXPECT_THROW(
    ugvcar_navigation2::evaluatePredictiveCollision(
      trajectory, {}, 0.2, 0.1, 0.0),
    std::invalid_argument);
}

TEST(PredictiveCollisionModel, ScoresWideClearanceBelowMarginalClearance)
{
  const ugvcar_navigation2::PredictiveCollisionResult wide{
    std::numeric_limits<double>::infinity(), 1.2};
  const ugvcar_navigation2::PredictiveCollisionResult marginal{
    std::numeric_limits<double>::infinity(), 0.2};

  const double wide_score = ugvcar_navigation2::predictiveTrajectoryScore(
    wide, 4.0, 0.8, 0.0001);
  const double marginal_score = ugvcar_navigation2::predictiveTrajectoryScore(
    marginal, 4.0, 0.8, 0.0001);
  EXPECT_DOUBLE_EQ(wide_score, 0.0);
  EXPECT_GT(marginal_score, wide_score);
  EXPECT_LT(marginal_score, 1.0);
}

TEST(PredictiveCollisionModel, CollidingCandidateWithLongerTtcScoresLower)
{
  const ugvcar_navigation2::PredictiveCollisionResult early{0.5, -0.1};
  const ugvcar_navigation2::PredictiveCollisionResult late{3.0, -0.1};
  EXPECT_GT(
    ugvcar_navigation2::predictiveTrajectoryScore(
      early, 4.0, 0.8, 0.0001),
    ugvcar_navigation2::predictiveTrajectoryScore(
      late, 4.0, 0.8, 0.0001));
}
