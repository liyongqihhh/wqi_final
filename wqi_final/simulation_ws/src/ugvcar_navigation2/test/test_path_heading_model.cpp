#include <gtest/gtest.h>

#include "ugvcar_navigation2/path_heading_model.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double score(
  const double current_heading,
  const double current_velocity,
  const double desired_heading,
  const double candidate_heading,
  const double candidate_velocity)
{
  return ugvcar_navigation2::pathHeadingScore(
    current_heading, current_velocity, desired_heading,
    candidate_heading, candidate_velocity,
    0.7, 0.08, 3.0, 0.6, 1.0);
}

TEST(PathHeadingModel, IsInactiveWhenAlreadyAligned)
{
  EXPECT_DOUBLE_EQ(score(0.0, 0.0, 0.3, 0.1, 0.0), 0.0);
}

TEST(PathHeadingModel, IsInactiveDuringNormalForwardTracking)
{
  EXPECT_DOUBLE_EQ(score(0.0, 0.2, 1.2, 0.3, 0.2), 0.0);
}

TEST(PathHeadingModel, CorrectTurnBeatsIdleAndWrongTurn)
{
  const double correct = score(0.0, 0.0, 2.0, 0.5, 0.0);
  const double idle = score(0.0, 0.0, 2.0, 0.0, 0.0);
  const double wrong = score(0.0, 0.0, 2.0, -0.5, 0.0);
  EXPECT_LT(correct, idle);
  EXPECT_LT(correct, wrong);
}

TEST(PathHeadingModel, AvoidsTranslatingBeforeLargeHeadingCorrection)
{
  const double rotate = score(0.0, 0.0, 2.0, 0.5, 0.0);
  const double moving_arc = score(0.0, 0.0, 2.0, 0.5, 0.2);
  EXPECT_LT(rotate, moving_arc);
}

TEST(PathHeadingModel, HandlesAngleWrapping)
{
  const double correct = score(kPi - 0.1, 0.0, -kPi + 0.9, -kPi + 0.2, 0.0);
  const double wrong = score(kPi - 0.1, 0.0, -kPi + 0.9, kPi - 0.4, 0.0);
  EXPECT_LT(correct, wrong);
}

}  // namespace
