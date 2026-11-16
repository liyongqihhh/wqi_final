#ifndef UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_MODEL_HPP_
#define UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_MODEL_HPP_

#include <limits>
#include <vector>

namespace ugvcar_navigation2
{

struct TimedPoint2D
{
  double x{0.0};
  double y{0.0};
  double time{0.0};
};

struct MovingCircle2D
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
};

struct PredictiveCollisionResult
{
  double earliest_collision_time{std::numeric_limits<double>::infinity()};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double collision_exposure{0.0};
};

PredictiveCollisionResult evaluatePredictiveCollision(
  const std::vector<TimedPoint2D> & trajectory,
  const std::vector<MovingCircle2D> & obstacles,
  double robot_radius,
  double safety_margin,
  double horizon);

double predictiveTrajectoryScore(
  const PredictiveCollisionResult & result,
  double horizon,
  double preferred_clearance,
  double clearance_weight);

double collisionFreeMotionPenalty(
  const PredictiveCollisionResult & result,
  double progress_speed,
  double minimum_speed,
  double weight);

// Signed arc progress plus return-to-path progress; path samples use XY only.
double pathAlignedProgressSpeed(
  const std::vector<TimedPoint2D> & path,
  const TimedPoint2D & start,
  const TimedPoint2D & end,
  double duration);

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__PREDICTIVE_COLLISION_MODEL_HPP_
