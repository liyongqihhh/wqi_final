#include "ugvcar_navigation2/predictive_collision_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ugvcar_navigation2
{

namespace
{
constexpr double kEpsilon = 1.0e-12;

void evaluateSegment(
  const TimedPoint2D & start,
  const TimedPoint2D & end,
  const MovingCircle2D & obstacle,
  const double combined_radius,
  PredictiveCollisionResult & result)
{
  const double duration = end.time - start.time;
  if (duration < -kEpsilon) {
    throw std::invalid_argument("Predictive trajectory times must be ordered");
  }
  const double obstacle_x = obstacle.x + obstacle.vx * start.time;
  const double obstacle_y = obstacle.y + obstacle.vy * start.time;
  const double relative_x = start.x - obstacle_x;
  const double relative_y = start.y - obstacle_y;
  const double start_distance = std::hypot(relative_x, relative_y);
  result.minimum_clearance = std::min(
    result.minimum_clearance, start_distance - combined_radius);
  if (start_distance <= combined_radius) {
    result.earliest_collision_time = std::min(
      result.earliest_collision_time, start.time);
    return;
  }
  if (duration <= kEpsilon) {
    return;
  }

  const double robot_vx = (end.x - start.x) / duration;
  const double robot_vy = (end.y - start.y) / duration;
  const double relative_vx = robot_vx - obstacle.vx;
  const double relative_vy = robot_vy - obstacle.vy;
  const double speed_squared =
    relative_vx * relative_vx + relative_vy * relative_vy;
  if (speed_squared <= kEpsilon) {
    return;
  }

  const double closest_time = std::clamp(
    -(relative_x * relative_vx + relative_y * relative_vy) /
    speed_squared,
    0.0,
    duration);
  const double closest_x = relative_x + relative_vx * closest_time;
  const double closest_y = relative_y + relative_vy * closest_time;
  result.minimum_clearance = std::min(
    result.minimum_clearance,
    std::hypot(closest_x, closest_y) - combined_radius);

  const double b = 2.0 * (
    relative_x * relative_vx + relative_y * relative_vy);
  const double c =
    relative_x * relative_x + relative_y * relative_y -
    combined_radius * combined_radius;
  const double discriminant = b * b - 4.0 * speed_squared * c;
  if (discriminant < 0.0) {
    return;
  }
  const double root =
    (-b - std::sqrt(std::max(0.0, discriminant))) /
    (2.0 * speed_squared);
  if (root >= 0.0 && root <= duration) {
    result.earliest_collision_time = std::min(
      result.earliest_collision_time, start.time + root);
  }
}
}  // namespace

PredictiveCollisionResult evaluatePredictiveCollision(
  const std::vector<TimedPoint2D> & trajectory,
  const std::vector<MovingCircle2D> & obstacles,
  const double robot_radius,
  const double safety_margin,
  const double horizon)
{
  if (
    trajectory.empty() || robot_radius <= 0.0 || safety_margin < 0.0 ||
    horizon <= 0.0 || !std::isfinite(robot_radius) ||
    !std::isfinite(safety_margin) || !std::isfinite(horizon))
  {
    throw std::invalid_argument("Predictive collision parameters are invalid");
  }
  for (const auto & point : trajectory) {
    if (
      !std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.time) || point.time < 0.0)
    {
      throw std::invalid_argument("Predictive trajectory contains invalid data");
    }
  }

  PredictiveCollisionResult result;
  for (const auto & obstacle : obstacles) {
    if (
      !std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
      !std::isfinite(obstacle.vx) || !std::isfinite(obstacle.vy) ||
      !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0)
    {
      continue;
    }
    const double combined_radius =
      robot_radius + safety_margin + obstacle.radius;
    for (std::size_t index = 0; index + 1 < trajectory.size(); ++index) {
      if (trajectory[index].time > horizon) {
        break;
      }
      TimedPoint2D end = trajectory[index + 1];
      if (end.time > horizon) {
        const double duration = end.time - trajectory[index].time;
        const double ratio = duration > kEpsilon ?
          (horizon - trajectory[index].time) / duration : 0.0;
        end.x = trajectory[index].x +
          ratio * (end.x - trajectory[index].x);
        end.y = trajectory[index].y +
          ratio * (end.y - trajectory[index].y);
        end.time = horizon;
      }
      evaluateSegment(
        trajectory[index], end, obstacle, combined_radius, result);
      if (end.time >= horizon) {
        break;
      }
    }

    const TimedPoint2D & last = trajectory.back();
    if (last.time < horizon) {
      evaluateSegment(
        last,
        TimedPoint2D{last.x, last.y, horizon},
        obstacle,
        combined_radius,
        result);
    } else if (trajectory.size() == 1) {
      evaluateSegment(last, last, obstacle, combined_radius, result);
    }
  }
  return result;
}

double predictiveTrajectoryScore(
  const PredictiveCollisionResult & result,
  const double horizon,
  const double preferred_clearance,
  const double clearance_weight)
{
  if (
    horizon <= 0.0 || preferred_clearance <= 0.0 ||
    clearance_weight < 0.0 || !std::isfinite(horizon) ||
    !std::isfinite(preferred_clearance) || !std::isfinite(clearance_weight))
  {
    throw std::invalid_argument("Predictive score parameters are invalid");
  }
  if (std::isfinite(result.earliest_collision_time)) {
    const double normalized_time = std::clamp(
      result.earliest_collision_time / horizon, 0.0, 1.0);
    // Every collision-free candidate scores below one. If every candidate
    // collides, the smallest value is the trajectory with the largest TTC.
    return 1.0 + (1.0 - normalized_time);
  }
  if (!std::isfinite(result.minimum_clearance)) {
    return 0.0;
  }
  const double clearance_deficit = std::clamp(
    (preferred_clearance - result.minimum_clearance) / preferred_clearance,
    0.0,
    1.0);
  return clearance_weight * clearance_deficit;
}

}  // namespace ugvcar_navigation2
