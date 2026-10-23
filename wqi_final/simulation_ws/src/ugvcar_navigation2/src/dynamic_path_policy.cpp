#include "ugvcar_navigation2/dynamic_path_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugvcar_navigation2
{
double pathLength(const PathPoints & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].first - path[i - 1].first, path[i].second - path[i - 1].second);
  }
  return length;
}

double remainingPathLength(const PathPoints & path, const std::pair<double, double> & robot)
{
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double nearest_distance = std::numeric_limits<double>::infinity();
  double nearest_progress = 0.0;
  double progress = 0.0;
  // Project onto segments rather than comparing complete path lengths: the
  // accepted path still contains the distance already driven by the robot.
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double squared_length = dx * dx + dy * dy;
    const double fraction = squared_length > 0.0 ? std::clamp(
      ((robot.first - a.first) * dx + (robot.second - a.second) * dy) / squared_length,
      0.0, 1.0) : 0.0;
    const double distance = std::hypot(
      robot.first - (a.first + fraction * dx), robot.second - (a.second + fraction * dy));
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_progress = progress + fraction * std::sqrt(squared_length);
    }
    progress += std::sqrt(squared_length);
  }
  return progress - nearest_progress;
}

bool shouldReplaceDynamicPath(
  const PathPoints & current, const PathPoints & candidate, const bool current_valid,
  const double minimum_improvement, const double improvement_ratio)
{
  if (candidate.empty()) {
    return false;
  }
  if (current.empty() || !current_valid) {
    return true;
  }
  const double remaining = remainingPathLength(current, candidate.front());
  const double improvement = remaining - pathLength(candidate);
  return improvement >= std::max(minimum_improvement, improvement_ratio * remaining);
}

std::pair<double, double> forwardPathTarget(
  const PathPoints & path, const std::pair<double, double> & robot, const double lookahead)
{
  if (path.empty()) {
    return robot;
  }
  const double target_progress =
    pathLength(path) - remainingPathLength(path, robot) + std::max(0.0, lookahead);
  double progress = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    const double length = std::hypot(b.first - a.first, b.second - a.second);
    if (length > 0.0 && progress + length >= target_progress) {
      const double fraction = (target_progress - progress) / length;
      return {a.first + fraction * (b.first - a.first),
        a.second + fraction * (b.second - a.second)};
    }
    progress += length;
  }
  return path.back();
}
}  // namespace ugvcar_navigation2
