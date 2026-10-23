#ifndef UGVCAR_NAVIGATION2__DYNAMIC_PATH_POLICY_HPP_
#define UGVCAR_NAVIGATION2__DYNAMIC_PATH_POLICY_HPP_

#include <utility>
#include <vector>

namespace ugvcar_navigation2
{
using PathPoints = std::vector<std::pair<double, double>>;

double pathLength(const PathPoints & path);
double remainingPathLength(const PathPoints & path, const std::pair<double, double> & robot);
std::pair<double, double> forwardPathTarget(
  const PathPoints & path, const std::pair<double, double> & robot, double lookahead);
bool shouldReplaceDynamicPath(
  const PathPoints & current, const PathPoints & candidate, bool current_valid,
  double minimum_improvement = 1.0, double improvement_ratio = 0.05);
}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__DYNAMIC_PATH_POLICY_HPP_
