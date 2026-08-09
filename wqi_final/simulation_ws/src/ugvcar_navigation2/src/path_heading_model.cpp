#include "ugvcar_navigation2/path_heading_model.hpp"

#include <cmath>

namespace ugvcar_navigation2
{

double normalizeHeading(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double pathHeadingScore(
  const double current_heading,
  const double current_linear_velocity,
  const double desired_heading,
  const double candidate_heading,
  const double candidate_linear_velocity,
  const double activation_angle,
  const double activation_linear_velocity,
  const double translation_weight,
  const double idle_penalty,
  const double wrong_turn_penalty)
{
  const double initial_error = normalizeHeading(
    desired_heading - current_heading);
  if (
    std::abs(initial_error) < activation_angle ||
    std::abs(current_linear_velocity) > activation_linear_velocity)
  {
    return 0.0;
  }

  const double candidate_turn = normalizeHeading(
    candidate_heading - current_heading);
  double score = std::abs(
    normalizeHeading(desired_heading - candidate_heading));
  score += translation_weight * std::abs(candidate_linear_velocity);
  if (std::abs(candidate_turn) < 1.0e-3) {
    score += idle_penalty;
  } else if (candidate_turn * initial_error < 0.0) {
    score += wrong_turn_penalty + std::abs(candidate_turn);
  }
  return score;
}

}  // namespace ugvcar_navigation2
