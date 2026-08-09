#ifndef UGVCAR_NAVIGATION2__PATH_HEADING_MODEL_HPP_
#define UGVCAR_NAVIGATION2__PATH_HEADING_MODEL_HPP_

namespace ugvcar_navigation2
{

double normalizeHeading(double angle);

double pathHeadingScore(
  double current_heading,
  double current_linear_velocity,
  double desired_heading,
  double candidate_heading,
  double candidate_linear_velocity,
  double activation_angle,
  double activation_linear_velocity,
  double translation_weight,
  double idle_penalty,
  double wrong_turn_penalty);

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__PATH_HEADING_MODEL_HPP_
