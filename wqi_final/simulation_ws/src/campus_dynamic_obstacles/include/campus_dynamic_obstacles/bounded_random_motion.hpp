#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace campus_dynamic_obstacles
{
using Vector = std::array<double, 3>;

// Random targets plus correlated (OU) velocity noise. Gazebo integrates the
// returned velocity; this controller never changes a body's position.
class BoundedRandomMotion
{
public:
  BoundedRandomMotion(Vector lower, Vector upper, double speed, double acceleration,
    std::uint32_t seed, double correlation = 2.5, double noise = .18)
  : lower_(lower), upper_(upper), speed_(speed), acceleration_(acceleration),
    correlation_(correlation), noise_(noise), generator_(seed)
  {
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(acceleration) ||
      acceleration <= 0 || !std::isfinite(correlation) || correlation <= 0 ||
      !std::isfinite(noise) || noise < 0 || noise > 1)
    {throw std::invalid_argument("Invalid random motion parameters");}
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > upper[i]) {
        throw std::invalid_argument("Invalid random motion bounds");
      }
    }
    if (upper[0] <= lower[0] || upper[1] <= lower[1]) {
      throw std::invalid_argument("Random motion needs a nonzero XY area");
    }
    DrawTarget();
  }

  Vector Step(const Vector & position, const Vector & velocity, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0 || dt > .1) {return velocity;}
    target_age_ += dt;
    noise_age_ += dt;
    // Draw on simulation time, not on the number of physics callbacks.
    while (noise_age_ >= .1) {
      noise_age_ -= .1;
      const double decay = std::exp(-.1 / correlation_);
      for (int i = 0; i < 3; ++i) {
        perturbation_[i] = decay * perturbation_[i] +
          noise_ * speed_ * std::sqrt(1 - decay * decay) * normal_(generator_);
      }
    }
    if (target_age_ >= target_duration_ || Norm(Subtract(target_, position)) < .25) {
      DrawTarget();
    }
    auto desired = Subtract(target_, position);
    const double distance = Norm(desired);
    const double pace = std::min(speed_, distance * .9);
    for (int i = 0; i < 3; ++i) {
      desired[i] = (distance > 1e-9 ? desired[i] * pace / distance : 0.) + perturbation_[i];
    }
    const double length = Norm(desired);
    if (length > speed_) {
      for (auto & value : desired) {value *= speed_ / length;}
    }
    Vector result{};
    // Per-axis acceleration bounds imply the requested Euclidean bound.
    const double axis_acceleration = acceleration_ / std::sqrt(3.);
    for (int i = 0; i < 3; ++i) {
      if (upper_[i] == lower_[i]) {result[i] = 0.; continue;}
      const double margin = std::min(.10, (upper_[i] - lower_[i]) * .1);
      const auto safe_speed = [&](double room) {
          const double a_dt = axis_acceleration * dt;
          return std::max(0., std::sqrt(a_dt * a_dt +
            2 * axis_acceleration * std::max(0., room - margin)) - a_dt);
        };
      const double positive = safe_speed(upper_[i] - position[i]);
      const double negative = safe_speed(position[i] - lower_[i]);
      desired[i] = std::clamp(desired[i], -negative, positive);
      result[i] = velocity[i] + std::clamp(desired[i] - velocity[i],
          -axis_acceleration * dt, axis_acceleration * dt);
      // Normally redundant: protects the envelope after external collisions.
      result[i] = std::clamp(result[i], -negative, positive);
    }
    const double result_length = Norm(result);
    if (result_length > speed_) {
      for (auto & value : result) {value *= speed_ / result_length;}
    }
    return result;
  }

private:
  static Vector Subtract(const Vector & a, const Vector & b)
  {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
  static double Norm(const Vector & a)
  {return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);}
  void DrawTarget()
  {
    for (int i = 0; i < 3; ++i) {
      const double midpoint = (lower_[i] + upper_[i]) / 2;
      const double half_span = (upper_[i] - lower_[i]) * .4;
      target_[i] = midpoint + uniform_(generator_) * half_span;
    }
    target_duration_ = 3. + uniform_(generator_) * 1.5;
    target_age_ = 0.;
  }
  Vector lower_, upper_, target_{}, perturbation_{};
  double speed_, acceleration_, correlation_, noise_;
  double target_age_{0.}, target_duration_{3.}, noise_age_{0.};
  std::mt19937 generator_;
  std::uniform_real_distribution<double> uniform_{-1., 1.};
  std::normal_distribution<double> normal_{0., 1.};
};
}  // namespace campus_dynamic_obstacles
