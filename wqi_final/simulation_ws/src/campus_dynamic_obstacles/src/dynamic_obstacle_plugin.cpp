#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class CampusDynamicObstaclePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    speed_ = sdf->HasElement("speed") ? sdf->Get<double>("speed") : 0.4;
    tolerance_ = sdf->HasElement("arrival_tolerance") ?
      sdf->Get<double>("arrival_tolerance") : 0.25;
    max_acceleration_ = sdf->HasElement("max_acceleration") ?
      sdf->Get<double>("max_acceleration") : 1.2;
    speed_ = std::max(0.01, speed_);
    tolerance_ = std::max(0.05, tolerance_);
    max_acceleration_ = std::max(0.1, max_acceleration_);

    if (sdf->HasElement("waypoint")) {
      auto waypoint = sdf->GetElement("waypoint");
      while (waypoint) {
        waypoints_.push_back(waypoint->Get<ignition::math::Vector3d>());
        waypoint = waypoint->GetNextElement("waypoint");
      }
    }

    if (waypoints_.size() < 2U) {
      gzerr << "CampusDynamicObstaclePlugin requires at least two waypoints\n";
      return;
    }

    model_->SetGravityMode(false);
    model_->SetAutoDisable(false);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(
        &CampusDynamicObstaclePlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void AdvanceTarget()
  {
    if (forward_) {
      if (target_index_ + 1U >= waypoints_.size()) {
        forward_ = false;
        target_index_ = waypoints_.size() - 2U;
      } else {
        ++target_index_;
      }
    } else if (target_index_ == 0U) {
      forward_ = true;
      target_index_ = 1U;
    } else {
      --target_index_;
    }
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!model_ || waypoints_.empty()) {
      return;
    }
    const auto position = model_->WorldPose().Pos();
    auto delta = waypoints_[target_index_] - position;
    if (delta.Length() <= tolerance_) {
      AdvanceTarget();
      delta = waypoints_[target_index_] - position;
    }
    if (delta.Length() < 1e-6) {
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      return;
    }
    delta.Normalize();
    const auto desired_velocity = delta * speed_;
    const double now = info.simTime.Double();
    if (last_update_time_seconds_ < 0.0) {
      last_update_time_seconds_ = now;
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      return;
    }
    const double dt = std::clamp(
      now - last_update_time_seconds_, 0.0, 0.1);
    last_update_time_seconds_ = now;
    auto velocity = model_->WorldLinearVel();
    auto change = desired_velocity - velocity;
    const double max_change = max_acceleration_ * dt;
    if (change.Length() > max_change && max_change > 0.0) {
      change.Normalize();
      change *= max_change;
    }
    velocity += change;
    model_->SetLinearVel(velocity);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  std::vector<ignition::math::Vector3d> waypoints_;
  std::size_t target_index_{1U};
  bool forward_{true};
  double speed_{0.4};
  double tolerance_{0.25};
  double max_acceleration_{1.2};
  double last_update_time_seconds_{-1.0};
};

GZ_REGISTER_MODEL_PLUGIN(CampusDynamicObstaclePlugin)
}  // namespace gazebo
