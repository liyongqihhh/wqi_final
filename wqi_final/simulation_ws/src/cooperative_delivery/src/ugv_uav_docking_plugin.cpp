#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_interfaces/srv/select_carrier.hpp>

#include <memory>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cooperative_delivery
{

class UgvUavDockingPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    uav_model_ = model;
    world_ = model->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    ugv_model_name_ = ReadString(sdf, "ugvModel", "ugvcar");
    ugv_link_name_ = ReadString(sdf, "ugvLink", "base_link");
    uav_link_name_ = ReadString(sdf, "uavLink", "base_link");
    docking_offset_.Pos().X() = ReadDouble(sdf, "dockingOffsetX", 0.0);
    docking_offset_.Pos().Y() = ReadDouble(sdf, "dockingOffsetY", 0.0);
    docking_offset_.Pos().Z() = ReadDouble(sdf, "dockingOffsetZ", 0.42);
    maximum_docking_distance_ = ReadDouble(
      sdf, "maximumDockingDistance", 0.8);
    strict_docking_ = sdf->HasElement("strictDocking") &&
      sdf->Get<bool>("strictDocking");
    if (strict_docking_) {
      maximum_docking_distance_ = 0.18;
    }
    // Keep the legacy two-car allowlist unless a scene explicitly supplies one.
    allowed_carriers_ = {{"ugvcar", "base_footprint"}, {"ugv_b", "ugv_b/base_footprint"}};
    if (sdf->HasElement("allowedCarrier")) {
      allowed_carriers_.clear();
      auto entry = sdf->GetElement("allowedCarrier");
      while (entry) {
        const auto name = ReadString(entry, "model", "");
        const auto link = ReadString(entry, "link", "");
        if (!name.empty() && !link.empty()) {
          allowed_carriers_.emplace_back(name, link);
        }
        entry = entry->GetNextElement("allowedCarrier");
      }
    }

    uav_link_ = uav_model_->GetLink(uav_link_name_);
    if (!uav_link_) {
      RCLCPP_FATAL(
        ros_node_->get_logger(), "UAV link '%s' was not found",
        uav_link_name_.c_str());
      return;
    }

    auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    docked_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
      "docked", status_qos);
    carrier_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "docked_carrier", status_qos);
    select_service_ = ros_node_->create_service<uav_interfaces::srv::SelectCarrier>(
      "select_carrier",
      [this](const std::shared_ptr<uav_interfaces::srv::SelectCarrier::Request> request,
      std::shared_ptr<uav_interfaces::srv::SelectCarrier::Response> response) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (fixed_joint_) {
          response->message = "Detach UAV before selecting another carrier";
          return;
        }
        bool allowed = false;
        for (const auto & carrier : allowed_carriers_) {
          allowed = allowed || (request->model_name == carrier.first &&
            request->link_name == carrier.second);
        }
        auto candidate = allowed ? world_->ModelByName(request->model_name) : nullptr;
        if (!candidate || !candidate->GetLink(request->link_name)) {
          response->message = "Requested carrier or docking link is unavailable";
          return;
        }
        ugv_model_name_ = request->model_name;
        ugv_link_name_ = request->link_name;
        response->success = true;
        response->message = "Docking target selected: " + ugv_model_name_;
      });
    attach_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "attach_uav",
      std::bind(
        &UgvUavDockingPlugin::Attach, this,
        std::placeholders::_1, std::placeholders::_2));
    detach_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "detach_uav",
      std::bind(
        &UgvUavDockingPlugin::Detach, this,
        std::placeholders::_1, std::placeholders::_2));

    PublishStatus(false);
    if (!sdf->HasElement("autoAttach") || sdf->Get<bool>("autoAttach")) {
      auto initial_docking_response =
        std::make_shared<std_srvs::srv::Trigger::Response>();
      Attach(nullptr, initial_docking_response);
      if (!initial_docking_response->success) {
        RCLCPP_WARN(
          ros_node_->get_logger(), "Initial UAV docking deferred: %s",
          initial_docking_response->message.c_str());
      }
    }
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "UGV-UAV docking plugin ready for model '%s'",
      ugv_model_name_.c_str());
  }

private:
  static std::string ReadString(
    const sdf::ElementPtr & sdf, const std::string & name,
    const std::string & fallback)
  {
    return sdf->HasElement(name) ?
      sdf->GetElement(name)->Get<std::string>() : fallback;
  }

  static double ReadDouble(
    const sdf::ElementPtr & sdf, const std::string & name, double fallback)
  {
    return sdf->HasElement(name) ?
      sdf->GetElement(name)->Get<double>() : fallback;
  }

  void PublishStatus(bool docked)
  {
    std_msgs::msg::Bool message;
    message.data = docked;
    docked_pub_->publish(message);
    std_msgs::msg::String carrier;
    carrier.data = docked ? ugv_model_name_ : "";
    carrier_pub_->publish(carrier);
  }

  void Attach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (fixed_joint_) {
      response->success = true;
      response->message = "UAV is already docked";
      PublishStatus(true);
      return;
    }

    auto ugv_model = world_->ModelByName(ugv_model_name_);
    if (!ugv_model) {
      response->success = false;
      response->message = "UGV model is unavailable";
      return;
    }
    auto ugv_link = ugv_model->GetLink(ugv_link_name_);
    if (!ugv_link) {
      response->success = false;
      response->message = "UGV docking link is unavailable";
      return;
    }

    const auto target_link_pose = ugv_link->WorldPose() * docking_offset_;
    const auto current_link_pose = uav_link_->WorldPose();
    const double ugv_linear = ugv_link->WorldLinearVel().Length();
    const double ugv_angular = ugv_link->WorldAngularVel().Length();
    const double uav_linear = uav_link_->WorldLinearVel().Length();
    const double uav_angular = uav_link_->WorldAngularVel().Length();
    if (strict_docking_ && (
        !std::isfinite(ugv_linear) || !std::isfinite(ugv_angular) ||
        !std::isfinite(uav_linear) || !std::isfinite(uav_angular) ||
        ugv_linear > 0.05 || ugv_angular > 0.10 ||
        uav_linear > 0.20 || uav_angular > 0.20))
    {
      response->success = false;
      response->message = "Both vehicles must be settled before capture; ugv_linear=" +
        std::to_string(ugv_linear) + ", ugv_angular=" + std::to_string(ugv_angular) +
        ", uav_linear=" + std::to_string(uav_linear) + ", uav_angular=" +
        std::to_string(uav_angular);
      RCLCPP_WARN(ros_node_->get_logger(), "%s", response->message.c_str());
      return;
    }
    const double docking_distance = (
      target_link_pose.Pos() - current_link_pose.Pos()).Length();
    if (docking_distance > maximum_docking_distance_) {
      response->success = false;
      response->message = "UAV is outside the docking capture distance";
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Docking rejected at %.3f m; limit is %.3f m",
        docking_distance, maximum_docking_distance_);
      return;
    }

    const auto model_to_link =
      uav_model_->WorldPose().Inverse() * current_link_pose;
    uav_model_->SetWorldPose(target_link_pose * model_to_link.Inverse());
    uav_model_->SetLinearVel(ignition::math::Vector3d::Zero);
    uav_model_->SetAngularVel(ignition::math::Vector3d::Zero);

    fixed_joint_ = world_->Physics()->CreateJoint("fixed", uav_model_);
    fixed_joint_->SetName("ugv_uav_docking_joint");
    fixed_joint_->Load(ugv_link, uav_link_, ignition::math::Pose3d());
    fixed_joint_->Attach(ugv_link, uav_link_);
    fixed_joint_->Init();

    response->success = true;
    response->message = "UAV docked to UGV";
    PublishStatus(true);
    RCLCPP_INFO(ros_node_->get_logger(), "%s", response->message.c_str());
  }

  void Detach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!fixed_joint_) {
      response->success = true;
      response->message = "UAV is already detached";
      PublishStatus(false);
      return;
    }

    fixed_joint_->Detach();
    fixed_joint_.reset();
    uav_model_->SetLinearVel(ignition::math::Vector3d::Zero);
    uav_model_->SetAngularVel(ignition::math::Vector3d::Zero);
    response->success = true;
    response->message = "UAV detached from UGV";
    PublishStatus(false);
    RCLCPP_INFO(ros_node_->get_logger(), "%s", response->message.c_str());
  }

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr uav_model_;
  gazebo::physics::LinkPtr uav_link_;
  gazebo::physics::JointPtr fixed_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docked_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr carrier_pub_;
  std::vector<std::pair<std::string, std::string>> allowed_carriers_;
  rclcpp::Service<uav_interfaces::srv::SelectCarrier>::SharedPtr select_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_service_;
  ignition::math::Pose3d docking_offset_;
  std::string ugv_model_name_;
  std::string ugv_link_name_;
  std::string uav_link_name_;
  double maximum_docking_distance_{0.8};
  bool strict_docking_{false};
  std::mutex mutex_;
};

GZ_REGISTER_MODEL_PLUGIN(UgvUavDockingPlugin)

}  // namespace cooperative_delivery
