#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <string>

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
    auto initial_docking_response =
      std::make_shared<std_srvs::srv::Trigger::Response>();
    Attach(nullptr, initial_docking_response);
    if (!initial_docking_response->success) {
      RCLCPP_WARN(
        ros_node_->get_logger(), "Initial UAV docking deferred: %s",
        initial_docking_response->message.c_str());
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_service_;
  ignition::math::Pose3d docking_offset_;
  std::string ugv_model_name_;
  std::string ugv_link_name_;
  std::string uav_link_name_;
  double maximum_docking_distance_{0.8};
  std::mutex mutex_;
};

GZ_REGISTER_MODEL_PLUGIN(UgvUavDockingPlugin)

}  // namespace cooperative_delivery
