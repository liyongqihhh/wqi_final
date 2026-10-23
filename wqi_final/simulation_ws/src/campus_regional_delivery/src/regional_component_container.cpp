#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <dlfcn.h>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

using namespace std::chrono_literals;

class PluginLibraries
{
public:
  PluginLibraries()
  {
    add<nav2_core::Behavior>("nav2_core::Behavior");
    add<nav2_core::GlobalPlanner>("nav2_core::GlobalPlanner");
    add<nav2_core::Controller>("nav2_core::Controller");
    add<nav2_core::Smoother>("nav2_core::Smoother");
    add<nav2_core::GoalChecker>("nav2_core::GoalChecker");
    add<nav2_core::ProgressChecker>("nav2_core::ProgressChecker");
    add<nav2_costmap_2d::Layer>("nav2_costmap_2d::Layer", "nav2_costmap_2d");
  }
  void pin_loaded()
  {
    for (const auto & path : paths_) {
      // NOLOAD never instantiates or preloads a plugin. NODELETE preserves code
      // for residual weak control blocks; dlclose balances our handle count.
      if (void * handle = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD | RTLD_NODELETE)) {
        dlclose(handle);
      }
    }
  }
private:
  template<typename Plugin>
  void add(const std::string & base, const std::string & package = "nav2_core")
  {
    pluginlib::ClassLoader<Plugin> metadata(package, base);
    for (const auto & name : metadata.getDeclaredClasses()) {
      paths_.insert(metadata.getClassLibraryPath(name));
    }
  }
  std::set<std::string> paths_;
};

class RegionalContext : public rclcpp::Context
{
public:
  rclcpp::PreShutdownCallbackHandle add_pre_shutdown_callback(PreShutdownCallback callback) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto handle = rclcpp::Context::add_pre_shutdown_callback(std::move(callback));
    if (finalized_) {remove_pre_shutdown_callback(handle);} else {handles_.push_back(handle);}
    return handle;
  }
  void release_finalized_node_callbacks()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    finalized_ = true;
    for (const auto & handle : handles_) {remove_pre_shutdown_callback(handle);}
    handles_.clear();
  }
private:
  std::mutex mutex_;
  bool finalized_{false};
  std::vector<rclcpp::PreShutdownCallbackHandle> handles_;
};

class RegionalContainer : public rclcpp_components::ComponentManager
{
public:
  RegionalContainer(std::weak_ptr<rclcpp::Executor> executor, std::shared_ptr<RegionalContext> context,
    PluginLibraries & libraries)
  : ComponentManager(executor, "ComponentManager", rclcpp::NodeOptions().context(context)),
    context_(std::move(context)), libraries_(libraries)
  {
    names_ = declare_parameter<std::vector<std::string>>("managed_node_names", {
      "controller_server", "smoother_server", "planner_server", "behavior_server",
      "bt_navigator", "waypoint_follower", "velocity_smoother", "collision_monitor"});
    if (names_.size() != 8) {throw std::runtime_error("Expected all eight navigation lifecycle nodes");}
    clients_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    prepare_ = create_service<std_srvs::srv::Trigger>("~/_container/prepare_shutdown",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          for (const auto & name : names_) {
            auto client = create_client<lifecycle_msgs::srv::GetState>(
              name + "/get_state", rmw_qos_profile_services_default, clients_group_);
            if (!client->wait_for_service(2s)) {throw std::runtime_error("Missing lifecycle state: " + name);}
            auto future = client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
            if (future.wait_for(2s) != std::future_status::ready ||
              future.get()->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
            {throw std::runtime_error("Component is not finalized: " + name);}
          }
          // Humble Nav2 keeps callbacks bound to raw node pointers. Release
          // them only after finalization, before component objects disappear.
          // Executor on-shutdown callbacks remain registered to wake spin().
          // Keep helper-node contexts alive until their owning components
          // have destroyed their clock/TF executors.
          context_->release_finalized_node_callbacks();
          response->success = true;
          response->message = "Finalized components may be safely unloaded";
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      });
  }
protected:
  rclcpp::NodeOptions create_node_options(const std::shared_ptr<LoadNode::Request> request) override
  {
    auto options = ComponentManager::create_node_options(request);
    options.context(context_);
    return options;
  }
  void on_unload_node(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override
  {
    libraries_.pin_loaded();
    ComponentManager::on_unload_node(header, request, response);
  }
private:
  std::shared_ptr<RegionalContext> context_;
  PluginLibraries & libraries_;
  std::vector<std::string> names_;
  rclcpp::CallbackGroup::SharedPtr clients_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto context = std::make_shared<RegionalContext>();
  context->init(argc, argv);
  rclcpp::ExecutorOptions options;
  options.context = context;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options, 4);
  // Plugins create lifecycle-managed publishers whose weak control blocks
  // outlive the plugin objects. Keep their code mapped until every
  // lifecycle node has been destroyed, including during emergency teardown.
  PluginLibraries libraries;
  auto manager = std::make_shared<RegionalContainer>(executor, context, libraries);
  executor->add_node(manager);
  executor->spin();
  executor->remove_node(manager);
  libraries.pin_loaded();
  manager.reset();
  executor.reset();
  rclcpp::shutdown(context);
  rclcpp::shutdown();
  return 0;
}
