#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include "gtest/gtest.h"
#include "cooperative_delivery/peer_vehicle_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

class TestLayer : public cooperative_delivery::PeerVehicleLayer
{
public:
  using PeerVehicleLayer::receive;
};

class FleetLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("fleet_layer_test", "", rclcpp::NodeOptions());
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    grid = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    grid->resizeMap(100, 100, .2, -10, -10);
    layer = std::make_shared<TestLayer>();
    grid->addPlugin(layer);
    layer->initialize(grid.get(), "peer", tf.get(), node, nullptr);
  }
  unsigned char cost(double x, double y)
  {
    unsigned int i, j;
    EXPECT_TRUE(grid->getCostmap()->worldToMap(x, y, i, j));
    return grid->getCostmap()->getCost(i, j);
  }
  void pose(double x, double y)
  {
    nav_msgs::msg::Odometry message;
    message.header.frame_id = "world";
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    layer->receive(message);
    grid->updateMap(0, 0, 0);
  }
  nav2_util::LifecycleNode::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> grid;
  std::shared_ptr<TestLayer> layer;
};

TEST_F(FleetLayerTest, MovingPeerClearsOldPositionWithoutGhostTrail)
{
  pose(0, 0);
  EXPECT_EQ(cost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0, .5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(2, 0), nav2_costmap_2d::FREE_SPACE);
  pose(3, 0);
  EXPECT_EQ(cost(0, 0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(3, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(1.5, 0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(FleetLayerTest, ClearRequestDoesNotEraseParkedPeer)
{
  pose(1, 2);
  layer->reset();
  grid->updateMap(0, 0, 0);
  EXPECT_EQ(cost(1, 2), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(layer->isCurrent());
  EXPECT_FALSE(layer->isClearable());
}

TEST_F(FleetLayerTest, MissingAndStalePoseFailClosed)
{
  grid->updateMap(0, 0, 0);
  EXPECT_FALSE(layer->isCurrent());
  pose(0, 0);
  EXPECT_TRUE(layer->isCurrent());
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  grid->updateMap(0, 0, 0);
  EXPECT_FALSE(layer->isCurrent());
  EXPECT_EQ(cost(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(FleetLayerTest, BoundedDiskMatchesFullGridReferenceAtEdgesAndOutsideMap)
{
  for (const auto & point : std::vector<std::pair<double, double>>{
      {0, 0}, {.15, -.35}, {-9.8, -9.8}, {9.9, 9.9}, {10.2, 0}, {30, 30}})
  {
    pose(point.first, point.second);
    nav2_costmap_2d::Costmap2D actual(100, 100, .2, -10, -10, 0);
    layer->updateCosts(actual, 0, 0, 100, 100);
    for (unsigned int j = 0; j < 100; ++j) {
      for (unsigned int i = 0; i < 100; ++i) {
        double x, y;
        actual.mapToWorld(i, j, x, y);
        const bool inside = std::hypot(x - point.first, y - point.second) <=
          .60 + .5 * std::sqrt(2.) * actual.getResolution();
        EXPECT_EQ(actual.getCost(i, j), inside ? 254 : 0);
      }
    }
  }
}

TEST_F(FleetLayerTest, BoundedDiskRespectsPartialDirtyRegion)
{
  pose(0, 0);
  nav2_costmap_2d::Costmap2D actual(100, 100, .2, -10, -10, 0);
  layer->updateCosts(actual, 50, 50, 51, 51);
  for (unsigned int j = 0; j < 100; ++j) {
    for (unsigned int i = 0; i < 100; ++i) {
      EXPECT_EQ(actual.getCost(i, j), i == 50 && j == 50 ? 254 : 0);
    }
  }
}
