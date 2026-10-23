#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "ugvcar_navigation2/road_footprint_model.hpp"

TEST(RoadFootprintModel, ClearCentreIsNotEnoughForTheWholeVehicle)
{
  std::vector<int8_t> road(144, 100);
  for (int y = 0; y < 12; ++y) {
    for (int x = 3; x <= 8; ++x) {
      road[y * 12 + x] = 0;
    }
  }
  const auto allowed = ugvcar_navigation2::roadFootprintMask(road, 12, 12, 1.0, 1.0);
  EXPECT_FALSE(allowed[6 * 12 + 3]);
  EXPECT_TRUE(allowed[6 * 12 + 4]);
  EXPECT_TRUE(allowed[6 * 12 + 7]);
  EXPECT_FALSE(allowed[6 * 12 + 8]);
  EXPECT_FALSE(allowed[4]);
}

TEST(RoadFootprintModel, ExistingSoftShoulderRemainsUsableButUnknownDoesNot)
{
  std::vector<int8_t> road(144, 35);
  auto allowed = ugvcar_navigation2::roadFootprintMask(road, 12, 12, 1.0, 1.0);
  EXPECT_TRUE(allowed[6 * 12 + 6]);
  road[6 * 12 + 7] = -1;
  allowed = ugvcar_navigation2::roadFootprintMask(road, 12, 12, 1.0, 1.0);
  EXPECT_FALSE(allowed[6 * 12 + 6]);
}

TEST(RoadFootprintModel, RejectsInconsistentGeometry)
{
  EXPECT_THROW(
    ugvcar_navigation2::roadFootprintMask({}, 12, 12, 1.0, 1.0),
    std::invalid_argument);
}
