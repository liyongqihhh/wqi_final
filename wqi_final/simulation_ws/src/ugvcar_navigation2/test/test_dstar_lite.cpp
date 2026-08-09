#include <algorithm>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "ugvcar_navigation2/dstar_lite.hpp"

namespace
{
std::size_t cell(const std::size_t x, const std::size_t y, const std::size_t width)
{
  return y * width + x;
}
}  // namespace

TEST(DStarLiteGrid, ReusesSearchWhenAnEdgeCostChanges)
{
  constexpr std::size_t width = 20;
  constexpr std::size_t height = 10;
  std::vector<std::uint8_t> costs(width * height, 0);
  ugvcar_navigation2::DStarLiteGrid planner(
    width, height, 0.1, 253, false, 3.0, 10000);
  const auto start = cell(1, 4, width);
  const auto goal = cell(18, 4, width);

  const auto initial = planner.plan(start, goal, costs);
  ASSERT_FALSE(initial.cells.empty());
  EXPECT_FALSE(initial.reused_search);
  EXPECT_EQ(initial.cells.front(), start);
  EXPECT_EQ(initial.cells.back(), goal);

  const auto blocked = cell(10, 4, width);
  costs[blocked] = 254;
  const auto repaired = planner.plan(start, goal, costs);
  ASSERT_FALSE(repaired.cells.empty());
  EXPECT_TRUE(repaired.reused_search);
  EXPECT_EQ(repaired.changed_cells, 1u);
  EXPECT_GT(repaired.expansions, 0u);
  EXPECT_EQ(repaired.cells.front(), start);
  EXPECT_EQ(repaired.cells.back(), goal);
  EXPECT_EQ(
    std::find(repaired.cells.begin(), repaired.cells.end(), blocked),
    repaired.cells.end());
}

TEST(DStarLiteGrid, RepairsPathWhenObstacleClears)
{
  constexpr std::size_t width = 16;
  constexpr std::size_t height = 7;
  std::vector<std::uint8_t> costs(width * height, 0);
  const auto blocked = cell(8, 3, width);
  costs[blocked] = 254;
  ugvcar_navigation2::DStarLiteGrid planner(
    width, height, 0.1, 253, false, 2.0, 10000);
  const auto start = cell(1, 3, width);
  const auto goal = cell(14, 3, width);
  ASSERT_FALSE(planner.plan(start, goal, costs).cells.empty());

  costs[blocked] = 0;
  const auto repaired = planner.plan(start, goal, costs);
  ASSERT_FALSE(repaired.cells.empty());
  EXPECT_TRUE(repaired.reused_search);
  EXPECT_EQ(repaired.changed_cells, 1u);
  EXPECT_NE(
    std::find(repaired.cells.begin(), repaired.cells.end(), blocked),
    repaired.cells.end());
}

TEST(DStarLiteGrid, UpdatesKeyModifierWhenRobotMoves)
{
  constexpr std::size_t width = 12;
  constexpr std::size_t height = 5;
  const std::vector<std::uint8_t> costs(width * height, 0);
  ugvcar_navigation2::DStarLiteGrid planner(
    width, height, 0.1, 253, false, 2.0, 10000);
  const auto goal = cell(10, 2, width);
  ASSERT_FALSE(planner.plan(cell(1, 2, width), goal, costs).cells.empty());

  const auto moved = planner.plan(cell(4, 2, width), goal, costs);
  ASSERT_FALSE(moved.cells.empty());
  EXPECT_TRUE(moved.reused_search);
  EXPECT_EQ(moved.changed_cells, 0u);
  EXPECT_EQ(moved.cells.front(), cell(4, 2, width));
}

TEST(DStarLiteGrid, DoesNotCutDiagonallyAcrossBlockedCorners)
{
  constexpr std::size_t width = 3;
  constexpr std::size_t height = 3;
  std::vector<std::uint8_t> costs(width * height, 0);
  costs[cell(1, 0, width)] = 254;
  costs[cell(0, 1, width)] = 254;
  ugvcar_navigation2::DStarLiteGrid planner(
    width, height, 1.0, 253, false, 1.0, 1000);

  const auto result = planner.plan(cell(0, 0, width), cell(1, 1, width), costs);
  EXPECT_TRUE(result.cells.empty());
}

TEST(DStarLiteGrid, RejectsUnknownCellsWhenConfigured)
{
  constexpr std::size_t width = 5;
  constexpr std::size_t height = 1;
  std::vector<std::uint8_t> costs(width * height, 0);
  costs[cell(2, 0, width)] = 255;
  ugvcar_navigation2::DStarLiteGrid planner(
    width, height, 1.0, 253, false, 1.0, 1000);

  EXPECT_TRUE(
    planner.plan(cell(0, 0, width), cell(4, 0, width), costs).cells.empty());
}
