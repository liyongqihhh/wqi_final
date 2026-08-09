#ifndef UGVCAR_NAVIGATION2__DSTAR_LITE_HPP_
#define UGVCAR_NAVIGATION2__DSTAR_LITE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace ugvcar_navigation2
{

struct DStarLiteResult
{
  std::vector<std::size_t> cells;
  std::size_t expansions{0};
  std::size_t changed_cells{0};
  bool reused_search{false};
};

class DStarLiteGrid
{
public:
  DStarLiteGrid(
    std::size_t width,
    std::size_t height,
    double resolution,
    std::uint8_t lethal_cost,
    bool allow_unknown,
    double cost_penalty,
    std::size_t max_expansions);

  DStarLiteResult plan(
    std::size_t start,
    std::size_t goal,
    const std::vector<std::uint8_t> & costs);

  bool traversable(std::size_t cell) const;
  std::size_t width() const {return width_;}
  std::size_t height() const {return height_;}

private:
  struct Key
  {
    double first{std::numeric_limits<double>::infinity()};
    double second{std::numeric_limits<double>::infinity()};
  };

  struct QueueEntry
  {
    Key key;
    std::size_t cell{0};
    std::uint64_t version{0};
  };

  struct QueueCompare
  {
    bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const;
  };

  static bool keyLess(const Key & lhs, const Key & rhs);
  static bool valuesEqual(double lhs, double rhs);

  void initialize(std::size_t start, std::size_t goal);
  std::size_t applyCostChanges(const std::vector<std::uint8_t> & costs);
  void updateVertex(std::size_t cell);
  bool computeShortestPath(std::size_t & expansions);
  std::vector<std::size_t> extractPath() const;
  Key calculateKey(std::size_t cell) const;
  Key topKey();
  QueueEntry popTop();
  void pushOpen(std::size_t cell, const Key & key);
  void discardStaleEntries();
  std::size_t neighbors(
    std::size_t cell,
    std::array<std::size_t, 8> & output) const;
  double edgeCost(std::size_t from, std::size_t to) const;
  double heuristic(std::size_t first, std::size_t second) const;
  bool isDiagonalClear(std::size_t from, std::size_t to) const;

  std::size_t width_;
  std::size_t height_;
  std::size_t cell_count_;
  double resolution_;
  std::uint8_t lethal_cost_;
  bool allow_unknown_;
  double cost_penalty_;
  std::size_t max_expansions_;
  bool initialized_{false};
  std::size_t start_{0};
  std::size_t last_start_{0};
  std::size_t goal_{0};
  double key_modifier_{0.0};
  std::vector<std::uint8_t> costs_;
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<std::uint64_t> versions_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open_;
};

}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__DSTAR_LITE_HPP_
