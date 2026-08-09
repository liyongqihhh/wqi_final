#include "ugvcar_navigation2/dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ugvcar_navigation2
{

namespace
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1.0e-9;
constexpr std::uint8_t kUnknownCost = 255;
constexpr double kNormalCostMaximum = 252.0;
}  // namespace

DStarLiteGrid::DStarLiteGrid(
  const std::size_t width,
  const std::size_t height,
  const double resolution,
  const std::uint8_t lethal_cost,
  const bool allow_unknown,
  const double cost_penalty,
  const std::size_t max_expansions)
: width_(width),
  height_(height),
  cell_count_(width * height),
  resolution_(resolution),
  lethal_cost_(lethal_cost),
  allow_unknown_(allow_unknown),
  cost_penalty_(cost_penalty),
  max_expansions_(max_expansions)
{
  if (
    width_ == 0 || height_ == 0 || resolution_ <= 0.0 ||
    lethal_cost_ == 0 || cost_penalty_ < 0.0 || max_expansions_ == 0)
  {
    throw std::invalid_argument("D* Lite grid parameters are invalid");
  }
}

bool DStarLiteGrid::QueueCompare::operator()(
  const QueueEntry & lhs, const QueueEntry & rhs) const
{
  if (DStarLiteGrid::keyLess(rhs.key, lhs.key)) {
    return true;
  }
  if (DStarLiteGrid::keyLess(lhs.key, rhs.key)) {
    return false;
  }
  return lhs.cell > rhs.cell;
}

bool DStarLiteGrid::keyLess(const Key & lhs, const Key & rhs)
{
  if (lhs.first < rhs.first - kEpsilon) {
    return true;
  }
  if (lhs.first > rhs.first + kEpsilon) {
    return false;
  }
  return lhs.second < rhs.second - kEpsilon;
}

bool DStarLiteGrid::valuesEqual(const double lhs, const double rhs)
{
  if (std::isinf(lhs) && std::isinf(rhs)) {
    return true;
  }
  return std::abs(lhs - rhs) <= kEpsilon;
}

bool DStarLiteGrid::traversable(const std::size_t cell) const
{
  if (cell >= costs_.size()) {
    return false;
  }
  const auto cost = costs_[cell];
  if (cost == kUnknownCost) {
    return allow_unknown_;
  }
  return cost < lethal_cost_;
}

void DStarLiteGrid::initialize(
  const std::size_t start, const std::size_t goal)
{
  start_ = start;
  last_start_ = start;
  goal_ = goal;
  key_modifier_ = 0.0;
  g_.assign(cell_count_, kInfinity);
  rhs_.assign(cell_count_, kInfinity);
  versions_.assign(cell_count_, 0);
  open_ = decltype(open_)();
  rhs_[goal_] = 0.0;
  pushOpen(goal_, calculateKey(goal_));
  initialized_ = true;
}

DStarLiteResult DStarLiteGrid::plan(
  const std::size_t start,
  const std::size_t goal,
  const std::vector<std::uint8_t> & costs)
{
  if (start >= cell_count_ || goal >= cell_count_) {
    throw std::out_of_range("D* Lite start or goal is outside the grid");
  }
  if (costs.size() != cell_count_) {
    throw std::invalid_argument("D* Lite cost array has the wrong size");
  }

  DStarLiteResult result;
  const bool can_reuse = initialized_ && goal == goal_;
  result.reused_search = can_reuse;
  if (!can_reuse) {
    costs_ = costs;
    initialize(start, goal);
  } else {
    if (start != start_) {
      key_modifier_ += heuristic(last_start_, start);
      start_ = start;
      last_start_ = start;
    }
    result.changed_cells = applyCostChanges(costs);
  }

  if (!traversable(start_) || !traversable(goal_)) {
    return result;
  }
  if (!computeShortestPath(result.expansions)) {
    return result;
  }
  result.cells = extractPath();
  return result;
}

std::size_t DStarLiteGrid::applyCostChanges(
  const std::vector<std::uint8_t> & costs)
{
  std::vector<std::size_t> changed;
  changed.reserve(128);
  for (std::size_t cell = 0; cell < cell_count_; ++cell) {
    if (costs_[cell] != costs[cell]) {
      changed.push_back(cell);
    }
  }
  if (changed.empty()) {
    return 0;
  }

  costs_ = costs;
  std::vector<std::uint8_t> touched(cell_count_, 0);
  std::array<std::size_t, 8> adjacent{};
  for (const auto cell : changed) {
    touched[cell] = 1;
    const auto count = neighbors(cell, adjacent);
    for (std::size_t index = 0; index < count; ++index) {
      touched[adjacent[index]] = 1;
    }
  }
  for (std::size_t cell = 0; cell < cell_count_; ++cell) {
    if (touched[cell] != 0) {
      updateVertex(cell);
    }
  }
  return changed.size();
}

void DStarLiteGrid::updateVertex(const std::size_t cell)
{
  if (cell != goal_) {
    double best = kInfinity;
    std::array<std::size_t, 8> adjacent{};
    const auto count = neighbors(cell, adjacent);
    for (std::size_t index = 0; index < count; ++index) {
      const auto next = adjacent[index];
      const double step = edgeCost(cell, next);
      if (std::isfinite(step) && std::isfinite(g_[next])) {
        best = std::min(best, step + g_[next]);
      }
    }
    rhs_[cell] = best;
  }

  ++versions_[cell];
  if (!valuesEqual(g_[cell], rhs_[cell])) {
    open_.push(QueueEntry{calculateKey(cell), cell, versions_[cell]});
  }
}

bool DStarLiteGrid::computeShortestPath(std::size_t & expansions)
{
  expansions = 0;
  while (
    keyLess(topKey(), calculateKey(start_)) ||
    !valuesEqual(rhs_[start_], g_[start_]))
  {
    if (expansions >= max_expansions_) {
      return false;
    }
    discardStaleEntries();
    if (open_.empty()) {
      return false;
    }
    const QueueEntry entry = popTop();
    const Key current_key = calculateKey(entry.cell);
    if (keyLess(entry.key, current_key)) {
      pushOpen(entry.cell, current_key);
    } else if (g_[entry.cell] > rhs_[entry.cell]) {
      g_[entry.cell] = rhs_[entry.cell];
      std::array<std::size_t, 8> adjacent{};
      const auto count = neighbors(entry.cell, adjacent);
      for (std::size_t index = 0; index < count; ++index) {
        updateVertex(adjacent[index]);
      }
    } else {
      g_[entry.cell] = kInfinity;
      updateVertex(entry.cell);
      std::array<std::size_t, 8> adjacent{};
      const auto count = neighbors(entry.cell, adjacent);
      for (std::size_t index = 0; index < count; ++index) {
        updateVertex(adjacent[index]);
      }
    }
    ++expansions;
  }
  return std::isfinite(g_[start_]);
}

std::vector<std::size_t> DStarLiteGrid::extractPath() const
{
  if (!std::isfinite(g_[start_])) {
    return {};
  }
  std::vector<std::size_t> path;
  path.reserve(width_ + height_);
  std::vector<std::uint8_t> visited(cell_count_, 0);
  std::size_t current = start_;
  path.push_back(current);
  visited[current] = 1;

  while (current != goal_ && path.size() <= cell_count_) {
    std::array<std::size_t, 8> adjacent{};
    const auto count = neighbors(current, adjacent);
    std::size_t best_cell = cell_count_;
    double best_value = kInfinity;
    double best_goal_distance = kInfinity;
    for (std::size_t index = 0; index < count; ++index) {
      const auto next = adjacent[index];
      const double step = edgeCost(current, next);
      if (!std::isfinite(step) || !std::isfinite(g_[next])) {
        continue;
      }
      const double value = step + g_[next];
      const double goal_distance = heuristic(next, goal_);
      if (
        value < best_value - kEpsilon ||
        (valuesEqual(value, best_value) &&
        goal_distance < best_goal_distance))
      {
        best_cell = next;
        best_value = value;
        best_goal_distance = goal_distance;
      }
    }
    if (best_cell >= cell_count_ || visited[best_cell] != 0) {
      return {};
    }
    current = best_cell;
    path.push_back(current);
    visited[current] = 1;
  }
  return current == goal_ ? path : std::vector<std::size_t>{};
}

DStarLiteGrid::Key DStarLiteGrid::calculateKey(
  const std::size_t cell) const
{
  const double best = std::min(g_[cell], rhs_[cell]);
  return Key{best + heuristic(start_, cell) + key_modifier_, best};
}

DStarLiteGrid::Key DStarLiteGrid::topKey()
{
  discardStaleEntries();
  return open_.empty() ? Key{} : open_.top().key;
}

DStarLiteGrid::QueueEntry DStarLiteGrid::popTop()
{
  discardStaleEntries();
  const QueueEntry result = open_.top();
  open_.pop();
  return result;
}

void DStarLiteGrid::pushOpen(const std::size_t cell, const Key & key)
{
  ++versions_[cell];
  open_.push(QueueEntry{key, cell, versions_[cell]});
}

void DStarLiteGrid::discardStaleEntries()
{
  while (!open_.empty()) {
    const auto & entry = open_.top();
    if (entry.version == versions_[entry.cell]) {
      return;
    }
    open_.pop();
  }
}

std::size_t DStarLiteGrid::neighbors(
  const std::size_t cell,
  std::array<std::size_t, 8> & output) const
{
  const int x = static_cast<int>(cell % width_);
  const int y = static_cast<int>(cell / width_);
  std::size_t count = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int next_x = x + dx;
      const int next_y = y + dy;
      if (
        next_x < 0 || next_y < 0 ||
        next_x >= static_cast<int>(width_) ||
        next_y >= static_cast<int>(height_))
      {
        continue;
      }
      output[count++] =
        static_cast<std::size_t>(next_y) * width_ +
        static_cast<std::size_t>(next_x);
    }
  }
  return count;
}

double DStarLiteGrid::edgeCost(
  const std::size_t from, const std::size_t to) const
{
  if (!traversable(from) || !traversable(to)) {
    return kInfinity;
  }
  if (!isDiagonalClear(from, to)) {
    return kInfinity;
  }
  const int from_x = static_cast<int>(from % width_);
  const int from_y = static_cast<int>(from / width_);
  const int to_x = static_cast<int>(to % width_);
  const int to_y = static_cast<int>(to / width_);
  const bool diagonal = from_x != to_x && from_y != to_y;
  const double distance = resolution_ * (diagonal ? std::sqrt(2.0) : 1.0);
  const auto normalized_cost = [](const std::uint8_t value) {
      return std::min<double>(value, kNormalCostMaximum) /
             kNormalCostMaximum;
    };
  const double occupancy = std::max(
    normalized_cost(costs_[from]), normalized_cost(costs_[to]));
  return distance * (1.0 + cost_penalty_ * occupancy);
}

double DStarLiteGrid::heuristic(
  const std::size_t first, const std::size_t second) const
{
  const double first_x = static_cast<double>(first % width_);
  const double first_y = static_cast<double>(first / width_);
  const double second_x = static_cast<double>(second % width_);
  const double second_y = static_cast<double>(second / width_);
  return resolution_ * std::hypot(first_x - second_x, first_y - second_y);
}

bool DStarLiteGrid::isDiagonalClear(
  const std::size_t from, const std::size_t to) const
{
  const std::size_t from_x = from % width_;
  const std::size_t from_y = from / width_;
  const std::size_t to_x = to % width_;
  const std::size_t to_y = to / width_;
  if (from_x == to_x || from_y == to_y) {
    return true;
  }
  const std::size_t side_a = from_y * width_ + to_x;
  const std::size_t side_b = to_y * width_ + from_x;
  return traversable(side_a) && traversable(side_b);
}

}  // namespace ugvcar_navigation2
