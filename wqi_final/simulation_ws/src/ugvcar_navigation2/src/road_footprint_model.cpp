#include "ugvcar_navigation2/road_footprint_model.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace ugvcar_navigation2
{
std::vector<bool> roadFootprintMask(
  const std::vector<int8_t> & mask, const unsigned int width, const unsigned int height,
  const double resolution, const double radius)
{
  if (
    width == 0 || height == 0 || mask.size() != static_cast<std::size_t>(width) * height ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(radius) || radius <= 0.0)
  {
    throw std::invalid_argument("Invalid road footprint mask geometry");
  }
  // Include cells intersecting the circular footprint, not just their centres.
  const double cell_radius = radius / resolution + std::sqrt(0.5);
  const int extent = static_cast<int>(std::ceil(cell_radius));
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -extent; dy <= extent; ++dy) {
    for (int dx = -extent; dx <= extent; ++dx) {
      if (std::hypot(dx, dy) <= cell_radius) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  std::vector<bool> allowed(mask.size(), false);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const std::size_t cell = static_cast<std::size_t>(y) * width + x;
      if (mask[cell] < 0 || mask[cell] >= 100) {
        continue;
      }
      bool fits = true;
      for (const auto & offset : offsets) {
        const int nx = static_cast<int>(x) + offset.first;
        const int ny = static_cast<int>(y) + offset.second;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
          fits = false;
          break;
        }
        const auto cost = mask[static_cast<std::size_t>(ny) * width + nx];
        if (cost < 0 || cost >= 100) {
          fits = false;
          break;
        }
      }
      allowed[cell] = fits;
    }
  }
  return allowed;
}
}  // namespace ugvcar_navigation2
