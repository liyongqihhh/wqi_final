#ifndef UGVCAR_NAVIGATION2__ROAD_FOOTPRINT_MODEL_HPP_
#define UGVCAR_NAVIGATION2__ROAD_FOOTPRINT_MODEL_HPP_

#include <cstdint>
#include <vector>

namespace ugvcar_navigation2
{
std::vector<bool> roadFootprintMask(
  const std::vector<int8_t> & mask, unsigned int width, unsigned int height,
  double resolution, double radius);
}  // namespace ugvcar_navigation2

#endif  // UGVCAR_NAVIGATION2__ROAD_FOOTPRINT_MODEL_HPP_
