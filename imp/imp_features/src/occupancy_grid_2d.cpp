#include <imp/features/occupancy_grid_2d.hpp>

namespace ze {

void OccupancyGrid2D::visualizeGrid(Image8uC1& img, real_t inv_scale)
{
  // Loop over all pixels:
  for (uint32_t y = 0u; y < img.height(); ++y)
  {
    for (uint32_t x = 0u; x < img.width(); ++x)
    {
      if (isOccupied(x, y, inv_scale))
      {
        img(x, y) = 50;
      }
      else
      {
        img(x, y) = 200;
      }
    }
  }
}

} // namespace ze
