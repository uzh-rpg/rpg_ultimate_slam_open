#include <imp/correspondence/epipolar_sampling.hpp>

#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_utils.hpp>

namespace ze {

//! @todo(cfo): PixelVector -> Eigen
PixelVector sampleGreatCircle(
    const Camera& cam,
    const real_t scale, // 1 << level
    const int border_margin,
    const Vector3& f_A,
    const Vector3& f_B,
    const Vector3& f_C,
    const int max_pixels_to_check)
{
  // calculate the step in angle
  constexpr real_t pixel_step = 0.7;
  const real_t angle_range = std::acos(f_A.dot(f_B));
  const real_t angle_step = scale * cam.getApproxAnglePerPixel() * pixel_step;
  const int img_width = cam.width() / scale;
  const int img_height = cam.height() / scale;
  int num_steps = angle_range / angle_step;
  PixelVector px_on_epipolar_line;
  px_on_epipolar_line.reserve(num_steps);
  if (num_steps <= 1)
  {
    Vector2i px = (cam.project(f_C) / scale + Vector2(0.5, 0.5)).cast<int>();
    if (isVisibleWithMargin(img_width, img_height, px, border_margin))
    {
      px_on_epipolar_line.push_back(px);
    }
    return PixelVector(px_on_epipolar_line);
  }
  num_steps = std::min(num_steps, max_pixels_to_check);
  int  half_num_steps = num_steps / 2;
  AngleAxis rotation_B_to_A(1.0,  (f_B.cross(f_A)).normalized());

  // search around center
  Vector2i last_checked_px(0, 0);
  for (int i = 0; i < num_steps; ++i)
  {
    // We rotate from the center to both ends until point is not visible anymore.
    real_t angle = (i < half_num_steps)
        ? i * angle_step                     // f_A <-- f_C
        : (i - half_num_steps) * (- angle_step); // f_C --> f_B
    rotation_B_to_A.setAngle(angle);

    // project in image, scale to get right pyramid level and round to int.
    Vector2i px = (cam.project(rotation_B_to_A.rotate(f_C)) / scale
                   + Vector2(0.5, 0.5)).cast<int>();
    if (px == last_checked_px)
    {
      // Don't add a pixel twice to the list.
      continue;
    }
    last_checked_px = px;

    if (!isVisibleWithMargin(img_width, img_height, px, border_margin))
    {
      if (i < half_num_steps)
      {
        // Switch search direction: f_A <-- f_C to f_C --> f_B
        i = half_num_steps;
        continue;
      }
      // end of search
      break;
    }

    px_on_epipolar_line.push_back(px);
  }

  return px_on_epipolar_line;
}

} // namespace ze
