#include <ze/cameras/camera_utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/types.hpp>
#include <ze/common/benchmark.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/features/occupancy_grid_2d.hpp>
#include <imp/imgproc/draw.hpp>

TEST(OccupancyGrid2DTests, test)
{
  using namespace ze;

  Size2u image_size(752, 480);
  OccupancyGrid2D grid(32, image_size);
  Keypoints keypoints = generateRandomKeypoints(image_size, 7u, 120u);
  ImageRaw8uC1 img(image_size);
  grid.fillWithKeypoints(keypoints);
  grid.visualizeGrid(img);

  drawFeatures<Pixel8uC1>(img, keypoints, 2, Pixel8uC1(255));

  // Visualize
  if (false)
  {
    cvBridgeShow("img", ImageCv8uC1(img));
    cv::waitKey(0);
  }

}

ZE_UNITTEST_ENTRYPOINT
