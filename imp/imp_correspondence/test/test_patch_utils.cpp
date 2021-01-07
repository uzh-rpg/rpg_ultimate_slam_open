#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>
#include <ze/common/timer.hpp>
#include <ze/cameras/camera.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/imgproc/draw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/correspondence/patch_utils.hpp>
#include <imp/correspondence/klt.hpp>

namespace ze {

int64 imageSum16sC1(const ImageRaw16sC1& img)
{
  int64_t sum = 0;
  for (uint32_t y = 0u; y < img.height(); ++y)
  {
    for (uint32_t x = 0u; x < img.width(); ++x)
    {
      sum += img(x,y);
    }
  }
  return sum;
}

} // namespace ze

TEST(ImpPatchUtilsTests, testExtractPatchInterpolated)
{
  using namespace ze;
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));

  constexpr int halfpatch_size = 2;
  constexpr int patch_size = 2 * halfpatch_size;
  ImageRaw8uC1 patch(patch_size * patch_size, 1);
  uint8_t* patch_data = reinterpret_cast<uint8_t*>(patch.data(0,0));
  const uint8_t* img_data = reinterpret_cast<const uint8_t*>(img->data());
  const int img_stride = img->stride();
  Keypoint px_ref(50.445155, 50.31251);

  auto extractPatchLambda = [&]() {
    extractPatchInterpolated8uC1(
          img_data, img_stride, px_ref, halfpatch_size, patch_data);
  };
  runTimingBenchmark(extractPatchLambda, 1000, 100, "Extract patch", true);

  // Check if patch is correctly extracted.
  int patch_sum = 0u;
  for (int i = 0; i < patch_size * patch_size; ++i)
  {
    patch_sum += patch_data[i];
  }
  EXPECT_EQ(patch_sum, 1279);
}

TEST(ImpPatchUtilsTests, testAffineWarp)
{
  using namespace ze;

  // Load two images:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img_ref = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  ImageCv8uC1::Ptr img_cur = cvBridgeLoad8uC1(joinPath(data_path, "img", "10.png"));

  // Load poses:
  std::map<int64_t, Transformation> T_W_C_vec =
      loadIndexedPosesFromCsv(data_path + "/traj_gt.csv");
  Transformation T_cur_ref = T_W_C_vec[10].inverse() * T_W_C_vec[1];
  
  // Load camera:
  Camera::Ptr cam = cameraFromYaml(data_path + "/calib.yaml");
  
  // Load depthmap for first image:
  ImageRaw32fC1 depth_ref(cam->width(), cam->height());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  loadDepthmapFromFile(data_path + "/depth/1.depth",
                       depth_ref.numel(),
                       reinterpret_cast<float*>(depth_ref.data()));
  
  // Check pixel correspondence
  Keypoint px_ref(50, 50);
  Bearing f_ref = cam->backProject(px_ref);
  f_ref.normalize();
  real_t depth = depth_ref(px_ref.cast<int>());
  Keypoint px_cur = cam->project(T_cur_ref * (f_ref * depth));

  // Verified correspondence between frame 1 and 10, and ref_px = (50,50).
  EXPECT_NEAR(px_cur(0), 211.329, 1e-3);
  EXPECT_NEAR(px_cur(1), 102.541, 1e-3);

  constexpr int halfpatch_size = 4;
  constexpr int patch_size = 2 * halfpatch_size;
  ImageRaw8uC1 patch(patch_size * patch_size, 3);
  patch.setValue(0);

  // Patch 1: Reference patch:
  extractPatchInterpolated8uC1(
        reinterpret_cast<const uint8_t*>(img_cur->data()), img_cur->stride(),
        px_ref, halfpatch_size,
        reinterpret_cast<uint8_t*>(patch.data(0,0)));

  // Patch 2: Warped reference patch:
  Matrix2 A_cur_ref =
      getAffineWarpMatrix(*cam, *cam, px_ref, f_ref, 1.0 / depth, T_cur_ref, 1.0);

  const uint8_t* img_ref_data = reinterpret_cast<const uint8_t*>(img_ref->data());
  const int img_width = img_ref->width();
  const int img_height = img_ref->height();
  const int img_stride = img_ref->stride();
  uint8_t* patch_data = reinterpret_cast<uint8_t*>(patch.data(0,1));
  int res = false;
  auto warpAffineLambda = [&]() {
    res = warpAffine8uC1(
          A_cur_ref, img_ref_data, img_width, img_height, img_stride,
          px_ref, 1.0, 1.0, halfpatch_size, patch_data);
    EXPECT_TRUE(res);
  };
  runTimingBenchmark(warpAffineLambda, 1000, 100, "Warp affine", true);

  // Check if patch is correctly warped.
  int patch_sum = 0u;
  for (int i = 0; i < patch_size * patch_size; ++i)
  {
    patch_sum += patch_data[i];
  }
  EXPECT_EQ(patch_sum, 5844); // Compiled with 64-bit.

  // Patch 3: Current patch that should look like warped patch:
  extractPatchInterpolated8uC1(
        reinterpret_cast<const uint8_t*>(img_cur->data()), img_cur->stride(),
        px_cur, halfpatch_size,
        reinterpret_cast<uint8_t*>(patch.data(0,2)));

  //! @todo Test a patch that actually rotates!

  // Display warped and actual patch:
  if(false)
  {
    ImageRaw8uC1 patches(patch.data(0,0), patch_size, 3 * patch_size, patch_size, true);
    cvBridgeShow("patches", ImageCv8uC1(patches));

    drawFeature<Pixel8uC1>(*img_ref, px_ref, 5, 255);
    drawFeature<Pixel8uC1>(*img_cur, px_cur, 5, 255);

    cvBridgeShow("ref_im", ImageCv8uC1(*img_ref));
    cvBridgeShow("cur_im", ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}

TEST(ImpPatchUtilsTests, testCreatePatch)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));

  constexpr int halfpatch_size = 4;
  constexpr int patch_size = 2 * halfpatch_size;
  constexpr int halfpatch_with_border_size = halfpatch_size + 1;
  constexpr int patch_with_border_size = 2 * halfpatch_with_border_size;

  // Data of each patch lies on a row.
  ImageRaw8uC1 patch_data(patch_with_border_size * patch_with_border_size, 3);
  patch_data.setValue(255);

  // Extract data
  Keypoint px_true = Keypoint(290.6, 170.2);
  extractPatch8uC1(
        *img, Vector2i(200, 200), halfpatch_with_border_size, patch_data.data(0, 0));
  extractPatchInterpolated8uC1(
        reinterpret_cast<const uint8_t*>(img->data()), img->stride(), px_true,
        halfpatch_with_border_size, reinterpret_cast<uint8_t*>(patch_data.data(0, 1)));

  // Create patch without border
  removeBorderFromPatch8uC1(patch_size,
                        reinterpret_cast<uint8_t*>(patch_data.data(0, 1)),
                        reinterpret_cast<uint8_t*>(patch_data.data(0, 2)));
}

TEST(ImpPatchUtilsTests, testPatchDerivative)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));

  constexpr int halfpatch_size = 2;
  constexpr int patch_size = 2 * halfpatch_size;
  constexpr int halfpatch_with_border_size = halfpatch_size + 1;
  constexpr int patch_with_border_size = 2 * halfpatch_with_border_size;

  // Data of each patch lies on a row.
  ImageRaw8uC1 patch_data(patch_with_border_size * patch_with_border_size, 1);
  ImageRaw16sC1 patch_deriv(patch_size * patch_size, 2);

  // Extract data
  Keypoint px_true = Keypoint(290.6, 170.2);

  auto extractPatchLambda = [&]() {
    extractPatchInterpolated8uC1(
          reinterpret_cast<const uint8_t*>(img->data()), img->stride(), px_true,
          halfpatch_with_border_size,
          reinterpret_cast<uint8_t*>(patch_data.data(0, 0)));
  };
  runTimingBenchmark(extractPatchLambda, 1000, 10, "Extract Patch Interpolated", true);

  auto computePatchDerivLambda = [&]() {
  computePatchDerivative8uC1(
        1,
        reinterpret_cast<uint8_t*>(patch_data.data(0, 0)),
        reinterpret_cast<int16_t*>(patch_deriv.data(0, 0)),
        reinterpret_cast<int16_t*>(patch_deriv.data(0, 1)));
  };
  runTimingBenchmark(computePatchDerivLambda, 1000, 10, "Compute Patch Derivative", true);

  CHECK_EQ(imageSum16sC1(patch_deriv), -47);
}

ZE_UNITTEST_ENTRYPOINT
