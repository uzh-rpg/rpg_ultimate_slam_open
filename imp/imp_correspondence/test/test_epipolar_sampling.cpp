#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/cameras/camera.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/imgproc/draw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/correspondence/epipolar_sampling.hpp>

TEST(ImpEpipolarSamplingTests, testEpipolarSampling)
{
  using namespace ze;

  // Load two images:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img_ref = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  ImageCv8uC1::Ptr img_cur = cvBridgeLoad8uC1(joinPath(data_path, "img", "10.png"));

  // Load poses:
  std::map<int64_t, Transformation> T_W_C_vec =
      loadIndexedPosesFromCsv(joinPath(data_path, "traj_gt.csv"));
  Transformation T_cur_ref = T_W_C_vec[10].inverse() * T_W_C_vec[1];
  
  // Load camera:
  Camera::Ptr cam = cameraFromYaml(joinPath(data_path, "calib.yaml"));
  
  // Load depthmap for first image:
  ImageRaw32fC1 depth_ref(cam->size());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  loadDepthmapFromFile(joinPath(data_path, "depth", "1.depth"),
                       depth_ref.numel(),
                       reinterpret_cast<float*>(depth_ref.data()));
  
  // Compute pixels on epipolar line
  Keypoint px_ref(120, 60);
  Bearing f_ref = cam->backProject(px_ref);
  real_t inv_depth_estimate = 1.0 / depth_ref(px_ref.cast<int>());
  real_t inv_depth_min = inv_depth_estimate * 2.0;
  real_t inv_depth_max = inv_depth_estimate * 0.5;
  Bearing f_A = (T_cur_ref * (f_ref * inv_depth_min)).normalized();
  Bearing f_B = (T_cur_ref * (f_ref * inv_depth_max)).normalized();
  Bearing f_C = (T_cur_ref * (f_ref * inv_depth_estimate)).normalized();
  Keypoint px_cur = cam->project(f_C);
  PixelVector pixels = sampleGreatCircle(*cam, 1.0, 8, f_A, f_B, f_C, 10000);

  // Check that px_cur is in pixels.
  bool res = false;
  for(const Vector2i& px : pixels)
  {
    if((px.cast<real_t>() - px_cur).norm() < 1.0)
      res = true;
  }
  EXPECT_TRUE(res);

  // Visualize
  if(false)
  {
    drawFeature<Pixel8uC1>(*img_ref, px_ref, 3, 255);
    for(auto px : pixels)
    {
      drawFeature<Pixel8uC1>(*img_cur, px(0), px(1), 1, 255);
    }
    cvBridgeShow("img_ref", ImageCv8uC1(*img_ref));
    cvBridgeShow("img_cur", ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}

ZE_UNITTEST_ENTRYPOINT
