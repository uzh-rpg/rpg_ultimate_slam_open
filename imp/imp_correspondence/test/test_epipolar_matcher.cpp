#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/cameras/camera.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/imgproc/draw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/correspondence/epipolar_matcher.hpp>

TEST(ImpEpipolarMatchingTests, testEpipolarMatching)
{
  using namespace ze;

  // Load two images:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img_ref = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  ImageCv8uC1::Ptr img_cur = cvBridgeLoad8uC1(joinPath(data_path, "img", "10.png"));
  ImagePyramid8uC1::Ptr pyr_ref = createImagePyramidCpu<Pixel8uC1>(img_ref, 0.5, 5, 8);
  ImagePyramid8uC1::Ptr pyr_cur = createImagePyramidCpu<Pixel8uC1>(img_cur, 0.5, 5, 8);

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
  
  // Compute pixels on epipolar line
  Keypoint px_ref(120, 50);
  Bearing f_ref = cam->backProject(px_ref);
  real_t inv_depth_estimate = 1.0f / depth_ref(px_ref.cast<int>());
  real_t inv_depth_min = inv_depth_estimate * 2.0f;
  real_t inv_depth_max = inv_depth_estimate * 0.5f;
  uint32_t level_ref = 0;
  EpipolarMatcherOptions options;
  options.subpix_refinement = false;
  EpipolarMatcher matcher(options);
  Keypoint px_cur_true = cam->project(T_cur_ref * (f_ref * 1.0 / inv_depth_estimate));
  Keypoint px_cur;
  EpipolarMatchResult res;
  std::tie(px_cur, res) = matcher.findEpipolarMatchDirect(
        T_cur_ref, *pyr_ref, *pyr_cur, *cam, *cam, px_ref, f_ref, level_ref,
        inv_depth_estimate, inv_depth_min, inv_depth_max);

  EXPECT_TRUE(res == EpipolarMatchResult::Success);
  EXPECT_LT((px_cur - px_cur_true).norm(), 0.3);

  // Visualize
  if(false)
  {
    drawFeature<Pixel8uC1>(*img_ref, px_ref, 3, 255);
    drawFeature<Pixel8uC1>(*img_cur, px_cur, 3, 255);
    cvBridgeShow("img_ref", ImageCv8uC1(*img_ref));
    cvBridgeShow("img_cur", ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}

ZE_UNITTEST_ENTRYPOINT
