#include <functional>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/benchmark.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/matrix.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_utils.hpp>

#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/correspondence/patch_utils.hpp>
#include <imp/correspondence/klt.hpp>
#include <imp/imgproc/draw.hpp>

TEST(KltTests, testKlt)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  KltParameters params;

  // Test alignment accuracy with 8x8 patch.
  // These checks compare with the results obtained at the time of implementation.
  {
    Keypoint px_true = Keypoint(290.6, 170.2);
    Keypoint px_estimate = px_true - Keypoint(0.4, 0.7);
    alignFeature(*img, *img, 1, params, px_true, px_estimate);
    EXPECT_NEAR((px_estimate - px_true).norm(), 0.000255088, 1e-4);

    // Timing benchmark: Problem, if we pass px_estimate as reference, then
    //                   the second iteration already starts with the solution!
    //px_estimate = px_true - Keypoint(0.4, 0.7);
    //auto fun = std::bind(alignFeature, *img, *img, px_true, px_estimate, 1);
    //runTimingBenchmark(fun, 100, 100, "alignFeature 8x8", true);
  }

  // Test alignment accuracy with 24x24 patch:
  {
    Keypoint px_true = Keypoint(290.6, 170.2);
    Keypoint px_estimate = px_true - Keypoint(0.4, 0.7);
    alignFeature(*img, *img, 3, params, px_true, px_estimate);
    EXPECT_NEAR((px_estimate - px_true).norm(), 0.00104593, 1e-4);
  }

  // Test vectorized alignment accuracy with 24x24 patch:
  {
    Keypoints px_true(2, 3);
    px_true << 290.6, 290.6, 290.6,
               170.2, 170.2, 170.2;
    Keypoints px_estimate = px_true;
    px_estimate.colwise() -= Keypoint(0.4, 0.7);
    alignFeatures(*img, *img, 3, params, px_true, px_estimate);
    EXPECT_NEAR((px_estimate.col(0) - px_true.col(0)).norm(), 0.00104593, 1e-4);
  }
}

TEST(KltTests, testKltPyramidal)
{
  using namespace ze;

  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  ImagePyramid8uC1::Ptr pyr = createImagePyramidCpu<Pixel8uC1>(img, 0.5, 5, 8);
  KltParameters params;

  // Test alignment accuracy with 8x8 patch:
  std::vector<int> patch_sizes_b8 = { 1, 2, 2, 2 };
  Keypoints px_true(2, 3);
  px_true << 290.6, 290.6, 290.6,
             170.2, 171.2, 172.2;
  Keypoints px_perturb(2, 3);
  px_perturb << 10.4, 12.5, 20.3,
                12.8, 5.32, 10.4;
  Keypoints px_estimate = px_true.array() + px_perturb.array();
  alignFeaturesPyr(*pyr, *pyr, patch_sizes_b8, params, px_true, px_estimate);
  // These checks compare with the results obtained at the time of implementation.
  EXPECT_NEAR((px_estimate.col(0) - px_true.col(0)).norm(), 0.000236231, 1e-5);
  EXPECT_NEAR((px_estimate.col(1) - px_true.col(1)).norm(), 0.000305851, 1e-5);
  EXPECT_NEAR((px_estimate.col(2) - px_true.col(2)).norm(), 0.000601515, 1e-5);
}

TEST(KltTests, testKltGuided)
{
  using namespace ze;

  // Load two images:
  std::string data_path = getTestDataDir("synthetic_room_pinhole");
  ImageCv8uC1::Ptr img_ref = cvBridgeLoad8uC1(joinPath(data_path, "img", "1.png"));
  ImageCv8uC1::Ptr img_cur = cvBridgeLoad8uC1(joinPath(data_path, "img", "3.png"));
  ImagePyramid8uC1::Ptr pyr_ref = createImagePyramidCpu<Pixel8uC1>(img_ref, 0.5, 5, 8);
  ImagePyramid8uC1::Ptr pyr_cur = createImagePyramidCpu<Pixel8uC1>(img_cur, 0.5, 5, 8);

  // Load poses:
  std::map<int64_t, Transformation> T_W_C_vec =
      loadIndexedPosesFromCsv(data_path + "/traj_gt.csv");
  Transformation T_cur_ref = T_W_C_vec[3].inverse() * T_W_C_vec[1];

  // Load camera:
  Camera::Ptr cam = cameraFromYaml(data_path + "/calib.yaml");

  // Load depthmap for first image:
  ImageRaw32fC1 depth_ref(cam->width(), cam->height());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  loadDepthmapFromFile(data_path + "/depth/1.depth",
                       depth_ref.numel(),
                       reinterpret_cast<float*>(depth_ref.data()));

  // Check pixel correspondence
  KltParameters params;
  constexpr int n = 100;
  Keypoints px_ref = generateRandomKeypoints(cam->size(), 0, n);
  Keypoints px_cur(2, n);
  std::vector<int> patch_sizes_b8 = { 2, 3, 3, 2 };
  Bearings f_ref = cam->backProjectVectorized(px_ref);
  VectorX invdepth_ref(n);
  for(int i = 0; i < invdepth_ref.size(); ++i)
  {
    invdepth_ref(i) = 1.0 / depth_ref(px_ref.col(i).cast<int>());
  }

  // KLT
  Transformation T_cur_ref_prior(T_cur_ref.getRotation(), Vector3::Zero());
  bool affine_warp = true;
  alignFeaturesGuidedPyr(*pyr_ref, *pyr_cur, *cam, *cam, patch_sizes_b8,
                         params, affine_warp, T_cur_ref_prior, px_ref, f_ref,
                         invdepth_ref, px_cur);

  // Display correspondences
  if(false)
  {
    drawFeatures<Pixel8uC1>(*img_ref, px_ref, 5, 255);
    drawFeatures<Pixel8uC1>(*img_cur, px_cur, 5, 255);
    drawLines<Pixel8uC1>(*img_cur, px_ref, px_cur, 255);

    cvBridgeShow("ref_im", ImageCv8uC1(*img_ref));
    cvBridgeShow("cur_im", ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}
ZE_UNITTEST_ENTRYPOINT
