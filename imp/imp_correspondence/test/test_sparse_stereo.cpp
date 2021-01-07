#include <functional>

#include <ze/common/benchmark.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/timer.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/cameras/camera_utils.hpp>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/correspondence/sparse_stereo.hpp>
#include <imp/features/fast_detector.hpp>
#include <imp/imgproc/draw.hpp>
#include <imp/imgproc/image_pyramid.hpp>

TEST(SparseStereoTests, testStereo)
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

  // Load depthmap from reference image:
  ImageRaw32fC1 depth_ref(cam->width(), cam->height());
  CHECK_EQ(depth_ref.width(), depth_ref.stride());
  loadDepthmapFromFile(data_path + "/depth/1.depth",
                       depth_ref.numel(),
                       reinterpret_cast<float*>(depth_ref.data()));

  // Detect keypoints in reference image:
  uint32_t max_fts = 300;
  Keypoints px_ref(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors;
  uint32_t num_features = 0u;
  KeypointsWrapper features(px_ref, score_vec, level_vec, angle_vec, type_vec,
                            descriptors, num_features);
  FastDetector detector(FastDetectorOptions(), img_ref->size());
  detector.detect(*pyr_ref, features);
  EXPECT_EQ(num_features, 271u);
  Bearings f_ref = cam->backProjectVectorized(px_ref.leftCols(num_features));

  // Compute ground-truth matches:
  Positions xyz_ref = f_ref;
  for (uint32_t i = 0; i < num_features; ++i)
  {
    xyz_ref.col(i) *= depth_ref(px_ref.col(i).cast<int>());
  }
  Positions xyz_cur = T_cur_ref.transformVectorized(xyz_ref);
  Keypoints px_cur_groundtruth = cam->projectVectorized(xyz_cur);

  // Run sparse stereo matcher:
  SparseStereoMatcher stereo_matcher(*cam, *cam, *pyr_ref, *pyr_cur,
                                     T_cur_ref,
                                     px_ref.leftCols(num_features),
                                     f_ref,
                                     level_vec.head(num_features));
  stereo_matcher.computeMatches(1.0 / 1.5, 1.0 / 0.2, 1.0 / 3.0);
  const Keypoints& px_cur = stereo_matcher.px_cur_;

  // Count inliers:
  int outlier_count = 0;
  int n = 0;
  Keypoints px_ref_matched(2, stereo_matcher.inliers_.size());
  for (auto i : stereo_matcher.inliers_)
  {
    px_ref_matched.col(n) = px_ref.col(i);
    real_t error = (px_cur_groundtruth.col(i) - px_cur.col(n)).norm();
    if(error > 1.0)
    {
      ++outlier_count;
    }
    ++n;
  }
  EXPECT_LE(outlier_count, 9);

  if (false)
  {
    drawFeatures<Pixel8uC1>(*img_ref, px_ref_matched, 5, 255);
    drawFeatures<Pixel8uC1>(*img_cur, px_cur, 5, 255);
    drawLines<Pixel8uC1>(*img_cur, px_ref_matched, px_cur, 255);

    cvBridgeShow("ref_im", ImageCv8uC1(*img_ref));
    cvBridgeShow("cur_im", ImageCv8uC1(*img_cur));
    cv::waitKey(0);
  }
}

ZE_UNITTEST_ENTRYPOINT
