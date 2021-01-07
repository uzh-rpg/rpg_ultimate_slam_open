#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/feature_matching/brute_force_matcher.hpp>
#include <imp/feature_matching/matching_viz.hpp>
#include <imp/features/brisk_detector.hpp>
#include <imp/features/brisk_descriptor.hpp>

using namespace ze;

TEST(Disabled_MatchingViz, test)
{
  /*
  std::string data_dir =
      joinPath(getTestDataDir("feature_matching"), "affine_covariant_features", "graffiti");
  ImageCv8uC1::Ptr img_a = cvBridgeLoad8uC1(joinPath(data_dir, "img1.ppm"));
  ImageCv8uC1::Ptr img_b = cvBridgeLoad8uC1(joinPath(data_dir, "img2.ppm"));
  */

  std::string data_dir = joinPath(getTestDataDir("loopclosure"), "dbow2_images");
  ImageCv8uC1::Ptr img_a = cvBridgeLoad8uC1(joinPath(data_dir, "image1.png"));
  ImageCv8uC1::Ptr img_b = cvBridgeLoad8uC1(joinPath(data_dir, "image3.png"));

  ImagePyramid8uC1::Ptr pyr_a = createImagePyramidCpu<Pixel8uC1>(img_a, 0.5, 5, 8);
  ImagePyramid8uC1::Ptr pyr_b = createImagePyramidCpu<Pixel8uC1>(img_b, 0.5, 5, 8);


  // Create detector:
  BriskDetectorOptions brisk_options;
  brisk_options.brisk_uniformity_radius = 20;
  brisk_options.brisk_absolute_threshold = 20;
  DetectionStrategy strategy;
  strategy.emplace_back(
        std::make_shared<BriskDetector>(brisk_options, img_a->size()));
  FeatureDetectionHandler detector(strategy);

  // Create descriptor:
  BriskDescriptorOptions descriptor_options;
  descriptor_options.rotation_invariance = true;
  descriptor_options.scale_invariance = true;
  AbstractDescriptorExtractor::Ptr descriptor_extractor =
      std::make_shared<BriskDescriptorExtractor>();

  // Detect:
  int max_fts = 750;
  Keypoints px_vec_a(2, max_fts), px_vec_b(2, max_fts);
  KeypointScores score_vec(max_fts);
  KeypointLevels level_vec(max_fts);
  KeypointAngles angle_vec(max_fts);
  KeypointTypes type_vec(max_fts);
  Descriptors descriptors_a, descriptors_b;
  uint32_t num_detected_a = 0, num_detected_b = 0;

  // Extract features in first frame:
  {
    KeypointsWrapper features(px_vec_a, score_vec, level_vec, angle_vec, type_vec,
                              descriptors_a, num_detected_a);
    detector.detect(*pyr_a, features);
    descriptor_extractor->extract(*pyr_a, features);
    VLOG(1) << "Detected " << features.num_detected << " BRISK features.";
  }

  // Extract features in second frame:
  {
    KeypointsWrapper features(px_vec_b, score_vec, level_vec, angle_vec, type_vec,
                              descriptors_b, num_detected_b);
    detector.detect(*pyr_b, features);
    descriptor_extractor->extract(*pyr_b, features);
    VLOG(1) << "Detected " << features.num_detected << " BRISK features.";
  }

  BruteForceMatcher matcher(70);
  auto matches_for_b = matcher.run(descriptors_a, descriptors_b);
  IndexPairList matches_a_b = indexPairListFromPairingList(matches_for_b);

  VLOG(1) << "Found " << matches_a_b.size() << " matches.";
  drawMatches(*img_a, *img_b, px_vec_a, px_vec_b, matches_a_b, 10);
}

TEST(MatchingViz, test2)
{
  std::string data_dir = joinPath(getTestDataDir("loopclosure"), "dbow2_images");
  cv::Mat img_a = cv::imread(joinPath(data_dir, "image1.png"), 0);
  cv::Mat img_b = cv::imread(joinPath(data_dir, "image3.png"), 0);

  CHECK(!img_a.empty());
  CHECK(!img_b.empty());

  // Extract features and descriptors:
  std::vector<cv::KeyPoint> kp_vec_a, kp_vec_b;
  cv::Mat desc_a, desc_b;
  cv::ORB orb_detector;
  orb_detector(img_a, cv::Mat(), kp_vec_a, desc_a);
  orb_detector(img_b, cv::Mat(), kp_vec_b, desc_b);

  // Create frame reference:
  Descriptors descriptors_a =
      Eigen::Map<Descriptors>(desc_a.data, desc_a.cols, desc_a.rows);
  Descriptors descriptors_b =
      Eigen::Map<Descriptors>(desc_b.data, desc_b.cols, desc_b.rows);

  Keypoints px_vec_a(2, kp_vec_a.size());
  for (size_t i = 0u; i < kp_vec_a.size(); ++i)
  {
    px_vec_a(0,i) = kp_vec_a.at(i).pt.x;
    px_vec_a(1,i) = kp_vec_a.at(i).pt.y;
  }
  Keypoints px_vec_b(2, kp_vec_b.size());
  for (size_t i = 0u; i < kp_vec_b.size(); ++i)
  {
    px_vec_b(0,i) = kp_vec_b.at(i).pt.x;
    px_vec_b(1,i) = kp_vec_b.at(i).pt.y;
  }

  BruteForceMatcher matcher(70);
  auto matches_for_b = matcher.run(descriptors_a, descriptors_b);
  IndexPairList matches_a_b = indexPairListFromPairingList(matches_for_b);

  VLOG(1) << "Found " << matches_a_b.size() << " matches.";

  drawMatches(ImageCv8uC1(img_a), ImageCv8uC1(img_b), px_vec_a, px_vec_b, matches_a_b,
              10);

}

ZE_UNITTEST_ENTRYPOINT
