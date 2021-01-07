#include <imp/feature_matching/matching_viz.hpp>

#include <imp/core/image_raw.hpp>
#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/imgproc/draw.hpp>

namespace ze {

void drawMatches(const Image8uC1& imp_img_a,
                 const Image8uC1& imp_img_b,
                 const Keypoints& px_vec_a,
                 const Keypoints& px_vec_b,
                 const IndexPairList& matches_a_b,
                 const int sleep)
{
  cv::Mat img_a = ImageCv8uC1(imp_img_a).cvMat();
  cv::Mat img_b = ImageCv8uC1(imp_img_b).cvMat();

  cv::Mat img_a_rgb = cv::Mat(img_a.size(), CV_8UC3);
  cv::Mat img_b_rgb = cv::Mat(img_b.size(), CV_8UC3);
  cv::cvtColor(img_a, img_a_rgb, cv::COLOR_GRAY2RGB);
  cv::cvtColor(img_b, img_b_rgb, cv::COLOR_GRAY2RGB);

  cv::Mat img_rgb(img_a.rows, 2 * img_a.cols, CV_8UC3);
  cv::Mat left_roi(img_rgb, cv::Rect(0, 0, img_a.cols, img_a.rows));
  img_a_rgb.copyTo(left_roi);
  cv::Mat right_roi(img_rgb, cv::Rect(img_a.cols, 0, img_a.cols, img_a.rows));
  img_b_rgb.copyTo(right_roi);

  // Draw features:
  for (int i = 0; i < px_vec_a.cols(); ++i)
  {
    cv::circle(img_rgb,
               cv::Point2f(px_vec_a(0,i), px_vec_a(1,i)),
               3, cv::Scalar(0,255), 1);
  }

  for (int i = 0; i < px_vec_b.cols(); ++i)
  {
    cv::circle(img_rgb,
               cv::Point2f(px_vec_b(0,i) + img_a.cols, px_vec_b(1,i)),
               3, cv::Scalar(0,255), 1);
  }

  // Draw all matches:
  for (const auto ab : matches_a_b)
  {

    auto pt_a = px_vec_a.col(ab.first);
    auto pt_b = px_vec_b.col(ab.second);
    cv::line(img_rgb,
             cv::Point2f(pt_a(0), pt_a(1)),
             cv::Point2f(pt_b(0) + img_a.cols, pt_b(1)), cv::Scalar(0,255,0), 1);
    cv::circle(img_rgb, cv::Point2f(pt_a(0), pt_a(1)), 3, cv::Scalar(0,0,255), 1);
    cv::circle(img_rgb, cv::Point2f(pt_b(0) + img_a.cols, pt_b(1)), 3, cv::Scalar(0,0,255), 1);
  }

  cv::imshow("matches", img_rgb);
  cv::waitKey(sleep);

}

} // namespace ze
