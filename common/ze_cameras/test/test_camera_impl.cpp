// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#include <string>
#include <vector>
#include <iostream>
#include <functional>

#include <ze/common/benchmark.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/manifold.hpp>
#include <ze/common/numerical_derivative.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/cameras/camera.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/cameras/camera_utils.hpp>

DEFINE_bool(run_benchmark, false, "Benchmark the camera models?");

namespace ze {

class CameraBenchmark
{
public:
  CameraBenchmark(const Camera& cam, size_t sample_size, const std::string& test_name)
    : cam_(cam)
    , sample_size_(sample_size)
    , test_name_(test_name)
  {
    px_ = generateRandomKeypoints(cam_.size(), 10u, sample_size_);
    f_ = cam.backProjectVectorized(px_);
  }

  void benchmarkAll()
  {
    if (!FLAGS_run_benchmark) {
      return;
    }

    real_t p1 = project();
    real_t p = (p1 - projectVectorized()) / p1;
    real_t p2 = backProject();
    real_t bp = (p2 - backProjectVectorized()) / p2;
    VLOG(1) << "[" << test_name_ << "]" << "Vectorized back projection " << bp * 100 << "% faster." << "\n"
            << "[" << test_name_ << "]" << "Vectorized projection " << p * 100 << "% faster.";
  }

  real_t backProjectVectorized()
  {
    // Test back-project vectorized
    Bearings f(3, sample_size_);
    auto backProjectVectorizedLambda = [&]()
    {
      f = cam_.backProjectVectorized(px_);
    };
    return runTimingBenchmark(backProjectVectorizedLambda, 10, 20,
                       test_name_ + ": Back-project vectorized", true);
  }

  real_t backProject()
  {
    // Test not vectorized.
    Bearings f(3, sample_size_);
    auto backProjectLambda = [&]()
    {
      for (size_t i = 0; i < sample_size_; ++i)
      {
        f.col(i) = cam_.backProjectVectorized(px_.col(i));
      }
    };
    return runTimingBenchmark(backProjectLambda, 10, 20,
                       test_name_ + ": Back-project N-times", true);
  }

  real_t projectVectorized()
  {
    // Test back-project vectorized
    Keypoints px(2, sample_size_);
    auto projectVectorizedLambda = [&]()
    {
      px = cam_.projectVectorized(f_);
    };
    return runTimingBenchmark(projectVectorizedLambda, 10, 20,
                       test_name_ + ": Project vectorized", true);
  }

  real_t project()
  {
    // Test not vectorized.
    Keypoints px(2, sample_size_);
    auto projectLambda = [&]()
    {
      for (size_t i = 0; i < sample_size_; ++i)
      {
        px.col(i) = cam_.project(f_.col(i));
      }
    };
    return runTimingBenchmark(projectLambda, 10, 20,
                       test_name_ + ": Project vectorized", true);
  }
private:
  const Camera& cam_;
  size_t sample_size_;
  std::string test_name_;
  Keypoints px_;
  Bearings f_;
};

class CameraTestHarness
{
public:
  CameraTestHarness(const Camera& cam, size_t sample_size, const std::string& test_name)
    : cam_(cam)
    , sample_size_(sample_size)
    , test_name_(test_name)
  {}

  void testProjection()
  {
    uint32_t N = 300;
    Keypoints px1 = generateRandomKeypoints(cam_.size(), 10u, sample_size_);
    Bearings f1 = cam_.backProjectVectorized(px1);
    Keypoints px2 = cam_.projectVectorized(f1);
    Keypoints px_error = px1 - px2;
    real_t max_error = px_error.colwise().norm().array().maxCoeff();
    EXPECT_LT(max_error, 1.3e-4);
  }

  void testProjectionSingle()
  {
    Vector3 bearing = cam_.backProject(Vector2(200, 300));
    Vector2 px = cam_.project(bearing);
#ifndef ZE_SINGLE_PRECISION_FLOAT
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200, 300), 1e-6));
#else
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200, 300), 1e-4));
#endif
  }

  void testHomogeneousProjection()
  {
    Vector3 bearing = cam_.backProject(Vector2(200.0, 300.0));
    Vector4 hom_position;
    {
      // normal case:
      hom_position << bearing, 1.0;
      Vector2 px = cam_.projectHomogeneous(hom_position);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200.0, 300.0), 1e-6));
    }

    {
      // point at infinity:
      hom_position << bearing, 0.0;
      Vector2 px = cam_.projectHomogeneous(hom_position);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200.0, 300.0), 1e-6));
    }

    {
      // point behind camera.
      hom_position << bearing, -1.0;
      Vector2 px = cam_.projectHomogeneous(hom_position);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200.0, 300.0), 1e-6));
    }

    {
      // some arbitrary scaling.
      hom_position << bearing, 10.0;
      Vector2 px = cam_.projectHomogeneous(hom_position);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200.0, 300.0), 1e-6));
    }
  }

  void testJacobian()
  {
#ifndef ZE_SINGLE_PRECISION_FLOAT
    Vector3 bearing = cam_.backProject(Vector2(200, 300));
    Matrix23 H = cam_.dProject_dLandmark(bearing);
    Matrix23 H_numerical =
        numericalDerivative<Vector2, Vector3>(
        std::bind(&Camera::project, &cam_, std::placeholders::_1), bearing);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
#else
    LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
  }

  void testHomogeneousJacobian()
  {
    Vector4 hom_position;
    hom_position.head<3>() = cam_.backProject(Vector2(200.0, 300.0));
    hom_position[3] = 1.0;
    Matrix24 H = cam_.dProjectHomogeneous_dLandmark(hom_position);
    Matrix24 H_numerical =
        numericalDerivative<Vector2, Vector4>(
          std::bind(&Camera::projectHomogeneous, &cam_, std::placeholders::_1),
          hom_position);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
  }

  void testAll()
  {
    {
      SCOPED_TRACE("Projection");
      testProjection();
    }
    {
      SCOPED_TRACE("ProjectionSingle");
      testProjectionSingle();
    }
    {
      SCOPED_TRACE("HomogeneousProjection");
      testHomogeneousProjection();
    }
    {
      SCOPED_TRACE("Jacobian");
      testJacobian();
    }
    {
      SCOPED_TRACE("HomogeneousJacobian");
      testHomogeneousJacobian();
    }
  }

private:
  const Camera& cam_;
  size_t sample_size_;
  std::string test_name_;
};

} // namespace ze

TEST(CameraImplTests, testPinhole)
{
  using namespace ze;
  PinholeCamera cam = createPinholeCamera(752, 480, 310, 320, 376.0, 240.0);
  CameraTestHarness test(cam, 300, "Pinhole");
  test.testAll();
  CameraBenchmark benchmark(cam, 300, "Pinhole");
  benchmark.benchmarkAll();
}

TEST(CameraImplTests, testFov)
{
  using namespace ze;
  FovCamera cam = createFovCamera(752, 480, 310, 320, 376.0, 240.0, 0.947367);
  CameraTestHarness test(cam, 300, "Fov");
  test.testAll();
  CameraBenchmark benchmark(cam, 300, "Fov");
  benchmark.benchmarkAll();
}

TEST(CameraImplTests, testRadTan)
{
  using namespace ze;
  RadTanCamera cam = createRadTanCamera(752, 480, 310, 320, 376.0, 240.0,
                                        -0.2834, 0.0739, 0.00019, 1.76e-05);
  CameraTestHarness test(cam, 300, "RadTan");
  test.testAll();
  CameraBenchmark benchmark(cam, 300, "RadTan");
  benchmark.benchmarkAll();
}

TEST(CameraImplTests, testEquidistant)
{
  using namespace ze;
  EquidistantCamera cam = createEquidistantCamera(752, 480, 310, 320, 376.0, 240.0,
                                                  -0.00279, 0.02414, -0.04304, 0.03118);
  CameraTestHarness test(cam, 300, "Equidistant");
  test.testAll();
  CameraBenchmark benchmark(cam, 300, "Equidistant");
  benchmark.benchmarkAll();
}

TEST(CameraImplTests, testYamlParsingPinhole)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_nodistortion.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::Camera::Ptr cam = ze::cameraFromYaml(yaml_file);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(1), 310.0);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(2), 376.5);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(3), 240.5);
}

TEST(CameraImplTests, testYamlParsingFoV)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_fov.yaml";
  ze::Camera::Ptr cam = ze::cameraFromYaml(yaml_file);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_FLOATTYPE_EQ(cam->distortionParameters()(0), 0.940454);
}

TEST(CameraImplTests, testYamlParsingRadTan)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_radtan.yaml";
  ze::Camera::Ptr cam = ze::cameraFromYaml(yaml_file);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_FLOATTYPE_EQ(cam->distortionParameters()(0), -0.28340811217029355);
}

TEST(CameraImplTests, testYamlParsingEquidistant)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_equidistant.yaml";
  ze::Camera::Ptr cam = ze::cameraFromYaml(yaml_file);
  EXPECT_FLOATTYPE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_FLOATTYPE_EQ(cam->distortionParameters()(0), -0.0027973061697674074);
}

ZE_UNITTEST_ENTRYPOINT
