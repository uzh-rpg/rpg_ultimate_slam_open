#include <ze/common/manifold.hpp>
#include <ze/common/numerical_derivative.hpp>
#include <ze/common/test_entrypoint.hpp>

using namespace ze;

// -----------------------------------------------------------------------------
// Test manifold concepts.
template<typename T>
void testManifoldInvariants(const T& a, const T& b, real_t tol = 1e-9)
{
  EXPECT_TRUE(traits<T>::equals(a, a));
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v);
  EXPECT_TRUE(traits<T>::equals(b, c, tol));
}

template<typename T>
void testRetractJacobians(const T& a, const T& b, real_t tol = 1e-9)
{
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v, &H1, &H2);
  ASSERT_TRUE(traits<T>::equals(b, c, tol));

  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<T, T>(
        std::bind(traits<T>::retract, _1, v, nullptr, nullptr), a);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<T, typename traits<T>::TangentVector>(
        std::bind(traits<T>::retract, a, _1, nullptr, nullptr), v);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H2, H2_numerical, tol));
}

template<typename T>
void testLocalJacobians(const T& a, const T& b, real_t tol = 1e-9)
{
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
        std::bind(traits<T>::local, _1, b, nullptr, nullptr), a);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
        std::bind(traits<T>::local, a, _1, nullptr, nullptr), b);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H2, H2_numerical, tol));
}

#ifndef ZE_SINGLE_PRECISION_FLOAT
constexpr real_t tol = 1e-9;
#else
constexpr real_t tol = 1e-7;
#endif

TEST(ManifoldTests, testScalarTraits)
{
  EXPECT_EQ(traits<real_t>::dimension, 1);
  testManifoldInvariants<real_t>(1.0, 1.5);
#ifndef ZE_SINGLE_PRECISION_FLOAT
  testRetractJacobians<real_t>(1.0, 1.5);
  testLocalJacobians<real_t>(1.0, 1.5);
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(ManifoldTests, testEigenTraits)
{
  EXPECT_EQ(traits<Vector3>::dimension, 3);
  testManifoldInvariants<Vector3>(
        Vector3(1.0, 1.2, 1.3), Vector3(2.0, 1.0, 0.0));
#ifndef ZE_SINGLE_PRECISION_FLOAT
  testRetractJacobians<Vector3>(
        Vector3(1.0, 1.2, 1.3), Vector3(2.0, 1.0, 0.0));
  testLocalJacobians<Vector3>(
        Vector3(1.0, 1.2, 1.3), Vector3(2.0, 1.0, 0.0));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(ManifoldTests, testManifoldSO3Invariants)
{
  EXPECT_EQ(traits<Quaternion>::dimension, 3);
  testManifoldInvariants<Quaternion>(
        Quaternion(Vector3(0.1, 0.2, 0.3)),
        Quaternion(Vector3(0.2, 0.3, 0.4)), tol);
}

#ifndef ZE_SINGLE_PRECISION_FLOAT
TEST(ManifoldTests, testManifoldSO3Retract)
{
  testRetractJacobians<Quaternion>(
        Quaternion(Vector3(0.1, 0.2, 0.3)),
        Quaternion(Vector3(0.2, 0.3, 0.4)));
}

TEST(ManifoldTests, testManifoldSO3Local)
{
  testLocalJacobians<Quaternion>(
        Quaternion(Vector3(0.1, 0.2, 0.3)),
        Quaternion(Vector3(0.2, 0.3, 0.4)));
}
#endif

TEST(ManifoldTests, testManifoldSE3Invariants)
{
  EXPECT_EQ(traits<Transformation>::dimension, 6);
  testManifoldInvariants<Transformation>(
        Transformation().setRandom(),
        Transformation().setRandom(), tol);
}

#ifndef ZE_SINGLE_PRECISION_FLOAT
TEST(ManifoldTests, testManifoldSE3LocalJacobians)
{
  Transformation origin;
  origin.setRandom();
  Transformation other = origin * Transformation().setRandom(0.2, 0.2);
  testLocalJacobians<Transformation>(origin, other);
}

TEST(ManifoldTests, testManifoldSE3RetractJacobians)
{
  Transformation origin, other;
  origin.setRandom(1.2, 0.8);
  other.setRandom();
  testRetractJacobians<Transformation>(origin, other);
  Vector6 v = Transformation::log(origin.inverse() * other);
  EXPECT_TRUE(traits<Vector6>::equals(v, traits<Transformation>::local(origin, other)));
}
#endif

ZE_UNITTEST_ENTRYPOINT
