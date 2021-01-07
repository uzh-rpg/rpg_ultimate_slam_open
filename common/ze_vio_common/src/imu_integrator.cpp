// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_common/imu_integrator.hpp>

#include <ze/common/logging.hpp>
#include <ze/common/time_conversions.hpp>


namespace ze {

Transformation ImuIntegrator::integrateImu(
    const ImuStamps& stamps,
    const ImuAccGyrContainer& acc_gyr,
    const Transformation& T_Bkm1_W,
    Vector3& v_W) const
{
  CHECK_EQ(stamps.size(), acc_gyr.cols());

  Transformation T_W_B = T_Bkm1_W.inverse();
  Quaternion& q = T_W_B.getRotation();
  Position& t = T_W_B.getPosition();

  for(int i = 0; i < (stamps.size() - 1); ++i)
  {
    real_t dt = nanosecToSecTrunc(
                  stamps(i+1) - stamps(i));
    propagate(q, t, v_W,
              acc_gyr.block<3,1>(0,i),
              acc_gyr.block<3,1>(3,i),
              dt);
  }

  VLOG(100) << "Integrated " << stamps.size() - 1 << " measurements";

  return T_Bkm1_W * T_W_B;
}

Quaternion ImuIntegrator::getInitialOrientation(
    const ImuAccGyrContainer& acc_gyr)
{
  // Set initial coordinate frame based on gravity direction.
  // This only works if the sensor is initially not moving and the accelerometer
  // has zero bias (which is never the case..).
  const Vector3 g = acc_gyr.topRows<3>().rowwise().sum();
  const Vector3 z = g.normalized();
  VLOG(3) << "Initial gravity direction = " << z.transpose();
  Vector3 p(1,0,0);
  Vector3 p_alternative(0,1,0);
  if(std::abs(z.dot(p)) > std::abs(z.dot(p_alternative)))
    p = p_alternative;
  Vector3 y = z.cross(p); // make sure gravity is not in x direction
  y.normalize();
  const Vector3 x = y.cross(z);
  Matrix3 R_B_W; // world unit vectors in imu coordinates
  R_B_W.col(0) = x;
  R_B_W.col(1) = y;
  R_B_W.col(2) = z;
  Quaternion R_W_B(Matrix3(R_B_W.transpose()));
  return R_W_B;
}

} // namespace ze
