// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! @todo(cfo): This is just a temporary class until Janosch has the same
//!             functionality in ze_imu
class ImuIntegrator
{
public:
  ImuIntegrator() = default;
  ~ImuIntegrator() = default;

  // No copy.
  ImuIntegrator(const ImuIntegrator&) = delete;
  ImuIntegrator& operator=(const ImuIntegrator&) = delete;

  //! All measurements are integrated except the last one, which is only there
  //! to provide the timestamp of the end-time of the second-last measurement.
  Transformation integrateImu(const ImuStamps& stamps,
                              const ImuAccGyrContainer& acc_gyr,
                              const Transformation &T_W_Bkm1,
                              Vector3 &v_W) const;


  //! Propagae IMU measurement.
  //! State must be given in world frame frame (i.e. q_W_B, p_W_B, v_W).
  inline void propagate(
      Quaternion& q,
      Position& p,
      Vector3& v,
      const Vector3& acc,
      const Vector3& gyr,
      const real_t dt) const
  {
    q = q * Quaternion::exp((gyr - gyr_bias_) * dt);
    p = p + v * dt;
    v = v + (q.rotate(acc - acc_bias_) - g_) * dt;
  }

  //! Get R_W_B assuming that dominant direction of acceleration is gravity.
  Quaternion getInitialOrientation(const ImuAccGyrContainer& acc_gyr);

  //! Set IMU Biases used for integration.
  void setBiases(const Vector3& acc_bias, const Vector3& gyr_bias)
  {
    acc_bias_ = acc_bias;
    gyr_bias_ = gyr_bias;
  }

  const Vector3& getGyrBias() const { return gyr_bias_; }
  const Vector3& getAccBias() const { return acc_bias_; }

private:
  Vector3 acc_bias_ = Vector3::Zero();
  Vector3 gyr_bias_ = Vector3::Zero();
  Vector3 g_ = Vector3(0., 0., 9.81);
};

} // namespace ze
