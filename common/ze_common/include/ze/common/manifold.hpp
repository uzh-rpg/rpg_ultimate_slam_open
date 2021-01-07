// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
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
//
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

#pragma once

#include <functional>
#include <Eigen/Dense>
#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! Manifold traits define a retract (boxPlus), and local (boxMinus) operation
//! for scalars, vectors, rotations and transformations. The traits are used for
//! simplified computation of numerical derivatives to verify the analytical ones
//! and in the optimizer. These traits are inspired by GTSAM and for rotations
//! and transformation we use the same convention (updates are applied on the
//! right hand side / in the body frame).
template<typename T> struct traits;

// -----------------------------------------------------------------------------
// Manifold traits for scalars.
namespace internal {
template<typename Scalar>
struct ScalarTraits
{
  enum { dimension = 1 }; //! @todo(cfo): static constexpr int fails on GCC 4.8, works for Eigen because no derived class.
  typedef Eigen::Matrix<Scalar, 1, 1> TangentVector;
  typedef Eigen::Matrix<Scalar, 1, 1> Jacobian;

  static int getDimension(const Scalar /*v*/)
  {
    return 1;
  }

  static bool equals(Scalar v1, Scalar v2, Scalar tol = 1e-8)
  {
    return std::abs(v1 - v2) < tol;
  }

  static TangentVector local(const Scalar origin, Scalar other,
                             Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      (*H1)[0] = -1.0; // dlocal(origin, other) / dorigin
    }
    if (H2)
    {
      (*H2)[0] =  1.0; // dlocal(origin, other) / dother
    }
    TangentVector result;
    result(0) = other - origin;
    return result;
  }

  static Scalar retract(const Scalar origin, const TangentVector& v,
                        Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      (*H1)[0] = 1.0; // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      (*H2)[0] = 1.0; // dretract(origin, v) / dv
    }
    return origin + v[0];
  }
};
} // namespace internal

// Define scalar traits for float and double
template<> struct traits<double> : public internal::ScalarTraits<double> {};
template<> struct traits<float> : public internal::ScalarTraits<float> {};


// -----------------------------------------------------------------------------
// Manifold traits for fixed-size Eigen matrices and vectors with double precision.
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<real_t, M, N, Options, MaxRows, MaxCols> >
{
  //static constexpr int dimension = M * N;
  enum { dimension = M * N };
  typedef Eigen::Matrix<real_t, M, N, Options, MaxRows, MaxCols> Matrix;
  typedef Eigen::Matrix<real_t, dimension, 1> TangentVector;
  typedef Eigen::Matrix<real_t, dimension, dimension> Jacobian;

  static int getDimension(const Matrix& /*v*/)
  {
    return M * N;
  }

  static bool equals(const Matrix& v1, const Matrix& v2, real_t tol = 1e-8)
  {
    if (v1.size() != v2.size())
    {
      return false;
    }
    return (v1 - v2).array().abs().maxCoeff() < tol;
    // TODO(cfo): Check for nan entries.
  }

  static TangentVector local(
      const Matrix& origin, Matrix other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = -Jacobian::Identity(); // dLocal(origin, other)/dOrigin
    }
    if (H2)
    {
      *H2 =  Jacobian::Identity(); // dLocal(origin, other)/dOther
    }
    TangentVector result;
    Eigen::Map<Matrix>(result.data()) = other - origin;
    return result;
  }

  static Matrix retract(
      const Matrix& origin, const TangentVector& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = Jacobian::Identity(); // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      *H2 = Jacobian::Identity(); // dretract(origin, v) / dv
    }
    return origin + Eigen::Map<const Matrix>(v.data());
  }
};

// -----------------------------------------------------------------------------
// Manifold traits for dynamic-size Eigen matrices and vectors with double precision.
namespace internal {

// traits for dynamic Eigen matrices
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct DynamicMatrixTraits {

  typedef Eigen::Matrix<real_t, M, N, Options, MaxRows, MaxCols> DynamicMatrix;

  enum Dimension : int { dimension = Eigen::Dynamic };

  typedef VectorX TangentVector;
  typedef Eigen::Matrix<real_t, dimension, dimension> Jacobian;
  typedef DynamicMatrix ManifoldType;

  static int getDimension(const DynamicMatrix& m)
  {
    return m.rows() * m.cols();
  }

  static Jacobian eye(const DynamicMatrix& m)
  {
    int dim = getDimension(m);
    return Eigen::Matrix<real_t, dimension, dimension>::Identity(dim, dim);
  }

  static TangentVector local(
      const DynamicMatrix& origin, const DynamicMatrix& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = -eye(origin); // dlocal(origin, other) / dorigin
    }
    if (H2)
    {
      *H2 =  eye(origin); // dlocal(origin, other) / dother
    }
    TangentVector v(getDimension(origin));
    Eigen::Map<DynamicMatrix>(v.data(), origin.rows(), origin.cols()) = other - origin;
    return v;
  }

  static DynamicMatrix retract(
      const DynamicMatrix& origin, const TangentVector& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = eye(origin); // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      *H2 = eye(origin); // dretract(origin, v) / dv
    }
    return origin + Eigen::Map<const DynamicMatrix>(v.data(), origin.rows(), origin.cols());
  }
};

} // namespace internal

// traits for fully dynamic matrix
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols> {
};

// traits for dynamic column vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<real_t, Eigen::Dynamic, 1, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<Eigen::Dynamic, 1, Options, MaxRows, MaxCols> {
};

// traits for dynamic row vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<real_t, 1, Eigen::Dynamic, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<1, Eigen::Dynamic, Options, MaxRows, MaxCols> {
};

// -----------------------------------------------------------------------------
// Manifold traits for SO(3)
template<> struct traits<Quaternion>
{
  enum { dimension = 3 }; // The dimension of the manifold.

  typedef Eigen::Matrix<real_t, dimension, 1> TangentVector;
  typedef Eigen::Matrix<real_t, dimension, dimension> Jacobian;

  static int getDimension(const Quaternion& /*v*/)
  {
    return 3;
  }

  static bool equals(
      const Quaternion& q1, const Quaternion& q2, real_t tol = 1e-8)
  {
    return (q1.getUnique().vector()
            - q2.getUnique().vector()).array().abs().maxCoeff() < tol;
  }

  static TangentVector local(
      const Quaternion& origin, const Quaternion& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion h = origin.inverse() * other;
    const TangentVector v = h.log();
    if(H1 || H2)
    {
      Jacobian D_v_h = logmapDerivativeSO3(v);
      if(H1)
      {
        // dlocal(origin, other) / dorigin, using that Adjoint(h.inverse()) = h.inverse()
        *H1 = - D_v_h * h.inverse().getRotationMatrix();
      }
      if(H2)
      {
        // dlocal(origin, other) / dother
        *H2 = D_v_h;
      }
    }
    return v;
  }

  static Quaternion retract(
      const Quaternion& origin, const Vector3& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion g = Quaternion::exp(v);
    const Quaternion h = origin * g;
    if (H1)
    {
      // dretract(origin, v) / dorigin
      *H1 = g.inverse().getRotationMatrix(); // Adjoint(g.inverse()) = g.inverse()
    }
    if (H2)
    {
      // dretract(origin, v) / dv
      *H2 = expmapDerivativeSO3(v);
    }
    return h;
  }
};

// -----------------------------------------------------------------------------
// Manifold traits for SE(3)
template<> struct traits<Transformation>
{
  enum { dimension = 6 }; // The dimension of the manifold.

  typedef Eigen::Matrix<real_t, dimension, 1> TangentVector;
  typedef Eigen::Matrix<real_t, dimension, dimension> Jacobian;

  static int getDimension(const Transformation& /*v*/)
  {
    return 6;
  }

  static bool equals(
      const Transformation& T1, const Transformation& T2, real_t tol = 1e-8)
  {
    return (T1.getRotation().getUnique().vector()
            - T2.getRotation().getUnique().vector()).array().abs().maxCoeff() < tol
        && (T1.getPosition() - T2.getPosition()).array().abs().maxCoeff() < tol;
  }

  static TangentVector local(
      const Transformation& origin, const Transformation& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Transformation h = origin.inverse() * other;
    const TangentVector v = (Vector6() << h.getPosition(), h.getRotation().log()).finished();
    if (H1 || H2)
    {
      Matrix3 J_r_inv = logmapDerivativeSO3(v.tail<3>());
      if (H1)
      {
        // dlocal(origin, other) / dorigin
        H1->block<3,3>(0,0) = -I_3x3;
        H1->block<3,3>(0,3) = skewSymmetric(v.head<3>());
        H1->block<3,3>(3,0) = Z_3x3;
        H1->block<3,3>(3,3) = - J_r_inv * h.getRotation().inverse().getRotationMatrix();
      }
      if(H2)
      {
        // dlocal(origin, other) / dother
        H2->block<3,3>(0,0) = h.getRotationMatrix();
        H2->block<3,3>(0,3) = Z_3x3;
        H2->block<3,3>(3,0) = Z_3x3;
        H2->block<3,3>(3,3) = J_r_inv;
      }
    }
    return v;
  }

  static Transformation retract(
      const Transformation& origin, const Vector6& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    Transformation g(Quaternion::exp(v.tail<3>()), v.head<3>());
    Transformation h  = origin * g;

    if (H1 || H2)
    {
      const Matrix3 R_CB = Quaternion::exp(v.tail<3>()).inverse().getRotationMatrix();
      if (H1)
      {
        // dretract(origin, other) / dorigin

        // Remember: Computation of the translation components must be in the
        // body frame of the result.
        // Retraction: T_AC = T_AB * exp(xi_BC) = T_AB * T_BC
        // Let's look at the translation component:
        //    A_t_AC = A_t_AB + R_AB * B_t_BC
        // Perturb it the translation (in the body frame B):
        //    A_t_AC = A_t_AB + (R_AB * B_dt) + R_AB * B_t_BC
        //           = A_t_AC + (R_AB * B_dt)
        // !!! What we want is the perturbation in the body frame of the result
        // i.e. in the frame C:
        //    A_t_AC = A_t_AC + (R_AB * B_dt)
        //    A_t_AC = A_t_AC + (R_AC * R_AC^-1) * (R_AB * B_dt) <- trick
        //           = A_t_AC + R_AC * (R_CB * B_dt)
        //           -> Jacobian = R_CB
        H1->block<3,3>(0,0) = R_CB;
        H1->block<3,3>(0,3) = - R_CB * skewSymmetric(v.head<3>());
        H1->block<3,3>(3,0) = Z_3x3;
        H1->block<3,3>(3,3) = g.getRotation().inverse().getRotationMatrix();
      }
      if(H2)
      {
        H2->block<3,3>(0,0) = R_CB;
        H2->block<3,3>(0,3) = Z_3x3;
        H2->block<3,3>(3,0) = Z_3x3;
        H2->block<3,3>(3,3) = expmapDerivativeSO3(v.tail<3>());
      }
    }
    return h;
  }
};

} // namespace ze
