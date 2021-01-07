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
//
// Modified: Robotics and Perception Group

#pragma once

#include <type_traits>
#include <vector>

#include <ze/common/types.hpp>
#include <ze/common/manifold.hpp>

namespace ze {

enum class SolverStrategy {
  GaussNewton,
  LevenbergMarquardt
};

struct LeastSquaresSolverOptions
{
  //! Solver strategy.
  SolverStrategy strategy = SolverStrategy::GaussNewton;

  //! Damping parameter. If mu > 0, coefficient matrix is positive definite, this
  //! ensures that x is a descent direction. If mu is large, x is a short step in
  //! the steepest direction. This is good if the current iterate is far from the
  //! solution. If mu is small, LM approximates gauss newton iteration and we
  //! have (almost) quadratic convergence in the final stages.
  real_t mu_init{0.01};

  //! Increase factor of mu after fail
  real_t nu_init{2.0};

  //! Max number of iterations
  uint32_t max_iter{15u};

  //! Max number of trials (used in LevenbergMarquardt)
  uint32_t max_trials{5u};

  //! Stop when error increases.
  bool stop_when_error_increases{false};

  //! Output Statistics
  bool verbose{false};

  //! Stop if update norm is smaller than eps
  real_t eps{1.0e-10};
};

//! Abstract Class for solving nonlinear least-squares (NLLS) problems.
//! Template Parameters: D: dimension of the state, T: type of the model
//! e.g. SE2, SE3
template <typename T, typename Implementation>
class LeastSquaresSolver
{
public:
  using State = T;
  enum Dimension : int { dimension = traits<State>::dimension };
  using HessianMatrix = Eigen::Matrix<real_t, dimension, dimension>;
  using GradientVector = Eigen::Matrix<real_t, dimension, 1>;
  using UpdateVector = Eigen::Matrix<real_t, dimension, 1>;

  LeastSquaresSolverOptions solver_options_;

protected:
  LeastSquaresSolver() = default;

  LeastSquaresSolver(const LeastSquaresSolverOptions& options);

  virtual ~LeastSquaresSolver() = default;

public:
  //! Calls the GaussNewton or LevenbergMarquardt optimization strategy.
  void optimize(State& state);

  //! Gauss Newton optimization strategy.
  void optimizeGaussNewton(State& state);

  //! Levenberg Marquardt optimization strategy.
  void optimizeLevenbergMarquardt(State& state);

  //! Reset all parameters to restart the optimization.
  void reset();

  //! Get the squared error.
  inline real_t error() const
  {
    return chi2_;
  }

  //! Get error at every iteration.
  inline const std::vector<real_t>& errors() const
  {
    return chi2_per_iter_;
  }

  //! The the Hessian matrix (Information Matrix).
  inline const HessianMatrix& hessian() const
  {
    return H_;
  }

protected:
  //! Get implementation (Curiously-Returning Template Pattern).
  Implementation& impl()
  {
    return *static_cast<Implementation*>(this);
  }

  //! Evaluates the error at provided state. Optional return variables are
  //! the Hessian matrix and the gradient vector (Jacobian * residual).
  //! If these parameters are requested, the system is linearized at the current
  //! state.
  real_t evaluateError(
      const State& state,
      HessianMatrix* H,
      GradientVector* g)
  {
    return impl().evaluateError(state, H, g);
  }

  //! Solve the linear system H*dx = g to obtain optimal perturbation dx.
  bool solve(
      const State& state,
      const HessianMatrix& H,
      const GradientVector& g,
      UpdateVector& dx)
  {
    if(&LeastSquaresSolver::solve != &Implementation::solve)
    {
      return impl().solve(state, H, g, dx);
    }
    return solveDefaultImpl(H, g, dx);
  }

  //! Apply the perturbation dx to the state.
  void update(
      const State& state,
      const UpdateVector& dx,
      State& new_state)
  {
    if(&LeastSquaresSolver::update != &Implementation::update)
    {
      return impl().update(state, dx, new_state);
    }
    return updateDefaultImpl(state, dx, new_state);
  }

  void startIteration()
  {
    if(&LeastSquaresSolver::startIteration != &Implementation::startIteration)
    {
      impl().startIteration();
    }
  }

  void finishIteration()
  {
    if(&LeastSquaresSolver::finishIteration != &Implementation::finishIteration)
    {
      impl().finishIteration();
    }
  }

  void finishTrial()
  {
    if(&LeastSquaresSolver::finishTrial != &Implementation::finishTrial)
    {
      impl().finishTrial();
    }
  }

private:
  //! Default implementation to solve the linear system H*dx = g to obtain optimal perturbation dx.
  bool solveDefaultImpl(
      const HessianMatrix& H,
      const GradientVector& g,
      UpdateVector& dx);

  void updateDefaultImpl(
      const State& state,
      const UpdateVector& dx,
      State& new_state);

  template<typename State, typename std::enable_if<(traits<State>::dimension == Eigen::Dynamic)>::type* = nullptr>
  inline void allocateMemory(State& state)
  {
    const int dim = state.getDimension();
    H_.resize(dim, dim);
    g_.resize(dim);
    dx_.resize(dim);
  }

  template<typename State, typename std::enable_if<(traits<State>::dimension != Eigen::Dynamic)>::type* = nullptr>
  inline void allocateMemory(State& /*state*/)
  {}

protected:
  //! Hessian or approximation Jacobian*Jacobian^T.
  HessianMatrix H_;

  //! Jacobian*residual.
  GradientVector g_;

  //! Update step.
  UpdateVector dx_;

  //! Whitened error / log-likelihood: 1/(2*sigma^2)*(z-h(x))^2.
  real_t chi2_{std::numeric_limits<real_t>::max()};

  //! Error reduction: chi2 - new_chi2.
  real_t rho_{0.0};

  //! Damping parameter.
  real_t mu_{0.01};

  //! Factor that specifies how much we increase mu at every trial.
  real_t nu_{2.0};

  //! Stop flag.
  bool stop_{false};

  //! Current Iteration.
  size_t iter_{0u};

  //! Current number of trials.
  size_t trials_{0u};

  // Statistics:
  std::vector<real_t> chi2_per_iter_;
};

} // namespace ze

#include <ze/geometry/lsq_solver-inl.hpp>
