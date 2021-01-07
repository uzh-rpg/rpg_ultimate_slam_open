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

#include <ze/geometry/lsq_solver.hpp>

#include <stdexcept>
#include <ze/common/logging.hpp>
#include <ze/common/matrix.hpp>

namespace ze {

template <typename T, typename Implementation>
LeastSquaresSolver<T, Implementation>::LeastSquaresSolver(
    const LeastSquaresSolverOptions& options)
  : solver_options_(options)
{
  chi2_per_iter_.reserve(std::min(options.max_iter, 100u));
}

template <typename T, typename Implementation>
void LeastSquaresSolver<T, Implementation>::optimize(State& state)
{
  // If state is of dynamic size, this resizes Hessian, dx, g.
  allocateMemory(state);

  if (solver_options_.strategy == SolverStrategy::GaussNewton)
  {
    optimizeGaussNewton(state);
  }
  else if (solver_options_.strategy == SolverStrategy::LevenbergMarquardt)
  {
    optimizeLevenbergMarquardt(state);
  }
}

template <typename T, typename Implementation>
void LeastSquaresSolver<T, Implementation>::optimizeGaussNewton(State& state)
{
  // Save the old model to rollback in case of unsuccessful update
  State old_state = state;

  // perform iterative estimation
  for (iter_ = 0; iter_<solver_options_.max_iter; ++iter_)
  {
    rho_ = 0;
    startIteration();

    H_.setZero();
    g_.setZero();

    // compute initial error
    real_t new_chi2 = evaluateError(state, &H_, &g_);

    // solve the linear system
    if (!solve(state, H_, g_, dx_))
    {
      LOG(WARNING) << "Matrix is close to singular! Stop Optimizing."
                   << "H = " << H_ << "g = " << g_;
      stop_ = true;
    }

    // check if error increased since last optimization
    if ((iter_ > 0 && new_chi2 > chi2_ && solver_options_.stop_when_error_increases) || stop_)
    {
      VLOG(400) << "It. " << iter_
                << "\t Failure"
                << "\t new_chi2 = " << new_chi2
                << "\t Error increased. Stop optimizing.";
      state = old_state; // rollback
      break;
    }

    // update the model
    State new_state;
    update(state, dx_, new_state);
    old_state = state;
    state = new_state;
    chi2_ = new_chi2;
    chi2_per_iter_.push_back(chi2_);
    real_t x_norm = normMax(dx_);
    VLOG(400) << "It. " << iter_
              << "\t Success"
              << "\t new_chi2 = " << new_chi2
              << "\t x_norm = " << x_norm;
    finishIteration();

    // stop when converged, i.e. update step too small
    if (x_norm < solver_options_.eps)
    {
      VLOG(400) << "Converged, x_norm " << x_norm << " < " << solver_options_.eps;
      break;
    }
  }
}

template <typename T, typename Implementation>
void LeastSquaresSolver<T, Implementation>::optimizeLevenbergMarquardt(State& state)
{
  // init parameters
  mu_ = solver_options_.mu_init;
  nu_ = solver_options_.nu_init;

  // compute the initial error
  chi2_ = evaluateError(state, nullptr, nullptr);
  VLOG(400) << "init chi2 = " << chi2_;

  // TODO: compute initial lambda
  // Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
  // average of the diagonal elements of J'J"
  // Compute Initial Lambda
  if (mu_ < 0)
  {
    real_t H_max_diag = maxAbsDiagonalElement(H_);
    real_t tau = 1e-4;
    mu_ = tau*H_max_diag;
  }

  // perform iterative estimation
  for (iter_ = 0; iter_<solver_options_.max_iter; ++iter_)
  {
    rho_ = 0;
    startIteration();

    // try to compute and update, if it fails, try with increased mu
    trials_ = 0;
    do
    {
      // init variables
      State new_model;
      real_t new_chi2 = -1;
      H_.setZero();
      g_.setZero();

      // linearize
      evaluateError(state, &H_, &g_);

      // add damping term:
      H_ += (H_.diagonal() * mu_).asDiagonal();

      // solve the linear system to obtain small perturbation in direction of gradient
      if (solve(state, H_, g_, dx_))
      {
        // apply perturbation to the state
        update(state, dx_, new_model);

        // compute error with new model and compare to old error
        new_chi2 = evaluateError(new_model, nullptr, nullptr);
        rho_ = chi2_-new_chi2;
      }
      else
      {
        LOG(WARNING) << "Matrix is close to singular! Stop Optimizing."
                     << "H = " << H_ << "g = " << g_;
        rho_ = -1;
      }

      if (rho_ > 0.0)
      {
        // update decrased the error -> success
        state = new_model;
        chi2_ = new_chi2;
        chi2_per_iter_.push_back(chi2_);
        stop_ = normMax(dx_) < solver_options_.eps;
        mu_ *= std::max(real_t{0.333f},
                        std::min(real_t{1.0f - 8.0f * rho_ * rho_ * rho_},
                                 real_t{0.666f}));
        nu_ = 2.;
        VLOG(400) << "It. " << iter_
                  << "\t Trial " << trials_
                  << "\t Success"
                  << "\t new_chi2 = " << new_chi2
                  << "\t mu = " << mu_
                  << "\t nu = " << nu_;
      }
      else
      {
        // update increased the error -> fail
        mu_ *= nu_;
        nu_ *= 2.;
        ++trials_;
        if (trials_ >= solver_options_.max_trials)
        {
          stop_ = true;
        }

        VLOG(400) << "It. " << iter_
                  << "\t Trial " << trials_
                  << "\t Failure"
                  << "\t new_chi2 = " << new_chi2
                  << "\t mu = " << mu_
                  << "\t nu = " << nu_;
      }
      finishTrial();

    } while (!(rho_>0 || stop_));

    if (stop_)
    {
      break;
    }

    finishIteration();
  }
}

template <typename T, typename Implementation>
void LeastSquaresSolver<T, Implementation>::reset()
{
  VLOG(400) << "Reset";
  chi2_ = std::numeric_limits<real_t>::max();
  mu_ = solver_options_.mu_init;
  nu_ = solver_options_.nu_init;
  iter_ = 0;
  trials_ = 0;
  stop_ = false;
}

template <typename T, typename Implementation>
bool LeastSquaresSolver<T, Implementation>::solveDefaultImpl(
    const HessianMatrix& H,
    const GradientVector& g,
    UpdateVector& dx)
{
  dx = H.ldlt().solve(g);
  if (std::isnan(dx[0]))
  {
    return false;
  }
  return true;
}

template <typename T, typename Implementation>
void LeastSquaresSolver<T, Implementation>::updateDefaultImpl(
    const State& state,
    const UpdateVector& dx,
    State& new_state)
{
  new_state = traits<State>::retract(state, dx);
}

} // namespace ze
