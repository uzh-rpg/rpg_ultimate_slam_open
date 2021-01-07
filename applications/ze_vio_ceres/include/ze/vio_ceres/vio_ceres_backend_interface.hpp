// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <ze/nlls/estimator.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/timer_collection.hpp>
#include <ze/vio_common/motion_type.hpp>

namespace ze {

// fwd
class CameraRig;
class ImuIntegrator;
class LandmarkTable;
class NFrameTable;

// callback declarations.
using BiasUpdateCallback = std::function<void(const Vector3& /*acc_bias*/,
                                              const Vector3& /*gyr_bias*/)>;

struct CeresBackendOptions
{
  // Optimization settings.
  size_t num_iterations{ 10 };
  size_t num_threads { 2 };
  bool verbose { false };

  // Marginalization settings.
  bool marginalize { true };
  size_t num_keyframes { 5 };
  size_t num_imu_frames { 3 };
};

class CeresBackendInterface : Noncopyable
{
public:
  CeresBackendInterface(LandmarkTable& landmarks_,
                        NFrameTable& states,
                        const std::shared_ptr<const CameraRig>& rig,
                        std::shared_ptr<ImuIntegrator> imu_integrator,
                        const CeresBackendOptions& settings_ = CeresBackendOptions());

  ~CeresBackendInterface();

  void startThread();

  void stopThread();

  void addTrackedNFrame(
      const std::shared_ptr<NFrame>& nframe_k,
      const ImuStamps& imu_stamps,
      const ImuAccGyrContainer& imu_accgyr,
      const VioMotionType motion_type,
      const std::vector<LandmarkHandle>& lm_opportunistic,
      const std::vector<LandmarkHandle>& lm_persistent_new,
      const std::vector<LandmarkHandle>& lm_persistent_continued);

  bool updateStateWithResultFromLastOptimization(
      const bool wait_for_backend = false);

private:
  void optimization();

  void optimizationLoop();

  bool addLandmarksAndObservationsToBackend(
      const std::vector<LandmarkHandle>& new_landmarks);

  bool addNewestObservationsToBackend(
      const std::vector<LandmarkHandle>& continued_landmarks,
      const NFrame::ConstPtr& nframe);

  ze::Estimator backend_;
  bool backend_initialized_{false}; //!< Is initialized as soon as 2 frames have been added.
  CeresBackendOptions settings_;
  const size_t num_cameras_;
  //! Imu integrator is only used to update biases for frontend.
  std::shared_ptr<ImuIntegrator> imu_integrator_;

  // State related.
  LandmarkTable& landmarks_;
  NFrameTable& states_;
  BackendId last_added_nframe_;
  BackendId last_optimized_nframe_;
  BackendId last_updated_nframe_;

  // Threading
  mutable std::condition_variable wait_condition_;
  mutable std::mutex mutex_backend_;
  std::unique_ptr<std::thread> thread_;
  std::atomic_bool stop_thread_ { false };

  // Timers
  DECLARE_TIMER(Timer, timers_,
                add_tracked_nframe, add_state, add_landmarks, optimize,
                marginalization, update_state);

};


} // namespace ze
