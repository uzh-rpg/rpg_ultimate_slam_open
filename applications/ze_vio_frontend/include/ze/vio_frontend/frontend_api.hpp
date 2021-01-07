// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>

DECLARE_bool(vio_use_events_and_images);
DECLARE_bool(vio_use_events);

namespace ze {

// Forward declarations.
class CameraImuSynchronizerBase;
class CameraRig;
class DataProviderBase;
class FrontendBase;

//! Status of visual-odometry frontend.
enum class FrontendStage : std::int8_t
{
  Paused,
  AttitudeEstimation,
  Initializing,
  Running
};

//! Visual-odometry result callback declaration.
using VisualOdometryCallback =
  std::function<void(const int64_t timestamp,
                     const Eigen::Quaterniond& orientation,
                     const Eigen::Vector3d& position,
                     const FrontendStage stage,
                     const uint32_t num_tracked_features)>;

//! Zurich Eye visual odometry interface.
class VisualOdometry
{
public:
  VisualOdometry() = default;
  ~VisualOdometry() = default;

  //! Initialize VO. Must be called before any processing. VO is configured
  //! using gflags.
  void initialize();

  //! Must be called to subscribe to data providers.
  void subscribeDataProviders();

  //! Start processing the data. This function call is blocking until all data
  //! is processed, Ctrl-C is pressed, or pause() is called from another thread.
  void startBlocking();

  //! Can be called from another thread to pause the VO. Continue using after
  //! calling reset() and then startBlocking().
  void pause();

  //! Shutdown VO. Call before destruction to ensure safe writing of tracefiles.
  void shutdown();

  //! Register a callback that is called after every processed frame.
  void registerResultCallback(const VisualOdometryCallback& cb);

  //! Access frontend.
  const std::shared_ptr<FrontendBase>& frontend() { return frontend_; }

private:
  std::shared_ptr<CameraRig> rig_;
  std::shared_ptr<FrontendBase> frontend_;
  std::shared_ptr<DataProviderBase> data_provider_;
  std::shared_ptr<CameraImuSynchronizerBase> data_sync_;
};

} // namespace ze
