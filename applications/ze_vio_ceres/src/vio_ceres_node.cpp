// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <gflags/gflags.h>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/vio_common/imu_integrator.hpp>
#include <ze/vio_common/landmark_table.hpp>
#include <ze/vio_common/nframe_table.hpp>
#include <ze/vio_frontend/frontend_api.hpp>
#include <ze/vio_frontend/frontend_base.hpp>
#include <ze/vio_ceres/vio_ceres_backend_interface.hpp>

#include <ros/ros.h>

DEFINE_bool(vio_ceres_verbose, false, "Output ceres optimization progress.");
DEFINE_bool(vio_ceres_marginalize, true, "Apply marginalization?");
DEFINE_int32(vio_ceres_iterations, 3, "Maximum number of iterations.");
DEFINE_int32(vio_ceres_sliding_window_size, 3, "Sliding window size of ceres backend.");
DEFINE_int32(vio_ceres_numkeyframes, 5, "Number of keyframes of ceres backend");
DEFINE_int32(vio_ceres_num_threads, 1, "Number of threads in ceres backend");

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  VLOG(1) << "Initialize ROS.";
  ros::init(argc, argv, "ze_vio");

  VLOG(1) << "Initialize Frontend.";
  ze::VisualOdometry vo;
  vo.initialize();

  VLOG(1) << "Initialize Backend.";
  ze::CeresBackendOptions backend_options;
  backend_options.verbose = FLAGS_vio_ceres_verbose;
  backend_options.marginalize = FLAGS_vio_ceres_marginalize;
  backend_options.num_iterations = FLAGS_vio_ceres_iterations;
  backend_options.num_imu_frames = FLAGS_vio_ceres_sliding_window_size;
  backend_options.num_keyframes = FLAGS_vio_ceres_numkeyframes;
  backend_options.num_threads = FLAGS_vio_ceres_num_threads;
  ze::CeresBackendInterface backend(
        vo.frontend()->landmarks_,
        vo.frontend()->states_,
        vo.frontend()->rig_,
        vo.frontend()->imu_integrator_,
        backend_options);

  VLOG(1) << "Register callbacks.";
  using namespace std::placeholders;
  vo.frontend()->registerTrackedNFrameCallback(
        std::bind(&ze::CeresBackendInterface::addTrackedNFrame,
                  &backend,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6,
                  std::placeholders::_7));
  vo.frontend()->registerUpdateStatesCallback(
        std::bind(&ze::CeresBackendInterface::updateStateWithResultFromLastOptimization,
                  &backend, std::placeholders::_1));

  vo.subscribeDataProviders();

  //! @todo register bias update callback.

  VLOG(1) << "Start Processing.";
  backend.startThread();
  vo.startBlocking();

  VLOG(1) << "Finish Processing.";
  vo.shutdown();
  backend.stopThread();

  VLOG(1) << "Node terminated cleanly.";
  return 0;
}
