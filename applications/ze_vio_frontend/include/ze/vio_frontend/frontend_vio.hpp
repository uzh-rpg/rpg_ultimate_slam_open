// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/frontend_base.hpp>

#include <ze/common/transformation.hpp>

namespace ze {

// fwd.
class LandmarksReprojector;

class FrontendVio : public FrontendBase
{
public:
  FrontendVio();

  FrontendVio(const CameraRigPtr& rig);

  // Modules
  std::shared_ptr<LandmarksReprojector> reprojector_;

protected:
  void init();

  virtual void processData(const Transformation& T_Bkm1_Bk);

  void makeKeyframeIfNecessary(const uint32_t num_tracked);

};


} // namespace ze
