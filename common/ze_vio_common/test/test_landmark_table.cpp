// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/common/test_entrypoint.hpp>
#include <ze/vio_common/landmark_table.hpp>

TEST(LandmarkStorageTests, testFunctionality)
{
  using namespace ze;

  LandmarkTable lms;
  LandmarkHandles h = lms.getNewLandmarkHandles(10, 0u);
  VLOG(1) << lms.typesFormattedString();
}

ZE_UNITTEST_ENTRYPOINT
