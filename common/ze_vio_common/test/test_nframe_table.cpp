// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <unordered_map>
#include <ze/common/random.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_utils.hpp>
#include <ze/common/types.hpp>
#include <ze/common/benchmark.hpp>
#include <ze/vio_common/nframe_table.hpp>

TEST(NFrameStorageTests, testFunctionality)
{
  using namespace ze;

  NFrameTable storage;

  EXPECT_FALSE(isValidNFrameHandle(storage.nframeHandleK()));
  EXPECT_FALSE(isValidNFrameHandle(storage.nframeHandleKm1()));

  NFrameHandle handle1 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle1.version, 1);
  EXPECT_EQ(handle1.slot, 0);
  EXPECT_EQ(storage.nframeHandleK(), handle1);

  NFrameHandle handle2 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle2.version, 1);
  EXPECT_EQ(handle2.slot, 1);
  storage.setKeyframe(handle2);
  EXPECT_EQ(storage.nframeHandleK(), handle2);
  EXPECT_EQ(storage.nframeHandleKm1(), handle1);

  // Re-use slot & handle at km1
  NFrameHandle handle3 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle3.version, 1);
  EXPECT_EQ(handle3.slot, 0);
  EXPECT_EQ(storage.nframeHandleK(), handle3);
  EXPECT_EQ(storage.nframeHandleKm1(), handle2);

  // New slot, because handle2 is keyframe.
  NFrameHandle handle4 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle4.version, 1);
  EXPECT_EQ(handle4.slot, 2);
  EXPECT_EQ(storage.nframeHandleK(), handle4);
  EXPECT_EQ(storage.nframeHandleKm1(), handle3);

  // Re-use slot at handle at km1
  NFrameHandle handle5 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle5.version, 1);
  EXPECT_EQ(handle5.slot, 0);
  EXPECT_EQ(storage.nframeHandleK(), handle5);
  EXPECT_EQ(storage.nframeHandleKm1(), handle4);

  // Re-use slot & handle at km2
  NFrameHandle handle6 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
  EXPECT_EQ(handle6.version, 1);
  EXPECT_EQ(handle6.slot, 2);
  EXPECT_EQ(storage.nframeHandleK(), handle6);
  EXPECT_EQ(storage.nframeHandleKm1(), handle5);
}

TEST(NFrameStorageTests, testIntrusive)
{
  using namespace ze;

  NFrameTable storage;

  NFrameHandle handle_k, handle_km1, handle_lkf, handle_lkfm1,
               handle_lkfm2, handle_lkfm3, handle_lkfm4;
  auto dist = getRandomGeneratorBinary(0.2);

  for (int i = 0; i < 1000000; ++i)
  {
    if (i == 100000)
    {
      dist = getRandomGeneratorBinary(0.0);
    }
    if (i == 200000)
    {
      dist = getRandomGeneratorBinary(0.8);
    }
    if (i == 300000)
    {
      dist = getRandomGeneratorBinary(1.0);
    }
    if (i == 400000)
    {
      dist = getRandomGeneratorBinary(0.5);
    }

    handle_km1 = handle_k;
    handle_k = storage.makeAndStoreNewEmptyTestNFrame()->handle();
    if (dist()) // Select a keyframe with random probability.
    {
      handle_lkfm4 = handle_lkfm3;
      handle_lkfm3 = handle_lkfm2;
      handle_lkfm2 = handle_lkfm1;
      handle_lkfm1 = handle_lkf;
      storage.setKeyframe(handle_k);
      handle_lkf = handle_k;
    }

    if (isValidNFrameHandle(handle_k))     { EXPECT_EQ(storage.nframeHandleK(), handle_k);     }
    if (isValidNFrameHandle(handle_km1))   { EXPECT_EQ(storage.nframeHandleKm1(), handle_km1); }
    if (isValidNFrameHandle(handle_lkf))   { EXPECT_EQ(storage.nframeHandleLkf(), handle_lkf); }
    if (isValidNFrameHandle(handle_k))     { EXPECT_TRUE(storage.isStored(handle_k));     }
    if (isValidNFrameHandle(handle_km1))   { EXPECT_TRUE(storage.isStored(handle_km1));   }
    if (isValidNFrameHandle(handle_lkf))   { EXPECT_TRUE(storage.isStored(handle_lkf));   }
    if (isValidNFrameHandle(handle_lkfm1)) { EXPECT_TRUE(storage.isStored(handle_lkfm1)); }
    if (isValidNFrameHandle(handle_lkfm2)) { EXPECT_TRUE(storage.isStored(handle_lkfm2)); }
    if (isValidNFrameHandle(handle_lkfm3)) { EXPECT_TRUE(storage.isStored(handle_lkfm3)); }
    if (isValidNFrameHandle(handle_lkfm4)) { EXPECT_TRUE(storage.isStored(handle_lkfm4)); }

    NFrameHandles last_keyframes = storage.getNLastKeyframes(5);
    if (isValidNFrameHandle(handle_lkf))
    {
      EXPECT_GE(last_keyframes.size(), 1u);
    }
    if (!last_keyframes.empty())
    {
      EXPECT_EQ(last_keyframes.front(), handle_lkf);
    }
    if (isValidNFrameHandle(handle_lkfm4))
    {
      EXPECT_EQ(last_keyframes.size(), 5u);
    }
    if (last_keyframes.size() == 5u)
    {
      EXPECT_EQ(last_keyframes.back(), handle_lkfm4);
    }
  }
}

TEST(NFrameStorageTests, benchmark)
{
  using namespace ze;

  // Benchmark NFrameStorage.
  {
    NFrameTable storage;
    Transformation T;
    NFrameHandle h1 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
    NFrameHandle h2 = storage.makeAndStoreNewEmptyTestNFrame()->handle();
    auto addAndQueryStorage = [&]() {
      storage.T_B_W(h1) = T;
      storage.setKeyframe(h2);
      if (storage.isStored(h2))
      {
        Transformation T2 = storage.T_B_W(h2);
        T = T2;
      }
    };
    runTimingBenchmark(addAndQueryStorage, 20, 100, "addAndQueryStorage", true);
  }

  // Benchmark Alternative with unordered_map.
  {
    std::unordered_map<uint16_t, Transformation> storage;
    Transformation T;
    uint16_t h2 = 0u;
    uint16_t h1 = 0u;
    auto addAndQueryStorage = [&]() {
      h1 += 1u;
      storage.insert(std::make_pair(h1, T));
      auto it = storage.find(h2);
      if (it != storage.end())
      {
        T = it->second;
      }
    };
    runTimingBenchmark(addAndQueryStorage, 20, 100, "addAndQueryAlternative", true);
  }
}


ZE_UNITTEST_ENTRYPOINT

