#include <imp/feature_matching/hamming.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/random_matrix.hpp>
#include <ze/common/benchmark.hpp>

using namespace ze;

// Hamming distance from ORB-SLAM, just used as reference:
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int orbHammingDistance(
    const Eigen::Ref<const Descriptors>& a,
    const Eigen::Ref<const Descriptors>& b)
{
  const int32_t *pa = reinterpret_cast<const int32_t*>(a.data());
  const int32_t *pb = reinterpret_cast<const int32_t*>(b.data());

  int dist = 0;
  for (int i = 0; i < 8; ++i, ++pa, ++pb)
  {
    unsigned  int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
  return dist;
}

TEST(HammingDistance, testHamming)
{
  const uint32_t descriptor_size_bytes = 32;
  const uint32_t num_descriptors = 500;
  Descriptors descriptors(descriptor_size_bytes, num_descriptors);

  descriptors.col(0).setConstant(0);
  descriptors.col(1).setConstant(std::numeric_limits<uint8_t>::max());
  int min_cost = Hamming::distance(descriptors.col(0).data(),
                                   descriptors.col(0).data(),
                                   descriptor_size_bytes);
  EXPECT_EQ(min_cost, 0);
  int max_cost = Hamming::distance(descriptors.col(0).data(),
                                   descriptors.col(1).data(),
                                   descriptor_size_bytes);
  EXPECT_EQ(max_cost, 256);

  descriptors.setRandom();

  auto benchmarkHammingSIMD = [&]() -> int
  {
    int sum_cost = 0;
    for (uint32_t i = 0; i < num_descriptors; ++i)
    {
      sum_cost += Hamming::distance(descriptors.col(0).data(),
                                    descriptors.col(i).data(),
                                    descriptor_size_bytes);
    }
    return sum_cost;
  };
  runTimingBenchmark(benchmarkHammingSIMD, 1, 20, "Hamming SIMD", true);

  auto benchmarkHammingBaseline = [&]() -> int
  {
    int sum_cost = 0;
    for (uint32_t i = 0; i < num_descriptors; ++i)
    {
      // Works only for 32-byte descriptors!
      sum_cost += orbHammingDistance(descriptors.col(0),
                                     descriptors.col(i));
    }
    return sum_cost;
  };
  runTimingBenchmark(benchmarkHammingBaseline, 1, 20, "Hamming Baseline", true);

  CHECK_EQ(benchmarkHammingSIMD(), benchmarkHammingBaseline());
}

ZE_UNITTEST_ENTRYPOINT
