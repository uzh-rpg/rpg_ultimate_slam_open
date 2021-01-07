#include <imp/feature_matching/brute_force_matcher.hpp>

#include <set>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/benchmark.hpp>

using namespace ze;

TEST(BruteForceMatcher, testTypes)
{
  EXPECT_EQ(sizeof(FeaturePairing), 4);
  EXPECT_EQ(sizeof(BruteForceMatcher::PairingCandidates), 18);
}

TEST(BruteForceMatcher, testAddingCandidates)
{
  BruteForceMatcher::PairingCandidates candidates;

  // Fill candidates:
  candidates.addPairingCandidate(FeaturePairing(0u, 20u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 20);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 20);

  candidates.addPairingCandidate(FeaturePairing(0u, 10u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 20);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 10);

  candidates.addPairingCandidate(FeaturePairing(0u, 30u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 30);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 10);

  candidates.addPairingCandidate(FeaturePairing(0u, 40u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 40);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 10);

  // Now, we exceed capacity.
  candidates.addPairingCandidate(FeaturePairing(0u, 50u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 40);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 10);

  candidates.addPairingCandidate(FeaturePairing(0u, 5u));
  EXPECT_EQ(candidates.candidates[candidates.worstPairingIndex()].distance, 30);
  EXPECT_EQ(candidates.candidates[candidates.bestPairingIndex()].distance, 5);
}

TEST(BruteForceMatcher, testMatcher)
{
  const uint32_t descriptor_size_bytes = 32;
  const uint32_t num_descriptors = 500;
  Descriptors descriptors_a(descriptor_size_bytes, num_descriptors);
  Descriptors descriptors_b(descriptor_size_bytes, num_descriptors);
  descriptors_a.setRandom();
  descriptors_b.setRandom();

  // Benchmark brute-force matcher.
  BruteForceMatcher matcher(110);
  auto benchmarkBruteforce = [&]() -> PairingList
  {
    return matcher.run(descriptors_a, descriptors_b);
  };
  runTimingBenchmark(benchmarkBruteforce, 1, 100, "Brute Force Matching", true);

  // Check uniqueness of matches.
  PairingList matches_for_b = benchmarkBruteforce();
  std::set<BruteForceMatcher::index_t> matched_indices;
  for (const FeaturePairing& pairing : matches_for_b)
  {
    if (pairing.isValid())
    {
      EXPECT_TRUE(matched_indices.find(pairing.other_index) == matched_indices.end());
      matched_indices.insert(pairing.other_index);
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
