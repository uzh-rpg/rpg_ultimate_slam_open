#pragma once

#include <imp/feature_matching/matching_types.hpp>

namespace ze {

//! Matches two blocks of descriptors. Returns optimal pairing.
class BruteForceMatcher
{
public:
  using distance_t = FeaturePairing::distance_t;
  using index_t = FeaturePairing::index_t;

  static constexpr uint32_t c_max_candidates_ { 4u };

  using Pairing = FeaturePairing;
  using PairingCandidates = FeaturePairingCandidates<c_max_candidates_>;
  using PairingCandidatesList = std::vector<PairingCandidates>;

  BruteForceMatcher() = delete;

  BruteForceMatcher(
      const distance_t hamming_theshold = 50);

  //! @return Returns best matches for B.
  PairingList run(
      const Descriptors& descriptors_a,
      const Descriptors& descriptors_b);

private:
  distance_t hamming_threshold_;
};

} // namespace ze
