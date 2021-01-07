#pragma once

#include <array>
#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>

namespace ze {

using IndexPairList = std::vector<std::pair<uint32_t, uint32_t>>;

// -----------------------------------------------------------------------------
//! Saves a match and distance pair.
struct FeaturePairing
{
  using index_t = int16_t;
  using distance_t = uint16_t;  //! Hamming distance returns an integer.

  index_t other_index { -1 };
  distance_t distance  { std::numeric_limits<distance_t>::max() };

  FeaturePairing() = default;

  FeaturePairing(index_t index, distance_t distance)
    : other_index(index)
    , distance(distance)
  {}

  inline bool isValid() const { return other_index != index_t(-1); }

  inline bool operator<(const FeaturePairing& rhs) const
  {
    return distance < rhs.distance;
  }
};
using PairingList = std::vector<FeaturePairing>;

IndexPairList indexPairListFromPairingList(const PairingList& best_matches_for_b);

// -----------------------------------------------------------------------------
//! Saves a list of candidate matches.
template<uint8_t MaxCandidates>
struct FeaturePairingCandidates
{
  uint8_t num_candidates { 0u };
  std::array<FeaturePairing, MaxCandidates> candidates;

  //! Inserts a new candidate if there is still space. Otherwise, it checks the
  //! existing candidates and inserts only if the new candidate is better than
  //! the worst candidate in the list.
  bool addPairingCandidate(const FeaturePairing& candidate);

  uint8_t bestPairingIndex() const;
  uint8_t worstPairingIndex() const;

  inline FeaturePairing& bestPairing() { return candidates[bestPairingIndex()]; }
  inline FeaturePairing& worstPairing() { return candidates[worstPairingIndex()]; }
};

//! Pretty printing.
template<size_t MaxCandidates>
std::ostream& operator<<(std::ostream& out,
                         const FeaturePairingCandidates<MaxCandidates>& candidates);

// -----------------------------------------------------------------------------
template<uint8_t MaxCandidates>
uint8_t FeaturePairingCandidates<MaxCandidates>::bestPairingIndex() const
{
  CHECK_GT(num_candidates, 0u);

  uint8_t best_candidate_index = 0u;
  for (uint8_t i = 1u; i < num_candidates; ++i)
  {
    if (candidates[i] < candidates[best_candidate_index])
    {
      best_candidate_index = i;
    }
  }
  return best_candidate_index;
}

// -----------------------------------------------------------------------------
template<uint8_t MaxCandidates>
uint8_t FeaturePairingCandidates<MaxCandidates>::worstPairingIndex() const
{
  CHECK_GT(num_candidates, 0u);

  uint8_t worst_candidate_index = 0u;
  for (uint8_t i = 1u; i < num_candidates; ++i)
  {
    if (candidates[worst_candidate_index] < candidates[i])
    {
      worst_candidate_index = i;
    }
  }
  return worst_candidate_index;
}

// -----------------------------------------------------------------------------
template<uint8_t MaxCandidates>
bool FeaturePairingCandidates<MaxCandidates>::addPairingCandidate(
    const FeaturePairing& candidate)
{
  if (num_candidates < MaxCandidates)
  {
    // Add new candidate to list:
    candidates[num_candidates] = candidate;
    ++num_candidates;
    return true;
  }
  else
  {
    // Replace worst candidate:
    FeaturePairing& worst_candidate = worstPairing();
    if (candidate < worst_candidate)
    {
      worst_candidate = candidate;
      return true;
    }
  }
  return false;
}

// -----------------------------------------------------------------------------
template<uint8_t MaxCandidates>
std::ostream& operator<<(std::ostream& out,
                         const FeaturePairingCandidates<MaxCandidates>& candidates)
{
  out << "size: " << static_cast<int>(candidates.num_candidates) << "\n"
      << "candidates: \n";
  for (uint8_t i = 0u; i < candidates.num_candidates; ++i)
  {
    out << "- index: " << candidates.candidates[i].other_index << "\n"
        << "  distance: " << candidates.candidates[i].distance << "\n";
  }
  return out;
}

} // namespace ze
