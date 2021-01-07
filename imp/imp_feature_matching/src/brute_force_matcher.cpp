#include <imp/feature_matching/brute_force_matcher.hpp>
#include <imp/feature_matching/hamming.hpp>

namespace ze {

// -----------------------------------------------------------------------------
BruteForceMatcher::BruteForceMatcher(
    const distance_t hamming_theshold)
  : hamming_threshold_(hamming_theshold)
{}

// -----------------------------------------------------------------------------
bool matchBruteForce(const Descriptors& descriptors_a,
                     const Descriptors& descriptors_b,
                     const uint32_t start_idx,
                     const uint32_t end_idx,
                     const BruteForceMatcher::distance_t hamming_threshold,
                     BruteForceMatcher::PairingCandidatesList& match_candidates_for_a)
{
  const int descriptor_size_bytes = descriptors_a.rows();

  for (uint32_t ia = start_idx; ia < end_idx; ++ia)
  {
    BruteForceMatcher::PairingCandidates& candidates = match_candidates_for_a[ia];
    BruteForceMatcher::distance_t worst_candidate_distance = hamming_threshold;

    //! @todo(cfo): PERFORMANCE: try to make it faster by implementing
    //!             Hamming::distance for a whole block: 1 against many.

    for (int ib = 0, ib_max = descriptors_b.cols(); ib < ib_max; ++ib)
    {

      BruteForceMatcher::distance_t distance =
          Hamming::distance(&descriptors_a(0, ia),
                            &descriptors_b(0, ib),
                            descriptor_size_bytes);

      if (distance < worst_candidate_distance)
      {
        // TODO: This should be improved!
        DEBUG_CHECK(candidates.addPairingCandidate(FeaturePairing(ib, distance)));
        if (candidates.num_candidates >= BruteForceMatcher::c_max_candidates_)
        {
          worst_candidate_distance = candidates.worstPairing().distance;
        }
      }
    }
  }
  return true;
}

// -----------------------------------------------------------------------------
PairingList BruteForceMatcher::run(
    const Descriptors& descriptors_a,
    const Descriptors& descriptors_b)
{
  DEBUG_CHECK_EQ(descriptors_a.rows(), descriptors_b.rows());

  PairingList best_matches_for_b(descriptors_b.cols());
  PairingCandidatesList match_candidates_for_a(descriptors_a.cols());

  // Find all match candidates. Can be multi-threaded but the
  // benefit is not economical due to false sharing.
  matchBruteForce(descriptors_a, descriptors_b, 0, descriptors_a.cols(),
                  hamming_threshold_, match_candidates_for_a);

  // Assign best candidate to B:
  std::vector<index_t> ia_to_assign;
  ia_to_assign.resize(descriptors_a.cols());
  std::iota(ia_to_assign.begin(), ia_to_assign.end(), 0);

  while (!ia_to_assign.empty())
  {
    VLOG(5) << "Need to re-assign " << ia_to_assign.size() << " indices.";

    std::vector<index_t> ia_to_assign_again;
    ia_to_assign_again.reserve(ia_to_assign.size());

    for (int ia : ia_to_assign)
    {
      PairingCandidates& candidates = match_candidates_for_a[ia];
      if (candidates.num_candidates == 0u)
      {
        continue;
      }

      index_t best_pairing_index = candidates.bestPairingIndex();
      FeaturePairing& candidate_pairing = candidates.candidates[best_pairing_index];
      FeaturePairing& best_pairing_for_b = best_matches_for_b[candidate_pairing.other_index];

      if (candidate_pairing < best_pairing_for_b)
      {
        if (best_pairing_for_b.isValid())
        {
          ia_to_assign_again.push_back(best_pairing_for_b.other_index);
        }

        // Note that in best_pairing we store the index of a and in candidate
        // pairing the index of b is stored.
        best_pairing_for_b.other_index = ia;
        best_pairing_for_b.distance = candidate_pairing.distance;

        // This candidate was assigned, so we set its cost to max.
        candidate_pairing.distance = std::numeric_limits<distance_t>::max();

        //! @todo(cfo): PERFORMANCE: Does decreasing candidates.num_candidates
        //!             speed-up the process? However, check that we are still
        //!             looping over all candidates to get the next best pairing.
      }

      //! @todo(cfo): We are just checking the best_pairing_index in the
      //!             candidates and if we fail to match, we discard.
      //!             However, we would get more matches (of questionable
      //!             quality, most likely) if we would also try the other
      //!             candidates.
    }

    ia_to_assign = ia_to_assign_again;
  }

  return best_matches_for_b;
}

} // namespace ze
