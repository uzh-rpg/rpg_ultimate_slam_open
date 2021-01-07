#include <imp/feature_matching/matching_types.hpp>

namespace ze {

IndexPairList indexPairListFromPairingList(const PairingList& best_matches_for_b)
{
  IndexPairList matches_a_b;
  matches_a_b.reserve(best_matches_for_b.size());
  for (size_t i = 0u; i < best_matches_for_b.size(); ++i)
  {
    if (best_matches_for_b[i].isValid())
    {
      matches_a_b.emplace_back(std::make_pair(best_matches_for_b[i].other_index, i));
    }
  }
  return matches_a_b;
}

} // namespace ze
