#pragma once

#include <imp/core/image.hpp>
#include <imp/feature_matching/matching_types.hpp>

namespace ze {

void drawMatches(const Image8uC1& img_a,
                 const Image8uC1& img_b,
                 const Keypoints& px_vec_a,
                 const Keypoints& px_vec_b,
                 const IndexPairList& matches_a_b,
                 const int sleep = 0);

} // namespace ze
