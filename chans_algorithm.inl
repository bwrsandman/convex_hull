#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>

namespace algorithm {
using namespace wykobi;
using namespace wykobi::algorithm;

template<typename T>
struct convex_hull_chans_algorithm<point2d<T>> {
 public:
  struct debug_data_t
  {
    point2d<T> first_point;
    std::vector<std::vector<std::vector<point2d<T>>>> intermediate_subsets;
    std::vector<uint32_t> intermediate_m;
    std::vector<std::vector<polygon<T, 2>>> intermediate_hulls;
    std::vector<std::vector<std::vector<point2d<T>>>> intermediate_found_hull_points;
    std::vector<std::vector<point2d<T>>> intermediate_found_points;
  };

  template<typename InputIterator, typename OutputIterator>
  convex_hull_chans_algorithm(InputIterator begin,
                              InputIterator end,
                              OutputIterator out,
                              debug_data_t& debug) {
    uint32_t n = std::distance(begin, end);

    // If the input is a point, line segment or triangle, then there is no work
    // to do. They will always form a convex set.
    if (n <= 3) {
      std::copy(begin, end, out);
      return;
    }

    // Pick a point which is guaranteed to be on the convex hull
    auto lowest_point = std::min_element(
        begin, end, [](const point2d<T> &a, const point2d<T> &b) -> bool {
          return a.y < b.y;
        });  // O(n) time
    debug.first_point = *lowest_point;

    // This point is used for the Javis march but is not part of the point set
    auto p0 = make_point(-std::numeric_limits<T>::infinity(), T(0));

    // Unlike source on wikipedia, I had to add 1 to the number of iterations
    auto iterations = std::log(std::log(n)) + 1;
    for (uint32_t iteration = 0; iteration < iterations; ++iteration) {
      // In order to find the number of points on the final convex hull,
      // we try to estimate it
      uint32_t output_size_estimate = std::pow(2, std::pow(2, iteration + 1));
      debug.intermediate_m.push_back(output_size_estimate);

      std::vector<point2d<T>> hull;
      hull.reserve(output_size_estimate);
      hull.push_back(*lowest_point);

      // Split the set into ceil(n/m) subsets
      std::vector<std::vector<point2d<T>>> subsets;
      subsets.resize(std::ceil(static_cast<T>(n) / output_size_estimate));
      {
        auto it = begin;
        for (auto& subset: subsets) {
          subset.reserve(output_size_estimate);
          for (uint32_t i = 0; i < output_size_estimate && it != end; ++i, ++it) {
            subset.push_back(*it);
          }
        }
      }
      debug.intermediate_subsets.push_back(subsets);

      // Compute a convex hull for each subset using graham scan
      std::vector<polygon<T, 2>> hulls;
      hulls.reserve(subsets.size());
      for (auto& subset : subsets) {
        auto& new_hull = hulls.emplace_back();
        convex_hull_graham_scan<point2d<T>>(subset.cbegin(), subset.cend(), std::back_inserter(new_hull));
      }
      debug.intermediate_hulls.push_back(hulls);
      auto& debug_intermediate_found_hull_points = debug.intermediate_found_hull_points.emplace_back();
      auto& debug_intermediate_found_point = debug.intermediate_found_points.emplace_back();

      // Use a modified version of the Jarvis march algorithm to compute the convex hull of the set
      point2d<T> p_i_m_1 = p0;
      point2d<T> p_i = *lowest_point;
      for (uint32_t i = 0; i < output_size_estimate; ++i) {
        // Use method from jarvis march to find the points in each hull
        // which form the largest angle between p_{i-1}, p_i and the point with
        // p_i being the connecting point and p_0 being at -infinity in x
        // (180 degrees) and p_1 being the lowest point in the set.
        std::vector<point2d<T>> jarvis_found_points;
        jarvis_found_points.reserve(hulls.size());
        for (uint32_t k = 0; k < hulls.size(); ++k) {
          jarvis_found_points.push_back(jarvis_binary_search(p_i_m_1, p_i, hulls[k].begin(), hulls[k].end()));
        }
        debug_intermediate_found_hull_points.push_back(jarvis_found_points);
        // From the found points from each full, evaluate which of them has
        // the largest angle again between between p_{i-1}, p_i and the point
        point2d<T> p_i_p_1 = jarvis_binary_search(p_i_m_1, p_i, jarvis_found_points.cbegin(), jarvis_found_points.cend());
        debug_intermediate_found_point.push_back(p_i_p_1);
        if (p_i_p_1 == *lowest_point) {
          for (auto& p : hull) {
            (*out++) = p;
          }
          return;
        }
        hull.push_back(p_i_p_1);
        p_i_m_1 = p_i;
        p_i = p_i_p_1;
      }
    }
  }

 private:
  /// Finds the point p in \ref convex_hull such that the angle between \ref a \ref b p is maximized
  template<typename InputIterator>
  point2d<T> jarvis_binary_search(const point2d<T>& a, const point2d<T>& b, InputIterator begin, InputIterator end) {
    point2d<T> current_point = a;
    for (auto it = begin; it != end; ++it)
    {
      if (orientation(b, current_point, *it) == RightHandSide)
      {
        current_point = *it;
      }
    }
    return current_point;
  }

};
} // namespace algorithm
