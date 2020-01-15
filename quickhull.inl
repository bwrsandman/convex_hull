#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>

namespace algorithm {
using namespace wykobi;
using namespace wykobi::algorithm;

template<typename T>
struct convex_hull_quickhull<point2d<T>>
{
public:
struct debug_data_t
{
  segment2d first_horizontal_split;
  std::vector<triangle2d> split_triangles;
};
template<typename InputIterator, typename OutputIterator>
convex_hull_quickhull(InputIterator begin,
                      InputIterator end,
                      OutputIterator out,
                      debug_data_t& debug)
{
  // If the input is a point, line segment or triangle, then there is no work
  // to do. They will always form a convex set.
  if (std::distance(begin, end) <= 3) {
    std::copy(begin, end, out);
    return;
  }

  std::list<point2d<T>> point_list;

  // Find leftmost and rightmost points and add to convex hull
  const auto [leftmost, rightmost] = std::minmax_element(
      begin, end, [](const point2d<T>& a, const point2d<T>& b) -> bool {
        return a.x < b.x;
      });
  debug.first_horizontal_split = make_segment(*leftmost, *rightmost);
  auto leftmost_it = point_list.insert(
      point_list.end(), make_point<T>(leftmost->x, leftmost->y));
  auto rightmost_it = point_list.insert(
      point_list.end(), make_point<T>(rightmost->x, rightmost->y));

  // Segment divides the remaining (n − 2) points into two groups S1 and S2
  // where S1 are points in the set which are on the right side of the
  // oriented line from leftmost to rightmost
  // and S2 are points in the set which are on the right side of the oriented
  // line from rightmost to leftmost
  std::vector<typename std::iterator_traits<InputIterator>::value_type> s1;
  std::vector<typename std::iterator_traits<InputIterator>::value_type> s2;
  s1.reserve(end - begin - 1);
  s2.reserve(end - begin - 1);
  for (InputIterator it = begin; it != end; ++it) {
    if (it == leftmost || it == rightmost) {
      continue;
    }
    switch (orientation(*leftmost, *rightmost, *it)) {
      case RightHandSide:
        s1.push_back(wykobi::make_point(it->x, it->y));
        break;
      case CollinearOrientation:
      case LeftHandSide:
        s2.push_back(wykobi::make_point(it->x, it->y));
        break;
    }
  }

  // insert with std::next(leftmost_it) which is rightmost_it so that it is
  // inserted after leftmost and before rightmost
  find_hull(s1.cbegin(),
            s1.cend(),
            leftmost,
            rightmost,
            point_list,
            std::next(leftmost_it),
            debug);
  // insert with std::next(leftmost_it) which is point_list.end so that it is
  // inserted after rightmost, which in a loop is before leftmost
  find_hull(s2.cbegin(),
            s2.cend(),
            rightmost,
            leftmost,
            point_list,
            std::next(rightmost_it),
            debug);

  for (const auto& point : point_list) {
    (*out++) = make_point<T>(point.x, point.y);
  }
}

private:
inline T distance2(const T& x1, const T& y1, const T& x2, const T& y2)
{
  const T dx = (x1 - x2);
  const T dy = (y1 - y2);

  return dx * dx + dy * dy;
}

inline T minimum_distance2_from_point_to_segment(const T& px,
                                                 const T& py,
                                                 const T& x1,
                                                 const T& y1,
                                                 const T& x2,
                                                 const T& y2)
{
  T nx = T(0.0);
  T ny = T(0.0);

  closest_point_on_segment_from_point(x1, y1, x2, y2, px, py, nx, ny);

  return distance2(px, py, nx, ny);
}

inline T minimum_distance2_from_point_to_segment(const point2d<T>& point,
                                                 const segment<T, 2>& segment)
{
  return minimum_distance2_from_point_to_segment(
      point.x, point.y, segment[0].x, segment[0].y, segment[1].x, segment[1].y);
}

/// Find points on convex hull from the set Sk of points
/// that are on the right side of the oriented line from P to Q
template<typename InputIterator, typename OutputIterator>
void find_hull(InputIterator begin,
               InputIterator end,
               InputIterator segment_0,
               InputIterator segment_1,
               std::list<point2d<T>> point_list,
               OutputIterator segment_it,
               debug_data_t& debug)
{
  // Sk has no points
  if (begin == end) {
    return;
  }
  // Find the farthest point from segment
  auto farthest = end;
  {
    T greatest_distance2 = T(0);
    for (auto it = begin; it != end; ++it) {
      auto d2 = minimum_distance2_from_point_to_segment(
          *it, make_segment(*segment_0, *segment_1));
      if (d2 > greatest_distance2) {
        greatest_distance2 = d2;
        farthest = it;
      }
    }
  }
  if (farthest == end) {
    return;
  }

  // Add point C to convex hull at the location between P and Q
  auto farthest_it =
      point_list.insert(segment_it, make_point<T>(farthest->x, farthest->y));
  // Three points P, Q, and C partition the remaining points of Sk into 3
  // subsets: S0, S1, and S2 where S0 are points inside triangle PCQ, S1 are
  // points on the right side of the oriented line from P to C, and S2 are
  // points on the right side of the oriented line from C to Q.
  std::vector<typename std::iterator_traits<InputIterator>::value_type> s1;
  std::vector<typename std::iterator_traits<InputIterator>::value_type> s2;
  s1.reserve(end - begin - 1);
  s2.reserve(end - begin - 1);
  for (InputIterator it = begin; it != end; ++it) {
    if (it == segment_0 || it == segment_1 || it == farthest) {
      continue;
    }
    if (orientation(*segment_0, *farthest, *it) == RightHandSide) {
      s1.push_back(wykobi::make_point(it->x, it->y));
    } else if (orientation(*farthest, *segment_1, *it) == RightHandSide) {
      s2.push_back(wykobi::make_point(it->x, it->y));
    }
  }
  debug.split_triangles.push_back(
      make_triangle(*segment_0, *farthest, *segment_1));
  find_hull(s1.cbegin(),
            s1.cend(),
            segment_0,
            farthest,
            point_list,
            farthest_it,
            debug);
  find_hull(s2.cbegin(),
            s2.cend(),
            farthest,
            segment_1,
            point_list,
            std::next(farthest_it),
            debug);
}
};
} // namespace algorithm
