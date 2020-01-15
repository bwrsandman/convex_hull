#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <wykobi.hpp>

namespace algorithm {
using namespace wykobi;

template<typename T>
struct convex_hull_quick_hull;

template <typename T> struct convex_hull_quick_hull;
template <typename T> struct convex_hull_quick_hull< point2d<T> >;

} // namespace algorithm

#include "quick_hull.inl"

#endif // ALGORITHM_H_
