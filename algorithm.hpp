#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <wykobi.hpp>

namespace algorithm {
using namespace wykobi;

template <typename T> struct convex_hull_quickhull;
template <typename T> struct convex_hull_quickhull<point2d<T> >;

template<typename T>
struct convex_hull_chans_algorithm;
template<typename T>
struct convex_hull_chans_algorithm<point2d<T>>;

} // namespace algorithm

#include "chans_algorithm.inl"
#include "quickhull.inl"

#endif // ALGORITHM_H_
