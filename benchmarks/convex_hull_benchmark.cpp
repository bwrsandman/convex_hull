#include <benchmark/benchmark.h>
#include "../algorithm.hpp"

class UniformRandomBox : public ::benchmark::Fixture
{
 protected:
  void SetUp(::benchmark::State& state) override
  {
    ::benchmark::Fixture::SetUp(state);
    point_list.clear();
    point_list.reserve(state.range(0));
    auto rectangle = wykobi::make_rectangle<double>(0.0, 0.0, 1.0, 1.0);
    wykobi::generate_random_points<double>(
        rectangle, state.range(0), std::back_inserter(point_list));
  }

  void TearDown(::benchmark::State& state) override
  {
    state.SetComplexityN(state.range(0));
    ::benchmark::Fixture::TearDown(state);
  }

  std::vector<wykobi::point2d<double>> point_list;
};

BENCHMARK_DEFINE_F(UniformRandomBox, GrahamScan)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  for (auto _ : state) {
    wykobi::algorithm::convex_hull_graham_scan<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
  }
}

BENCHMARK_DEFINE_F(UniformRandomBox, JarvisMarch)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  for (auto _ : state) {
    wykobi::algorithm::convex_hull_jarvis_march<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
  }
}

BENCHMARK_DEFINE_F(UniformRandomBox, QuickHull)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  typename algorithm::convex_hull_quickhull<wykobi::point2d<double>>::debug_data_t
      debug;

  for (auto _ : state) {
    algorithm::convex_hull_quickhull<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull), debug);
  }
}

BENCHMARK_DEFINE_F(UniformRandomBox, ChansAlgorithm)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  typename algorithm::convex_hull_chans_algorithm<wykobi::point2d<double>>::debug_data_t
      debug;

  for (auto _ : state) {
    algorithm::convex_hull_chans_algorithm<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull), debug);
  }
}

BENCHMARK_REGISTER_F(UniformRandomBox, GrahamScan)->RangeMultiplier(2)->Range(1<<10, 1<<18);
BENCHMARK_REGISTER_F(UniformRandomBox, JarvisMarch)->RangeMultiplier(2)->Range(1<<10, 1<<18);
BENCHMARK_REGISTER_F(UniformRandomBox, QuickHull)->RangeMultiplier(2)->Range(1<<10, 1<<18);
BENCHMARK_REGISTER_F(UniformRandomBox, ChansAlgorithm)->RangeMultiplier(2)->Range(1<<10, 1<<18);

class UniformRandomBoxBorder : public ::benchmark::Fixture
{
 protected:
  void SetUp(::benchmark::State& state) override
  {
    ::benchmark::Fixture::SetUp(state);
    point_list.clear();
    point_list.reserve(state.range(0));
    auto left = wykobi::make_segment<double>(0, 0, 0, 1);
    auto right = wykobi::make_segment<double>(1, 0, 1, 1);
    auto top = wykobi::make_segment<double>(0, 1, 1, 1);
    auto bottom = wykobi::make_segment<double>(0, 0, 1, 0);
    wykobi::generate_random_points<double>(
        left, state.range(0) / 4, std::back_inserter(point_list));
    wykobi::generate_random_points<double>(
        right, state.range(0) / 4, std::back_inserter(point_list));
    wykobi::generate_random_points<double>(
        top, state.range(0) / 4, std::back_inserter(point_list));
    wykobi::generate_random_points<double>(
        bottom, state.range(0) / 4, std::back_inserter(point_list));
  }

  void TearDown(::benchmark::State& state) override
  {
    state.SetComplexityN(state.range(0));
    ::benchmark::Fixture::TearDown(state);
  }

  std::vector<wykobi::point2d<double>> point_list;
};

BENCHMARK_DEFINE_F(UniformRandomBoxBorder, GrahamScan)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  for (auto _ : state) {
    wykobi::algorithm::convex_hull_graham_scan<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
  }
}

BENCHMARK_DEFINE_F(UniformRandomBoxBorder, JarvisMarch)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  for (auto _ : state) {
    wykobi::algorithm::convex_hull_jarvis_march<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull));
  }
}

BENCHMARK_DEFINE_F(UniformRandomBoxBorder, QuickHull)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  typename algorithm::convex_hull_quickhull<wykobi::point2d<double>>::debug_data_t
      debug;

  for (auto _ : state) {
    algorithm::convex_hull_quickhull<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull), debug);
  }
}

BENCHMARK_DEFINE_F(UniformRandomBoxBorder, ChansAlgorithm)(benchmark::State& state)
{
  wykobi::polygon<double, 2> convex_hull;
  typename algorithm::convex_hull_chans_algorithm<wykobi::point2d<double>>::debug_data_t
      debug;

  for (auto _ : state) {
    algorithm::convex_hull_chans_algorithm<wykobi::point2d<double>>(
        point_list.cbegin(), point_list.cend(), std::back_inserter(convex_hull), debug);
  }
}

BENCHMARK_REGISTER_F(UniformRandomBoxBorder, GrahamScan)->RangeMultiplier(2)->Range(1<<10, 1<<18);
BENCHMARK_REGISTER_F(UniformRandomBoxBorder, JarvisMarch)->RangeMultiplier(2)->Range(1<<10, 1<<18);
BENCHMARK_REGISTER_F(UniformRandomBoxBorder, QuickHull)->RangeMultiplier(2)->Range(1<<10, 1<<16);
BENCHMARK_REGISTER_F(UniformRandomBoxBorder, ChansAlgorithm)->RangeMultiplier(2)->Range(1<<10, 1<<18);
