find_package(benchmark QUIET COMPONENTS benchmark_main)

if (benchmark_FOUND)
  add_executable(convex_hull_benchmark convex_hull_benchmark.cpp)
  target_link_libraries(convex_hull_benchmark PRIVATE benchmark::benchmark_main wykobi)
endif()
