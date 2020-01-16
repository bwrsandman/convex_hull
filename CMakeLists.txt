cmake_minimum_required(VERSION 3.15)
project(convex_hull)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/wykobi-0.0.5)
add_subdirectory(third_party/imgui-1.74)

add_executable(convex_hull main.cpp)
target_link_libraries(convex_hull PRIVATE wykobi imgui)

if (EMSCRIPTEN)
  set_target_properties(convex_hull PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  target_link_libraries(convex_hull PRIVATE SDL2::SDL2)
endif()

add_subdirectory(benchmarks)
