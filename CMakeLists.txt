cmake_minimum_required(VERSION 3.15)
project(quickhull)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/wykobi-0.0.5)

add_executable(quickhull main.cpp)
target_link_libraries(quickhull PRIVATE wykobi)

if (EMSCRIPTEN)
  set_target_properties(quickhull PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  target_link_libraries(quickhull PRIVATE SDL2::SDL2)
endif()
