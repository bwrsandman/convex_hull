include(FetchContent)

FetchContent_Declare(
  wykobi
  URL http://www.wykobi.com/downloads/wykobi.tgz
  URL_HASH SHA512=e4b6e963cf8338d3a6877f4e9d1927ed4be5df7489116aa93d5f91d8ac8dc32bf6ba8e073421497586a325059d612960a351e48978ed2920c6a4dec3873b6122
  PATCH_COMMAND patch -p1 -i "${CMAKE_CURRENT_SOURCE_DIR}/random_shuffle.patch"
)

FetchContent_MakeAvailable(wykobi)
add_library(wykobi INTERFACE)
target_include_directories(wykobi INTERFACE ${wykobi_SOURCE_DIR})
