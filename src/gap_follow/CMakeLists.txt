cmake_minimum_required(VERSION 3.8)
project(gap_follow)

set(gap_follow_MAJOR_VERSION 0)
set(gap_follow_MINOR_VERSION 0)
set(gap_follow_PATCH_VERSION 0)
set(gap_follow_VERSION
  ${gap_follow_MAJOR_VERSION}.${gap_follow_MINOR_VERSION}.${gap_follow_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
