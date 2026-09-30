cmake_minimum_required(VERSION 3.20)
project(hpolytope-cpp LANGUAGES C CXX)

option(HPOLYTOPE_USE_GMP "Use GMP rational arithmetic in cddlib" OFF)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

if (HPOLYTOPE_USE_GMP)
    pkg_check_modules(cddlib REQUIRED IMPORTED_TARGET cddgmp)
else()
    pkg_check_modules(cddlib REQUIRED IMPORTED_TARGET cddlib)
endif()

add_library(hpolytope
    src/hpolytope.cpp
    src/cddlib/adapter.cpp
    src/cddlib/runtime.cpp
    src/cddlib/linear_programming.cpp
)

add_library(hpolytope::hpolytope ALIAS hpolytope)

target_compile_features(hpolytope PUBLIC cxx_std_20)

target_include_directories(hpolytope
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(hpolytope
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        PkgConfig::cddlib
)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
