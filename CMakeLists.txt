cmake_minimum_required(VERSION 3.20)
project(hpolytope-cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(HPOLYTOPE_USE_GMP "Use GMP rational arithmetic in cddlib" OFF)

find_package(Eigen3 REQUIRED)
find_package(highs CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)

if (HPOLYTOPE_USE_GMP)
    pkg_check_modules(cddlib REQUIRED IMPORTED_TARGET cddgmp)
else()
    pkg_check_modules(cddlib REQUIRED IMPORTED_TARGET cddlib)
endif()

add_executable(
    ${PROJECT_NAME}
    main.cpp
    src/hpolytope.cpp
    src/cddlib/adapter.cpp
    src/cddlib/runtime.cpp
    src/cddlib/linear_programming.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    highs::highs
    Eigen3::Eigen
    PkgConfig::cddlib
)

