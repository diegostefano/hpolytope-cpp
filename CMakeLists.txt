cmake_minimum_required(VERSION 3.20)
project(hpolytope-cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  highs
  GIT_REPOSITORY https://github.com/ERGO-Code/HiGHS/
  GIT_TAG v1.13.1
)
FetchContent_MakeAvailable(highs)

find_package(Eigen3 REQUIRED)

include_directories(./include)
add_executable(${PROJECT_NAME} main.cpp src/solver/linear_program.cpp src/solver/highs_solver.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE highs)
