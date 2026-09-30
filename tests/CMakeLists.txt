find_package(GTest REQUIRED CONFIG)

add_executable(hpolytope_test
    construction_test.cpp
    operations_test.cpp
)

target_link_libraries(hpolytope_test
    PRIVATE
        hpolytope::hpolytope
        GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(hpolytope_test
    PROPERTIES
        LABELS unit
)
