add_library(doctest_header INTERFACE)
target_include_directories(doctest_header SYSTEM INTERFACE ${doctest_SOURCE_DIR})

add_executable(armor_tests
    test_main.cpp
    test_math.cpp
    test_kernel_grid.cpp
    test_solver.cpp
    test_stage_b.cpp
)
target_link_libraries(armor_tests PRIVATE armor_sim doctest_header)
armor_set_warnings(armor_tests)

add_test(NAME armor_tests COMMAND armor_tests)
