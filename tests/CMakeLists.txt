add_library(doctest_header INTERFACE)
target_include_directories(doctest_header SYSTEM INTERFACE ${doctest_SOURCE_DIR})

add_executable(armor_tests
    test_main.cpp
    test_math.cpp
    test_kernel_grid.cpp
    test_solver.cpp
    test_stage_b.cpp
    test_stage_d.cpp
    test_stage_e.cpp
)
target_link_libraries(armor_tests PRIVATE armor_sim armor_io doctest_header)
# Example scenario files are checked by the tests.
target_compile_definitions(armor_tests PRIVATE ARMOR_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
    ARMOR_LIBRARY_DIR="${CMAKE_SOURCE_DIR}/library")
armor_set_warnings(armor_tests)

add_test(NAME armor_tests COMMAND armor_tests)
