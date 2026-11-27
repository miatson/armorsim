# Header-only maths library.
add_library(armor_core INTERFACE)
target_include_directories(armor_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(armor_core INTERFACE cxx_std_20)

# Determinism: no floating-point shortcuts that change results between builds.
if(MSVC)
    target_compile_options(armor_core INTERFACE /fp:precise)
else()
    target_compile_options(armor_core INTERFACE -ffp-contract=off)
endif()
