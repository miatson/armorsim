# The SPH solver library. No rendering, no UI - standard library + threads only.
add_library(armor_sim STATIC
    src/Material.cpp
    src/Scenario.cpp
    src/NeighborGrid.cpp
    src/Solver.cpp
)
target_include_directories(armor_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(armor_sim PUBLIC armor_core Threads::Threads)
armor_set_warnings(armor_sim)
