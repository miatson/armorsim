# File input/output (Stage D): scenario files (JSON) and probe time histories
# (CSV). Kept apart from the solver, which stays dependency-free.
add_library(armor_io STATIC
    src/ScenarioFile.cpp
    src/Probes.cpp
)
target_include_directories(armor_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(armor_io PUBLIC armor_sim PRIVATE nlohmann_json::nlohmann_json)
armor_set_warnings(armor_io)
