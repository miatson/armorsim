# The user interface: raylib draws the particles, Dear ImGui draws the panels.
add_executable(miataphysim
    src/main.cpp
    src/App.cpp
    src/Presets.cpp
    src/Headless.cpp
)
target_include_directories(miataphysim PRIVATE src)
# Fallback location of the example scenario files and library (see Presets.cpp).
if(MIATAPHYSIM_SOURCE_PATHS)
    target_compile_definitions(miataphysim PRIVATE ARMOR_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
        ARMOR_LIBRARY_DIR="${CMAKE_SOURCE_DIR}/library")
endif()
target_compile_definitions(miataphysim PRIVATE MIATAPHYSIM_VERSION="${PROJECT_VERSION}")
target_link_libraries(miataphysim PRIVATE armor_sim armor_io imgui_rl raylib)
armor_set_warnings(miataphysim)

if(MSVC)
    set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT miataphysim)
endif()
