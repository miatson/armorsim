# The user interface: raylib draws the particles, Dear ImGui draws the panels.
add_executable(armorsim
    src/main.cpp
    src/App.cpp
    src/Presets.cpp
    src/Headless.cpp
)
target_include_directories(armorsim PRIVATE src)
# Fallback location of the example scenario files (see Presets.cpp).
target_compile_definitions(armorsim PRIVATE ARMOR_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios")
target_link_libraries(armorsim PRIVATE armor_sim armor_io imgui_rl raylib)
armor_set_warnings(armorsim)

if(MSVC)
    set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT armorsim)
endif()
