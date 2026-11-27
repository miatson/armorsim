cmake_minimum_required(VERSION 3.25)

# C is enabled because raylib is written in C.
project(miataphysim VERSION 0.4.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ARMOR_BUILD_APP "Build the raylib + Dear ImGui application" ON)
option(ARMOR_BUILD_TESTS "Build the unit tests" ON)
option(ARMOR_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
# Compile the source tree's scenarios/ and library/ folders into the app as a
# fallback location. Handy while developing; switch OFF for builds you give to
# others (it would embed this machine's folder path in the executable), and
# ship the two folders next to the .exe instead.
option(MIATAPHYSIM_SOURCE_PATHS "Embed source-tree scenario/library paths in the app" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Strict warnings for OUR targets only (third-party code is left alone).
function(armor_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
        if(ARMOR_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
        if(ARMOR_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

include(FetchContent)
find_package(Threads REQUIRED)

# ---- maths + solver: standard library only ----
add_subdirectory(core)
add_subdirectory(sim)

# ---- file I/O (Stage D): scenario files (JSON) and probe CSV ----
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM)
FetchContent_MakeAvailable(nlohmann_json)
add_subdirectory(io)

# ---- application: raylib (window/drawing) + Dear ImGui (UI widgets) ----
if(ARMOR_BUILD_APP)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM)
    # Dear ImGui and rlImGui (the raylib backend) have no usable CMake files,
    # so we only download them (SOURCE_SUBDIR points nowhere) and compile
    # their sources ourselves below. rlImGui has no releases: pinned to a commit.
    FetchContent_Declare(imgui
        URL https://github.com/ocornut/imgui/archive/refs/tags/v1.93.1.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR no-cmake)
    FetchContent_Declare(rlimgui
        URL https://github.com/raylib-extras/rlImGui/archive/5c17e70e98ae9f50ca94bdcad0e3b3536b46a203.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR no-cmake)
    FetchContent_MakeAvailable(raylib imgui rlimgui)

    add_library(imgui_rl STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
    target_include_directories(imgui_rl SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${rlimgui_SOURCE_DIR})
    target_link_libraries(imgui_rl PUBLIC raylib)

    add_subdirectory(app)
endif()

# ---- tests ----
if(ARMOR_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(doctest
        URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR no-cmake)
    FetchContent_MakeAvailable(doctest)
    add_subdirectory(tests)
endif()
