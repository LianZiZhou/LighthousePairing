cmake_minimum_required(VERSION 3.25)
project(LighthousePairing VERSION 0.4.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(TP ${CMAKE_SOURCE_DIR}/third_party)
foreach(dep imgui/imgui.cpp implot/implot.cpp freetype/CMakeLists.txt)
    if(NOT EXISTS ${TP}/${dep})
        message(FATAL_ERROR "third_party/${dep} missing - run: git submodule update --init")
    endif()
endforeach()

# Distribution build (build.ps1 -Dist): whole-program optimisation, smaller binary, no PDB.
option(LHP_DIST "Optimised single-exe distribution build" OFF)
if(LHP_DIST)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    add_compile_options(/O2 /Gy /Gw)
    add_link_options(/OPT:REF /OPT:ICF /INCREMENTAL:NO)
endif()

# ---------------------------------------------------------------- FreeType (static, no optional deps)
set(FT_DISABLE_ZLIB ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BZIP2 ON CACHE BOOL "" FORCE)
set(FT_DISABLE_PNG ON CACHE BOOL "" FORCE)
set(FT_DISABLE_HARFBUZZ ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BROTLI ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(${TP}/freetype ${CMAKE_BINARY_DIR}/freetype EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------- Dear ImGui (docking) + ImPlot
add_library(imgui STATIC
    ${TP}/imgui/imgui.cpp
    ${TP}/imgui/imgui_draw.cpp
    ${TP}/imgui/imgui_tables.cpp
    ${TP}/imgui/imgui_widgets.cpp
    ${TP}/imgui/imgui_demo.cpp
    ${TP}/imgui/backends/imgui_impl_win32.cpp
    ${TP}/imgui/backends/imgui_impl_dx11.cpp
    ${TP}/imgui/misc/freetype/imgui_freetype.cpp
    ${TP}/implot/implot.cpp
    ${TP}/implot/implot_items.cpp
    ${TP}/implot/implot_demo.cpp
)
target_include_directories(imgui PUBLIC ${TP}/imgui ${TP}/imgui/backends ${TP}/imgui/misc/freetype ${TP}/implot)
target_compile_definitions(imgui PUBLIC IMGUI_ENABLE_FREETYPE IMGUI_DISABLE_OBSOLETE_FUNCTIONS NOMINMAX WIN32_LEAN_AND_MEAN UNICODE _UNICODE)
target_link_libraries(imgui PUBLIC freetype d3d11 dxgi d3dcompiler)

# ---------------------------------------------------------------- core library (no UI) – also used by tests
add_library(lhp_core STATIC
    src/util/Text.cpp
    src/backend/ConsoleParser.cpp
    src/backend/DeviceConfig.cpp
    src/backend/ConsoleSession.cpp
    src/backend/UsbTopology.cpp
    src/backend/SystemInfo.cpp
    src/core/HistoryDb.cpp
    src/core/RealService.cpp
    src/core/DemoService.cpp
    src/core/Store.cpp
)
target_include_directories(lhp_core PUBLIC src ${TP})
target_compile_definitions(lhp_core PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN UNICODE _UNICODE)
target_link_libraries(lhp_core PUBLIC setupapi cfgmgr32 shell32 ole32 dwmapi)

# ---------------------------------------------------------------- application
option(LHP_CORE_ONLY "Build only the core library and tests" OFF)
if(NOT LHP_CORE_ONLY)
add_executable(LighthousePairing WIN32
    src/main.cpp
    src/platform/Window.cpp
    src/ui/App.cpp
    src/ui/Theme.cpp
    src/ui/Widgets.cpp
    src/ui/Views.cpp
    src/ui/Dialogs.cpp
    src/app.rc
)
target_link_libraries(LighthousePairing PRIVATE lhp_core imgui dcomp)
target_compile_options(LighthousePairing PRIVATE /W4 /permissive- /utf-8 /EHsc /Zc:__cplusplus /wd4100 /wd4505)
endif()

# ---------------------------------------------------------------- tests
add_executable(lhp_tests tests/ParserTests.cpp tests/StoreTests.cpp tests/main.cpp)
target_include_directories(lhp_tests PRIVATE ${TP}/doctest)
target_compile_definitions(lhp_tests PRIVATE LHP_TRANSCRIPTS="${CMAKE_SOURCE_DIR}/tests/transcripts")
target_link_libraries(lhp_tests PRIVATE lhp_core)

foreach(t lhp_core lhp_tests)
    target_compile_options(${t} PRIVATE /W4 /permissive- /utf-8 /EHsc /Zc:__cplusplus /wd4100 /wd4505)
endforeach()
target_compile_options(imgui PRIVATE /utf-8 /W0)
