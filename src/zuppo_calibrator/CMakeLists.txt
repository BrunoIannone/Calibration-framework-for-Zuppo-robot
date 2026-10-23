cmake_minimum_required(VERSION 3.16)
project(dmap_localizer CXX)

# ============================================================
# C++ Standard
# ============================================================

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()


# ============================================================
# Project Compiler Options
# Applied ONLY to project-owned targets
# ============================================================

set(PROJECT_COMPILE_OPTIONS
    -fno-omit-frame-pointer
    -g
)

set(PROJECT_WARNING_OPTIONS
    -Wall
    -Wextra
    -Wpedantic
)


# ============================================================
# External Dependencies
# ============================================================
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# FetchContent - Third Party
# ============================================================

include(FetchContent)

FetchContent_Declare(
    unordered_dense
    GIT_REPOSITORY https://github.com/martinus/unordered_dense.git
    GIT_TAG v4.4.0
)

FetchContent_Declare(
    polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    GIT_TAG v2.6.1
    SYSTEM
)

FetchContent_MakeAvailable(unordered_dense polyscope)


# ============================================================
# ImPlot - Third Party
# ============================================================

add_library(implot STATIC
    ${polyscope_SOURCE_DIR}/deps/imgui/implot/implot.cpp
    ${polyscope_SOURCE_DIR}/deps/imgui/implot/implot_items.cpp
)

# Third-party headers: don't warn about them
target_include_directories(implot SYSTEM PUBLIC
    ${polyscope_SOURCE_DIR}/deps/imgui/implot
    ${polyscope_SOURCE_DIR}/deps/imgui/imgui
)

target_link_libraries(implot PUBLIC
    polyscope
)

# Do NOT apply project warnings to ImPlot
# If another dependency propagates warnings to it,
# explicitly suppress them here.
target_compile_options(implot PRIVATE
    -w
)


# ============================================================
# Project Library: helpers
# ============================================================

add_library(helpers
    src/helpers.cpp
)

target_include_directories(helpers
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
)

target_link_libraries(helpers
    PRIVATE
    yaml-cpp
    unordered_dense::unordered_dense
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

target_compile_options(helpers PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(helpers PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
    <yaml-cpp/yaml.h>
)


# ============================================================
# Project Library: draw_primitives
# ============================================================

add_library(draw_primitives
     src/draw_primitives.cpp
)

target_include_directories(draw_primitives
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include

    SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(draw_primitives
    PUBLIC
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

target_compile_options(draw_primitives PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(draw_primitives PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
)

# ============================================================
# Project Library: plot
# ============================================================

add_library(plot
     src/plot.cpp
)

target_include_directories(plot
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include

    SYSTEM STATIC
    ${polyscope_SOURCE_DIR}/deps/imgui/implot
    ${polyscope_SOURCE_DIR}/deps/imgui/imgui

    SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(plot
    PUBLIC
    Eigen3::Eigen
    OpenGL::GL
    glfw
    polyscope
    ${OpenCV_LIBRARIES}


)

target_compile_options(plot PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(plot PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
)


# ============================================================
# Project Library: grids
# ============================================================

add_library(grids
     src/grid_map.cpp
     src/distance_map.cpp
)

target_include_directories(grids
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include

    SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(grids
    PUBLIC
    draw_primitives
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
)

target_compile_options(grids PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(grids PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
)


# ============================================================
# Project Library: packet_assembler
# ============================================================

add_library(packet_assembler
     src/packet_assembler.cpp
)

target_include_directories(packet_assembler
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

target_link_libraries(packet_assembler
    PUBLIC
    Eigen3::Eigen
)

target_compile_options(packet_assembler PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(packet_assembler PRIVATE
    <Eigen/Dense>
)


# ============================================================
# Project Library: localizer
# ============================================================

add_library(localizer
     src/dmap_localizer.cpp
)

target_include_directories(localizer
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include

    SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(localizer
    PUBLIC
    draw_primitives
    Eigen3::Eigen
)

target_compile_options(localizer PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(localizer PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
)


# ============================================================
# Project Library: Calibrator
# ============================================================

add_library(calibrator
     src/hlidar_calibrator.cpp   src/slidar_calibrator.cpp
)

target_include_directories(calibrator
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    
)

target_link_libraries(calibrator
    PUBLIC
    Eigen3::Eigen
    grids
    localizer
)

target_compile_options(calibrator PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(calibrator PRIVATE
    <Eigen/Dense>
)


# ============================================================
# Main Executable
# ============================================================

add_executable(zuppo_calibrator
     src/main.cpp  src/display_utils.cpp  src/hpipeline.cpp  src/spipeline.cpp
)

target_include_directories(zuppo_calibrator
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include

    SYSTEM PRIVATE
    ${polyscope_SOURCE_DIR}/deps/imgui/implot
    ${polyscope_SOURCE_DIR}/deps/imgui
)

target_link_libraries(zuppo_calibrator
    PRIVATE
    draw_primitives
    plot
    grids
    localizer
    calibrator
    packet_assembler
    polyscope
    glfw
    yaml-cpp
    Eigen3::Eigen
    unordered_dense::unordered_dense
    OpenGL::GL
    helpers
)

target_compile_options(zuppo_calibrator PRIVATE
    ${PROJECT_COMPILE_OPTIONS}
    ${PROJECT_WARNING_OPTIONS}
)

target_precompile_headers(zuppo_calibrator PRIVATE
    <Eigen/Dense>
    <opencv2/opencv.hpp>
)