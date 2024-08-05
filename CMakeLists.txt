cmake_minimum_required(VERSION 3.10)
project(BrushSim VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Open3D package
find_package(Open3D REQUIRED)

# Gather all source files
file(GLOB_RECURSE SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# Add executable
add_executable(BrushSim
    ${PROJECT_SOURCE_DIR}/main.cpp
    ${SOURCE_FILES}
)

# Specify include directories for BrushSim
target_include_directories(BrushSim PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external/json/single_include/nlohmann
    ${Open3D_INCLUDE_DIRS}
)

# Include testing if necessary
include(CTest)
enable_testing()
