cmake_minimum_required(VERSION 3.10)
project(BrushSim VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find Open3D package
find_package(Open3D REQUIRED)

# Include directories
include_directories(${Open3D_INCLUDE_DIRS})

# Add executable
add_executable(BrushSim main.cpp ${SOURCE_FILES})

# Link Open3D libraries
target_link_libraries(BrushSim ${Open3D_LIBRARIES})

# Include testing if necessary
include(CTest)
enable_testing()
