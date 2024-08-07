cmake_minimum_required(VERSION 3.10)
project(BrushSim VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Open3D package
find_package(Open3D REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

#disables the opencv dependency on <unsupported/Eigen/CXX11/Tensor>
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

include_directories(${OpenCV_INCLUDE_DIRS})

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
    ${OpenCV_INCLUDE_DIRS}
)

# Link Open3D to your target
target_link_libraries(BrushSim PRIVATE Open3D::Open3D)

# Include Open3D headers
target_include_directories(BrushSim PRIVATE ${Open3D_INCLUDE_DIRS})

target_link_libraries(BrushSim PRIVATE ${OpenCV_LIBS})

# Include testing if necessary
include(CTest)
enable_testing()
