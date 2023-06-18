cmake_minimum_required(VERSION 3.0)

project(pointcloud_to_image)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find Eigen3 package
find_package(Eigen3 REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Include Eigen3 headers
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find PCL package
find_package(PCL  REQUIRED COMPONENTS common visualization)
# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)

# Add the include directories for the PCL headers
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Set source files
# set(
#     SOURCES
#     src/main.cc
#     include/pointcloud_to_image.cc
#     include/pointcloud_to_image.hpp
# )


# Create executable target
add_executable(main src/main.cc src/yaml_parser.hpp)

# add_executable(pointcloud_to_image 
#     src/poiintcloud_to_image.cc
#     include/pointcloud_to_image.hpp
# )
# Link with OpenCV PCL_LIBRARIES Eigen3  libraries
target_link_libraries(main ${OpenCV_LIBS} Eigen3::Eigen  ${PCL_LIBRARIES} yaml-cpp)

