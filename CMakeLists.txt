cmake_minimum_required(VERSION 3.12)
project(pysee)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

# nlohmann_json

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/3rdPatry)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/see_core/inc)
include_directories(${CMAKE_SOURCE_DIR}/see_common/inc)

# Add source files
set(SOURCE_FILES
    SEE.cpp
    SEE.h
    python_bindings.cpp
    see_core/src/see_core.cpp
)

# Add the pybind11 module
#pybind11_add_module(pysee ${SOURCE_FILES})

# Link libraries
target_link_libraries(pysee PRIVATE 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Set output directory
set_target_properties(pysee PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/pysee
)

# Copy Python package files (if they exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pysee/__init__.py")
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/pysee/__init__.py
         DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/pysee)
endif()

# Install rules
install(TARGETS pysee DESTINATION .)
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/pysee/__init__.py")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pysee/__init__.py DESTINATION .)
endif()

# Print debug information
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Project source dir: ${CMAKE_SOURCE_DIR}")