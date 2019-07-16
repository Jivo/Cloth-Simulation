################################################################
#                          Created by:                         #
#                         James Miller                         #
################################################################

cmake_minimum_required(VERSION 3.1)

# Include Extra CMake capabilities
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# Assignment Name
set(CGRA_PROJECT "a1" CACHE STRING "CGRA Project Name")

# Project
project("CGRA_PROJECT_${CGRA_PROJECT}" CXX C)

include(CGRACompiler)
include(CGRAFunctions)

# Enable IDE Project Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Force Output Directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Vendored libraries
add_subdirectory(vendor/glew)
include_directories("${PROJECT_SOURCE_DIR}/vendor/glfw/include")
add_subdirectory(vendor/glfw)
add_subdirectory(vendor/glm)
add_subdirectory(vendor/imgui)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Add subdirectories
add_subdirectory(src) # Primary Source Files
add_subdirectory(res) # Resources; for example shaders

set_property(TARGET ${CGRA_PROJECT} PROPERTY FOLDER "CGRA")
