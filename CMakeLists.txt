cmake_minimum_required(VERSION 3.25)

cmake_policy(SET CMP0091 NEW)

project(
    QuantumCoasterEngine
    VERSION 0.1.0
    DESCRIPTION "QUANTUM SplineWorks Engine"
    LANGUAGES CXX
)

if(MSVC)
    set(
        CMAKE_MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------

find_package(
    Vulkan
    REQUIRED
    COMPONENTS
        glslangValidator
)

find_package(SDL3 CONFIG REQUIRED)

# ------------------------------------------------------------
# Project targets
# ------------------------------------------------------------

add_subdirectory(engine)
add_subdirectory(editor)

# ------------------------------------------------------------
# Configuration information
# ------------------------------------------------------------

message(STATUS "Vulkan version: ${Vulkan_VERSION}")
message(STATUS "Vulkan include directory: ${Vulkan_INCLUDE_DIRS}")
message(
    STATUS
    "glslangValidator: ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
)