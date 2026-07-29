cmake_minimum_required(VERSION 3.25)

project(
    QuantumCoasterEngine
    VERSION 0.1.0
    DESCRIPTION "QUANTUM SplineWorks Engine"
    LANGUAGES CXX
)

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

find_package(glfw3 CONFIG REQUIRED)

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