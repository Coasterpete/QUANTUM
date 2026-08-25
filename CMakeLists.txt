cmake_minimum_required(VERSION 3.25)

cmake_policy(SET CMP0091 NEW)

project(
    QUANTUM
    VERSION 0.1.0
    DESCRIPTION "QUANTUM"
    LANGUAGES CXX
)

if(MSVC)
    set(
        CMAKE_MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

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
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# ------------------------------------------------------------
# Project targets
# ------------------------------------------------------------

add_subdirectory(core)
add_subdirectory(engine)
add_subdirectory(editor)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# ------------------------------------------------------------
# Configuration information
# ------------------------------------------------------------

message(STATUS "Vulkan version: ${Vulkan_VERSION}")
message(STATUS "Vulkan include directory: ${Vulkan_INCLUDE_DIRS}")
message(
    STATUS
    "glslangValidator: ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
)
