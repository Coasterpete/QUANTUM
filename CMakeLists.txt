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
# Component options
#
# QuantumCore is the portable simulation core and has no
# windowing/rendering dependencies. QuantumEngine adds the
# Vulkan/SDL3 renderer and audio. QUANTUM (editor) builds on
# top of the engine. The editor implies the engine.
# ------------------------------------------------------------

option(QUANTUM_ENABLE_ENGINE "Build the QuantumEngine renderer/audio target" ON)
option(QUANTUM_ENABLE_EDITOR "Build the QUANTUM editor application" ON)

if(QUANTUM_ENABLE_EDITOR AND NOT QUANTUM_ENABLE_ENGINE)
    set(QUANTUM_ENABLE_ENGINE ON)
endif()

# ------------------------------------------------------------
# Asset staging layout
#
# Multi-config generators (Visual Studio) place each runtime in a
# per-config subdirectory, so staged editor assets follow ../$<CONFIG>.
# Single-config generators (Ninja/Make) place the runtime directly in
# the binary directory, so assets must be staged a level up.
# ------------------------------------------------------------

if(CMAKE_CONFIGURATION_TYPES)
    set(QUANTUM_ASSET_CONFIG_SUBDIR "/$<CONFIG>")
else()
    set(QUANTUM_ASSET_CONFIG_SUBDIR "")
endif()

# ------------------------------------------------------------
# Dependencies
#
# Core always needs glm and nlohmann_json. They are resolved
# with plain CONFIG find_package, so a core-only Linux build can
# use distro packages instead of the full vcpkg manifest.
# Vulkan, SDL3, VMA, and imgui are only required when the engine
# or editor is built.
# ------------------------------------------------------------

find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

if(QUANTUM_ENABLE_ENGINE)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        # vcpkg's VulkanLoader is compiled without the platform WSI surface
        # extensions, so it cannot present under X11 or Wayland. Linux
        # distros ship the WSI-capable loader as a system component; link
        # that loader while continuing to use vcpkg's Vulkan headers.
        block(SCOPE_FOR VARIABLES)
            set(CMAKE_FIND_USE_CMAKE_PATH OFF)
            set(CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH OFF)
            set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
            set(CMAKE_FIND_ROOT_PATH "")
            set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
            unset(QUANTUM_VULKAN_SYSTEM_LIBRARY CACHE)
            find_library(QUANTUM_VULKAN_SYSTEM_LIBRARY NAMES vulkan)
        endblock()
        if(QUANTUM_VULKAN_SYSTEM_LIBRARY)
            set(
                Vulkan_LIBRARY
                "${QUANTUM_VULKAN_SYSTEM_LIBRARY}"
                CACHE FILEPATH "System Vulkan loader" FORCE
            )
        endif()
    endif()
    find_package(
        Vulkan
        REQUIRED
        COMPONENTS
            glslangValidator
    )
    find_package(SDL3 CONFIG REQUIRED)
    find_package(VulkanMemoryAllocator CONFIG REQUIRED)
    if(QUANTUM_ENABLE_EDITOR)
        find_package(imgui CONFIG REQUIRED)
    endif()
endif()

# ------------------------------------------------------------
# Project targets
# ------------------------------------------------------------

add_subdirectory(core)

if(QUANTUM_ENABLE_ENGINE)
    add_subdirectory(engine)
endif()

if(QUANTUM_ENABLE_EDITOR)
    add_subdirectory(editor)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# ------------------------------------------------------------
# Configuration information
# ------------------------------------------------------------

if(QUANTUM_ENABLE_ENGINE)
    message(STATUS "Vulkan version: ${Vulkan_VERSION}")
    message(STATUS "Vulkan include directory: ${Vulkan_INCLUDE_DIRS}")
    message(
        STATUS
        "glslangValidator: ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
    )
endif()
