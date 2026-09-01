add_library(QuantumEngine STATIC)

add_library(Quantum::Engine ALIAS QuantumEngine)

set(QUANTUM_SHADER_BINARY_DIR "${CMAKE_BINARY_DIR}/shaders")
set(
    QUANTUM_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/centerline.vert.spv"
)
set(
    QUANTUM_FRAGMENT_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/centerline.frag.spv"
)
set(QUANTUM_TRACK_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/track.vert.spv")
set(QUANTUM_TRACK_FRAGMENT_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/track.frag.spv")
set(QUANTUM_HARDWARE_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/hardware.vert.spv")

add_custom_command(
    OUTPUT
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
    COMMAND
        ${CMAKE_COMMAND} -E make_directory
        "${QUANTUM_SHADER_BINARY_DIR}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/centerline.vert"
        -o "${QUANTUM_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/centerline.frag"
        -o "${QUANTUM_FRAGMENT_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track.vert"
        -o "${QUANTUM_TRACK_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track.frag"
        -o "${QUANTUM_TRACK_FRAGMENT_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/hardware.vert"
        -o "${QUANTUM_HARDWARE_VERTEX_SHADER}"
    DEPENDS
        shaders/centerline.vert
        shaders/centerline.frag
        shaders/track.vert
        shaders/track.frag
        shaders/hardware.vert
    VERBATIM
)

add_custom_target(
    QuantumShaders
    DEPENDS
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
)

install(
    FILES
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
    DESTINATION shaders
    COMPONENT Runtime
)

target_sources(
    QuantumEngine
    PRIVATE
        src/Logging.cpp
        src/VulkanContext.cpp
        src/ViewportAids.cpp
        include/quantum/renderer/ViewportAids.hpp
        include/quantum/renderer/ViewportTrackPresentation.hpp
        include/quantum/engine/Logging.hpp
        include/quantum/renderer/VulkanContext.hpp
)

target_include_directories(
    QuantumEngine
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
    QuantumEngine
    PUBLIC
        cxx_std_23
)

add_dependencies(QuantumEngine QuantumShaders)

target_compile_definitions(
    QuantumEngine
    PRIVATE
        $<$<CONFIG:Debug>:QUANTUM_ENABLE_VULKAN_VALIDATION>
)

target_link_libraries(
    QuantumEngine
    PUBLIC
        # VulkanContext.hpp is a public header that includes <vulkan/vulkan.h>,
        # so consumers compiling against engine headers need Vulkan includes.
        Vulkan::Vulkan
        GPUOpen::VulkanMemoryAllocator
        Quantum::Core
    PRIVATE
        SDL3::SDL3
)

set_target_properties(
    QuantumEngine
    PROPERTIES
        CXX_EXTENSIONS OFF
)
