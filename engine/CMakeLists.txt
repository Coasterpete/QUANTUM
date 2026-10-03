add_subdirectory(audio)

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
set(QUANTUM_SKY_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/sky.vert.spv")
set(QUANTUM_SKY_FRAGMENT_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/sky.frag.spv")
set(QUANTUM_HARDWARE_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/hardware.vert.spv")
set(QUANTUM_GROUND_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/ground.vert.spv")
set(QUANTUM_GROUND_FRAGMENT_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/ground.frag.spv")
set(QUANTUM_SUPPORT_VERTEX_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/support.vert.spv")
set(QUANTUM_SUPPORT_FRAGMENT_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/support.frag.spv")
set(QUANTUM_PHYSICS_COMPUTE_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/track_sample.comp.spv")
set(QUANTUM_RIGID_BOGIE_COMPUTE_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/rigid_bogie.comp.spv")
set(QUANTUM_TRAIN_POSE_COMPUTE_SHADER
    "${QUANTUM_SHADER_BINARY_DIR}/train_pose.comp.spv")

add_custom_command(
    OUTPUT
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_SKY_VERTEX_SHADER}"
        "${QUANTUM_SKY_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
        "${QUANTUM_SUPPORT_VERTEX_SHADER}"
        "${QUANTUM_SUPPORT_FRAGMENT_SHADER}"
        "${QUANTUM_PHYSICS_COMPUTE_SHADER}"
        "${QUANTUM_RIGID_BOGIE_COMPUTE_SHADER}"
        "${QUANTUM_TRAIN_POSE_COMPUTE_SHADER}"
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
        -V --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky.vert"
        -o "${QUANTUM_SKY_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky.frag"
        -o "${QUANTUM_SKY_FRAGMENT_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/hardware.vert"
        -o "${QUANTUM_HARDWARE_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ground.vert"
        -o "${QUANTUM_GROUND_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ground.frag"
        -o "${QUANTUM_GROUND_FRAGMENT_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/support.vert"
        -o "${QUANTUM_SUPPORT_VERTEX_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/support.frag"
        -o "${QUANTUM_SUPPORT_FRAGMENT_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/physics/track_sample.comp"
        -o "${QUANTUM_PHYSICS_COMPUTE_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/physics/rigid_bogie.comp"
        -o "${QUANTUM_RIGID_BOGIE_COMPUTE_SHADER}"
    COMMAND
        "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}"
        -V
        --target-env vulkan1.3
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/physics/train_pose.comp"
        -o "${QUANTUM_TRAIN_POSE_COMPUTE_SHADER}"
    DEPENDS
        shaders/centerline.vert
        shaders/centerline.frag
        shaders/track.vert
        shaders/track.frag
        shaders/sky.vert
        shaders/sky.frag
        shaders/hardware.vert
        shaders/ground.vert
        shaders/ground.frag
        shaders/support.vert
        shaders/support.frag
        shaders/physics/track_sample.comp
        shaders/physics/rigid_bogie.comp
        shaders/physics/train_pose.comp
    VERBATIM
)

add_custom_target(
    QuantumShaders
    DEPENDS
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_SKY_VERTEX_SHADER}"
        "${QUANTUM_SKY_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
        "${QUANTUM_GROUND_VERTEX_SHADER}"
        "${QUANTUM_GROUND_FRAGMENT_SHADER}"
        "${QUANTUM_SUPPORT_VERTEX_SHADER}"
        "${QUANTUM_SUPPORT_FRAGMENT_SHADER}"
        "${QUANTUM_PHYSICS_COMPUTE_SHADER}"
        "${QUANTUM_RIGID_BOGIE_COMPUTE_SHADER}"
        "${QUANTUM_TRAIN_POSE_COMPUTE_SHADER}"
)

install(
    FILES
        "${QUANTUM_VERTEX_SHADER}"
        "${QUANTUM_FRAGMENT_SHADER}"
        "${QUANTUM_TRACK_VERTEX_SHADER}"
        "${QUANTUM_TRACK_FRAGMENT_SHADER}"
        "${QUANTUM_SKY_VERTEX_SHADER}"
        "${QUANTUM_SKY_FRAGMENT_SHADER}"
        "${QUANTUM_HARDWARE_VERTEX_SHADER}"
        "${QUANTUM_GROUND_VERTEX_SHADER}"
        "${QUANTUM_GROUND_FRAGMENT_SHADER}"
        "${QUANTUM_SUPPORT_VERTEX_SHADER}"
        "${QUANTUM_SUPPORT_FRAGMENT_SHADER}"
        "${QUANTUM_PHYSICS_COMPUTE_SHADER}"
        "${QUANTUM_RIGID_BOGIE_COMPUTE_SHADER}"
        "${QUANTUM_TRAIN_POSE_COMPUTE_SHADER}"
    DESTINATION shaders
    COMPONENT Runtime
)

target_sources(
    QuantumEngine
    PRIVATE
        src/Logging.cpp
        src/EnvironmentAssets.cpp
        src/EnvironmentMap.cpp
        src/EnvironmentVulkan.cpp
        src/GroundSurface.cpp
        src/GroundSurfaceVulkan.cpp
        src/StaticMeshAssets.cpp
        src/SupportSolidGeometry.cpp
        src/SupportSolidVulkan.cpp
        src/VulkanContext.cpp
        src/ViewportAids.cpp
        src/physics/gpu/GpuPhysicsContext.cpp
        src/physics/RigidBodyWorld.cpp
        include/quantum/physics/RigidBodyWorld.hpp
        include/quantum/renderer/ViewportAids.hpp
        include/quantum/renderer/ViewportTrackPresentation.hpp
        include/quantum/engine/Logging.hpp
        include/quantum/renderer/EnvironmentAssets.hpp
        include/quantum/renderer/GroundSurface.hpp
        include/quantum/renderer/StaticMeshAssets.hpp
        include/quantum/renderer/SupportSolidGeometry.hpp
        include/quantum/renderer/Renderer.hpp
        include/quantum/renderer/EnvironmentMap.hpp
        include/quantum/renderer/VulkanContext.hpp
        include/quantum/physics/gpu/GpuPhysicsContext.hpp
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
        Jolt::Jolt
        nlohmann_json::nlohmann_json
)

set_target_properties(
    QuantumEngine
    PROPERTIES
        CXX_EXTENSIONS OFF
)

