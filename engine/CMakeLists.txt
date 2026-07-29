add_library(QuantumEngine STATIC)

add_library(Quantum::Engine ALIAS QuantumEngine)

target_sources(
    QuantumEngine
    PRIVATE
        src/Application.cpp
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

target_compile_definitions(
    QuantumEngine
    PRIVATE
        GLFW_INCLUDE_NONE
)

target_link_libraries(
    QuantumEngine
    PRIVATE
        glfw
        Vulkan::Vulkan
)

set_target_properties(
    QuantumEngine
    PROPERTIES
        CXX_EXTENSIONS OFF
)