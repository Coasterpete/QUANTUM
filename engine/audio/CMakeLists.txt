add_library(QuantumAudio STATIC)

add_library(Quantum::Audio ALIAS QuantumAudio)

target_sources(
    QuantumAudio
    PRIVATE
        AudioEngine.cpp
        AudioEngine.hpp

        backend/AudioBackend.cpp
        backend/AudioBackend.hpp
        backend/AudioDevice.cpp
        backend/AudioDevice.hpp
)

target_include_directories(
    QuantumAudio
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(
    QuantumAudio
    PUBLIC
        cxx_std_23
)

target_link_libraries(
    QuantumAudio
    PRIVATE
        SDL3::SDL3
)

set_target_properties(
    QuantumAudio
    PROPERTIES
        CXX_EXTENSIONS OFF
)
