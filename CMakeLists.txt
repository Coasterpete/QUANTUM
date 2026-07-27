cmake_minimum_required(VERSION 3.25)

project(
    QUANTUMSplineWorksEngine
    VERSION 0.1.0
    LANGUAGES CXX
)

add_executable(quantum_splineworks
    src/main.cpp
)

target_compile_features(quantum_splineworks
    PRIVATE
        cxx_std_20
)

set_target_properties(quantum_splineworks PROPERTIES
    CXX_EXTENSIONS OFF
)