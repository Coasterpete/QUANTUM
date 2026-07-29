cmake_minimum_required(VERSION 3.25)

project(
    QuantumCoasterEngine
    VERSION 0.1.0
    LANGUAGES CXX
)

add_executable(
    QUANTUM
    src/main.cpp
    src/Application.cpp
    src/Window.cpp
)

target_include_directories(
    QUANTUM
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
    QUANTUM
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QUANTUM
    PROPERTIES
        CXX_EXTENSIONS OFF
)