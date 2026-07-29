add_executable(
    QUANTUM
    src/main.cpp
)

target_link_libraries(
    QUANTUM
    PRIVATE
        Quantum::Engine
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