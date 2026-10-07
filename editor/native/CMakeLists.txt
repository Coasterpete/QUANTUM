add_library(QuantumNative SHARED QuantumNative.cpp NativeViewport.cpp)
target_include_directories(QuantumNative PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(QuantumNative PRIVATE QuantumApplication SDL3::SDL3 imgui::imgui)
target_compile_definitions(QuantumNative PRIVATE QUANTUM_NATIVE_EXPORTS)
target_compile_features(QuantumNative PUBLIC cxx_std_23)
set_target_properties(QuantumNative PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
if(BUILD_TESTING)
    add_executable(QuantumNativeTests QuantumNativeTests.cpp)
    target_link_libraries(QuantumNativeTests PRIVATE QuantumNative)
    add_test(NAME QuantumApplication.NativeBoundary COMMAND QuantumNativeTests
        "${PROJECT_SOURCE_DIR}/smoke-tests/modern-steel-validation.quantum")
endif()
add_custom_command(TARGET QuantumNative POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_BINARY_DIR}/shaders" "$<TARGET_FILE_DIR:QuantumNative>/shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:QuantumNative>/assets")
