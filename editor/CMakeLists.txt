add_executable(
    QUANTUM
    src/main.cpp
    src/CenterlineVisualization.cpp
    src/DocumentState.cpp
    src/EditorStyle.cpp
    src/EditorUi.cpp
    src/PlatformDialogs.cpp
    src/RegionSummary.cpp
    src/TransitionEditorModel.cpp
    src/ViewportCamera.cpp
    src/ViewportPicking.cpp
    include/quantum/editor/CenterlineVisualization.hpp
    include/quantum/editor/DocumentState.hpp
    include/quantum/editor/EditorStyle.hpp
    include/quantum/editor/EditorUi.hpp
    include/quantum/editor/PlatformDialogs.hpp
    include/quantum/editor/RegionSummary.hpp
    include/quantum/editor/RegionSelection.hpp
    include/quantum/editor/TransitionEditorModel.hpp
    include/quantum/editor/TransitionTypePresets.hpp
    include/quantum/editor/ViewportCamera.hpp
    include/quantum/editor/ViewportPicking.hpp
    ${PROJECT_SOURCE_DIR}/engine/src/Application.cpp
    ${PROJECT_SOURCE_DIR}/engine/include/quantum/engine/Application.hpp
)

target_include_directories(
    QUANTUM
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    QUANTUM
    PRIVATE
        Quantum::Engine
        Quantum::Core
        SDL3::SDL3
        imgui::imgui
)

target_compile_features(
    QUANTUM
    PRIVATE
        cxx_std_23
)

set(
    quantum_editor_font_source
    "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Red_Hat_Mono/static/RedHatMono-SemiBold.ttf"
)
set(
    quantum_editor_font_destination
    "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/fonts/RedHatMono-SemiBold.ttf"
)

add_custom_command(
    OUTPUT "${quantum_editor_font_destination}"
    COMMAND
        ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/fonts"
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
        "${quantum_editor_font_source}"
        "${quantum_editor_font_destination}"
    DEPENDS "${quantum_editor_font_source}"
    COMMENT "Deploying the QUANTUM Editor UI font"
    VERBATIM
)

add_custom_target(
    QUANTUMEditorFont ALL
    DEPENDS "${quantum_editor_font_destination}"
)
add_dependencies(QUANTUM QUANTUMEditorFont)

add_custom_command(
    TARGET QUANTUM
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_BINARY_DIR}/shaders"
        "$<TARGET_FILE_DIR:QUANTUM>/shaders"
    VERBATIM
)

set_target_properties(
    QUANTUM
    PROPERTIES
        CXX_EXTENSIONS OFF
)
