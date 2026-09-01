add_executable(
    QUANTUM
    src/main.cpp
    src/CenterlineVisualization.cpp
    src/DocumentState.cpp
    src/EditorIcons.cpp
    src/EditorStyle.cpp
    src/EditorUi.cpp
    src/PlatformDialogs.cpp
    src/RegionSummary.cpp
    src/ReadmeCapture.cpp
    src/ReadmeCaptureRunner.cpp
    src/RiderLoadDiagnostics.cpp
    src/TransitionEditorModel.cpp
    src/ViewportCamera.cpp
    src/ViewportPicking.cpp
    src/ViewportTrackAnchors.cpp
    include/quantum/editor/AuthoredTrackEditTransaction.hpp
    include/quantum/editor/CenterlineVisualization.hpp
    include/quantum/editor/DocumentState.hpp
    include/quantum/editor/EditorIcons.hpp
    include/quantum/editor/EditorStyle.hpp
    include/quantum/editor/EditorUi.hpp
    include/quantum/editor/PlatformDialogs.hpp
    include/quantum/editor/RegionSummary.hpp
    include/quantum/editor/ReadmeCapture.hpp
    include/quantum/editor/RegionSelection.hpp
    include/quantum/editor/RiderLoadDiagnostics.hpp
    include/quantum/editor/TransitionEditorModel.hpp
    include/quantum/editor/TransitionTypePresets.hpp
    include/quantum/editor/ViewportCamera.hpp
    include/quantum/editor/ViewportPicking.hpp
    include/quantum/editor/ViewportTrackAnchors.hpp
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

set(quantum_editor_font_files
    overpass-regular.otf
    overpass-semibold.otf
    overpass-mono-regular.otf
    LICENSE.md
    README.md
)
set(quantum_editor_font_outputs)
foreach(font_file IN LISTS quantum_editor_font_files)
    set(font_source "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Overpass/${font_file}")
    set(font_destination "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/fonts/Overpass/${font_file}")
    add_custom_command(
        OUTPUT "${font_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/fonts/Overpass"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${font_source}" "${font_destination}"
        DEPENDS "${font_source}"
        COMMENT "Deploying QUANTUM typography asset ${font_file}"
        VERBATIM
    )
    list(APPEND quantum_editor_font_outputs "${font_destination}")
endforeach()
add_custom_target(QUANTUMEditorFont ALL DEPENDS ${quantum_editor_font_outputs})
add_dependencies(QUANTUM QUANTUMEditorFont)

set(quantum_editor_icon_files
    mouse-pointer-2.svg
    move-3d.svg
    rotate-3d.svg
    orbit.svg
    hand.svg
    focus.svg
    camera.svg
    axis-3d.svg
    maximize.svg
    play.svg
    settings.svg
)
set(quantum_editor_icon_outputs)
foreach(icon_file IN LISTS quantum_editor_icon_files)
    set(icon_source "${PROJECT_SOURCE_DIR}/assets/icons/lucide/${icon_file}")
    set(icon_destination "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/icons/lucide/${icon_file}")
    add_custom_command(
        OUTPUT "${icon_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/assets/icons/lucide"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${icon_source}" "${icon_destination}"
        DEPENDS "${icon_source}"
        COMMENT "Deploying QUANTUM Lucide toolbar icon ${icon_file}"
        VERBATIM
    )
    list(APPEND quantum_editor_icon_outputs "${icon_destination}")
endforeach()
add_custom_target(QUANTUMEditorIcons ALL DEPENDS ${quantum_editor_icon_outputs})
add_dependencies(QUANTUM QUANTUMEditorIcons)

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
