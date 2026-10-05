add_executable(
    QUANTUM
    src/main.cpp
    src/CenterlineVisualization.cpp
    src/SupportPicking.cpp
    src/SupportNodeManipulation.cpp
    src/SupportVisualization.cpp
    src/CoasterSetupUi.cpp
    src/DocumentHistory.cpp
    src/DocumentState.cpp
    src/EditorIcons.cpp
    src/EditorStyle.cpp
    src/EditorUi.cpp
    src/EditorUiCommands.cpp
    src/EditorUiTrain.cpp
    src/EditorUiPerformanceTelemetry.cpp
    src/EditorUiSimulationTelemetry.cpp
    src/EditorUiSupportAppearance.cpp
    src/PlatformDialogs.cpp
    src/PreviewSmoke.cpp
    src/RegionSummary.cpp
    src/ReadmeCapture.cpp
    src/ReadmeCaptureRunner.cpp
    src/RiderLoadDiagnostics.cpp
    src/RigidBodyMechanismProof.cpp
    src/SimulationPreview.cpp
    src/TrainMeshPresentation.cpp
    src/TransitionEditorModel.cpp
    src/TrackStylePresentation.cpp
    src/ViewportCamera.cpp
    src/ViewportPicking.cpp
    src/ViewportTrackAnchors.cpp
    include/quantum/editor/AuthoredTrackEditTransaction.hpp
    include/quantum/editor/CenterlineVisualization.hpp
    include/quantum/editor/SupportPicking.hpp
    include/quantum/editor/SupportNodeManipulation.hpp
    include/quantum/editor/SupportVisualization.hpp
    include/quantum/editor/CoasterSetupUi.hpp
    include/quantum/editor/DocumentHistory.hpp
    include/quantum/editor/DocumentState.hpp
    include/quantum/editor/EditorIcons.hpp
    include/quantum/editor/EditorStyle.hpp
    include/quantum/editor/EditorUi.hpp
    include/quantum/editor/FramePerformanceTelemetry.hpp
    include/quantum/editor/PlatformDialogs.hpp
    include/quantum/editor/PreviewSmoke.hpp
    include/quantum/editor/RegionSummary.hpp
    include/quantum/editor/ReadmeCapture.hpp
    include/quantum/editor/RegionSelection.hpp
    include/quantum/editor/RiderLoadDiagnostics.hpp
    include/quantum/editor/RigidBodyMechanismProof.hpp
    include/quantum/editor/SimulationPreview.hpp
    include/quantum/editor/TrainMeshPresentation.hpp
    include/quantum/editor/TransitionEditorModel.hpp
    include/quantum/editor/TrainPreviewInspection.hpp
    include/quantum/editor/TrackStylePresentation.hpp
    include/quantum/editor/TransitionTypePresets.hpp
    include/quantum/editor/ViewportCamera.hpp
    include/quantum/editor/ViewportPicking.hpp
    include/quantum/editor/ViewportTrackAnchors.hpp
    include/quantum/editor/WorkspaceComposition.hpp
    ${PROJECT_SOURCE_DIR}/engine/src/Application.cpp
    ${PROJECT_SOURCE_DIR}/engine/include/quantum/engine/Application.hpp
)

if(WIN32)
    target_sources(
        QUANTUM
        PRIVATE
            platform/windows/QUANTUM.rc
    )
endif()

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
        Quantum::Audio
        SDL3::SDL3
        imgui::imgui
)

target_compile_features(
    QUANTUM
    PRIVATE
        cxx_std_23
)

if(MSVC)
    # Debug's deeper Application stack plus Vulkan validation's device-state
    # construction can exceed the default 1 MiB Windows stack reserve.
    target_link_options(QUANTUM PRIVATE "$<$<CONFIG:Debug>:/STACK:4194304>")
endif()

set(quantum_editor_font_files
    Overpass/overpass-regular.otf
    Overpass/LICENSE.md
    Overpass/README.md
    Red_Hat_Mono/static/RedHatMono-Regular.ttf
    Red_Hat_Mono/static/RedHatMono-SemiBold.ttf
    Red_Hat_Mono/OFL.txt
    Red_Hat_Mono/README.txt
)
set(quantum_editor_font_outputs)
foreach(font_file IN LISTS quantum_editor_font_files)
    set(font_source "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/${font_file}")
    set(font_destination "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/fonts/${font_file}")
    get_filename_component(font_destination_directory "${font_destination}" DIRECTORY)
    add_custom_command(
        OUTPUT "${font_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${font_destination_directory}"
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
    folder-open.svg
    save.svg
    undo-2.svg
    redo-2.svg
    eye.svg
    eye-off.svg
    lock.svg
    lock-open.svg
    zoom-in.svg
    zoom-out.svg
    play.svg
    pause.svg
    square-stop.svg
    settings.svg
)
set(quantum_editor_icon_outputs)
foreach(icon_file IN LISTS quantum_editor_icon_files)
    set(icon_source "${PROJECT_SOURCE_DIR}/assets/icons/lucide/${icon_file}")
    set(icon_destination "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/icons/lucide/${icon_file}")
    add_custom_command(
        OUTPUT "${icon_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/icons/lucide"
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

set(quantum_track_asset_source
    "${PROJECT_SOURCE_DIR}/assets/track/test-crosstie-placeholder.glb")
set(quantum_track_asset_destination
    "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/track/test-crosstie-placeholder.glb")
add_custom_command(
    OUTPUT "${quantum_track_asset_destination}"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/track"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${quantum_track_asset_source}"
        "${quantum_track_asset_destination}"
    DEPENDS "${quantum_track_asset_source}"
    COMMENT "Deploying QUANTUM test track-hardware GLB"
    VERBATIM
)
add_custom_target(QUANTUMTrackAssets ALL
    DEPENDS "${quantum_track_asset_destination}")
add_dependencies(QUANTUM QUANTUMTrackAssets)

set(quantum_mechanical_asset_files
    rotating-arm-placeholder.glb
    hanging-carrier-placeholder.glb)
set(quantum_mechanical_asset_outputs)
foreach(mechanical_asset_file IN LISTS quantum_mechanical_asset_files)
    set(quantum_mechanical_asset_source
        "${PROJECT_SOURCE_DIR}/assets/mechanical/${mechanical_asset_file}")
    set(quantum_mechanical_asset_destination
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/mechanical/${mechanical_asset_file}")
    add_custom_command(
        OUTPUT "${quantum_mechanical_asset_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/mechanical"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${quantum_mechanical_asset_source}" "${quantum_mechanical_asset_destination}"
        DEPENDS "${quantum_mechanical_asset_source}"
        COMMENT "Deploying QUANTUM mechanical placeholder ${mechanical_asset_file}"
        VERBATIM)
    list(APPEND quantum_mechanical_asset_outputs "${quantum_mechanical_asset_destination}")
endforeach()
add_custom_target(QUANTUMMechanicalAssets ALL
    DEPENDS ${quantum_mechanical_asset_outputs})
add_dependencies(QUANTUM QUANTUMMechanicalAssets)

set(quantum_train_asset_source
    "${PROJECT_SOURCE_DIR}/assets/train/placeholder-car-shell.glb")
set(quantum_train_asset_destination
    "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/train/placeholder-car-shell.glb")
add_custom_command(
    OUTPUT "${quantum_train_asset_destination}"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/train"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${quantum_train_asset_source}" "${quantum_train_asset_destination}"
    DEPENDS "${quantum_train_asset_source}"
    COMMENT "Deploying QUANTUM placeholder train shell"
    VERBATIM)
add_custom_target(QUANTUMTrainAssets ALL DEPENDS "${quantum_train_asset_destination}")
add_dependencies(QUANTUM QUANTUMTrainAssets)
install(FILES "${quantum_train_asset_source}" DESTINATION assets/train COMPONENT Runtime)

set(quantum_environment_asset_files
    rooitou_park_1k.hdr
    dayskyhdri027b_1k.hdr
)
set(quantum_environment_asset_outputs)
foreach(environment_asset_file IN LISTS quantum_environment_asset_files)
    set(environment_asset_source
        "${PROJECT_SOURCE_DIR}/assets/environment/${environment_asset_file}")
    set(environment_asset_destination
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/environment/${environment_asset_file}")
    add_custom_command(
        OUTPUT "${environment_asset_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/environment"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${environment_asset_source}" "${environment_asset_destination}"
        DEPENDS "${environment_asset_source}"
        VERBATIM
    )
    list(APPEND quantum_environment_asset_outputs
        "${environment_asset_destination}")
endforeach()
add_custom_target(QUANTUMEnvironmentAssets ALL
    DEPENDS ${quantum_environment_asset_outputs})
add_dependencies(QUANTUM QUANTUMEnvironmentAssets)

# Ground Surface M0 resolves assets://ground/... beneath the runtime asset
# root, so the whole folder is staged. The test patterns are the only bundled
# maps; a user-supplied PNG dropped here becomes selectable in the Editor.
set(quantum_ground_asset_files
    test-ground-albedo.png
    test-ground-normal.png
    test-ground-roughness.png
    README.md
)
set(quantum_ground_asset_outputs)
foreach(ground_asset_file IN LISTS quantum_ground_asset_files)
    set(ground_asset_source
        "${PROJECT_SOURCE_DIR}/assets/ground/${ground_asset_file}")
    set(ground_asset_destination
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/ground/${ground_asset_file}")
    add_custom_command(
        OUTPUT "${ground_asset_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/ground"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ground_asset_source}" "${ground_asset_destination}"
        DEPENDS "${ground_asset_source}"
        COMMENT "Deploying QUANTUM ground asset ${ground_asset_file}"
        VERBATIM
    )
    list(APPEND quantum_ground_asset_outputs "${ground_asset_destination}")
endforeach()
add_custom_target(QUANTUMGroundAssets ALL
    DEPENDS ${quantum_ground_asset_outputs})
add_dependencies(QUANTUM QUANTUMGroundAssets)

# Solid Supports M2A resolves assets://materials/timber/... beneath the runtime
# asset root. Only the three maps the solid timber shader samples are staged;
# the reserved AO, height, and edge sources ship through the repository and the
# install rule below, not into the editor's working asset directory, so nothing
# implies they are consumed.
set(quantum_timber_asset_files
    pinewood_basecolor_neutral.png
    pinewood_normal.png
    pinewood_roughness.png
)
set(quantum_timber_asset_outputs)
foreach(timber_asset_file IN LISTS quantum_timber_asset_files)
    set(timber_asset_source
        "${PROJECT_SOURCE_DIR}/assets/materials/timber/pine/${timber_asset_file}")
    set(timber_asset_destination
        "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/materials/timber/pine/${timber_asset_file}")
    add_custom_command(
        OUTPUT "${timber_asset_destination}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}${QUANTUM_ASSET_CONFIG_SUBDIR}/assets/materials/timber/pine"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${timber_asset_source}"
            "${timber_asset_destination}"
        DEPENDS "${timber_asset_source}"
        COMMENT "Deploying QUANTUM timber asset ${timber_asset_file}"
        VERBATIM
    )
    list(APPEND quantum_timber_asset_outputs "${timber_asset_destination}")
endforeach()
add_custom_target(QUANTUMTimberAssets ALL
    DEPENDS ${quantum_timber_asset_outputs})
add_dependencies(QUANTUM QUANTUMTimberAssets)

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

install(
    TARGETS QUANTUM
    RUNTIME DESTINATION "."
    COMPONENT Runtime
)

install(
    FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Overpass/overpass-regular.otf"
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Overpass/LICENSE.md"
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Overpass/README.md"
    DESTINATION assets/fonts/Overpass
    COMPONENT Runtime
)

install(
    FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Red_Hat_Mono/static/RedHatMono-Regular.ttf"
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Red_Hat_Mono/static/RedHatMono-SemiBold.ttf"
    DESTINATION assets/fonts/Red_Hat_Mono/static
    COMPONENT Runtime
)
install(
    FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Red_Hat_Mono/OFL.txt"
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/Red_Hat_Mono/README.txt"
    DESTINATION assets/fonts/Red_Hat_Mono
    COMPONENT Runtime
)

set(quantum_editor_runtime_icon_files)
foreach(icon_file IN LISTS quantum_editor_icon_files)
    list(APPEND quantum_editor_runtime_icon_files
        "${PROJECT_SOURCE_DIR}/assets/icons/lucide/${icon_file}"
    )
endforeach()

install(
    FILES ${quantum_editor_runtime_icon_files}
    DESTINATION assets/icons/lucide
    COMPONENT Runtime
)

install(
    FILES
        "${PROJECT_SOURCE_DIR}/assets/track/test-crosstie-placeholder.glb"
    DESTINATION assets/track
    COMPONENT Runtime
)

foreach(environment_asset_file IN LISTS quantum_environment_asset_files)
    install(
        FILES "${PROJECT_SOURCE_DIR}/assets/environment/${environment_asset_file}"
        DESTINATION assets/environment
        COMPONENT Runtime
    )
endforeach()

foreach(mechanical_asset_file IN LISTS quantum_mechanical_asset_files)
    install(FILES "${PROJECT_SOURCE_DIR}/assets/mechanical/${mechanical_asset_file}"
        DESTINATION assets/mechanical COMPONENT Runtime)
endforeach()

foreach(ground_asset_file IN LISTS quantum_ground_asset_files)
    install(
        FILES "${PROJECT_SOURCE_DIR}/assets/ground/${ground_asset_file}"
        DESTINATION assets/ground
        COMPONENT Runtime
    )
endforeach()

# The full user-authored timber set ships, including the height and edge
# sources that M2A intentionally does not read. They are kept together so a
# future material-authoring pass can use them without re-acquiring the set.
foreach(timber_asset_file IN ITEMS
    ${quantum_timber_asset_files}
    pinewood_ao.png
    pinewood_height.png
    pinewood_edge.png
    README.md)
    install(
        FILES
            "${PROJECT_SOURCE_DIR}/assets/materials/timber/pine/${timber_asset_file}"
        DESTINATION assets/materials/timber/pine
        COMPONENT Runtime
    )
endforeach()

if(WIN32)
    install(
        FILES "$<TARGET_FILE_DIR:QUANTUM>/vulkan-1.dll"
        DESTINATION "."
        COMPONENT Runtime
    )
endif()
