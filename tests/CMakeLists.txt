add_executable(
    QuantumEngineLoggingTests
    LoggingTests.cpp
)

add_executable(QuantumCoreTrackFollowerPhysicsTests TrackFollowerPhysicsTests.cpp)
target_link_libraries(QuantumCoreTrackFollowerPhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreTrackFollowerPhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreTrackFollowerPhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.TrackFollowerPhysics COMMAND QuantumCoreTrackFollowerPhysicsTests)
set_tests_properties(QuantumCore.TrackFollowerPhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreCarPosePhysicsTests CarPosePhysicsTests.cpp)
target_link_libraries(QuantumCoreCarPosePhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreCarPosePhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreCarPosePhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.CarPosePhysics COMMAND QuantumCoreCarPosePhysicsTests)
set_tests_properties(QuantumCore.CarPosePhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreTrainPhysicsTests TrainPhysicsTests.cpp)
target_link_libraries(QuantumCoreTrainPhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreTrainPhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreTrainPhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.TrainPhysics COMMAND QuantumCoreTrainPhysicsTests)
set_tests_properties(QuantumCore.TrainPhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreConnectorLoadPhysicsTests ConnectorLoadPhysicsTests.cpp)
target_link_libraries(QuantumCoreConnectorLoadPhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreConnectorLoadPhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreConnectorLoadPhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.ConnectorLoadPhysics COMMAND QuantumCoreConnectorLoadPhysicsTests)
set_tests_properties(QuantumCore.ConnectorLoadPhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreExternalForcePhysicsTests ExternalForcePhysicsTests.cpp)
target_link_libraries(QuantumCoreExternalForcePhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreExternalForcePhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreExternalForcePhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.ExternalForcePhysics COMMAND QuantumCoreExternalForcePhysicsTests)
set_tests_properties(QuantumCore.ExternalForcePhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreResistancePhysicsTests ResistancePhysicsTests.cpp)
target_link_libraries(QuantumCoreResistancePhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreResistancePhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreResistancePhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.ResistancePhysics COMMAND QuantumCoreResistancePhysicsTests)
set_tests_properties(QuantumCore.ResistancePhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreBogieReactionPhysicsTests BogieReactionPhysicsTests.cpp)
target_link_libraries(QuantumCoreBogieReactionPhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreBogieReactionPhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreBogieReactionPhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.BogieReactionPhysics COMMAND QuantumCoreBogieReactionPhysicsTests)
set_tests_properties(QuantumCore.BogieReactionPhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreBogieContactFeasibilityTests BogieContactFeasibilityTests.cpp)
target_link_libraries(QuantumCoreBogieContactFeasibilityTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreBogieContactFeasibilityTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreBogieContactFeasibilityTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.BogieContactFeasibility COMMAND QuantumCoreBogieContactFeasibilityTests)
set_tests_properties(QuantumCore.BogieContactFeasibility PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreRotationalPhysicsTests RotationalPhysicsTests.cpp)
target_link_libraries(QuantumCoreRotationalPhysicsTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreRotationalPhysicsTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreRotationalPhysicsTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.RotationalPhysics COMMAND QuantumCoreRotationalPhysicsTests)
set_tests_properties(QuantumCore.RotationalPhysics PROPERTIES LABELS "core;coaster;physics")

add_executable(QuantumCoreTrackStyleTests TrackStyleTests.cpp)
target_link_libraries(QuantumCoreTrackStyleTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreTrackStyleTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreTrackStyleTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.TrackStyle COMMAND QuantumCoreTrackStyleTests)
set_tests_properties(QuantumCore.TrackStyle PROPERTIES LABELS "core;coaster;track-style;geometry")

add_executable(QuantumCoreTrackHardwarePlacementTests TrackHardwarePlacementTests.cpp)
target_link_libraries(QuantumCoreTrackHardwarePlacementTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreTrackHardwarePlacementTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreTrackHardwarePlacementTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.TrackHardwarePlacement COMMAND QuantumCoreTrackHardwarePlacementTests)
set_tests_properties(QuantumCore.TrackHardwarePlacement PROPERTIES LABELS "core;coaster;track-style;hardware")

add_executable(QuantumEngineStaticMeshAssetTests StaticMeshAssetTests.cpp)
target_link_libraries(QuantumEngineStaticMeshAssetTests PRIVATE Quantum::Engine)
target_compile_features(QuantumEngineStaticMeshAssetTests PRIVATE cxx_std_23)
set_target_properties(QuantumEngineStaticMeshAssetTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumEngine.StaticMeshAssets COMMAND QuantumEngineStaticMeshAssetTests
    "${PROJECT_SOURCE_DIR}")
set_tests_properties(QuantumEngine.StaticMeshAssets PROPERTIES
    LABELS "engine;renderer;assets;hardware")

add_executable(QuantumEditorReadmeCaptureTests
    ReadmeCaptureTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ReadmeCapture.cpp
)
target_include_directories(QuantumEditorReadmeCaptureTests PRIVATE ${PROJECT_SOURCE_DIR}/editor/include)
target_link_libraries(QuantumEditorReadmeCaptureTests PRIVATE Quantum::Core)
target_compile_features(QuantumEditorReadmeCaptureTests PRIVATE cxx_std_23)
set_target_properties(QuantumEditorReadmeCaptureTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumEditor.ReadmeCapture COMMAND QuantumEditorReadmeCaptureTests)
set_tests_properties(QuantumEditor.ReadmeCapture PROPERTIES LABELS "editor;capture")

add_executable(QuantumEditorViewportPresentationTests
    ViewportPresentationTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/CenterlineVisualization.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/EditorStyle.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportTrackAnchors.cpp
)
target_include_directories(QuantumEditorViewportPresentationTests PRIVATE ${PROJECT_SOURCE_DIR}/editor/include)
target_link_libraries(QuantumEditorViewportPresentationTests PRIVATE Quantum::Core Quantum::Engine SDL3::SDL3 imgui::imgui)
target_compile_features(QuantumEditorViewportPresentationTests PRIVATE cxx_std_23)
set_target_properties(QuantumEditorViewportPresentationTests PROPERTIES CXX_EXTENSIONS OFF)
add_dependencies(QuantumEditorViewportPresentationTests QUANTUMEditorFont)
add_test(NAME QuantumEditor.ViewportPresentation COMMAND QuantumEditorViewportPresentationTests
    "${PROJECT_BINARY_DIR}/editor/$<CONFIG>")
set_tests_properties(QuantumEditor.ViewportPresentation PROPERTIES LABELS "editor;camera;viewport;typography")

add_executable(QuantumEditorIconsTests
    EditorIconsTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/EditorIcons.cpp
)
target_include_directories(QuantumEditorIconsTests PRIVATE ${PROJECT_SOURCE_DIR}/editor/include)
target_link_libraries(QuantumEditorIconsTests PRIVATE imgui::imgui)
target_compile_features(QuantumEditorIconsTests PRIVATE cxx_std_23)
set_target_properties(QuantumEditorIconsTests PROPERTIES CXX_EXTENSIONS OFF)
add_dependencies(QuantumEditorIconsTests QUANTUMEditorIcons)
add_test(NAME QuantumEditor.Icons COMMAND QuantumEditorIconsTests
    "${PROJECT_BINARY_DIR}/editor/$<CONFIG>")
set_tests_properties(QuantumEditor.Icons PROPERTIES LABELS "editor;icons")

add_executable(QuantumCoreForceDrivenRegionTests ForceDrivenRegionTests.cpp)
target_link_libraries(QuantumCoreForceDrivenRegionTests PRIVATE Quantum::Core)
target_compile_features(QuantumCoreForceDrivenRegionTests PRIVATE cxx_std_23)
set_target_properties(QuantumCoreForceDrivenRegionTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumCore.ForceDrivenRegion COMMAND QuantumCoreForceDrivenRegionTests)
set_tests_properties(QuantumCore.ForceDrivenRegion PROPERTIES LABELS "core;coaster;physics;force")

add_executable(QuantumEditorForceDrivenAcceptanceTests
    ForceDrivenAcceptanceTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/CenterlineVisualization.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/RiderLoadDiagnostics.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportTrackAnchors.cpp
)
target_include_directories(QuantumEditorForceDrivenAcceptanceTests PRIVATE ${PROJECT_SOURCE_DIR}/editor/include)
target_link_libraries(QuantumEditorForceDrivenAcceptanceTests PRIVATE Quantum::Core Quantum::Engine)
target_compile_features(QuantumEditorForceDrivenAcceptanceTests PRIVATE cxx_std_23)
set_target_properties(QuantumEditorForceDrivenAcceptanceTests PROPERTIES CXX_EXTENSIONS OFF)
add_test(NAME QuantumEditor.ForceDrivenAcceptance COMMAND QuantumEditorForceDrivenAcceptanceTests)
set_tests_properties(QuantumEditor.ForceDrivenAcceptance PROPERTIES LABELS "editor;force;transaction;physics")

target_link_libraries(
    QuantumEngineLoggingTests
    PRIVATE
        Quantum::Engine
)

target_compile_features(
    QuantumEngineLoggingTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEngineLoggingTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEngine.Logging
    COMMAND QuantumEngineLoggingTests
)

set_tests_properties(
    QuantumEngine.Logging
    PROPERTIES
        LABELS "engine;logging"
)

add_executable(
    QuantumCoreTests
    BSplineCurveTests.cpp
)

target_link_libraries(
    QuantumCoreTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.BSplineCurve
    COMMAND QuantumCoreTests
)

set_tests_properties(
    QuantumCore.BSplineCurve
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreNurbsTests
    NurbsCurveTests.cpp
)

target_link_libraries(
    QuantumCoreNurbsTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreNurbsTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreNurbsTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.NurbsCurve
    COMMAND QuantumCoreNurbsTests
)

set_tests_properties(
    QuantumCore.NurbsCurve
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreCurveGeometryTests
    CurveGeometryTests.cpp
)

target_link_libraries(
    QuantumCoreCurveGeometryTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCurveGeometryTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCurveGeometryTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CurveGeometry
    COMMAND QuantumCoreCurveGeometryTests
)

set_tests_properties(
    QuantumCore.CurveGeometry
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreArcLengthTests
    ArcLengthTests.cpp
)

target_link_libraries(
    QuantumCoreArcLengthTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreArcLengthTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreArcLengthTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.ArcLength
    COMMAND QuantumCoreArcLengthTests
)

set_tests_properties(
    QuantumCore.ArcLength
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreArcLengthLUTTests
    ArcLengthLUTTests.cpp
)

target_link_libraries(
    QuantumCoreArcLengthLUTTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreArcLengthLUTTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreArcLengthLUTTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.ArcLengthLUT
    COMMAND QuantumCoreArcLengthLUTTests
)

set_tests_properties(
    QuantumCore.ArcLengthLUT
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreCurveSamplingTests
    CurveSamplingTests.cpp
)

target_link_libraries(
    QuantumCoreCurveSamplingTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCurveSamplingTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCurveSamplingTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CurveSampling
    COMMAND QuantumCoreCurveSamplingTests
)

set_tests_properties(
    QuantumCore.CurveSampling
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreRotationMinimizingFramesTests
    RotationMinimizingFramesTests.cpp
)

target_link_libraries(
    QuantumCoreRotationMinimizingFramesTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreRotationMinimizingFramesTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreRotationMinimizingFramesTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.RotationMinimizingFrames
    COMMAND QuantumCoreRotationMinimizingFramesTests
)

set_tests_properties(
    QuantumCore.RotationMinimizingFrames
    PROPERTIES
        LABELS "core;geometry"
)

add_executable(
    QuantumCoreTransitionFunctionsTests
    TransitionFunctionsTests.cpp
)

target_link_libraries(
    QuantumCoreTransitionFunctionsTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreTransitionFunctionsTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreTransitionFunctionsTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.TransitionFunctions
    COMMAND QuantumCoreTransitionFunctionsTests
)

set_tests_properties(
    QuantumCore.TransitionFunctions
    PROPERTIES
        LABELS "core;math"
)

add_executable(
    QuantumCoreScalarTransitionTests
    ScalarTransitionTests.cpp
)

target_link_libraries(
    QuantumCoreScalarTransitionTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreScalarTransitionTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreScalarTransitionTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.ScalarTransition
    COMMAND QuantumCoreScalarTransitionTests
)

set_tests_properties(
    QuantumCore.ScalarTransition
    PROPERTIES
        LABELS "core;math"
)

add_executable(
    QuantumCoreAuthoredSectionTests
    AuthoredSectionTests.cpp
)

target_link_libraries(
    QuantumCoreAuthoredSectionTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreAuthoredSectionTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreAuthoredSectionTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.AuthoredSections
    COMMAND QuantumCoreAuthoredSectionTests
)

set_tests_properties(
    QuantumCore.AuthoredSections
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreRiderLocalGeometryTests
    RiderLocalGeometryTests.cpp
)

target_link_libraries(
    QuantumCoreRiderLocalGeometryTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreRiderLocalGeometryTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreRiderLocalGeometryTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.RiderLocalGeometry
    COMMAND QuantumCoreRiderLocalGeometryTests
)

set_tests_properties(
    QuantumCore.RiderLocalGeometry
    PROPERTIES
        LABELS "core;coaster;geometry"
)

add_executable(
    QuantumCoreRiderLocalGeometryEndpointTests
    RiderLocalGeometryEndpointTests.cpp
)

target_link_libraries(
    QuantumCoreRiderLocalGeometryEndpointTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreRiderLocalGeometryEndpointTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreRiderLocalGeometryEndpointTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.RiderLocalGeometryEndpoint
    COMMAND QuantumCoreRiderLocalGeometryEndpointTests
)

set_tests_properties(
    QuantumCore.RiderLocalGeometryEndpoint
    PROPERTIES
        LABELS "core;coaster;geometry"
)

add_executable(
    QuantumCoreVariableLocalPitchRateTests
    VariableLocalPitchRateTests.cpp
)

target_link_libraries(
    QuantumCoreVariableLocalPitchRateTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreVariableLocalPitchRateTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreVariableLocalPitchRateTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.VariableLocalPitchRate
    COMMAND QuantumCoreVariableLocalPitchRateTests
)

set_tests_properties(
    QuantumCore.VariableLocalPitchRate
    PROPERTIES
        LABELS "core;coaster;geometry;math"
)

add_executable(
    QuantumCoreLocalYawRateTests
    LocalYawRateTests.cpp
)

target_link_libraries(
    QuantumCoreLocalYawRateTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreLocalYawRateTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreLocalYawRateTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.LocalYawRate
    COMMAND QuantumCoreLocalYawRateTests
)

set_tests_properties(
    QuantumCore.LocalYawRate
    PROPERTIES
        LABELS "core;coaster;geometry;math"
)

add_executable(
    QuantumCoreLocalRollRateTests
    LocalRollRateTests.cpp
)

target_link_libraries(
    QuantumCoreLocalRollRateTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreLocalRollRateTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreLocalRollRateTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.LocalRollRate
    COMMAND QuantumCoreLocalRollRateTests
)

set_tests_properties(
    QuantumCore.LocalRollRate
    PROPERTIES
        LABELS "core;coaster;geometry;math"
)

add_executable(
    QuantumCoreCoupledLocalPitchYawRateTests
    CoupledLocalPitchYawRateTests.cpp
)

target_link_libraries(
    QuantumCoreCoupledLocalPitchYawRateTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCoupledLocalPitchYawRateTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCoupledLocalPitchYawRateTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CoupledLocalPitchYawRate
    COMMAND QuantumCoreCoupledLocalPitchYawRateTests
)

set_tests_properties(
    QuantumCore.CoupledLocalPitchYawRate
    PROPERTIES
        LABELS "core;coaster;geometry;math"
)

add_executable(
    QuantumCoreLocalRollPitchYawRateTests
    LocalRollPitchYawRateTests.cpp
)

target_link_libraries(
    QuantumCoreLocalRollPitchYawRateTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreLocalRollPitchYawRateTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreLocalRollPitchYawRateTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.LocalRollPitchYawRate
    COMMAND QuantumCoreLocalRollPitchYawRateTests
)

set_tests_properties(
    QuantumCore.LocalRollPitchYawRate
    PROPERTIES
        LABELS "core;coaster;geometry;math"
)

add_executable(
    QuantumEditorViewportCameraTests
    ViewportCameraTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
)

target_include_directories(
    QuantumEditorViewportCameraTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorViewportCameraTests
    PRIVATE
        glm::glm
)

target_compile_features(
    QuantumEditorViewportCameraTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorViewportCameraTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.ViewportCamera
    COMMAND QuantumEditorViewportCameraTests
)

set_tests_properties(
    QuantumEditor.ViewportCamera
    PROPERTIES
        LABELS "editor;camera;math"
)

add_executable(
    QuantumEditorViewportPickingTests
    ViewportPickingTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportPicking.cpp
)

target_include_directories(
    QuantumEditorViewportPickingTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorViewportPickingTests
    PRIVATE
        Quantum::Core
        Quantum::Engine
        glm::glm
)

target_compile_features(
    QuantumEditorViewportPickingTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorViewportPickingTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.ViewportPicking
    COMMAND QuantumEditorViewportPickingTests
)

set_tests_properties(
    QuantumEditor.ViewportPicking
    PROPERTIES
        LABELS "editor;camera;selection;math"
)

add_executable(
    QuantumEditorViewportTrackAnchorTests
    ViewportTrackAnchorTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportTrackAnchors.cpp
)

target_include_directories(
    QuantumEditorViewportTrackAnchorTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorViewportTrackAnchorTests
    PRIVATE
        Quantum::Core
        glm::glm
)

target_compile_features(
    QuantumEditorViewportTrackAnchorTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorViewportTrackAnchorTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.ViewportTrackAnchors
    COMMAND QuantumEditorViewportTrackAnchorTests
)

set_tests_properties(
    QuantumEditor.ViewportTrackAnchors
    PROPERTIES
        LABELS "editor;camera;selection;coaster"
)

add_executable(
    QuantumEditorAuthoredTrackEditTransactionTests
    AuthoredTrackEditTransactionTests.cpp
)

target_include_directories(
    QuantumEditorAuthoredTrackEditTransactionTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorAuthoredTrackEditTransactionTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorAuthoredTrackEditTransactionTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorAuthoredTrackEditTransactionTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.AuthoredTrackEditTransaction
    COMMAND QuantumEditorAuthoredTrackEditTransactionTests
)

set_tests_properties(
    QuantumEditor.AuthoredTrackEditTransaction
    PROPERTIES
        LABELS "editor;coaster;transaction;selection"
)

add_executable(
    QuantumCoreAuthoredTrackTests
    AuthoredTrackTests.cpp
)

add_executable(
    QuantumEditorAuthoredStartPoseTests
    AuthoredStartPoseTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/CenterlineVisualization.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/RiderLoadDiagnostics.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportTrackAnchors.cpp
)

target_include_directories(
    QuantumEditorAuthoredStartPoseTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorAuthoredStartPoseTests
    PRIVATE
        Quantum::Core
        Quantum::Engine
        glm::glm
)

target_compile_features(
    QuantumEditorAuthoredStartPoseTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorAuthoredStartPoseTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.AuthoredStartPose
    COMMAND QuantumEditorAuthoredStartPoseTests
)

set_tests_properties(
    QuantumEditor.AuthoredStartPose
    PROPERTIES
        LABELS "editor;core;coaster;transaction;camera;physics;serialization"
)

target_link_libraries(
    QuantumCoreAuthoredTrackTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreAuthoredTrackTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreAuthoredTrackTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.AuthoredTrack
    COMMAND QuantumCoreAuthoredTrackTests
)

set_tests_properties(
    QuantumCore.AuthoredTrack
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreAuthoredTrackGoldenTests
    AuthoredTrackGoldenTests.cpp
)

target_link_libraries(
    QuantumCoreAuthoredTrackGoldenTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreAuthoredTrackGoldenTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreAuthoredTrackGoldenTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.AuthoredTrackGolden
    COMMAND QuantumCoreAuthoredTrackGoldenTests
)

set_tests_properties(
    QuantumCore.AuthoredTrackGolden
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreChannelProfileTests
    ChannelProfileTests.cpp
)

target_link_libraries(
    QuantumCoreChannelProfileTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreChannelProfileTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreChannelProfileTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.ChannelProfile
    COMMAND QuantumCoreChannelProfileTests
)

set_tests_properties(
    QuantumCore.ChannelProfile
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreChannelProfileEditingTests
    ChannelProfileEditingTests.cpp
)

target_link_libraries(
    QuantumCoreChannelProfileEditingTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreChannelProfileEditingTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreChannelProfileEditingTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.ChannelProfileEditing
    COMMAND QuantumCoreChannelProfileEditingTests
)

set_tests_properties(
    QuantumCore.ChannelProfileEditing
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCorePlanarArcRegionTests
    PlanarArcRegionTests.cpp
)

target_link_libraries(
    QuantumCorePlanarArcRegionTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCorePlanarArcRegionTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCorePlanarArcRegionTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.PlanarArcRegion
    COMMAND QuantumCorePlanarArcRegionTests
)

set_tests_properties(
    QuantumCore.PlanarArcRegion
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreRiderLoadTests
    RiderLoadTests.cpp
)

target_link_libraries(
    QuantumCoreRiderLoadTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreRiderLoadTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreRiderLoadTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.RiderLoads
    COMMAND QuantumCoreRiderLoadTests
)

set_tests_properties(
    QuantumCore.RiderLoads
    PROPERTIES
        LABELS "core;coaster;physics"
)

add_executable(
    QuantumEditorRiderLoadDiagnosticsTests
    RiderLoadDiagnosticsTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/RiderLoadDiagnostics.cpp
)

target_include_directories(
    QuantumEditorRiderLoadDiagnosticsTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorRiderLoadDiagnosticsTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorRiderLoadDiagnosticsTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorRiderLoadDiagnosticsTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.RiderLoadDiagnostics
    COMMAND QuantumEditorRiderLoadDiagnosticsTests
)

set_tests_properties(
    QuantumEditor.RiderLoadDiagnostics
    PROPERTIES
        LABELS "editor;coaster;physics;diagnostics"
)

add_executable(
    QuantumCoreRegionAuthoringWorkflowTests
    RegionAuthoringWorkflowTests.cpp
)

target_link_libraries(
    QuantumCoreRegionAuthoringWorkflowTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreRegionAuthoringWorkflowTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreRegionAuthoringWorkflowTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.RegionAuthoringWorkflow
    COMMAND QuantumCoreRegionAuthoringWorkflowTests
)

set_tests_properties(
    QuantumCore.RegionAuthoringWorkflow
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumEditorRegionSummaryTests
    RegionSummaryTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/RegionSummary.cpp
)

target_include_directories(
    QuantumEditorRegionSummaryTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorRegionSummaryTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorRegionSummaryTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorRegionSummaryTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.RegionSummary
    COMMAND QuantumEditorRegionSummaryTests
)

set_tests_properties(
    QuantumEditor.RegionSummary
    PROPERTIES
        LABELS "editor;coaster"
)

add_executable(
    QuantumEditorTransitionEditorModelTests
    TransitionEditorModelTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/TransitionEditorModel.cpp
)

target_include_directories(
    QuantumEditorTransitionEditorModelTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorTransitionEditorModelTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorTransitionEditorModelTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorTransitionEditorModelTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.TransitionEditorModel
    COMMAND QuantumEditorTransitionEditorModelTests
)

set_tests_properties(
    QuantumEditor.TransitionEditorModel
    PROPERTIES
        LABELS "editor;coaster;transition"
)

add_executable(
    QuantumEditorRollProfileAuthoringTests
    RollProfileAuthoringTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/RegionSummary.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/TransitionEditorModel.cpp
)

target_include_directories(
    QuantumEditorRollProfileAuthoringTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorRollProfileAuthoringTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorRollProfileAuthoringTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorRollProfileAuthoringTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.RollProfileAuthoring
    COMMAND QuantumEditorRollProfileAuthoringTests
)

set_tests_properties(
    QuantumEditor.RollProfileAuthoring
    PROPERTIES
        LABELS "editor;coaster;transition;roll"
)

add_executable(
    QuantumEditorCenterlineVisualizationTests
    CenterlineVisualizationTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/CenterlineVisualization.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportCamera.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/ViewportTrackAnchors.cpp
)

target_include_directories(
    QuantumEditorCenterlineVisualizationTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorCenterlineVisualizationTests
    PRIVATE
        Quantum::Core
        Quantum::Engine
)

target_compile_features(
    QuantumEditorCenterlineVisualizationTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorCenterlineVisualizationTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.CenterlineVisualization
    COMMAND QuantumEditorCenterlineVisualizationTests
)

set_tests_properties(
    QuantumEditor.CenterlineVisualization
    PROPERTIES
        LABELS "editor;coaster;visualization"
)

add_executable(
    QuantumCoreAuthoredTrackStructureTests
    AuthoredTrackStructureTests.cpp
)

target_link_libraries(
    QuantumCoreAuthoredTrackStructureTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreAuthoredTrackStructureTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreAuthoredTrackStructureTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.AuthoredTrackStructure
    COMMAND QuantumCoreAuthoredTrackStructureTests
)

set_tests_properties(
    QuantumCore.AuthoredTrackStructure
    PROPERTIES
        LABELS "core;coaster"
)

add_executable(
    QuantumCoreCoasterDocumentTests
    CoasterDocumentTests.cpp
)

target_link_libraries(
    QuantumCoreCoasterDocumentTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCoasterDocumentTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCoasterDocumentTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CoasterDocument
    COMMAND QuantumCoreCoasterDocumentTests
)

set_tests_properties(
    QuantumCore.CoasterDocument
    PROPERTIES
        LABELS "core;coaster;serialization"
)

add_executable(
    QuantumEditorDocumentTests
    EditorDocumentTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/DocumentState.cpp
)

target_include_directories(
    QuantumEditorDocumentTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorDocumentTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorDocumentTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorDocumentTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.DocumentState
    COMMAND QuantumEditorDocumentTests
)

set_tests_properties(
    QuantumEditor.DocumentState
    PROPERTIES
        LABELS "editor;document"
)

add_executable(
    QuantumEditorDocumentHistoryTests
    DocumentHistoryTests.cpp
    ${PROJECT_SOURCE_DIR}/editor/src/DocumentHistory.cpp
)

target_include_directories(
    QuantumEditorDocumentHistoryTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/editor/include
)

target_link_libraries(
    QuantumEditorDocumentHistoryTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumEditorDocumentHistoryTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumEditorDocumentHistoryTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumEditor.DocumentHistory
    COMMAND QuantumEditorDocumentHistoryTests
)

set_tests_properties(
    QuantumEditor.DocumentHistory
    PROPERTIES
        LABELS "editor;document;history;transaction"
)

add_executable(
    QuantumCoreTrackTopologyTests
    TrackTopologyTests.cpp
)

target_link_libraries(
    QuantumCoreTrackTopologyTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreTrackTopologyTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreTrackTopologyTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.TrackTopology
    COMMAND QuantumCoreTrackTopologyTests
)

set_tests_properties(
    QuantumCore.TrackTopology
    PROPERTIES
        LABELS "core;coaster;topology"
)

add_executable(
    QuantumCoreLayoutModeTests
    LayoutModeTests.cpp
)

target_link_libraries(
    QuantumCoreLayoutModeTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreLayoutModeTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreLayoutModeTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.LayoutMode
    COMMAND QuantumCoreLayoutModeTests
)

set_tests_properties(
    QuantumCore.LayoutMode
    PROPERTIES
        LABELS "core;coaster;layout"
)

add_executable(
    QuantumCoreCircuitCompletionTests
    CircuitCompletionTests.cpp
)

target_link_libraries(
    QuantumCoreCircuitCompletionTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCircuitCompletionTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCircuitCompletionTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CircuitCompletion
    COMMAND QuantumCoreCircuitCompletionTests
)

set_tests_properties(
    QuantumCore.CircuitCompletion
    PROPERTIES
        LABELS "core;coaster;topology"
)

add_executable(
    QuantumCoreCircuitCompletionSensitivityTests
    CircuitCompletionSensitivityTests.cpp
)

target_link_libraries(
    QuantumCoreCircuitCompletionSensitivityTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCircuitCompletionSensitivityTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCircuitCompletionSensitivityTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CircuitCompletionSensitivity
    COMMAND QuantumCoreCircuitCompletionSensitivityTests
)

set_tests_properties(
    QuantumCore.CircuitCompletionSensitivity
    PROPERTIES
        LABELS "core;coaster;geometry;topology"
)

add_executable(
    QuantumCoreCircuitCompletionLmShadowTests
    CircuitCompletionLmShadowTests.cpp
)

target_link_libraries(
    QuantumCoreCircuitCompletionLmShadowTests
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCircuitCompletionLmShadowTests
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCircuitCompletionLmShadowTests
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_test(
    NAME QuantumCore.CircuitCompletionLmShadow
    COMMAND QuantumCoreCircuitCompletionLmShadowTests
)

set_tests_properties(
    QuantumCore.CircuitCompletionLmShadow
    PROPERTIES
        LABELS "core;coaster;geometry;topology;performance"
)

add_executable(
    QuantumCoreCircuitCompletionSensitivityBenchmark
    CircuitCompletionSensitivityBenchmark.cpp
)

target_link_libraries(
    QuantumCoreCircuitCompletionSensitivityBenchmark
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuantumCoreCircuitCompletionSensitivityBenchmark
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuantumCoreCircuitCompletionSensitivityBenchmark
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_executable(
    CompletionDebug
    CompletionDebugMain.cpp
)

target_link_libraries(
    CompletionDebug
    PRIVATE
        Quantum::Core
)

target_compile_features(
    CompletionDebug
    PRIVATE
        cxx_std_23
)

set_target_properties(
    CompletionDebug
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_executable(
    FeasibilityTest
    FeasibilityTest.cpp
)

target_link_libraries(
    FeasibilityTest
    PRIVATE
        Quantum::Core
)

target_compile_features(
    FeasibilityTest
    PRIVATE
        cxx_std_23
)

set_target_properties(
    FeasibilityTest
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_executable(
    InitialSeedSweep
    InitialSeedSweep.cpp
)

target_link_libraries(
    InitialSeedSweep
    PRIVATE
        Quantum::Core
)

target_compile_features(
    InitialSeedSweep
    PRIVATE
        cxx_std_23
)

set_target_properties(
    InitialSeedSweep
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_executable(
    QuickSeedTest
    QuickSeedTest.cpp
)

target_link_libraries(
    QuickSeedTest
    PRIVATE
        Quantum::Core
)

target_compile_features(
    QuickSeedTest
    PRIVATE
        cxx_std_23
)

set_target_properties(
    QuickSeedTest
    PROPERTIES
        CXX_EXTENSIONS OFF
)

add_executable(
    OmegaSweep
    OmegaSweep.cpp
)

target_compile_features(
    OmegaSweep
    PRIVATE
        cxx_std_23
)

add_executable(
    JacobianRankTest
    JacobianRankTest.cpp
)

target_link_libraries(
    JacobianRankTest
    PRIVATE
        Quantum::Core
)

target_compile_features(
    JacobianRankTest
    PRIVATE
        cxx_std_23
)

set_target_properties(
    JacobianRankTest
    PROPERTIES
        CXX_EXTENSIONS OFF
)

set_target_properties(
    OmegaSweep
    PROPERTIES
        CXX_EXTENSIONS OFF
)
