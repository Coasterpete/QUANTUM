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
    QuantumCoreAuthoredTrackTests
    AuthoredTrackTests.cpp
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
