add_library(QuantumCore STATIC)

add_library(Quantum::Core ALIAS QuantumCore)

target_sources(
    QuantumCore
    PRIVATE
        src/ArcLengthLUT.cpp
        src/AuthoredSections.cpp
        src/AuthoredTrack.cpp
        src/ChannelProfileEditing.cpp
        src/CircuitCompletion.cpp
        src/CoasterDocument.cpp
        src/ForceDrivenRegion.cpp
        src/TrackPhysicalSettings.cpp
        src/TrackStyle.cpp
        src/BSplineCurve.cpp
        src/NurbsCurve.cpp
        src/PlanarArcRegion.cpp
        src/RiderLoads.cpp
        src/RiderLocalGeometry.cpp
        src/ScalarTransition.cpp
        src/TrackTopology.cpp
        src/TransitionFunctions.cpp
        include/quantum/coaster/AuthoredTrack.hpp
        include/quantum/coaster/ChannelProfileEditing.hpp
        include/quantum/coaster/CircuitCompletion.hpp
        include/quantum/coaster/CoasterDocument.hpp
        include/quantum/coaster/GeometricSection.hpp
        include/quantum/coaster/ForceDrivenRegion.hpp
        include/quantum/coaster/TrackGeneration.hpp
        include/quantum/coaster/TrackPhysicalSettings.hpp
        include/quantum/coaster/TrackStyle.hpp
        include/quantum/coaster/PlanarArcRegion.hpp
        include/quantum/coaster/RiderLoads.hpp
        include/quantum/coaster/RiderLocalGeometry.hpp
        include/quantum/coaster/TrackKinematics.hpp
        include/quantum/coaster/TrackTopology.hpp
        include/quantum/coaster/detail/CircuitCompletionDetail.hpp
        include/quantum/coaster/detail/RiderLocalGeometryDetail.hpp
        include/quantum/geometry/ArcLengthLUT.hpp
        include/quantum/geometry/BSplineCurve.hpp
        include/quantum/geometry/CurveGeometry.hpp
        include/quantum/geometry/CurveSampling.hpp
        include/quantum/geometry/NurbsCurve.hpp
        include/quantum/geometry/RotationMinimizingFrames.hpp
        include/quantum/math/ScalarTransition.hpp
        include/quantum/math/TransitionFunctions.hpp
)

target_include_directories(
    QuantumCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(
    QuantumCore
    PUBLIC
        cxx_std_23
)

target_link_libraries(
    QuantumCore
    PUBLIC
        glm::glm
        nlohmann_json::nlohmann_json
)

set_target_properties(
    QuantumCore
    PROPERTIES
        CXX_EXTENSIONS OFF
)
