#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <glm/vec3.hpp>

namespace quantum::coaster
{
    enum class TrackEndpointRole
    {
        Start,
        End
    };

    struct TrackEndpoint
    {
        glm::dvec3 position{0.0};
        glm::dvec3 tangent{1.0, 0.0, 0.0};
        glm::dvec3 up{0.0, 0.0, 1.0};
        TrackEndpointRole role = TrackEndpointRole::Start;
    };

    enum class TopologyKind
    {
        OpenLinear,
        ClosedCircuit
    };

    struct TopologyTolerances
    {
        double positionTolerance = 0.001;
        double angleTolerance = 0.5;
        double closureGapTolerance = 0.01;
    };

    struct ClosureDiagnostics
    {
        double positionalGap = 0.0;
        double tangentMismatchDegrees = 0.0;
        double frameMismatchDegrees = 0.0;
        bool closureValid = false;
    };

    struct TrackTopology
    {
        TopologyKind kind = TopologyKind::OpenLinear;
        TrackEndpoint startEndpoint{};
        TrackEndpoint endEndpoint{};
        ClosureDiagnostics diagnostics{};
    };

    [[nodiscard]] TrackEndpoint extractStartEndpoint(
        const AuthoredTrack& track,
        double integrationSpacing = 0.75
    );

    [[nodiscard]] TrackEndpoint extractEndEndpoint(
        const AuthoredTrack& track,
        double integrationSpacing = 0.75
    );

    [[nodiscard]] TrackTopology computeTrackTopology(
        const AuthoredTrack& track,
        const TopologyTolerances& tolerances = TopologyTolerances{},
        double integrationSpacing = 0.75
    );
}
