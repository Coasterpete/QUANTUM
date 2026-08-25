#include <quantum/coaster/TrackTopology.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace quantum::coaster
{
    namespace
    {
        constexpr double piRadians = 3.14159265358979323846;
        constexpr double degreesPerRadian = 180.0 / piRadians;

        [[nodiscard]] TrackEndpoint makeEndpoint(
            const RiderLocalGeometryState& state,
            TrackEndpointRole role)
        {
            return {state.position, state.frame.tangent, state.frame.up, role};
        }

        [[nodiscard]] std::vector<RiderLocalGeometryState> solveTrack(
            const AuthoredTrack& track,
            double spacing)
        {
            return integrateAuthoredTrack(track, spacing);
        }
    }

    TrackEndpoint extractStartEndpoint(
        const AuthoredTrack& track,
        double integrationSpacing)
    {
        if (track.sectionCount() == 0)
        {
            throw std::invalid_argument(
                "An authored track requires at least one section to "
                "extract endpoints."
            );
        }

        const std::vector<RiderLocalGeometryState> states =
            solveTrack(track, integrationSpacing);

        return makeEndpoint(states.front(), TrackEndpointRole::Start);
    }

    TrackEndpoint extractEndEndpoint(
        const AuthoredTrack& track,
        double integrationSpacing)
    {
        if (track.sectionCount() == 0)
        {
            throw std::invalid_argument(
                "An authored track requires at least one section to "
                "extract endpoints."
            );
        }

        const std::vector<RiderLocalGeometryState> states =
            solveTrack(track, integrationSpacing);

        return makeEndpoint(states.back(), TrackEndpointRole::End);
    }

    TrackTopology computeTrackTopology(
        const AuthoredTrack& track,
        const TopologyTolerances& tolerances,
        double integrationSpacing)
    {
        if (track.sectionCount() == 0)
        {
            throw std::invalid_argument(
                "An authored track requires at least one section to "
                "compute topology."
            );
        }

        const std::vector<RiderLocalGeometryState> states =
            solveTrack(track, integrationSpacing);

        TrackTopology topology;
        topology.startEndpoint = makeEndpoint(
            states.front(), TrackEndpointRole::Start);
        topology.endEndpoint = makeEndpoint(
            states.back(), TrackEndpointRole::End);

        const glm::dvec3 gap =
            topology.endEndpoint.position - topology.startEndpoint.position;
        topology.diagnostics.positionalGap = glm::length(gap);

        const double tangentDot = glm::clamp(
            glm::dot(
                topology.startEndpoint.tangent,
                topology.endEndpoint.tangent),
            -1.0,
            1.0);
        topology.diagnostics.tangentMismatchDegrees =
            std::acos(tangentDot) * degreesPerRadian;

        const double upDot = glm::clamp(
            glm::dot(
                topology.startEndpoint.up,
                topology.endEndpoint.up),
            -1.0,
            1.0);
        topology.diagnostics.frameMismatchDegrees =
            std::acos(upDot) * degreesPerRadian;

        topology.diagnostics.closureValid =
            topology.diagnostics.positionalGap
                <= tolerances.closureGapTolerance
            && topology.diagnostics.tangentMismatchDegrees
                <= tolerances.angleTolerance
            && topology.diagnostics.frameMismatchDegrees
                <= tolerances.angleTolerance;

        topology.kind = topology.diagnostics.closureValid
            ? TopologyKind::ClosedCircuit
            : TopologyKind::OpenLinear;

        return topology;
    }
}
