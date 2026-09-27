#pragma once

#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <glm/vec3.hpp>

namespace quantum::coaster
{
    // Construction-independent geometric-reference state. Distance and
    // position describe the track construction reference used by rails,
    // supports, bogies, and train poses. centerlineCurvature is dT/ds.
    // localFrameRates=(roll,pitch,yaw), rollRateDerivative, and
    // pitchRateDerivative retain exactly the frame motion needed to evaluate
    // a rider reference offset without reinterpreting either position.
    struct TrackKinematicState
    {
        double distance;
        glm::dvec3 position;
        geometry::CurveFrame frame;
        glm::dvec3 centerlineCurvature;
        glm::dvec3 localFrameRates{0.0};
        double rollRateDerivative = 0.0;
        double pitchRateDerivative = 0.0;
    };

    [[nodiscard]] inline glm::dvec3 riderReferencePosition(
        const TrackKinematicState& state,
        const double offsetCoordinateUnits) noexcept
    {
        return state.position + offsetCoordinateUnits * state.frame.up;
    }
}
