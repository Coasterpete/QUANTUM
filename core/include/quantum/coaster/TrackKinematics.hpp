#pragma once

#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <glm/vec3.hpp>

namespace quantum::coaster
{
    // Construction-independent centerline state. Distance, position, and
    // centerlineCurvature use the project's coordinate units; curvature is
    // the world-space derivative dT/ds in inverse coordinate units.
    struct TrackKinematicState
    {
        double distance;
        glm::dvec3 position;
        geometry::CurveFrame frame;
        glm::dvec3 centerlineCurvature;
    };
}
