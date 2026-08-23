#pragma once

#include <quantum/math/ScalarTransition.hpp>

namespace quantum::coaster
{
    // Initial authored geometric-section channels. Angles use radians. No
    // rider-local or world-oriented reference-frame meaning is assigned yet.
    struct GeometricSection
    {
        math::ScalarTransition pitch;
        math::ScalarTransition yaw;
        math::ScalarTransition roll;
    };

    // Authored channel values evaluated at one location in the section domain.
    // Their eventual solver interpretation is intentionally unspecified.
    struct GeometricSectionState
    {
        double pitch;
        double yaw;
        double roll;
    };

    // Throws std::invalid_argument when a channel is malformed or when the
    // authored channel domains do not match exactly.
    void validateGeometricSection(const GeometricSection& section);

    // Evaluates all authored channels at the same independent value. Section
    // validation and scalar-transition domain behavior remain authoritative.
    [[nodiscard]] GeometricSectionState evaluateGeometricSection(
        const GeometricSection& section,
        double independentValue
    );
}
