#pragma once

#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackPhysicalSettings.hpp>
#include <quantum/coaster/CoasterSetup.hpp>

#include <optional>
#include <span>
#include <vector>

namespace quantum::coaster
{
    struct RiderLoadEvaluationSettings
    {
        // Vehicle speed is SI even though authored Core geometry remains
        // unit-neutral.
        double initialSpeed = 0.0;
        double metersPerCoordinateUnit = 1.0;
        double gravityAcceleration = standardGravityAcceleration;
        // SI distance from the construction reference along local +U. Zero
        // selects the construction reference and reproduces legacy loads.
        double riderReferenceOffsetMeters = 0.0;
    };

    [[nodiscard]] inline RiderLoadEvaluationSettings riderLoadEvaluationSettings(
        const TrackPhysicalSettings& settings)
    {
        return {settings.initialSpeed, settings.metersPerCoordinateUnit,
            settings.gravityAcceleration};
    }

    [[nodiscard]] inline RiderLoadEvaluationSettings riderLoadEvaluationSettings(
        const TrackPhysicalSettings& settings,
        const HeartlineSettings& heartline)
    {
        return {settings.initialSpeed, settings.metersPerCoordinateUnit,
            settings.gravityAcceleration,
            heartline.enabled ? heartline.offsetMeters : 0.0};
    }

    // Mass-independent rider specific force expressed in the rider frame.
    // Distance remains in Core coordinate units and speed is metres/second.
    struct RiderLoadState
    {
        double distance;
        double vehicleSpeed;
        double normalG;
        double lateralG;
        double longitudinalG;
        glm::dvec3 riderReferencePosition{0.0};

        [[nodiscard]] friend bool operator==(
            const RiderLoadState&,
            const RiderLoadState&) = default;
    };

    // The first sampled canonical state whose gravity-only energy would
    // require a materially negative speed squared.
    struct RiderLoadUnreachableState
    {
        double distance;
        double speedSquared;
    };

    struct RiderLoadHistory
    {
        std::vector<RiderLoadState> states;
        std::optional<RiderLoadUnreachableState> unreachable;

        [[nodiscard]] bool completed() const noexcept
        {
            return !unreachable.has_value();
        }
    };

    // Evaluates one continuous gravity-only construction-station speed/load
    // history at the configured rider reference C+hU. Construction energy and
    // station dynamics remain authoritative; offset-frame derivatives add the
    // rider point's rotational acceleration. The evaluator consumes only
    // canonical kinematics and never authored region kinds. Invalid settings
    // or malformed canonical input throw std::invalid_argument.
    [[nodiscard]] RiderLoadHistory evaluateRiderLoads(
        std::span<const TrackKinematicState> kinematics,
        const RiderLoadEvaluationSettings& settings
    );
}
