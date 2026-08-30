#pragma once

#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackPhysicalSettings.hpp>

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
    };

    [[nodiscard]] inline RiderLoadEvaluationSettings riderLoadEvaluationSettings(
        const TrackPhysicalSettings& settings)
    {
        return {settings.initialSpeed, settings.metersPerCoordinateUnit,
            settings.gravityAcceleration};
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

    // Evaluates one continuous gravity-only point-mass speed/load history.
    // The evaluator consumes only canonical kinematics and never authored
    // region kinds. Invalid settings or malformed canonical input throw
    // std::invalid_argument. A physically unreachable sample is reported in
    // the result and evaluation stops before fabricating subsequent states.
    [[nodiscard]] RiderLoadHistory evaluateRiderLoads(
        std::span<const TrackKinematicState> kinematics,
        const RiderLoadEvaluationSettings& settings
    );
}
