#pragma once

#include <quantum/coaster/TrackDevices.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <span>
#include <vector>

namespace quantum::physics
{
    // Transient Simulator control, initialized from an authored default.
    // Future interlocks can change this without mutating the document.
    struct TrackDeviceRuntimeState
    {
        coaster::TrackDeviceId id = 0;
        bool armed = false;
        std::optional<double> signedAccelerationCommand;
        bool compensateGravity = false;
    };

    [[nodiscard]] std::vector<TrackDeviceRuntimeState>
        initialTrackDeviceRuntimeStates(
            std::span<const coaster::TrackDevice> devices);

    struct TrackDeviceForceTelemetry
    {
        coaster::TrackDeviceId id = 0;
        std::size_t occupiedBogieCount = 0;
        double commandedAccelerationMetersPerSecondSquared = 0.0;
        double appliedForceNewtons = 0.0;
        // Launch profile request before target-speed limiting, averaged over
        // occupied bogies even when reaching the target releases propulsion.
        double profileAccelerationMetersPerSecondSquared = 0.0;
        bool targetSpeedLimited = false;
        // Requests after occupancy/controller/interlocks, before the force cap.
        double requestedForceNewtons = 0.0;
        double requestedAbsoluteForceNewtons = 0.0;
        double suppliedAbsoluteForceNewtons = 0.0;
        bool forceLimitReached = false;
        bool armed = false;
        std::size_t applicationBegin = 0, applicationCount = 0;
        // Published by the preview only after a successful production step.
        // Q = sum(F dot dp/dq), not the net force after gravity/resistance or
        // an allocation of endpoint reaction to this mechanism.
        std::optional<double> generalizedForceNewtons;
    };

    struct TrackDeviceForceResult
    {
        std::vector<ExternalForceApplication> applications;
        std::vector<TrackDeviceForceTelemetry> devices;
    };

    // Uses the already-solved pose at the beginning of the fixed step.
    // The output is ordinary per-car external force input to stepTrain.
    [[nodiscard]] TrackDeviceForceResult evaluateTrackDeviceForces(
        std::span<const coaster::TrackDevice> devices,
        std::span<const TrackDeviceRuntimeState> runtimeStates,
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrainPose& pose,
        double signedVelocityMetersPerSecond,
        double gravityAccelerationMetersPerSecondSquared = 9.80665);
}
