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
        double signedVelocityMetersPerSecond);
}
