#pragma once
#include <quantum/physics/TrackFollower.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace quantum::editor
{
    struct SimulationCarDiagnostics
    {
        std::uint64_t id = 0;
        double loadedMassKilograms = 0, referenceStationMeters = 0;
        double frontBogieStationMeters = 0, rearBogieStationMeters = 0;
    };

    // Captured from committed physics, never from render interpolation. Force
    // values describe the most recent fixed step, not an unstepped estimate.
    struct SimulationDynamicsDiagnostics
    {
        bool placementValid = false, hasStep = false, circuit = false;
        std::uint64_t tick = 0;
        double trackLengthMeters = 0, straightBogieSpanMeters = 0, bodyEnvelopeLengthMeters = 0;
        double minimumBogieStationMeters = 0, maximumBogieStationMeters = 0;
        double loadedMassKilograms = 0, effectiveMassKilograms = 0;
        double signedSpeedMetersPerSecond = 0, accelerationMetersPerSecondSquared = 0;
        double gravityForceNewtons = 0, externalForceNewtons = 0, resistanceForceNewtons = 0;
        double massGradientForceNewtons = 0, constraintForceNewtons = 0, netForceNewtons = 0;
        physics::BasicResistanceForceBreakdown resistance;
        physics::TrackBoundary boundary = physics::TrackBoundary::None;
        double maximumConnectorResidualMeters = 0;
        std::vector<SimulationCarDiagnostics> cars;
    };
}
