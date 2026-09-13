#pragma once

#include <quantum/physics/TrainPhysics.hpp>

#include <cstddef>

namespace quantum::coaster
{
    // Authored physical data for the single repeated-car train shape needed by
    // the current preview. Resolution expands it into the ordered runtime
    // physics definition consumed by TrainPhysics.
    struct TrainConfiguration
    {
        physics::TrainCarDefinition repeatedCar;
        std::size_t carCount = 1;
        physics::InterCarConnectionDefinition repeatedConnection;
        physics::BasicResistance resistance;
    };

    [[nodiscard]] TrainConfiguration createDefaultTrainConfiguration();

    // Returns a complete validated runtime physics definition. The authored
    // configuration remains independent of editor and renderer state.
    [[nodiscard]] physics::TrainDefinition resolveTrainConfiguration(
        const TrainConfiguration& configuration);
}
