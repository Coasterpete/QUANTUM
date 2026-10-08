#pragma once

#include <quantum/physics/TrainPhysics.hpp>

#include <cstddef>
#include <optional>

namespace quantum::editor
{
    // Copies of accepted document inputs, published only after rebuild.
    struct TrainPreviewInspection
    {
        std::size_t carCount = 0;
        physics::TrainCarDefinition repeatedCar;
        double loadedCarMassKilograms = 0.0;
        double totalTrainMassKilograms = 0.0;
        std::optional<double> connectorLengthMeters;
        physics::BasicResistance resistance;
        std::vector<physics::TrainCarDefinition> cars;
    };
}
