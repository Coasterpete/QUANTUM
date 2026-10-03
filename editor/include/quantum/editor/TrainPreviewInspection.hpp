#pragma once

#include <quantum/physics/TrainPhysics.hpp>

#include <cstddef>
#include <optional>

namespace quantum::editor
{
    // A copy of accepted preview inputs, published only after rebuild. These
    // backend defaults are not a second authored train or saved document data.
    struct TrainPreviewInspection
    {
        std::size_t carCount = 0;
        physics::TrainCarDefinition repeatedCar;
        double loadedCarMassKilograms = 0.0;
        double totalTrainMassKilograms = 0.0;
        std::optional<double> connectorLengthMeters;
        physics::BasicResistance resistance;
    };
}
