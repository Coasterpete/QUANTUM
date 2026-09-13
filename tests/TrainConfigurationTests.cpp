#include <quantum/coaster/TrainConfiguration.hpp>

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum;

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireEqual(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const std::string& message)
    {
        require(actual == expected, message);
    }

    void requireEqual(
        const glm::dmat3& actual,
        const glm::dmat3& expected,
        const std::string& message)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int row = 0; row < 3; ++row)
            {
                require(actual[column][row] == expected[column][row], message);
            }
        }
    }

    void defaultConfigurationResolvesToPreviousPreviewDefinition()
    {
        const coaster::TrainConfiguration configuration =
            coaster::createDefaultTrainConfiguration();
        const physics::TrainDefinition definition =
            coaster::resolveTrainConfiguration(configuration);

        require(definition.cars.size() == 4, "preview car count");
        require(definition.connections.size() == 3, "preview connection count");

        const glm::dvec3 expectedDimensions{4.0, 1.35, 1.4};
        const glm::dmat3 expectedInertia =
            physics::makeUniformBoxInertiaTensorBodyKgM2(
                800.0,
                expectedDimensions);
        for (std::size_t index = 0; index < definition.cars.size(); ++index)
        {
            const physics::TrainCarDefinition& trainCar =
                definition.cars[index];
            const physics::CarDefinition& car = trainCar.car;

            require(car.dryMassKilograms == 800.0, "car dry mass");
            requireEqual(
                car.dryCenterOfGravityMeters,
                {0.0, 0.0, 0.55},
                "car dry center of gravity");
            requireEqual(
                car.dryInertiaTensorBodyKgM2,
                expectedInertia,
                "car dry inertia tensor");
            requireEqual(
                car.bodyDimensionsMeters,
                expectedDimensions,
                "car body dimensions");
            requireEqual(
                car.frontHitchPositionMeters,
                {2.0, 0.0, 0.2},
                "front hitch position");
            requireEqual(
                car.rearHitchPositionMeters,
                {-2.0, 0.0, 0.2},
                "rear hitch position");
            require(car.bogies.size() == 2, "bogie count");
            requireEqual(
                car.bogies[0].referencePositionMeters,
                {1.15, 0.0, 0.0},
                "front bogie position");
            requireEqual(
                car.bogies[1].referencePositionMeters,
                {-1.15, 0.0, 0.0},
                "rear bogie position");
            require(car.bogies[0].contacts.empty()
                    && car.bogies[1].contacts.empty(),
                "preview bogie contacts remain empty");
            require(car.aerodynamicDragAreaSquareMeters == 0.0,
                "per-car aerodynamic drag remains disabled");
            requireEqual(
                car.aerodynamicCenterLocalMeters,
                {0.0, 0.0, 0.0},
                "per-car aerodynamic center");
            require(trainCar.loadout.massKilograms == 200.0,
                "car load mass");
            requireEqual(
                trainCar.loadout.centerOfMassMeters,
                {0.0, 0.0, 0.9},
                "car load center of mass");
        }

        for (const physics::InterCarConnectionDefinition& connection
            : definition.connections)
        {
            require(connection.rigidLengthMeters == 0.5,
                "connector rigid length");
        }

        require(definition.resistance.constantMechanicalForceNewtons == 500.0,
            "constant resistance");
        require(definition.resistance
                .linearResistanceCoefficientNewtonSecondsPerMeter == 50.0,
            "linear resistance");
        require(definition.resistance.airDensityKilogramsPerCubicMeter == 1.225,
            "air density");
        require(definition.resistance.dragAreaSquareMeters == 2.5,
            "aggregate drag area");
        require(definition.resistance.rollingResistanceCoefficient == 0.01,
            "rolling resistance");

        physics::validateTrainDefinition(definition);
    }
}

int main()
{
    try
    {
        defaultConfigurationResolvesToPreviousPreviewDefinition();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "TrainConfigurationTests failed: "
                  << exception.what() << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "TrainConfigurationTests passed\n";
    return EXIT_SUCCESS;
}
