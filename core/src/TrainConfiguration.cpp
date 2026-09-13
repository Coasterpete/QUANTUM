#include <quantum/coaster/TrainConfiguration.hpp>

namespace quantum::coaster
{
    TrainConfiguration createDefaultTrainConfiguration()
    {
        TrainConfiguration configuration;

        physics::CarDefinition& car = configuration.repeatedCar.car;
        car.dryMassKilograms = 800.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.55};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.dryInertiaTensorBodyKgM2 =
            physics::makeUniformBoxInertiaTensorBodyKgM2(
                car.dryMassKilograms,
                car.bodyDimensionsMeters);
        car.frontHitchPositionMeters = {2.0, 0.0, 0.2};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.2};
        car.bogies = {
            physics::BogieDefinition{{1.15, 0.0, 0.0}},
            physics::BogieDefinition{{-1.15, 0.0, 0.0}}
        };

        configuration.repeatedCar.loadout = {
            200.0,
            {0.0, 0.0, 0.9}
        };
        configuration.carCount = 4;
        configuration.repeatedConnection.rigidLengthMeters = 0.5;

        // This train retains the existing aggregate resistance law; no
        // preview-specific motion or operations force is authored here.
        configuration.resistance.constantMechanicalForceNewtons = 500.0;
        configuration.resistance
            .linearResistanceCoefficientNewtonSecondsPerMeter = 50.0;
        configuration.resistance.airDensityKilogramsPerCubicMeter = 1.225;
        configuration.resistance.dragAreaSquareMeters = 2.5;
        configuration.resistance.rollingResistanceCoefficient = 0.01;

        return configuration;
    }

    physics::TrainDefinition resolveTrainConfiguration(
        const TrainConfiguration& configuration)
    {
        physics::TrainDefinition definition;
        definition.cars.assign(
            configuration.carCount,
            configuration.repeatedCar);
        if (configuration.carCount > 1)
        {
            definition.connections.assign(
                configuration.carCount - 1,
                configuration.repeatedConnection);
        }
        definition.resistance = configuration.resistance;

        physics::validateTrainDefinition(definition);
        return definition;
    }
}
