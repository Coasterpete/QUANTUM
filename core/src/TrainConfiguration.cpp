#include <quantum/coaster/TrainConfiguration.hpp>
#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/coaster/CoasterSetup.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <glm/common.hpp>
#include <glm/vector_relational.hpp>

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
        definition.trainId = configuration.trainId;
        if (configuration.cars.empty())
        {
            definition.cars.assign(configuration.carCount, configuration.repeatedCar);
            if (configuration.carCount > 1)
                definition.connections.assign(configuration.carCount - 1, configuration.repeatedConnection);
        }
        else
        {
            for (std::size_t i = 0; i < configuration.cars.size(); ++i)
            {
                auto car = configuration.cars[i].physical;
                car.authoredCarId = configuration.cars[i].id;
                definition.cars.push_back(std::move(car));
                if (i + 1 < configuration.cars.size())
                    definition.connections.push_back(configuration.cars[i].rearConnection);
            }
        }
        definition.resistance = configuration.resistance;

        physics::validateTrainDefinition(definition);
        return definition;
    }

    const char* trainCarRoleName(TrainCarRole role)
    {
        switch (role)
        {
        case TrainCarRole::Lead: return "Lead";
        case TrainCarRole::Middle: return "Middle";
        case TrainCarRole::Rear: return "Rear";
        case TrainCarRole::Zero: return "Zero";
        case TrainCarRole::Trailer: return "Trailer";
        case TrainCarRole::Custom: return "Custom";
        }
        throw std::invalid_argument("Unknown train car role.");
    }

    TrainCarRole trainCarRoleFromName(std::string_view name)
    {
        for (auto role : {TrainCarRole::Lead, TrainCarRole::Middle, TrainCarRole::Rear,
            TrainCarRole::Zero, TrainCarRole::Trailer, TrainCarRole::Custom})
            if (name == trainCarRoleName(role)) return role;
        throw std::invalid_argument("Unknown train car role: " + std::string(name));
    }

    AuthoredTrainCar createTrainCarPrototype(TrainCarRole role)
    {
        const auto defaults = createDefaultTrainConfiguration();
        AuthoredTrainCar result;
        result.role = role;
        result.name = trainCarRoleName(role);
        result.physical = defaults.repeatedCar;
        result.rearConnection = defaults.repeatedConnection;
        result.visualAsset = "assets://train/placeholder-car-shell.glb";
        if (role == TrainCarRole::Lead || role == TrainCarRole::Zero)
            result.visualAsset = "assets://train/placeholder-lead-car-shell.glb";
        if (role == TrainCarRole::Rear)
            result.visualAsset = "assets://train/placeholder-rear-car-shell.glb";
        // Generic engineering presets, always editable. The two-bogie rigid
        // model is retained for Zero and Trailer; shared/single bogies wait.
        auto& car = result.physical.car;
        if (role == TrainCarRole::Zero || role == TrainCarRole::Trailer)
        {
            car.dryMassKilograms = role == TrainCarRole::Zero ? 350.0 : 550.0;
            car.bodyDimensionsMeters = role == TrainCarRole::Zero
                ? glm::dvec3{2.0, 1.2, 0.8} : glm::dvec3{3.0, 1.35, 1.2};
            car.dryCenterOfGravityMeters.z = role == TrainCarRole::Zero ? 0.3 : 0.5;
            car.frontHitchPositionMeters.x = car.bodyDimensionsMeters.x * 0.5;
            car.rearHitchPositionMeters.x = -car.frontHitchPositionMeters.x;
            car.bogies[0].referencePositionMeters.x = car.bodyDimensionsMeters.x * 0.3;
            car.bogies[1].referencePositionMeters.x = -car.bogies[0].referencePositionMeters.x;
            if (role == TrainCarRole::Zero) result.physical.loadout.massKilograms = 0;
            car.dryInertiaTensorBodyKgM2 = physics::makeUniformBoxInertiaTensorBodyKgM2(
                car.dryMassKilograms, car.bodyDimensionsMeters);
        }
        return result;
    }

    TrainConfiguration createDefaultOrderedTrainConfiguration(std::size_t count)
    {
        if (count < minimumCarsPerTrain || count > maximumCarsPerTrain)
            throw std::invalid_argument("Cars per train must be within [1, 32].");
        auto result = createDefaultTrainConfiguration();
        result.cars.clear();
        for (std::size_t i = 0; i < count; ++i)
        {
            auto car = createTrainCarPrototype(i == 0 ? TrainCarRole::Lead
                : i + 1 == count ? TrainCarRole::Rear : TrainCarRole::Middle);
            car.id = result.nextCarId++;
            result.cars.push_back(std::move(car));
        }
        result.carCount = count;
        return result;
    }

    TrainCarId insertTrainCar(TrainConfiguration& configuration, std::size_t index, AuthoredTrainCar car)
    {
        if (index > configuration.cars.size() || configuration.cars.size() >= maximumCarsPerTrain)
            throw std::invalid_argument("Consist insertion is outside the supported 1–32 car range.");
        if (!configuration.nextCarId || configuration.nextCarId == std::numeric_limits<TrainCarId>::max())
            throw std::invalid_argument("Train car identity space is exhausted.");
        car.id = configuration.nextCarId++;
        car.physical.authoredCarId = 0;
        const auto id = car.id;
        configuration.cars.insert(configuration.cars.begin() + index, std::move(car));
        configuration.carCount = configuration.cars.size();
        return id;
    }

    void resizeTrainConsist(TrainConfiguration& configuration, std::size_t count)
    {
        if (count < minimumCarsPerTrain || count > maximumCarsPerTrain)
            throw std::invalid_argument("Cars per train must be within [1, 32].");
        while (configuration.cars.size() < count)
        {
            auto car = createTrainCarPrototype(TrainCarRole::Middle);
            car.physical = configuration.repeatedCar;
            car.rearConnection = configuration.repeatedConnection;
            static_cast<void>(insertTrainCar(configuration, configuration.cars.size(), std::move(car)));
        }
        configuration.cars.resize(count);
        configuration.carCount = count;
    }

    void removeTrainCar(TrainConfiguration& configuration, TrainCarId id)
    {
        if (configuration.cars.size() <= 1) throw std::invalid_argument("A train needs at least one car.");
        const auto it = std::ranges::find(configuration.cars, id, &AuthoredTrainCar::id);
        if (it == configuration.cars.end()) throw std::invalid_argument("Unknown train car ID.");
        configuration.cars.erase(it);
        configuration.carCount = configuration.cars.size();
    }

    void reorderTrainCar(TrainConfiguration& configuration, TrainCarId id, std::size_t index)
    {
        auto it = std::ranges::find(configuration.cars, id, &AuthoredTrainCar::id);
        if (it == configuration.cars.end() || index >= configuration.cars.size())
            throw std::invalid_argument("Invalid car ID or destination index.");
        auto car = std::move(*it);
        configuration.cars.erase(it);
        configuration.cars.insert(configuration.cars.begin() + index, std::move(car));
    }

    TrainSummary summarizeTrainConfiguration(const TrainConfiguration& configuration)
    {
        TrainSummary result;
        result.carCount = configuration.cars.size();
        result.connectorCount = result.carCount ? result.carCount - 1 : 0;
        const auto issue = [&](bool error, TrainCarId id, std::string message)
            { result.diagnostics.push_back({error, id, std::move(message)}); };
        if (!configuration.trainId) issue(true, 0, "Train ID must be nonzero.");
        if (result.carCount < minimumCarsPerTrain || result.carCount > maximumCarsPerTrain)
            issue(true, 0, "The supported consist contains 1–32 cars.");
        std::set<TrainCarId> ids;
        result.frontExtentMeters = -std::numeric_limits<double>::infinity();
        result.rearExtentMeters = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < result.carCount; ++i)
        {
            const auto& authored = configuration.cars[i];
            const auto& car = authored.physical.car;
            if (!authored.id || !ids.insert(authored.id).second || authored.id >= configuration.nextCarId)
                issue(true, authored.id, "Car IDs must be unique, nonzero and below nextCarId.");
            try
            {
                static_cast<void>(trainCarRoleName(authored.role));
                physics::validateCarDefinition(car);
                physics::validateCarLoadout(authored.physical.loadout);
                physics::validateInterCarConnectionDefinition(authored.rearConnection);
            }
            catch (const std::exception& error) { issue(true, authored.id, error.what()); continue; }
            if (car.bogies.size() != 2 || std::abs(car.bogies[0].referencePositionMeters.x
                - car.bogies[1].referencePositionMeters.x) <= 1e-6)
                issue(true, authored.id, "The rigid solver requires exactly two longitudinally separated bogies.");
            for (const auto& bogie : car.bogies)
                if (std::abs(bogie.referencePositionMeters.x) > car.bodyDimensionsMeters.x * .5)
                    issue(false, authored.id, "Bogie lies outside the longitudinal body/reference envelope.");
            for (int axis = 0; axis < 3; ++axis)
            {
                if (!std::isfinite(authored.visualAlignment.translationMeters[axis])
                    || !std::isfinite(authored.visualAlignment.rotationDegrees[axis]))
                    issue(true, authored.id, "Visual alignment must be finite.");
            }
            if (authored.visualAsset.empty()) issue(false, authored.id, "No GLB shell assigned; engineering overlays remain available.");
            const auto mass = physics::totalCarMassKilograms(car, authored.physical.loadout);
            const auto cog = physics::loadedCarCenterOfGravityMeters(car, authored.physical.loadout);
            if (glm::any(glm::greaterThan(glm::abs(cog), car.bodyDimensionsMeters * .5)))
                issue(false, authored.id, "Loaded COG lies outside the body/reference envelope.");
            result.cars.push_back({authored.id, car.dryMassKilograms, mass, cog, 0, 0});
            result.bogieCount += car.bogies.size();
            result.dryMassKilograms += car.dryMassKilograms;
            result.loadedMassKilograms += mass;
        }
        try
        {
            physics::validateCarDefinition(configuration.repeatedCar.car);
            physics::validateCarLoadout(configuration.repeatedCar.loadout);
            physics::validateInterCarConnectionDefinition(configuration.repeatedConnection);
            const auto definition = resolveTrainConfiguration(configuration);
            // Use the existing car and connector solver on a straight reference
            // rail. This preserves asymmetric bogies and body-origin placement.
            double extent = 100;
            for (const auto& car : definition.cars)
            {
                extent += car.car.bodyDimensionsMeters.x
                    + glm::length(car.car.frontHitchPositionMeters)
                    + glm::length(car.car.rearHitchPositionMeters);
                for (const auto& bogie : car.car.bogies) extent += glm::length(bogie.referencePositionMeters);
            }
            for (const auto& connection : definition.connections) extent += connection.rigidLengthMeters;
            const geometry::CurveFrame frame{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            const physics::CompiledPhysicsTrack rail{std::vector<TrackKinematicState>{
                {0, {0, 0, 0}, frame, {0, 0, 0}}, {extent * 2, {extent * 2, 0, 0}, frame, {0, 0, 0}}}, 1, TopologyKind::OpenLinear};
            const auto pose = physics::solveTrainPose(rail, definition,
                {physics::primaryTrackPathId, extent, physics::TravelDirection::IncreasingStation});
            const auto origin = pose.cars().front().carPose().bodyWorldPositionMeters();
            result.frontBogieExtentMeters = -std::numeric_limits<double>::infinity();
            result.rearBogieExtentMeters = std::numeric_limits<double>::infinity();
            std::vector<std::pair<double, double>> bounds;
            for (std::size_t i = 0; i < pose.carCount(); ++i)
            {
                const auto& carPose = pose.cars()[i].carPose();
                for (const auto* bogie : {&carPose.frontBogie(), &carPose.rearBogie()})
                {
                    const double station = bogie->location().stationMeters - extent;
                    result.frontBogieExtentMeters = std::max(result.frontBogieExtentMeters, station);
                    result.rearBogieExtentMeters = std::min(result.rearBogieExtentMeters, station);
                }
                const auto dimensions = definition.cars[i].car.bodyDimensionsMeters;
                double low = std::numeric_limits<double>::infinity(), high = -low;
                for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1})
                {
                    const auto point = carPose.transformLocalPoint(dimensions * glm::dvec3{x, y, z} * .5) - origin;
                    low = std::min(low, point.x); high = std::max(high, point.x);
                }
                bounds.emplace_back(low, high);
                result.frontExtentMeters = std::max(result.frontExtentMeters, high);
                result.rearExtentMeters = std::min(result.rearExtentMeters, low);
                if (i < result.cars.size()) result.cars[i].referenceXMeters = carPose.bodyWorldPositionMeters().x - origin.x;
                if (i)
                {
                    const double gap = bounds[i - 1].first - high;
                    result.cars[i - 1].gapToNextMeters = gap;
                    if (gap < -1e-6) issue(false, configuration.cars[i].id, "Car " + std::to_string(i - 1)
                        + " overlaps Car " + std::to_string(i) + " reference envelope by "
                        + std::to_string(-gap) + " m. Check hitches or body length.");
                }
            }
            result.straightLoadedCenterOfGravityMeters = pose.aggregateWorldCenterOfGravityMeters() - origin;
            result.straightAssemblyAvailable = true;
            result.straightBogieSpanMeters = result.frontBogieExtentMeters - result.rearBogieExtentMeters;
        }
        catch (const std::invalid_argument& error) { issue(true, 0, error.what()); }
        catch (const std::exception& error) { issue(false, 0, "Straight reference assembly unavailable: " + std::string(error.what())); }
        if (!std::isfinite(result.frontExtentMeters) || !std::isfinite(result.rearExtentMeters))
            result.frontExtentMeters = result.rearExtentMeters = 0;
        result.bodyEnvelopeLengthMeters = result.frontExtentMeters - result.rearExtentMeters;
        return result;
    }

    void validateTrainConfiguration(const TrainConfiguration& configuration)
    {
        for (const auto& diagnostic : summarizeTrainConfiguration(configuration).diagnostics)
            if (diagnostic.error) throw std::invalid_argument("Car #" + std::to_string(diagnostic.carId) + ": " + diagnostic.message);
    }
}
