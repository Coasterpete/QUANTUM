#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;
    using namespace quantum::physics;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view message)
    {
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(message) + ": expected "
                + std::to_string(expected) + ", got "
                + std::to_string(actual));
        }
    }

    void requireNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const std::string_view message)
    {
        if (!std::isfinite(actual.x)
            || !std::isfinite(actual.y)
            || !std::isfinite(actual.z)
            || glm::length(actual - expected) > tolerance)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename Exception = std::invalid_argument, typename Function>
    void requireThrows(Function&& function, const std::string_view message)
    {
        bool threw = false;
        try
        {
            function();
        }
        catch (const Exception&)
        {
            threw = true;
        }
        require(threw, message);
    }

    [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    [[nodiscard]] TrackLocation locationAt(
        const double stationMeters,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return {primaryTrackPathId, stationMeters, direction};
    }

    [[nodiscard]] CurveFrame frameForTangent(
        const glm::dvec3& tangent,
        const bool inverted = false)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        const glm::dvec3 lateral = inverted
            ? glm::dvec3{0.0, -1.0, 0.0}
            : glm::dvec3{0.0, 1.0, 0.0};
        return {
            unit,
            lateral,
            glm::normalize(glm::cross(unit, lateral))
        };
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const double lengthMeters = 200.0,
        const bool inverted = false)
    {
        const CurveFrame frame = frameForTangent(
            {1.0, 0.0, 0.0}, inverted);
        const std::vector<TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {lengthMeters,
                {lengthMeters, 0.0, 0.0},
                frame,
                {0.0, 0.0, 0.0}}
        };
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalArcTrack(const bool crest)
    {
        constexpr double radius = 24.0;
        constexpr double startAngle = -0.9;
        constexpr double endAngle = 0.9;
        // Keep canonical sample spacing below the 1 cm physics derivative
        // step so these tests exercise curvature rather than a polyline knot.
        constexpr int count = 6'000;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = startAngle
                + (endAngle - startAngle)
                    * static_cast<double>(index) / count;
            const double verticalSign = crest ? 1.0 : -1.0;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, -verticalSign * std::sin(angle)};
            const CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            samples.push_back({
                radius * (angle - startAngle),
                {
                    radius * std::sin(angle),
                    0.0,
                    verticalSign * radius * std::cos(angle)
                },
                frame,
                {
                    -std::sin(angle) / radius,
                    0.0,
                    -verticalSign * std::cos(angle) / radius
                }
            });
        }
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalCircuit(
        const double bankRadians = 0.0)
    {
        constexpr double radius = 25.0;
        constexpr int count = 20'000;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / count;
            CurveFrame frame{
                {std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0},
                {0.0, 0.0, 1.0}
            };
            frame = quantum::geometry::applyRoll(frame, bankRadians);
            samples.push_back({
                radius * angle,
                {
                    radius * std::sin(angle),
                    radius * (1.0 - std::cos(angle)),
                    0.0
                },
                frame,
                {
                    -std::sin(angle) / radius,
                    std::cos(angle) / radius,
                    0.0
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalLoopCircuit()
    {
        constexpr double radius = 20.0;
        constexpr int count = 16'000;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / count;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, std::sin(angle)};
            const CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            samples.push_back({
                radius * angle,
                {
                    radius * std::sin(angle),
                    0.0,
                    radius * (1.0 - std::cos(angle))
                },
                frame,
                {
                    -std::sin(angle) / radius,
                    0.0,
                    std::cos(angle) / radius
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CarDefinition carDefinition(
        const double massKilograms = 1'000.0,
        const double bogieHalfSpacingMeters = 1.15,
        const double centerOfGravityX = 0.0,
        const double cdaSquareMeters = 0.0)
    {
        CarDefinition car;
        car.dryMassKilograms = massKilograms;
        car.dryCenterOfGravityMeters = {centerOfGravityX, 0.0, 0.65};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.25};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.25};
        // Intentionally authored rear-first. Role comes from Phase 2, not
        // storage parity, and is observable through definitionIndex.
        car.bogies = {
            BogieDefinition{{-bogieHalfSpacingMeters, 0.0, 0.0}},
            BogieDefinition{{bogieHalfSpacingMeters, 0.0, 0.0}}
        };
        car.aerodynamicDragAreaSquareMeters = cdaSquareMeters;
        car.aerodynamicCenterLocalMeters = {0.4, 0.0, 0.9};
        return car;
    }

    [[nodiscard]] TrainDefinition singleCarTrain(
        const CarDefinition& car = carDefinition())
    {
        TrainDefinition train;
        train.cars.push_back({car, {}});
        return train;
    }

    [[nodiscard]] TrainDefinition twoCarTrain()
    {
        TrainDefinition train;
        train.cars.push_back({carDefinition(650.0), {}});
        train.cars.push_back({carDefinition(1'750.0), {}});
        train.connections.push_back({0.5});
        return train;
    }

    [[nodiscard]] TrainDynamicsState consistentState(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const double velocityMetersPerSecond = 0.0,
        const std::span<const ExternalForceApplication> forces = {})
    {
        const TrainKinematicEvaluation kinematics = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, location, forces);
        TrainDynamicsState state;
        state.generalizedReferenceLocation = location;
        state.signedVelocityMetersPerSecond = velocityMetersPerSecond;
        state.generalizedAccelerationMetersPerSecondSquared =
            (kinematics.generalizedGravityForceNewtons
                + kinematics.generalizedExternalForceNewtons
                - 0.5
                    * kinematics.effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * velocityMetersPerSecond * velocityMetersPerSecond)
            / kinematics.effectiveGeneralizedMassKilograms;
        state.runState = velocityMetersPerSecond == 0.0
            ? FollowerRunState::Resting
            : FollowerRunState::Running;
        return state;
    }

    [[nodiscard]] BogieReactionAnalysis analyze(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const double velocityMetersPerSecond = 0.0,
        const std::span<const ExternalForceApplication> forces = {})
    {
        return evaluateBogieReactions(
            track,
            train,
            PhysicsEnvironment{},
            consistentState(track, train, location, velocityMetersPerSecond,
                forces),
            forces);
    }

    [[nodiscard]] CarTrackReaction availableCar(
        const BogieReactionAnalysis& analysis,
        const std::size_t index = 0)
    {
        require(analysis.exactAggregateRecoveryAvailable(),
            "aggregate reaction analysis available");
        require(analysis.status == CarTrackReactionRecoveryStatus::Available,
            "aggregate analysis status available");
        require(index < analysis.cars.size(), "car reaction index");
        const CarTrackReaction& car = analysis.cars[index];
        require(car.status == CarTrackReactionRecoveryStatus::Available,
            "car reaction status available");
        require(car.worldCenterOfGravityAccelerationMetersPerSecondSquared
                .has_value()
            && car.aggregateWorldBogieReactionNewtons.has_value()
            && car.aggregateMagnitudeNewtons.has_value()
            && car.aggregateBodyFrameComponentsNewtons.has_value()
            && car.forceBalanceResidualNewtons.has_value()
            && car.rotationalInertialMomentNewtonMeters.has_value()
            && car.knownAppliedMomentNewtonMeters.has_value(),
            "available car fields");
        return car;
    }

    void requireAvailableSplit(
        const CarTrackReaction& car,
        const std::string_view context)
    {
        require(car.frontBogie.status == BogieReactionRecoveryStatus::Available
                && car.rearBogie.status
                    == BogieReactionRecoveryStatus::Available,
            std::string(context) + " split available (front status "
                + std::to_string(static_cast<int>(car.frontBogie.status))
                + ", rank "
                + std::to_string(car.frontBogie.reactionSolveRank)
                + ", force residual "
                + std::to_string(car.forceBalanceResidualNewtons
                    ? glm::length(*car.forceBalanceResidualNewtons) : -1.0)
                + ", force tolerance "
                + std::to_string(car.forceBalanceToleranceNewtons)
                + ", moment residual "
                + std::to_string(car.momentBalanceResidualNewtonMeters
                    ? glm::length(*car.momentBalanceResidualNewtonMeters) : -1.0)
                + ", moment tolerance "
                + std::to_string(car.momentBalanceToleranceNewtonMeters)
                + ")");
        require(car.frontBogie.worldReactionNewtons.has_value()
                && car.rearBogie.worldReactionNewtons.has_value()
                && car.frontBogie.magnitudeNewtons.has_value()
                && car.rearBogie.magnitudeNewtons.has_value()
                && car.frontBogie.trackFrameComponentsNewtons.has_value()
                && car.rearBogie.trackFrameComponentsNewtons.has_value()
                && car.frontBogie.bodyFrameComponentsNewtons.has_value()
                && car.rearBogie.bodyFrameComponentsNewtons.has_value()
                && car.momentBalanceResidualNewtonMeters.has_value(),
            std::string(context) + " split fields");
        require(car.frontBogie.reactionSolveRank == 4
                && car.rearBogie.reactionSolveRank == 4
                && car.frontBogie.reactionSolveConditionEstimate.has_value()
                && car.frontBogie.reactionSolveConditionEstimate.value()
                    <= bogieReactionMaximumConditionEstimate
                && car.frontBogie.momentRowScalePerMeter > 0.0,
            std::string(context) + " conditioning diagnostics");
        requireNear(
            *car.frontBogie.worldReactionNewtons
                + *car.rearBogie.worldReactionNewtons,
            *car.aggregateWorldBogieReactionNewtons,
            car.forceBalanceToleranceNewtons,
            std::string(context) + " Phase 7 aggregate equality");
        require(glm::length(*car.forceBalanceResidualNewtons)
                    <= car.forceBalanceToleranceNewtons
                && glm::length(*car.momentBalanceResidualNewtonMeters)
                    <= car.momentBalanceToleranceNewtonMeters,
            std::string(context) + " force and moment residuals");

        for (const BogieReaction* bogie
            : {&car.frontBogie, &car.rearBogie})
        {
            const glm::dvec3& world = *bogie->worldReactionNewtons;
            require(std::abs(glm::dot(world, bogie->trackFrame.tangent))
                    <= car.forceBalanceToleranceNewtons,
                std::string(context) + " zero tangent work");
            const glm::dvec3& components =
                *bogie->trackFrameComponentsNewtons;
            requireNear(
                components,
                {
                    glm::dot(world, bogie->trackFrame.tangent),
                    glm::dot(world, bogie->trackFrame.lateral),
                    glm::dot(world, bogie->trackFrame.up)
                },
                1.0e-9,
                std::string(context) + " bogie-frame projection");
            requireNear(
                components.x * bogie->trackFrame.tangent
                    + components.y * bogie->trackFrame.lateral
                    + components.z * bogie->trackFrame.up,
                world,
                1.0e-9,
                std::string(context) + " world projection reconstruction");
        }
        requireNear(
            *car.frontBogie.bodyFrameComponentsNewtons
                + *car.rearBogie.bodyFrameComponentsNewtons,
            *car.aggregateBodyFrameComponentsNewtons,
            car.forceBalanceToleranceNewtons,
            std::string(context) + " body-frame projection sum");
    }

    void staticLevelCenteredSplit()
    {
        constexpr double mass = 1'000.0;
        constexpr double gravity = 9.80665;
        const BogieReactionAnalysis analysis = analyze(
            straightTrack(), singleCarTrain(), locationAt(50.0));
        const CarTrackReaction& car = availableCar(analysis);
        requireNear(*car.worldCenterOfGravityAccelerationMetersPerSecondSquared,
            {0.0, 0.0, 0.0}, 1.0e-9, "static COG acceleration");
        requireNear(*car.aggregateWorldBogieReactionNewtons,
            {0.0, 0.0, mass * gravity}, 1.0e-6,
            "static aggregate support equals m g");
        requireNear(*car.aggregateMagnitudeNewtons,
            mass * gravity, 1.0e-6, "static aggregate magnitude");
        require(car.frontBogie.role == BogieRole::Front
                && car.rearBogie.role == BogieRole::Rear,
            "named bogie roles");
        require(car.frontBogie.bogieDefinitionIndex == 1
                && car.rearBogie.bogieDefinitionIndex == 0,
            "Phase 2 ordering reused");
        requireAvailableSplit(car, "static centered");
        requireNear(*car.frontBogie.worldReactionNewtons,
            {0.0, 0.0, 0.5 * mass * gravity}, 1.0e-6,
            "centered front support");
        requireNear(*car.rearBogie.worldReactionNewtons,
            {0.0, 0.0, 0.5 * mass * gravity}, 1.0e-6,
            "centered rear support");
        requireNear(*car.forceBalanceResidualNewtons,
            {0.0, 0.0, 0.0}, car.forceBalanceToleranceNewtons,
            "static force closure");
        require(analysis.generalizedBalanceResidualNewtons.has_value()
                && std::abs(*analysis.generalizedBalanceResidualNewtons)
                    <= analysis.generalizedBalanceToleranceNewtons,
            "ideal generalized constraint closure");
    }

    void staticLevelOffCenterCogAnalyticSplit()
    {
        constexpr double mass = 1'000.0;
        constexpr double gravity = 9.80665;
        constexpr double halfSpacing = 1.15;
        constexpr double centerOfGravityX = 0.7;
        const BogieReactionAnalysis centered = analyze(
            straightTrack(), singleCarTrain(), locationAt(50.0));
        const BogieReactionAnalysis shifted = analyze(
            straightTrack(),
            singleCarTrain(carDefinition(
                mass, halfSpacing, centerOfGravityX)),
            locationAt(50.0));
        const CarTrackReaction& centeredCar = availableCar(centered);
        const CarTrackReaction& shiftedCar = availableCar(shifted);
        requireNear(*shiftedCar.aggregateWorldBogieReactionNewtons,
            *centeredCar.aggregateWorldBogieReactionNewtons,
            1.0e-6,
            "off-center COG preserves aggregate static reaction");
        requireAvailableSplit(shiftedCar, "static off-center COG");
        const double weight = mass * gravity;
        const double expectedFront = weight
            * (halfSpacing + centerOfGravityX) / (2.0 * halfSpacing);
        const double expectedRear = weight - expectedFront;
        requireNear(shiftedCar.frontBogie.worldReactionNewtons->z,
            expectedFront, 1.0e-6, "off-center analytic front support");
        requireNear(shiftedCar.rearBogie.worldReactionNewtons->z,
            expectedRear, 1.0e-6, "off-center analytic rear support");
        require(std::abs(expectedFront - expectedRear) > 1'000.0,
            "off-center split is not 50/50");
    }

    void crestValleyAndSpeedDependence()
    {
        constexpr double radius = 24.0;
        const double station = radius * 0.9;
        const TrainDefinition train = singleCarTrain();
        const CompiledPhysicsTrack crestTrack = verticalArcTrack(true);
        const CompiledPhysicsTrack valleyTrack = verticalArcTrack(false);
        const CarTrackReaction& crestStatic = availableCar(
            analyze(crestTrack, train, locationAt(station), 0.0));
        const CarTrackReaction& crestMoving = availableCar(
            analyze(crestTrack, train, locationAt(station), 3.0));
        const CarTrackReaction& valleyStatic = availableCar(
            analyze(valleyTrack, train, locationAt(station), 0.0));
        const CarTrackReaction& valleyMoving = availableCar(
            analyze(valleyTrack, train, locationAt(station), 3.0));
        requireAvailableSplit(crestStatic, "crest static");
        requireAvailableSplit(crestMoving, "crest moving");
        requireAvailableSplit(valleyStatic, "valley static");
        requireAvailableSplit(valleyMoving, "valley moving");
        if (!(crestMoving.aggregateMagnitudeNewtons.value()
                < crestStatic.aggregateMagnitudeNewtons.value()))
        {
            throw TestFailure(
                "crest support decreases with speed: static "
                + std::to_string(crestStatic.aggregateMagnitudeNewtons.value())
                + ", moving "
                + std::to_string(crestMoving.aggregateMagnitudeNewtons.value())
                + ", static z "
                + std::to_string(
                    crestStatic.aggregateWorldBogieReactionNewtons->z)
                + ", moving z "
                + std::to_string(
                    crestMoving.aggregateWorldBogieReactionNewtons->z));
        }
        require(valleyMoving.aggregateMagnitudeNewtons.value()
                > valleyStatic.aggregateMagnitudeNewtons.value(),
            "valley support increases with speed");
        require(crestMoving.aggregateWorldBogieReactionNewtons->z
                < crestStatic.aggregateWorldBogieReactionNewtons->z,
            "velocity-squared crest acceleration included");
        require(valleyMoving.aggregateWorldBogieReactionNewtons->z
                > valleyStatic.aggregateWorldBogieReactionNewtons->z,
            "velocity-squared valley acceleration included");
        require(std::abs(crestMoving.frontBogie.worldReactionNewtons->z
                    - crestStatic.frontBogie.worldReactionNewtons->z) > 1.0
                && std::abs(valleyMoving.rearBogie.worldReactionNewtons->z
                    - valleyStatic.rearBogie.worldReactionNewtons->z) > 1.0,
            "speed changes front/rear reaction distribution");
    }

    void invertedLoopAndBankedFrames()
    {
        constexpr double mass = 1'000.0;
        constexpr double gravity = 9.80665;
        const CarTrackReaction& inverted = availableCar(analyze(
            straightTrack(200.0, true),
            singleCarTrain(),
            locationAt(50.0)));
        requireNear(*inverted.aggregateWorldBogieReactionNewtons,
            {0.0, 0.0, mass * gravity}, 1.0e-6,
            "inverted world reaction remains upward");
        require(inverted.aggregateBodyFrameComponentsNewtons->z < 0.0,
            "inverted body-up component is negative");
        requireAvailableSplit(inverted, "inverted");
        require(inverted.frontBogie.trackFrameComponentsNewtons->z < 0.0
                && inverted.rearBogie.trackFrameComponentsNewtons->z < 0.0,
            "inverted bogie up components may be negative");

        const CompiledPhysicsTrack loop = verticalLoopCircuit();
        const double radius = 20.0;
        const CarTrackReaction& bottom = availableCar(analyze(
            loop, singleCarTrain(), locationAt(0.0), 30.0));
        const CarTrackReaction& side = availableCar(analyze(
            loop,
            singleCarTrain(),
            locationAt(0.5 * std::numbers::pi * radius),
            30.0));
        const CarTrackReaction& top = availableCar(analyze(
            loop,
            singleCarTrain(),
            locationAt(std::numbers::pi * radius),
            30.0));
        const CarTrackReaction& beforeTop = availableCar(analyze(
            loop,
            singleCarTrain(),
            locationAt(std::numbers::pi * radius - 0.02),
            30.0));
        const CarTrackReaction& afterTop = availableCar(analyze(
            loop,
            singleCarTrain(),
            locationAt(std::numbers::pi * radius + 0.02),
            30.0));
        requireAvailableSplit(bottom, "loop bottom");
        requireAvailableSplit(side, "loop side");
        requireAvailableSplit(top, "loop top");
        requireAvailableSplit(beforeTop, "loop before top");
        requireAvailableSplit(afterTop, "loop after top");
        require(bottom.aggregateWorldBogieReactionNewtons->z > 0.0,
            "loop bottom reaction points upward");
        if (!(top.aggregateWorldBogieReactionNewtons->z < 0.0))
        {
            throw TestFailure(
                "loop top reaction points downward: reaction z "
                + std::to_string(
                    top.aggregateWorldBogieReactionNewtons->z)
                + ", acceleration z "
                + std::to_string(
                    top.worldCenterOfGravityAccelerationMetersPerSecondSquared
                        ->z));
        }
        require(finite(*side.aggregateWorldBogieReactionNewtons)
                && side.aggregateMagnitudeNewtons.value() > 0.0,
            "loop side reaction finite");
        require(glm::length(
                    *beforeTop.aggregateWorldBogieReactionNewtons
                    - *afterTop.aggregateWorldBogieReactionNewtons)
                < 250.0,
            "loop reaction is continuous through the top neighborhood");

        CarDefinition bankedCarDefinition = carDefinition();
        bankedCarDefinition.dryCenterOfGravityMeters.z = 0.0;
        const CarTrackReaction& banked = availableCar(analyze(
            horizontalCircuit(0.55),
            singleCarTrain(bankedCarDefinition),
            locationAt(12.0),
            14.0));
        require(std::abs(banked.aggregateWorldBogieReactionNewtons->x) > 1.0
                || std::abs(banked.aggregateWorldBogieReactionNewtons->y)
                    > 1.0,
            "banked turn has horizontal world reaction");
        require(std::abs(banked.aggregateBodyFrameComponentsNewtons->y) > 1.0
                && std::abs(banked.aggregateBodyFrameComponentsNewtons->z)
                    > 1.0,
            "banked reaction has lateral and up frame components");
        requireAvailableSplit(banked, "banked curve");
        require(std::abs(
                    banked.frontBogie.trackFrameComponentsNewtons->y) > 1.0
                || std::abs(
                    banked.rearBogie.trackFrameComponentsNewtons->y) > 1.0,
            "banked bogie reactions include lateral components");
    }

    void connectorAndHeterogeneousTrainEffects()
    {
        const CompiledPhysicsTrack track = verticalArcTrack(true);
        const TrainDefinition train = twoCarTrain();
        const TrackLocation location = locationAt(24.0 * 0.9);
        const TrainDynamicsState state = consistentState(
            track, train, location, 13.0);
        const RigidConnectorLoadAnalysis connector =
            evaluateRigidConnectorLoads(
                track, train, PhysicsEnvironment{}, state);
        require(connector.exactRecoveryAvailable()
                && connector.connectorLoads().size() == 1
                && connector.connectorLoads()[0]
                    .worldForceOnLeadingCarNewtons().has_value(),
            "heterogeneous connector load available");
        const glm::dvec3 forceOnLead = *connector.connectorLoads()[0]
            .worldForceOnLeadingCarNewtons();
        require(glm::length(forceOnLead) > 1.0,
            "connector force materially nonzero");

        const BogieReactionAnalysis reactions = evaluateBogieReactions(
            track, train, PhysicsEnvironment{}, state);
        require(reactions.cars.size() == 2
                && reactions.connectorLoadStatus
                    == RigidConnectorLoadRecoveryStatus::Available,
            "heterogeneous per-car reaction count");
        const CarTrackReaction& lead = availableCar(reactions, 0);
        const CarTrackReaction& following = availableCar(reactions, 1);
        requireAvailableSplit(lead, "connector leading car");
        requireAvailableSplit(following, "connector following car");
        const glm::dvec3 acceleration = *lead
            .worldCenterOfGravityAccelerationMetersPerSecondSquared;
        const glm::dvec3 withoutConnector =
            train.cars[0].car.dryMassKilograms * acceleration
            - glm::dvec3{0.0, 0.0,
                -train.cars[0].car.dryMassKilograms * 9.80665};
        requireNear(*lead.aggregateWorldBogieReactionNewtons,
            withoutConnector - forceOnLead,
            1.0e-6,
            "connector world force enters car reaction balance");
        require(glm::length(*lead.knownAppliedMomentNewtonMeters) > 1.0
                && glm::length(*following.knownAppliedMomentNewtonMeters) > 1.0,
            "connector forces act at actual hitch points");
    }

    void explicitForcesAndAerodynamicPath()
    {
        const CompiledPhysicsTrack track = straightTrack();
        const TrainDefinition train = singleCarTrain();
        const std::vector<ExternalForceApplication> vertical{
            {0, {0.8, 0.0, 1.2}, {0.0, 0.0, 2'000.0}}
        };
        const CarTrackReaction& verticalCar = availableCar(analyze(
            track, train, locationAt(50.0), 0.0, vertical));
        requireNear(*verticalCar.aggregateWorldBogieReactionNewtons,
            {0.0, 0.0, 9'806.65 - 2'000.0}, 1.0e-6,
            "off-center vertical external force");
        requireAvailableSplit(verticalCar, "off-center vertical force");
        const std::vector<ExternalForceApplication> verticalThroughCog{
            {0, {0.0, 0.0, 0.65}, {0.0, 0.0, 2'000.0}}
        };
        const CarTrackReaction& verticalThroughCogCar = availableCar(analyze(
            track, train, locationAt(50.0), 0.0, verticalThroughCog));
        requireAvailableSplit(verticalThroughCogCar, "vertical force through COG");
        requireNear(*verticalCar.aggregateWorldBogieReactionNewtons,
            *verticalThroughCogCar.aggregateWorldBogieReactionNewtons,
            1.0e-9, "same force preserves aggregate reaction");
        require(std::abs(verticalCar.frontBogie.worldReactionNewtons->z
                    - verticalThroughCogCar.frontBogie
                        .worldReactionNewtons->z) > 100.0,
            "application point changes split without changing aggregate");

        const std::vector<ExternalForceApplication> lateral{
            {0, {0.8, 0.0, 0.0}, {0.0, 1'500.0, 0.0}}
        };
        const CarTrackReaction& lateralCar = availableCar(analyze(
            track, train, locationAt(50.0), 0.0, lateral));
        requireNear(*lateralCar.aggregateWorldBogieReactionNewtons,
            {0.0, -1'500.0, 9'806.65}, 1.0e-6,
            "lateral external force changes reaction direction");
        requireAvailableSplit(lateralCar, "off-center lateral force");
        require(std::abs(lateralCar.frontBogie.worldReactionNewtons->y
                    - lateralCar.rearBogie.worldReactionNewtons->y) > 100.0,
            "off-center lateral force changes front/rear split");

        TrainDefinition aerodynamic = singleCarTrain(
            carDefinition(1'000.0, 1.15, 0.0, 2.0));
        TrainDynamicsState seed;
        seed.generalizedReferenceLocation = locationAt(50.0);
        seed.signedVelocityMetersPerSecond = 10.0;
        seed.runState = FollowerRunState::Running;
        std::vector<ExternalForceApplication> drag;
        const ExplicitResistanceTelemetry telemetry =
            generateExplicitResistanceForces(
                track, aerodynamic, PhysicsEnvironment{}, seed, drag);
        require(telemetry.aerodynamicApplicationCount == 1
                && drag.size() == 1
                && drag[0].worldForceNewtons.x < 0.0,
            "explicit aerodynamic application generated");
        const BogieReactionAnalysis dragReaction = analyze(
            track,
            aerodynamic,
            locationAt(50.0),
            10.0,
            drag);
        const CarTrackReaction& dragCar = availableCar(dragReaction);
        requireNear(*dragCar.aggregateWorldBogieReactionNewtons,
            {0.0, 0.0, 9'806.65}, 2.0e-5,
            "drag uses ordinary external-force balance path");
        requireAvailableSplit(dragCar, "aerodynamic center");
        require(glm::length(*dragCar.knownAppliedMomentNewtonMeters) > 1.0,
            "aerodynamic center contributes an ordinary force moment");
        require(std::abs(dragCar.frontBogie.worldReactionNewtons->z
                    - dragCar.rearBogie.worldReactionNewtons->z) > 1.0,
            "aerodynamic-center moment changes bogie split");
    }

    void longitudinalForceAndUnsupportedRollMoment()
    {
        const CompiledPhysicsTrack track = straightTrack();
        const TrainDefinition train = singleCarTrain();
        const std::vector<ExternalForceApplication> longitudinal{
            {0, {0.0, 0.0, 0.65}, {2'500.0, 0.0, 0.0}}
        };
        const CarTrackReaction& driven = availableCar(analyze(
            track, train, locationAt(50.0), 4.0, longitudinal));
        requireAvailableSplit(driven, "longitudinal force");
        requireNear(*driven.aggregateWorldBogieReactionNewtons,
            {0.0, 0.0, 9'806.65}, 1.0e-6,
            "longitudinal demand is handled by generalized motion");
        require(std::abs(driven.frontBogie.trackFrameComponentsNewtons->x)
                    <= driven.forceBalanceToleranceNewtons
                && std::abs(
                    driven.rearBogie.trackFrameComponentsNewtons->x)
                    <= driven.forceBalanceToleranceNewtons,
            "bogies do not absorb longitudinal force");

        const std::vector<ExternalForceApplication> unsupportedLateral{
            {0, {0.0, 0.0, 0.65}, {0.0, 1'500.0, 0.0}}
        };
        const CarTrackReaction& unsupported = availableCar(analyze(
            track, train, locationAt(50.0), 0.0, unsupportedLateral));
        require(unsupported.frontBogie.status
                    == BogieReactionRecoveryStatus::ForceBalanceInconsistent
                || unsupported.frontBogie.status
                    == BogieReactionRecoveryStatus::MomentBalanceInconsistent,
            "unsupported roll demand is explicitly inconsistent");
        require(!unsupported.frontBogie.worldReactionNewtons
                && !unsupported.rearBogie.worldReactionNewtons
                && unsupported.momentBalanceResidualNewtonMeters.has_value(),
            "inconsistent candidate is not published as authoritative");
    }

    void connectorTensionAndCompressionMoments()
    {
        TrainDefinition lightLead = twoCarTrain();
        TrainDefinition heavyLead = lightLead;
        std::swap(heavyLead.cars[0], heavyLead.cars[1]);
        bool sawTension = false;
        bool sawCompression = false;
        for (const bool crest : {true, false})
        {
            const CompiledPhysicsTrack track = verticalArcTrack(crest);
            for (const TrainDefinition* train : {&lightLead, &heavyLead})
            {
                const TrackLocation location = locationAt(24.0 * 0.9);
                const TrainDynamicsState state = consistentState(
                    track, *train, location, 13.0);
                const RigidConnectorLoadAnalysis loads =
                    evaluateRigidConnectorLoads(
                        track, *train, PhysicsEnvironment{}, state);
                require(loads.exactRecoveryAvailable()
                        && loads.connectorLoads().size() == 1,
                    "connector sign fixture is available");
                const RigidConnectorLoadClassification classification =
                    loads.connectorLoads()[0].classification();
                sawTension = sawTension
                    || classification
                        == RigidConnectorLoadClassification::Tension;
                sawCompression = sawCompression
                    || classification
                        == RigidConnectorLoadClassification::Compression;

                const BogieReactionAnalysis reactions = evaluateBogieReactions(
                    track, *train, PhysicsEnvironment{}, state);
                for (std::size_t index = 0; index < reactions.cars.size(); ++index)
                {
                    const CarTrackReaction& car = availableCar(reactions, index);
                    requireAvailableSplit(car, "signed connector load");
                    require(glm::length(*car.knownAppliedMomentNewtonMeters)
                            > 1.0,
                        "signed connector force contributes hitch moment");
                }
            }
        }
        require(sawTension && sawCompression,
            "fixtures exercise both connector tension and compression");
    }

    void accelerationInertiaAndSpacingEffects()
    {
        const CompiledPhysicsTrack track = verticalArcTrack(false);
        const TrackLocation location = locationAt(24.0 * 0.9);
        CarDefinition drivenCarDefinition = carDefinition();
        drivenCarDefinition.dryInertiaTensorBodyKgM2 = glm::dmat3{4'000.0};
        const TrainDefinition baselineTrain = singleCarTrain(
            drivenCarDefinition);
        const CarPose pose = solveCarPose(
            track, baselineTrain.cars[0].car, location);
        const std::vector<ExternalForceApplication> drive{
            {0,
                baselineTrain.cars[0].car.dryCenterOfGravityMeters,
                3'000.0 * pose.bodyFrame().tangent}
        };
        const CarTrackReaction& coasting = availableCar(analyze(
            track, baselineTrain, location, 8.0));
        const CarTrackReaction& accelerating = availableCar(analyze(
            track, baselineTrain, location, 8.0, drive));
        requireAvailableSplit(coasting, "coasting qdd");
        requireAvailableSplit(accelerating, "driven qdd");
        require(glm::length(
                    *accelerating.rotationalInertialMomentNewtonMeters
                    - *coasting.rotationalInertialMomentNewtonMeters) > 1.0
                && glm::length(
                    *accelerating.frontBogie.worldReactionNewtons
                    - *coasting.frontBogie.worldReactionNewtons) > 1.0,
            "qdd changes angular demand and bogie split");

        CarDefinition lowInertia = carDefinition();
        CarDefinition highInertia = lowInertia;
        lowInertia.dryInertiaTensorBodyKgM2 = glm::dmat3{100.0};
        highInertia.dryInertiaTensorBodyKgM2 = glm::dmat3{4'000.0};
        const CarTrackReaction& low = availableCar(analyze(
            track, singleCarTrain(lowInertia), location, 8.0, drive));
        const CarTrackReaction& high = availableCar(analyze(
            track, singleCarTrain(highInertia), location, 8.0, drive));
        requireAvailableSplit(low, "low inertia");
        requireAvailableSplit(high, "high inertia");
        require(glm::length(*low.frontBogie.worldReactionNewtons
                    - *high.frontBogie.worldReactionNewtons) > 1.0,
            "heterogeneous inertia changes per-car split");

        const CarTrackReaction& shortSpacing = availableCar(analyze(
            straightTrack(),
            singleCarTrain(carDefinition(1'000.0, 0.8, 0.35)),
            locationAt(50.0)));
        const CarTrackReaction& longSpacing = availableCar(analyze(
            straightTrack(),
            singleCarTrain(carDefinition(1'000.0, 1.6, 0.35)),
            locationAt(50.0)));
        requireAvailableSplit(shortSpacing, "short spacing");
        requireAvailableSplit(longSpacing, "long spacing");
        require(std::abs(shortSpacing.frontBogie.worldReactionNewtons->z
                    - longSpacing.frontBogie.worldReactionNewtons->z) > 100.0,
            "bogie leverage changes load distribution");

        TrainDefinition heterogeneous;
        CarDefinition first = carDefinition(700.0, 0.8, -0.2);
        first.dryInertiaTensorBodyKgM2 = glm::dmat3{250.0};
        CarDefinition second = carDefinition(1'800.0, 1.6, 0.3);
        second.dryInertiaTensorBodyKgM2 = glm::dmat3{1'500.0};
        heterogeneous.cars.push_back({first, {120.0, {0.1, 0.0, 0.8}}});
        heterogeneous.cars.push_back({second, {300.0, {-0.2, 0.0, 0.9}}});
        heterogeneous.connections.push_back({0.5});
        const BogieReactionAnalysis heterogeneousResult = analyze(
            track, heterogeneous, location, 7.0);
        require(heterogeneousResult.cars.size() == 2,
            "heterogeneous car reaction count");
        requireAvailableSplit(
            availableCar(heterogeneousResult, 0), "heterogeneous front car");
        requireAvailableSplit(
            availableCar(heterogeneousResult, 1), "heterogeneous rear car");
    }

    void reverseSeamEndpointAndDeterminism()
    {
        const CompiledPhysicsTrack circuit = horizontalCircuit();
        CarDefinition circuitCar = carDefinition();
        circuitCar.dryCenterOfGravityMeters.z = 0.0;
        const TrainDefinition train = singleCarTrain(circuitCar);
        const BogieReactionAnalysis forward = analyze(
            circuit,
            train,
            locationAt(0.005, TravelDirection::IncreasingStation),
            12.0);
        const BogieReactionAnalysis repeat = analyze(
            circuit,
            train,
            locationAt(0.005, TravelDirection::IncreasingStation),
            12.0);
        const BogieReactionAnalysis reverse = analyze(
            circuit,
            train,
            locationAt(0.005, TravelDirection::DecreasingStation),
            -12.0);
        const CarTrackReaction& forwardCar = availableCar(forward);
        const CarTrackReaction& repeatCar = availableCar(repeat);
        const CarTrackReaction& reverseCar = availableCar(reverse);
        requireNear(*forwardCar.aggregateWorldBogieReactionNewtons,
            *repeatCar.aggregateWorldBogieReactionNewtons,
            0.0,
            "circuit-seam reaction deterministic");
        requireNear(forwardCar.aggregateMagnitudeNewtons.value(),
            reverseCar.aggregateMagnitudeNewtons.value(),
            2.0e-5,
            "reverse speed-squared reaction magnitude");
        require(finite(*forwardCar.aggregateWorldBogieReactionNewtons)
                && finite(*reverseCar.aggregateWorldBogieReactionNewtons),
            "seam and reverse outputs finite");
        requireAvailableSplit(forwardCar, "circuit seam forward");
        requireAvailableSplit(repeatCar, "circuit seam repeat");
        requireAvailableSplit(reverseCar, "circuit seam reverse");
        require(forwardCar.frontBogie.role == BogieRole::Front
                && reverseCar.frontBogie.role == BogieRole::Front
                && forwardCar.frontBogie.bogieDefinitionIndex
                    == reverseCar.frontBogie.bogieDefinitionIndex,
            "reverse travel does not swap bogie roles");
        requireNear(*forwardCar.frontBogie.worldReactionNewtons,
            *repeatCar.frontBogie.worldReactionNewtons,
            0.0, "front split deterministic");

        const CompiledPhysicsTrack open = straightTrack();
        const BogieReactionAnalysis atStart = analyze(
            open, train, locationAt(1.15));
        const BogieReactionAnalysis atEnd = analyze(
            open, train, locationAt(200.0 - 1.15));
        require(atStart.finiteDifferenceKind
                == TrainFiniteDifferenceKind::Forward
                && atEnd.finiteDifferenceKind
                    == TrainFiniteDifferenceKind::Backward,
            "open endpoints use legal one-sided derivatives");
        static_cast<void>(availableCar(atStart));
        static_cast<void>(availableCar(atEnd));
        requireAvailableSplit(availableCar(atStart), "open start");
        requireAvailableSplit(availableCar(atEnd), "open end");
    }

    void explicitUnavailabilityAndConditioning()
    {
        TrainDefinition resistance = singleCarTrain();
        resistance.resistance.rollingResistanceCoefficient = 0.02;
        TrainDynamicsState staticState;
        staticState.generalizedReferenceLocation = locationAt(50.0);
        const BogieReactionAnalysis underdetermined = evaluateBogieReactions(
            straightTrack(),
            resistance,
            PhysicsEnvironment{},
            staticState);
        require(underdetermined.status
                == CarTrackReactionRecoveryStatus::
                    AggregateResistanceUnderdetermined
                && !underdetermined.exactAggregateRecoveryAvailable()
                && !underdetermined.cars[0]
                    .aggregateWorldBogieReactionNewtons,
            "aggregate resistance is explicitly underdetermined");

        const TrainDefinition singular = singleCarTrain(
            carDefinition(1'000.0, 2.5e-7));
        const CarTrackReaction& singularCar = availableCar(analyze(
            straightTrack(), singular, locationAt(50.0)));
        require(singularCar.frontBogie.status
                == BogieReactionRecoveryStatus::SingularGeometry
                && singularCar.rearBogie.status
                    == BogieReactionRecoveryStatus::SingularGeometry,
            "sub-micrometre bogie leverage is singular");
        require(!singularCar.frontBogie.worldReactionNewtons
                && !singularCar.rearBogie.worldReactionNewtons,
            "singular geometry publishes no split");

        const TrainDefinition illConditioned = singleCarTrain(
            carDefinition(1'000.0, 2.5e-6));
        const CarTrackReaction& illConditionedCar = availableCar(analyze(
            straightTrack(), illConditioned, locationAt(50.0)));
        require(illConditionedCar.frontBogie.status
                    == BogieReactionRecoveryStatus::IllConditioned
                && illConditionedCar.frontBogie.reactionSolveRank == 4
                && illConditionedCar.frontBogie
                    .reactionSolveConditionEstimate.has_value()
                && *illConditionedCar.frontBogie
                    .reactionSolveConditionEstimate
                    > bogieReactionMaximumConditionEstimate
                && !illConditionedCar.frontBogie.worldReactionNewtons,
            "near-coincident supports are ill-conditioned, not explosive");

        constexpr double circuitRadius = 25.0;
        CarDefinition rankDeficientCar = carDefinition(
            1'000.0,
            0.5 * std::numbers::pi * circuitRadius);
        rankDeficientCar.dryCenterOfGravityMeters.z = 0.0;
        const CarTrackReaction& rankDeficient = availableCar(analyze(
            horizontalCircuit(),
            singleCarTrain(rankDeficientCar),
            locationAt(std::numbers::pi * circuitRadius)));
        require(rankDeficient.frontBogie.status
                    == BogieReactionRecoveryStatus::RankDeficient
                && rankDeficient.frontBogie.reactionSolveRank < 4
                && !rankDeficient.frontBogie.worldReactionNewtons,
            "constraint-plane rank deficiency is explicit");

        const CompiledPhysicsTrack exactFit = straightTrack(2.3);
        const TrainDefinition exactFitTrain = singleCarTrain();
        TrainDynamicsState exactFitState;
        exactFitState.generalizedReferenceLocation = locationAt(1.15);
        const BogieReactionAnalysis noDerivative = evaluateBogieReactions(
            exactFit,
            exactFitTrain,
            PhysicsEnvironment{},
            exactFitState);
        require(noDerivative.status
                == CarTrackReactionRecoveryStatus::KinematicsUnavailable
                && !noDerivative.cars[0]
                    .aggregateWorldBogieReactionNewtons,
            "impossible endpoint derivative is unavailable");

        TrainDynamicsState inconsistent;
        inconsistent.generalizedReferenceLocation = locationAt(50.0);
        inconsistent.generalizedAccelerationMetersPerSecondSquared = 1.0;
        const BogieReactionAnalysis imbalance = evaluateBogieReactions(
            straightTrack(),
            singleCarTrain(),
            PhysicsEnvironment{},
            inconsistent);
        require(imbalance.status
                == CarTrackReactionRecoveryStatus::
                    InconsistentGeneralizedBalance
                && imbalance.generalizedBalanceResidualNewtons.has_value()
                && std::abs(*imbalance.generalizedBalanceResidualNewtons)
                    > imbalance.generalizedBalanceToleranceNewtons,
            "hidden propulsion is rejected by ideal-constraint balance");

        inconsistent.signedVelocityMetersPerSecond =
            std::numeric_limits<double>::quiet_NaN();
        requireThrows([&] {
            static_cast<void>(evaluateBogieReactions(
                straightTrack(),
                singleCarTrain(),
                PhysicsEnvironment{},
                inconsistent));
        }, "non-finite state rejected");
    }

    template<typename Function>
    void run(const char* const name, Function&& function)
    {
        try
        {
            function();
            std::printf("[PASS] %s\n", name);
        }
        catch (const std::exception& error)
        {
            std::fprintf(stderr, "[FAIL] %s: %s\n", name, error.what());
            throw;
        }
    }
}

int main()
{
    run("static level centered split", staticLevelCenteredSplit);
    run("static level off-center COG", staticLevelOffCenterCogAnalyticSplit);
    run("crest valley and speed", crestValleyAndSpeedDependence);
    run("inverted loop and banked frames", invertedLoopAndBankedFrames);
    run("heterogeneous connector effects", connectorAndHeterogeneousTrainEffects);
    run("explicit forces and aerodynamics", explicitForcesAndAerodynamicPath);
    run("longitudinal and unsupported moment",
        longitudinalForceAndUnsupportedRollMoment);
    run("connector tension compression moments",
        connectorTensionAndCompressionMoments);
    run("acceleration inertia spacing effects",
        accelerationInertiaAndSpacingEffects);
    run("reverse seam endpoint determinism", reverseSeamEndpointAndDeterminism);
    run("unavailability and conditioning", explicitUnavailabilityAndConditioning);
    return 0;
}
