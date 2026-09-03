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
            && car.forceBalanceResidualNewtons.has_value(),
            "available car fields");
        return car;
    }

    void staticLevelAndUnavailableSplit()
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
        for (const BogieReaction* bogie
            : {&car.frontBogie, &car.rearBogie})
        {
            require(bogie->status
                    == BogieReactionRecoveryStatus::MomentBalanceNotImplemented,
                "front/rear split explicitly unavailable");
            require(!bogie->worldReactionNewtons
                    && !bogie->magnitudeNewtons
                    && !bogie->trackFrameComponentsNewtons,
                "no fabricated bogie force");
        }
        requireNear(*car.forceBalanceResidualNewtons,
            {0.0, 0.0, 0.0}, car.forceBalanceToleranceNewtons,
            "static force closure");
        require(analysis.generalizedBalanceResidualNewtons.has_value()
                && std::abs(*analysis.generalizedBalanceResidualNewtons)
                    <= analysis.generalizedBalanceToleranceNewtons,
            "ideal generalized constraint closure");
    }

    void offCenterCogDoesNotInventSplit()
    {
        const BogieReactionAnalysis centered = analyze(
            straightTrack(), singleCarTrain(), locationAt(50.0));
        const BogieReactionAnalysis shifted = analyze(
            straightTrack(),
            singleCarTrain(carDefinition(1'000.0, 1.15, 0.7)),
            locationAt(50.0));
        const CarTrackReaction& centeredCar = availableCar(centered);
        const CarTrackReaction& shiftedCar = availableCar(shifted);
        requireNear(*shiftedCar.aggregateWorldBogieReactionNewtons,
            *centeredCar.aggregateWorldBogieReactionNewtons,
            1.0e-6,
            "off-center COG preserves aggregate static reaction");
        require(shiftedCar.frontBogie.status
                == BogieReactionRecoveryStatus::MomentBalanceNotImplemented
                && shiftedCar.rearBogie.status
                    == BogieReactionRecoveryStatus::MomentBalanceNotImplemented,
            "off-center COG load split remains unavailable");
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

        const CarTrackReaction& banked = availableCar(analyze(
            horizontalCircuit(0.55),
            singleCarTrain(),
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
        require(verticalCar.frontBogie.status
                == BogieReactionRecoveryStatus::MomentBalanceNotImplemented,
            "off-center force does not invent a moment split");

        const std::vector<ExternalForceApplication> lateral{
            {0, {0.0, 0.0, 0.65}, {0.0, 1'500.0, 0.0}}
        };
        const CarTrackReaction& lateralCar = availableCar(analyze(
            track, train, locationAt(50.0), 0.0, lateral));
        requireNear(*lateralCar.aggregateWorldBogieReactionNewtons,
            {0.0, -1'500.0, 9'806.65}, 1.0e-6,
            "lateral external force changes reaction direction");

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
    }

    void reverseSeamEndpointAndDeterminism()
    {
        const CompiledPhysicsTrack circuit = horizontalCircuit();
        const TrainDefinition train = singleCarTrain();
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
    run("static level and unavailable split", staticLevelAndUnavailableSplit);
    run("off-center COG", offCenterCogDoesNotInventSplit);
    run("crest valley and speed", crestValleyAndSpeedDependence);
    run("inverted loop and banked frames", invertedLoopAndBankedFrames);
    run("heterogeneous connector effects", connectorAndHeterogeneousTrainEffects);
    run("explicit forces and aerodynamics", explicitForcesAndAerodynamicPath);
    run("reverse seam endpoint determinism", reverseSeamEndpointAndDeterminism);
    run("unavailability and conditioning", explicitUnavailabilityAndConditioning);
    return 0;
}
