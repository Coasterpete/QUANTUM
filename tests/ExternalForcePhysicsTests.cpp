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
        const double station,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return {primaryTrackPathId, station, direction};
    }

    [[nodiscard]] CurveFrame frameForTangent(const glm::dvec3& tangent)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        return {
            unit,
            {0.0, 1.0, 0.0},
            glm::normalize(glm::cross(unit, glm::dvec3{0.0, 1.0, 0.0}))
        };
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const double length = 200.0,
        const glm::dvec3& tangent = {1.0, 0.0, 0.0})
    {
        const CurveFrame frame = frameForTangent(tangent);
        const std::vector<TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {length, length * frame.tangent, frame, {0.0, 0.0, 0.0}}
        };
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalCircuit()
    {
        constexpr double radius = 25.0;
        constexpr int count = 720;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / count;
            const CurveFrame frame{
                {std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0},
                {0.0, 0.0, 1.0}
            };
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

    [[nodiscard]] CarDefinition carDefinition(
        const double mass = 1'000.0,
        const double length = 4.0,
        const double bogieHalfSpacing = 1.15)
    {
        CarDefinition car;
        car.dryMassKilograms = mass;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.65};
        car.bodyDimensionsMeters = {length, 1.35, 1.4};
        car.frontHitchPositionMeters = {0.5 * length, 0.0, 0.25};
        car.rearHitchPositionMeters = {-0.5 * length, 0.0, 0.25};
        car.bogies = {
            BogieDefinition{{-bogieHalfSpacing, 0.0, 0.0}},
            BogieDefinition{{bogieHalfSpacing, 0.0, 0.0}}
        };
        return car;
    }

    [[nodiscard]] TrainDefinition trainOf(
        const std::size_t count,
        const double connectorLength = 0.5)
    {
        TrainDefinition train;
        for (std::size_t index = 0; index < count; ++index)
        {
            train.cars.push_back({carDefinition(), {}});
            if (index != 0)
            {
                train.connections.push_back({connectorLength});
            }
        }
        return train;
    }

    [[nodiscard]] TrainDynamicsState stateAt(
        const double station,
        const double velocity = 0.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        TrainDynamicsState state;
        state.generalizedReferenceLocation = locationAt(station, direction);
        state.signedVelocityMetersPerSecond = velocity;
        state.runState = velocity == 0.0
            ? FollowerRunState::Resting
            : FollowerRunState::Running;
        return state;
    }

    [[nodiscard]] TrainDynamicsState consistentState(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const std::span<const ExternalForceApplication> forces,
        const double velocity = 0.0)
    {
        const auto kinematics = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, location, forces);
        TrainDynamicsState state;
        state.generalizedReferenceLocation = location;
        state.signedVelocityMetersPerSecond = velocity;
        state.generalizedAccelerationMetersPerSecondSquared =
            (kinematics.generalizedGravityForceNewtons
                + kinematics.generalizedExternalForceNewtons
                - 0.5
                    * kinematics.effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * velocity * velocity)
            / kinematics.effectiveGeneralizedMassKilograms;
        state.runState = velocity == 0.0
            ? FollowerRunState::Resting
            : FollowerRunState::Running;
        return state;
    }

    [[nodiscard]] RigidConnectorLoadAnalysis analyze(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const std::span<const ExternalForceApplication> forces,
        const double velocity = 0.0)
    {
        return evaluateRigidConnectorLoads(
            track,
            train,
            PhysicsEnvironment{},
            consistentState(track, train, location, forces, velocity),
            forces);
    }

    [[nodiscard]] double connectorForce(
        const RigidConnectorLoadAnalysis& analysis,
        const std::size_t index)
    {
        require(analysis.exactRecoveryAvailable(),
            "connector recovery must be available");
        require(analysis.connectorLoads()[index].axialForceNewtons().has_value(),
            "connector load value must be available");
        return *analysis.connectorLoads()[index].axialForceNewtons();
    }

    void emptyValidationAndZeroForce()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(2);
        const auto baseline = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(50.0));
        const std::vector<ExternalForceApplication> empty;
        const auto explicitEmpty = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(50.0), empty);
        requireNear(explicitEmpty.generalizedGravityForceNewtons,
            baseline.generalizedGravityForceNewtons, 0.0,
            "empty force list preserves gravity");
        requireNear(explicitEmpty.effectiveGeneralizedMassKilograms,
            baseline.effectiveGeneralizedMassKilograms, 0.0,
            "empty force list preserves mass");

        requireThrows([&] {
            const std::vector<ExternalForceApplication> forces{
                {2, {}, {1.0, 0.0, 0.0}}};
            static_cast<void>(evaluateTrainKinematics(
                track, train, PhysicsEnvironment{}, locationAt(50.0), forces));
        }, "invalid car index");
        requireThrows([&] {
            const std::vector<ExternalForceApplication> forces{
                {0, {}, {std::numeric_limits<double>::infinity(), 0.0, 0.0}}};
            static_cast<void>(evaluateTrainKinematics(
                track, train, PhysicsEnvironment{}, locationAt(50.0), forces));
        }, "non-finite world force");
        requireThrows([&] {
            const std::vector<ExternalForceApplication> forces{
                {0, {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}, {}}};
            static_cast<void>(evaluateRigidConnectorLoads(
                track, train, PhysicsEnvironment{}, stateAt(50.0), forces));
        }, "non-finite local application point");
        requireThrows<std::domain_error>([&] {
            const double component = std::sqrt(0.5);
            const std::vector<ExternalForceApplication> forces{{
                0,
                {},
                {
                    std::numeric_limits<double>::max(),
                    0.0,
                    std::numeric_limits<double>::max()
                }
            }};
            static_cast<void>(evaluateTrainKinematics(
                straightTrack(200.0, {component, 0.0, component}),
                train,
                PhysicsEnvironment{},
                locationAt(50.0),
                forces));
        }, "non-finite generalized projection");

        const std::vector<ExternalForceApplication> zero{{1, {2.0, 1.0, 0.5}, {}}};
        const auto zeroEvaluation = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(50.0), zero);
        requireNear(zeroEvaluation.generalizedExternalForceNewtons,
            0.0, 0.0, "zero force generalized contribution");
        require(zeroEvaluation.externalForces.size() == 1,
            "zero force remains observable");
        const auto zeroLoads = analyze(track, train, locationAt(50.0), zero);
        requireNear(connectorForce(zeroLoads, 0), 0.0, 1.0e-4,
            "zero force connector contribution");
    }

    void straightVirtualWorkAndStepTelemetry()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(3);
        const std::vector<ExternalForceApplication> forward{
            {1, {1.5, 0.4, 0.8}, {3'000.0, 0.0, 0.0}}};
        const auto evaluation = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(60.0), forward);
        requireNear(evaluation.generalizedExternalForceNewtons,
            3'000.0, 2.0e-6, "straight forward virtual work");
        requireNear(evaluation.externalForces[0]
                .worldApplicationPointDerivativePerGeneralizedMeter,
            {1.0, 0.0, 0.0}, 2.0e-9,
            "straight application-point derivative");

        const auto result = stepTrain(
            track, train, PhysicsEnvironment{}, stateAt(60.0, 2.0),
            FixedStepSettings{0.01}, forward);
        requireNear(result.telemetry.generalizedAccelerationMetersPerSecondSquared,
            1.0, 2.0e-9, "forward analytic acceleration");
        requireNear(result.telemetry.generalizedExternalForceNewtons,
            3'000.0, 2.0e-6, "external-force telemetry");
        require(result.telemetry.externalForceApplicationCount == 1,
            "external-force telemetry count");

        const std::vector<ExternalForceApplication> backward{
            {0, {}, {-3'000.0, 0.0, 0.0}}};
        const auto braking = stepTrain(
            track, train, PhysicsEnvironment{}, stateAt(60.0, 5.0),
            FixedStepSettings{0.01}, backward);
        requireNear(braking.telemetry.generalizedAccelerationMetersPerSecondSquared,
            -1.0, 2.0e-9, "backward analytic deceleration");
    }

    void arbitraryDirectionCancellationAndSummation()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(2);
        const std::vector<ExternalForceApplication> arbitrary{
            {0, {0.2, 0.3, 1.1}, {700.0, -350.0, 125.0}}};
        const auto arbitraryResult = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(50.0), arbitrary);
        requireNear(arbitraryResult.generalizedExternalForceNewtons,
            700.0, 2.0e-6, "arbitrary 3D virtual-work projection");

        const std::vector<ExternalForceApplication> vertical{
            {0, {}, {0.0, 0.0, 4'000.0}}};
        requireNear(evaluateTrainKinematics(
                track, train, PhysicsEnvironment{}, locationAt(50.0), vertical)
                .generalizedExternalForceNewtons,
            0.0, 2.0e-9, "stationary virtual-work direction");

        const std::vector<ExternalForceApplication> cancelling{
            {1, {0.5, 0.2, 0.7}, {1'200.0, -80.0, 30.0}},
            {1, {0.5, 0.2, 0.7}, {-1'200.0, 80.0, -30.0}}};
        requireNear(evaluateTrainKinematics(
                track, train, PhysicsEnvironment{}, locationAt(50.0), cancelling)
                .generalizedExternalForceNewtons,
            0.0, 1.0e-10, "equal and opposite applications cancel");

        const std::vector<ExternalForceApplication> multiple{
            {0, {}, {400.0, 0.0, 0.0}},
            {0, {1.0, 0.0, 0.0}, {600.0, 20.0, 0.0}},
            {1, {}, {-250.0, 0.0, 0.0}}};
        const auto multipleResult = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(50.0), multiple);
        requireNear(multipleResult.generalizedExternalForceNewtons,
            750.0, 2.0e-6, "multiple applications sum independently");
        require(multipleResult.externalForces.size() == multiple.size(),
            "per-application evaluations retained");
    }

    void applicationTransformAndPointMotionMatter()
    {
        const auto track = horizontalCircuit();
        const TrainDefinition train = trainOf(2, 0.4);
        const TrackLocation location = locationAt(35.0);
        const glm::dvec3 localPoint{1.2, -0.7, 0.9};
        const glm::dvec3 tangent = track.sample(location).frame.tangent;
        const std::vector<ExternalForceApplication> force{
            {0, localPoint, 2'000.0 * tangent}};
        const auto evaluation = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, location, force);
        requireNear(evaluation.externalForces[0].worldApplicationPointMeters,
            evaluation.pose.cars()[0].carPose().transformLocalPoint(localPoint),
            1.0e-12, "application point world transform");

        const std::vector<ExternalForceApplication> differentPoints{
            {0, {0.0, -2.0, 0.0}, 2'000.0 * tangent},
            {0, {0.0, 2.0, 0.0}, 2'000.0 * tangent}};
        const auto pointResult = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, location, differentPoints);
        require(std::abs(pointResult.externalForces[0].generalizedForceNewtons
                - pointResult.externalForces[1].generalizedForceNewtons)
                > 100.0,
            "different application points have different generalized motion");
    }

    void heterogeneousCarsAndForceLocations()
    {
        const auto track = horizontalCircuit();
        TrainDefinition train = trainOf(3, 0.4);
        train.cars[0].car = carDefinition(700.0, 3.2, 0.8);
        train.cars[1].car = carDefinition(1'500.0, 4.8, 1.5);
        train.cars[2].car = carDefinition(900.0, 3.8, 1.0);
        const TrackLocation location = locationAt(40.0);
        const glm::dvec3 forceVector =
            1'000.0 * track.sample(location).frame.tangent;
        std::vector<double> contributions;
        for (std::size_t carIndex = 0; carIndex < train.cars.size(); ++carIndex)
        {
            const std::vector<ExternalForceApplication> force{
                {carIndex, {0.8, 0.5, 0.9}, forceVector}};
            const auto result = evaluateTrainKinematics(
                track, train, PhysicsEnvironment{}, location, force);
            require(std::isfinite(result.generalizedExternalForceNewtons),
                "heterogeneous car projection is finite");
            contributions.push_back(result.generalizedExternalForceNewtons);
        }
        require(std::abs(contributions.front() - contributions.back()) > 1.0,
            "different articulated cars retain car-specific projection");
    }

    void connectorPropulsionAndBrakingPatterns()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(3);
        const TrackLocation location = locationAt(60.0);
        const auto check = [&](const std::size_t carIndex,
                               const double force,
                               const double expectedFront,
                               const double expectedRear,
                               const std::string_view message)
        {
            const std::vector<ExternalForceApplication> applications{
                {carIndex, {0.6, 0.0, 0.4}, {force, 0.0, 0.0}}};
            const auto result = analyze(track, train, location, applications);
            requireNear(connectorForce(result, 0), expectedFront, 2.0e-3,
                message);
            requireNear(connectorForce(result, 1), expectedRear, 2.0e-3,
                message);
        };

        check(0, 3'000.0, 2'000.0, 1'000.0, "lead propulsion loads");
        check(1, 3'000.0, -1'000.0, 1'000.0, "middle propulsion loads");
        check(2, 3'000.0, -1'000.0, -2'000.0, "rear propulsion loads");
        check(0, -3'000.0, -2'000.0, -1'000.0, "lead braking loads");
        check(2, -3'000.0, 1'000.0, 2'000.0, "rear braking loads");

        const std::vector<ExternalForceApplication> sameCarForces{
            {0, {-0.8, 0.2, 0.3}, {1'250.0, 0.0, 0.0}},
            {0, {1.1, -0.4, 0.9}, {1'750.0, 0.0, 0.0}}};
        const auto sameCarResult = analyze(
            track, train, location, sameCarForces);
        requireNear(connectorForce(sameCarResult, 0), 2'000.0, 2.0e-3,
            "multiple lead-car applications front load");
        requireNear(connectorForce(sameCarResult, 1), 1'000.0, 2.0e-3,
            "multiple lead-car applications rear load");

        const std::vector<ExternalForceApplication> distributed{
            {0, {}, {1'000.0, 0.0, 0.0}},
            {1, {}, {1'000.0, 0.0, 0.0}},
            {2, {}, {1'000.0, 0.0, 0.0}}};
        const auto distributedResult = analyze(
            track, train, location, distributed);
        requireNear(connectorForce(distributedResult, 0), 0.0, 2.0e-3,
            "distributed propulsion front load");
        requireNear(connectorForce(distributedResult, 1), 0.0, 2.0e-3,
            "distributed propulsion rear load");
    }

    void gravityResistanceAndStaticHoldRemainSeparate()
    {
        const glm::dvec3 slope{
            std::sqrt(1.0 - 0.2 * 0.2), 0.0, -0.2};
        const auto incline = straightTrack(200.0, slope);
        const TrainDefinition train = trainOf(2);
        const std::vector<ExternalForceApplication> forces{
            {1, {}, 1'500.0 * slope}};
        const auto evaluation = evaluateTrainKinematics(
            incline, train, PhysicsEnvironment{}, locationAt(60.0), forces);
        const auto step = stepTrain(
            incline, train, PhysicsEnvironment{}, stateAt(60.0, 3.0),
            FixedStepSettings{0.01}, forces);
        requireNear(step.telemetry.totalGeneralizedForceNewtons,
            evaluation.generalizedGravityForceNewtons
                + evaluation.generalizedExternalForceNewtons,
            2.0e-4, "gravity and external generalized forces add");
        const auto inclineLoads = analyze(
            incline, train, locationAt(60.0), forces);
        requireNear(connectorForce(inclineLoads, 0), -750.0, 2.0e-3,
            "gravity and rear propulsion share connector balance");

        TrainDefinition resisted = train;
        resisted.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            20.0;
        const auto resistedStep = stepTrain(
            straightTrack(), resisted, PhysicsEnvironment{}, stateAt(60.0, 5.0),
            FixedStepSettings{0.01}, forces);
        requireNear(resistedStep.telemetry.totalGeneralizedForceNewtons,
            resistedStep.telemetry.generalizedExternalForceNewtons
                + resistedStep.telemetry.resistanceForceNewtons,
            2.0e-6, "aggregate resistance and explicit force each counted once");
        const auto unavailable = evaluateRigidConnectorLoads(
            straightTrack(),
            resisted,
            PhysicsEnvironment{},
            consistentState(
                straightTrack(), resisted, locationAt(60.0), forces, 5.0),
            forces);
        require(unavailable.status()
                == RigidConnectorLoadRecoveryStatus::
                    AggregateResistanceUnderdetermined,
            "aggregate resistance remains underdetermined");

        TrainDefinition held = train;
        held.resistance.constantMechanicalForceNewtons = 2'000.0;
        const std::vector<ExternalForceApplication> heldForce{
            {0, {}, {1'500.0, 0.0, 0.0}}};
        const auto holdStep = stepTrain(
            straightTrack(), held, PhysicsEnvironment{}, stateAt(60.0),
            FixedStepSettings{0.01}, heldForce);
        requireNear(holdStep.state.signedVelocityMetersPerSecond,
            0.0, 0.0, "aggregate dry resistance holds explicit force at rest");
        requireNear(holdStep.telemetry.resistanceForceNewtons,
            -1'500.0, 2.0e-6, "static resistance opposes total impending force");
    }

    void seamReverseBoundaryAndDeterminism()
    {
        const auto circuit = horizontalCircuit();
        const TrainDefinition train = trainOf(3, 0.4);
        const std::vector<ExternalForceApplication> seamForce{
            {2, {1.0, 0.5, 0.8}, {800.0, -250.0, 100.0}}};
        const TrackLocation seamLocation = locationAt(0.005);
        const auto seamFirst = evaluateTrainKinematics(
            circuit, train, PhysicsEnvironment{}, seamLocation, seamForce);
        const auto seamSecond = evaluateTrainKinematics(
            circuit, train, PhysicsEnvironment{}, seamLocation, seamForce);
        requireNear(seamFirst.generalizedExternalForceNewtons,
            seamSecond.generalizedExternalForceNewtons, 0.0,
            "circuit-seam projection is deterministic");
        require(finite(seamFirst.externalForces[0]
                .worldApplicationPointDerivativePerGeneralizedMeter),
            "circuit-seam derivative is finite");
        const auto seamLoads = analyze(
            circuit, train, seamLocation, seamForce, 8.0);
        require(seamLoads.exactRecoveryAvailable(),
            "circuit-seam connector recovery is available");

        const std::vector<ExternalForceApplication> reverseForce{
            {1, {}, {900.0, 0.0, 0.0}}};
        const auto reverseStep = stepTrain(
            straightTrack(),
            train,
            PhysicsEnvironment{},
            stateAt(60.0, -4.0, TravelDirection::DecreasingStation),
            FixedStepSettings{0.01},
            reverseForce);
        require(reverseStep.telemetry.generalizedExternalForceNewtons > 0.0
                && reverseStep.telemetry.generalizedAccelerationMetersPerSecondSquared
                    > 0.0,
            "world force is not flipped for reverse velocity");

        const auto boundaryTrack = straightTrack(80.0);
        const TrainDefinition boundaryTrain = trainOf(2);
        const std::vector<ExternalForceApplication> boundaryForce{
            {0, {1.0, 0.2, 0.5}, {500.0, 0.0, 0.0}}};
        const auto boundary = evaluateTrainKinematics(
            boundaryTrack,
            boundaryTrain,
            PhysicsEnvironment{},
            locationAt(78.849),
            boundaryForce);
        require(boundary.finiteDifferenceKind
                == TrainFiniteDifferenceKind::Backward,
            "open endpoint uses legal one-sided force derivative");
        require(std::isfinite(boundary.generalizedExternalForceNewtons),
            "open endpoint force projection is finite");
    }

    void connectorAnalysisDoesNotFeedBackIntoDynamics()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(3);
        const std::vector<ExternalForceApplication> forces{
            {0, {}, {2'000.0, 0.0, 0.0}},
            {2, {}, {-300.0, 0.0, 0.0}}};
        const TrainDynamicsState initial = stateAt(60.0, 2.0);
        const auto before = stepTrain(
            track, train, PhysicsEnvironment{}, initial,
            FixedStepSettings{0.01}, forces);
        static_cast<void>(evaluateRigidConnectorLoads(
            track,
            train,
            PhysicsEnvironment{},
            consistentState(track, train, locationAt(60.0), forces, 2.0),
            forces));
        const auto after = stepTrain(
            track, train, PhysicsEnvironment{}, initial,
            FixedStepSettings{0.01}, forces);
        requireNear(after.state.signedVelocityMetersPerSecond,
            before.state.signedVelocityMetersPerSecond, 0.0,
            "connector recovery does not alter train dynamics");
        requireNear(after.telemetry.totalGeneralizedForceNewtons,
            before.telemetry.totalGeneralizedForceNewtons, 0.0,
            "internal connector forces do not enter generalized force");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;
    const auto run = [&](const char* name, void (*test)())
    {
        try
        {
            test();
            ++passed;
            std::fprintf(stdout, "  PASS  %s\n", name);
        }
        catch (const std::exception& error)
        {
            ++failed;
            std::fprintf(stderr, "  FAIL  %s: %s\n", name, error.what());
        }
    };

    std::fprintf(stdout, "External Force Physics Tests\n");
    run("empty validation and zero", emptyValidationAndZeroForce);
    run("straight virtual work and telemetry", straightVirtualWorkAndStepTelemetry);
    run("3D projection cancellation and summation", arbitraryDirectionCancellationAndSummation);
    run("application transform and point motion", applicationTransformAndPointMotionMatter);
    run("heterogeneous cars and locations", heterogeneousCarsAndForceLocations);
    run("connector propulsion and braking", connectorPropulsionAndBrakingPatterns);
    run("gravity resistance and static hold", gravityResistanceAndStaticHoldRemainSeparate);
    run("seam reverse boundary determinism", seamReverseBoundaryAndDeterminism);
    run("connector loads do not feed back", connectorAnalysisDoesNotFeedBackIntoDynamics);

    std::fprintf(stdout, "%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
