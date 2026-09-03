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
        const double cda = 0.0,
        const glm::dvec3 aerodynamicCenter = {0.0, 0.0, 0.8})
    {
        CarDefinition car;
        car.dryMassKilograms = 1'000.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.65};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.25};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.25};
        car.bogies = {
            BogieDefinition{{-1.15, 0.0, 0.0}},
            BogieDefinition{{1.15, 0.0, 0.0}}
        };
        car.aerodynamicDragAreaSquareMeters = cda;
        car.aerodynamicCenterLocalMeters = aerodynamicCenter;
        return car;
    }

    [[nodiscard]] TrainDefinition trainWithCdas(
        const std::initializer_list<double> cdas,
        const double connectorLength = 0.5)
    {
        TrainDefinition train;
        for (const double cda : cdas)
        {
            train.cars.push_back({carDefinition(cda), {}});
            if (train.cars.size() > 1)
            {
                train.connections.push_back({connectorLength});
            }
        }
        return train;
    }

    [[nodiscard]] PhysicsEnvironment environment(
        const double density = 1.2,
        const glm::dvec3& wind = {0.0, 0.0, 0.0})
    {
        PhysicsEnvironment result;
        result.airDensityKilogramsPerCubicMeter = density;
        result.windVelocityMetersPerSecond = wind;
        return result;
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
        const PhysicsEnvironment& physicsEnvironment,
        const TrainDynamicsState& state,
        const std::span<const ExternalForceApplication> forces)
    {
        const auto kinematics = evaluateTrainKinematics(
            track,
            train,
            physicsEnvironment,
            state.generalizedReferenceLocation,
            forces);
        TrainDynamicsState result = state;
        result.generalizedAccelerationMetersPerSecondSquared =
            (kinematics.generalizedGravityForceNewtons
                + kinematics.generalizedExternalForceNewtons
                - 0.5
                    * kinematics.effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * state.signedVelocityMetersPerSecond
                    * state.signedVelocityMetersPerSecond)
            / kinematics.effectiveGeneralizedMassKilograms;
        return result;
    }

    [[nodiscard]] RigidConnectorLoadAnalysis analyze(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const PhysicsEnvironment& physicsEnvironment,
        const TrainDynamicsState& state,
        const std::span<const ExternalForceApplication> forces)
    {
        return evaluateRigidConnectorLoads(
            track,
            train,
            physicsEnvironment,
            consistentState(track, train, physicsEnvironment, state, forces),
            forces);
    }

    [[nodiscard]] double connectorForce(
        const RigidConnectorLoadAnalysis& analysis,
        const std::size_t index)
    {
        require(analysis.exactRecoveryAvailable(),
            "connector recovery must be available");
        require(analysis.connectorLoads()[index].axialForceNewtons().has_value(),
            "connector force must have a value");
        return *analysis.connectorLoads()[index].axialForceNewtons();
    }

    void noExplicitResistanceAndZeroCda()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainWithCdas({0.0, 0.0});
        std::vector<ExternalForceApplication> forces;
        forces.reserve(8);
        forces.push_back({0, {}, {123.0, 0.0, 0.0}});
        const std::size_t originalCapacity = forces.capacity();

        const auto telemetry = generateExplicitResistanceForces(
            track, train, environment(), stateAt(50.0, 10.0), forces);
        require(forces.empty(), "zero-CdA cars emit no applications");
        require(forces.capacity() == originalCapacity,
            "caller-owned output capacity is retained");
        require(telemetry.generatedApplicationCount == 0,
            "empty resistance telemetry count");
        requireNear(telemetry.totalGeneralizedExplicitResistanceForceNewtons,
            0.0, 0.0, "empty resistance telemetry force");

        const auto baseline = evaluateTrainKinematics(
            track, train, environment(), locationAt(50.0));
        const auto empty = evaluateTrainKinematics(
            track, train, environment(), locationAt(50.0), forces);
        requireNear(empty.generalizedExternalForceNewtons,
            baseline.generalizedExternalForceNewtons, 0.0,
            "no explicit resistance preserves Phase 5 behavior");
    }

    void analyticStraightDragAndIntegration()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainWithCdas({1.8});
        const PhysicsEnvironment physicsEnvironment = environment();
        const TrainDynamicsState state = stateAt(50.0, 10.0);
        std::vector<ExternalForceApplication> forces;
        const auto telemetry = generateExplicitResistanceForces(
            track, train, physicsEnvironment, state, forces);
        const double expected = -0.5 * 1.2 * 1.8 * 10.0 * 10.0;

        require(forces.size() == 1, "one aerodynamic application is emitted");
        require(forces[0].carIndex == 0, "aerodynamic target car");
        requireNear(forces[0].localApplicationPointMeters,
            {0.0, 0.0, 0.8}, 0.0, "authored aerodynamic center");
        requireNear(forces[0].worldForceNewtons,
            {expected, 0.0, 0.0}, 2.0e-8, "analytic world drag");
        requireNear(telemetry.generalizedAerodynamicForceNewtons,
            expected, 2.0e-8, "analytic generated generalized drag");

        const auto kinematics = evaluateTrainKinematics(
            track, train, physicsEnvironment, locationAt(50.0), forces);
        requireNear(kinematics.generalizedExternalForceNewtons,
            expected, 2.0e-8, "Phase 5 generalized-force integration");
        const auto step = stepTrain(
            track,
            train,
            physicsEnvironment,
            state,
            FixedStepSettings{0.01},
            forces);
        requireNear(step.telemetry.generalizedAccelerationMetersPerSecondSquared,
            expected / 1'000.0, 2.0e-8,
            "analytic aerodynamic acceleration");
        requireNear(step.telemetry.generalizedExternalForceNewtons,
            expected, 2.0e-8, "step external-force telemetry");
        require(step.telemetry.externalForceApplicationCount == 1,
            "step application telemetry count");
    }

    void zeroReverseAndSpeedSquared()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainWithCdas({2.0});
        const PhysicsEnvironment physicsEnvironment = environment();
        std::vector<ExternalForceApplication> zero;
        static_cast<void>(generateExplicitResistanceForces(
            track, train, physicsEnvironment, stateAt(50.0), zero));
        require(zero.size() == 1, "configured aero remains observable at rest");
        requireNear(zero[0].worldForceNewtons, {}, 0.0,
            "zero relative airspeed gives exact zero drag");

        std::vector<ExternalForceApplication> forward;
        std::vector<ExternalForceApplication> faster;
        std::vector<ExternalForceApplication> reverse;
        static_cast<void>(generateExplicitResistanceForces(
            track, train, physicsEnvironment, stateAt(50.0, 5.0), forward));
        static_cast<void>(generateExplicitResistanceForces(
            track, train, physicsEnvironment, stateAt(50.0, 10.0), faster));
        static_cast<void>(generateExplicitResistanceForces(
            track,
            train,
            physicsEnvironment,
            stateAt(50.0, -5.0, TravelDirection::DecreasingStation),
            reverse));
        requireNear(glm::length(faster[0].worldForceNewtons),
            4.0 * glm::length(forward[0].worldForceNewtons), 1.0e-8,
            "doubling speed quadruples drag");
        require(forward[0].worldForceNewtons.x < 0.0,
            "forward drag opposes forward motion");
        require(reverse[0].worldForceNewtons.x > 0.0,
            "reverse drag opposes reverse motion");
    }

    void relativeWindPhysics()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainWithCdas({2.0});
        std::vector<ExternalForceApplication> forces;
        static_cast<void>(generateExplicitResistanceForces(
            track,
            train,
            environment(1.2, {15.0, 0.0, 0.0}),
            stateAt(50.0, 10.0),
            forces));
        requireNear(forces[0].worldForceNewtons,
            {30.0, 0.0, 0.0}, 2.0e-8,
            "tailwind drag follows relative airflow");

        static_cast<void>(generateExplicitResistanceForces(
            track,
            train,
            environment(1.2, {10.0, 0.0, 0.0}),
            stateAt(50.0, 10.0),
            forces));
        requireNear(forces[0].worldForceNewtons, {}, 1.0e-20,
            "matching wind produces exact zero relative drag");
    }

    void invalidAerodynamicsAndEnvironment()
    {
        const auto track = straightTrack();
        std::vector<ExternalForceApplication> forces;

        for (const double invalid : {
            -1.0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()})
        {
            TrainDefinition train = trainWithCdas({1.0});
            train.cars[0].car.aerodynamicDragAreaSquareMeters = invalid;
            requireThrows([&] {
                static_cast<void>(generateExplicitResistanceForces(
                    track, train, environment(), stateAt(50.0, 5.0), forces));
            }, "invalid CdA must be rejected");
        }

        TrainDefinition invalidCenter = trainWithCdas({1.0});
        invalidCenter.cars[0].car.aerodynamicCenterLocalMeters.y =
            std::numeric_limits<double>::quiet_NaN();
        requireThrows([&] {
            static_cast<void>(generateExplicitResistanceForces(
                track,
                invalidCenter,
                environment(),
                stateAt(50.0, 5.0),
                forces));
        }, "invalid aerodynamic center must be rejected");

        for (const double invalidDensity : {
            -1.0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()})
        {
            requireThrows([&] {
                static_cast<void>(generateExplicitResistanceForces(
                    track,
                    trainWithCdas({1.0}),
                    environment(invalidDensity),
                    stateAt(50.0, 5.0),
                    forces));
            }, "invalid air density must be rejected");
        }

        PhysicsEnvironment invalidWind = environment();
        invalidWind.windVelocityMetersPerSecond.z =
            std::numeric_limits<double>::infinity();
        requireThrows([&] {
            static_cast<void>(generateExplicitResistanceForces(
                track,
                trainWithCdas({1.0}),
                invalidWind,
                stateAt(50.0, 5.0),
                forces));
        }, "invalid wind must be rejected");

        TrainDefinition overflowing = trainWithCdas({
            std::numeric_limits<double>::max()});
        requireThrows<std::domain_error>([&] {
            static_cast<void>(generateExplicitResistanceForces(
                track,
                overflowing,
                environment(),
                stateAt(50.0, 2.0),
                forces));
        }, "non-finite generated force must be rejected");
    }

    [[nodiscard]] std::vector<ExternalForceApplication> generatedFor(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrainDynamicsState& state)
    {
        std::vector<ExternalForceApplication> forces;
        static_cast<void>(generateExplicitResistanceForces(
            track, train, environment(), state, forces));
        return forces;
    }

    void heterogeneousTotalAndConnectorPatterns()
    {
        const auto track = straightTrack();
        const TrainDynamicsState state = stateAt(60.0, 10.0);
        const TrainDefinition heterogeneous = trainWithCdas({3.0, 1.0, 2.0});
        const auto forces = generatedFor(track, heterogeneous, state);
        require(forces.size() == 3, "heterogeneous cars each emit drag");
        requireNear(forces[0].worldForceNewtons.x, -180.0, 2.0e-8,
            "lead heterogeneous drag");
        requireNear(forces[1].worldForceNewtons.x, -60.0, 2.0e-8,
            "middle heterogeneous drag");
        requireNear(forces[2].worldForceNewtons.x, -120.0, 2.0e-8,
            "rear heterogeneous drag");
        const auto evaluation = evaluateTrainKinematics(
            track, heterogeneous, environment(), locationAt(60.0), forces);
        requireNear(evaluation.generalizedExternalForceNewtons,
            -360.0, 5.0e-8, "per-car generalized drag sums");

        const TrainDefinition leadHeavy = trainWithCdas({3.0, 1.0, 2.0});
        const TrainDefinition rearHeavy = trainWithCdas({1.0, 1.0, 4.0});
        const TrainDefinition distributed = trainWithCdas({2.0, 2.0, 2.0});
        const auto leadForces = generatedFor(track, leadHeavy, state);
        const auto rearForces = generatedFor(track, rearHeavy, state);
        const auto distributedForces = generatedFor(track, distributed, state);
        const auto leadLoads = analyze(
            track, leadHeavy, environment(), state, leadForces);
        const auto rearLoads = analyze(
            track, rearHeavy, environment(), state, rearForces);
        const auto distributedLoads = analyze(
            track, distributed, environment(), state, distributedForces);
        requireNear(connectorForce(leadLoads, 0), -60.0, 3.0e-3,
            "lead-heavy front connector pattern");
        requireNear(connectorForce(leadLoads, 1), 0.0, 3.0e-3,
            "lead-heavy rear connector pattern");
        requireNear(connectorForce(rearLoads, 0), 60.0, 3.0e-3,
            "rear-heavy front connector pattern");
        requireNear(connectorForce(rearLoads, 1), 120.0, 3.0e-3,
            "rear-heavy rear connector pattern");
        requireNear(connectorForce(distributedLoads, 0), 0.0, 3.0e-3,
            "distributed front connector pattern");
        requireNear(connectorForce(distributedLoads, 1), 0.0, 3.0e-3,
            "distributed rear connector pattern");
    }

    void applicationCenterCurvedMotionAndEnergy()
    {
        const auto track = horizontalCircuit();
        TrainDefinition train = trainWithCdas({1.7});
        train.cars[0].car.aerodynamicCenterLocalMeters = {1.2, -0.8, 1.1};
        const TrainDynamicsState state = stateAt(35.0, 12.0);
        const auto forces = generatedFor(track, train, state);
        const auto evaluation = evaluateTrainKinematics(
            track, train, environment(), locationAt(35.0), forces);
        const auto& forceEvaluation = evaluation.externalForces[0];
        requireNear(forceEvaluation.worldApplicationPointMeters,
            evaluation.pose.cars()[0].carPose().transformLocalPoint(
                train.cars[0].car.aerodynamicCenterLocalMeters),
            1.0e-12,
            "aerodynamic center world transform");
        const glm::dvec3 pointVelocity =
            forceEvaluation.worldApplicationPointDerivativePerGeneralizedMeter
            * state.signedVelocityMetersPerSecond;
        const glm::dvec3 expectedForce = -0.5 * 1.2 * 1.7
            * glm::length(pointVelocity) * pointVelocity;
        requireNear(forces[0].worldForceNewtons,
            expectedForce, 2.0e-8,
            "curved drag follows aerodynamic-center velocity");
        require(glm::dot(forces[0].worldForceNewtons, pointVelocity) <= 0.0,
            "still-air aerodynamic drag cannot add mechanical energy");
    }

    void circuitSeamBoundaryAndDeterminism()
    {
        const auto circuit = horizontalCircuit();
        const TrainDefinition circuitTrain = trainWithCdas({2.0, 1.0, 1.5});
        const TrainDynamicsState seamState = stateAt(0.005, 8.0);
        std::vector<ExternalForceApplication> first;
        std::vector<ExternalForceApplication> second;
        const auto firstTelemetry = generateExplicitResistanceForces(
            circuit, circuitTrain, environment(), seamState, first);
        const auto secondTelemetry = generateExplicitResistanceForces(
            circuit, circuitTrain, environment(), seamState, second);
        require(first.size() == second.size(),
            "circuit-seam application count is deterministic");
        requireNear(firstTelemetry.generalizedAerodynamicForceNewtons,
            secondTelemetry.generalizedAerodynamicForceNewtons, 0.0,
            "circuit-seam generalized drag is deterministic");
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            require(finite(first[index].worldForceNewtons),
                "circuit-seam drag is finite");
            requireNear(first[index].worldForceNewtons,
                second[index].worldForceNewtons, 0.0,
                "circuit-seam force is deterministic");
        }

        const auto open = straightTrack();
        const TrainDefinition openTrain = trainWithCdas({1.0});
        std::vector<ExternalForceApplication> boundaryForces;
        const auto boundaryTelemetry = generateExplicitResistanceForces(
            open,
            openTrain,
            environment(),
            stateAt(198.849, 7.0),
            boundaryForces);
        require(boundaryTelemetry.finiteDifferenceKind
                == TrainFiniteDifferenceKind::Backward,
            "open endpoint uses the legal one-sided derivative");
        require(finite(boundaryForces[0].worldForceNewtons),
            "open-endpoint drag is finite");
    }

    void doubleCountAndLegacyPolicies()
    {
        const auto track = straightTrack();
        std::vector<ExternalForceApplication> forces;
        TrainDefinition conflicting = trainWithCdas({1.0, 1.0});
        conflicting.resistance.dragAreaSquareMeters = 2.0;
        requireThrows([&] {
            static_cast<void>(generateExplicitResistanceForces(
                track,
                conflicting,
                environment(),
                stateAt(50.0, 5.0),
                forces));
        }, "aggregate and per-car aero cannot be combined");

        TrainDefinition legacy = trainWithCdas({0.0, 0.0});
        legacy.resistance.dragAreaSquareMeters = 2.0;
        const auto legacyStep = stepTrain(
            track,
            legacy,
            PhysicsEnvironment{},
            stateAt(50.0, 5.0),
            FixedStepSettings{0.01});
        require(legacyStep.telemetry.resistanceForceNewtons < 0.0,
            "aggregate aerodynamic compatibility remains active");

        TrainDefinition aggregateOther = trainWithCdas({1.0, 1.0});
        aggregateOther.resistance.constantMechanicalForceNewtons = 10.0;
        aggregateOther.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            2.0;
        aggregateOther.resistance.rollingResistanceCoefficient = 0.001;
        static_cast<void>(generateExplicitResistanceForces(
            track,
            aggregateOther,
            environment(),
            stateAt(50.0, 5.0),
            forces));
        const auto unavailable = evaluateRigidConnectorLoads(
            track,
            aggregateOther,
            environment(),
            consistentState(
                track,
                aggregateOther,
                environment(),
                stateAt(50.0, 5.0),
                forces),
            forces);
        require(unavailable.status()
                == RigidConnectorLoadRecoveryStatus::
                    AggregateResistanceUnderdetermined,
            "aggregate mechanical, linear, and provisional rolling resistance remain underdetermined");
    }

    void aggregateStaticHoldRemainsIntact()
    {
        const auto track = straightTrack();
        TrainDefinition train = trainWithCdas({0.0, 0.0});
        train.resistance.constantMechanicalForceNewtons = 2'000.0;
        const std::vector<ExternalForceApplication> applied{
            {0, {}, {1'500.0, 0.0, 0.0}}};
        const auto step = stepTrain(
            track,
            train,
            PhysicsEnvironment{},
            stateAt(50.0),
            FixedStepSettings{0.01},
            applied);
        requireNear(step.state.signedVelocityMetersPerSecond,
            0.0, 0.0, "aggregate static hold remains intact");
        requireNear(step.telemetry.resistanceForceNewtons,
            -1'500.0, 2.0e-6,
            "aggregate static hold opposes the total impending force");
    }

    void finiteGeneratedOutput()
    {
        const auto track = horizontalCircuit();
        const TrainDefinition train = trainWithCdas({3.0, 0.0, 1.0});
        std::vector<ExternalForceApplication> forces;
        const auto telemetry = generateExplicitResistanceForces(
            track,
            train,
            environment(0.0, {-2.0, 4.0, 1.0}),
            stateAt(73.0, -9.0, TravelDirection::DecreasingStation),
            forces);
        require(telemetry.generatedApplicationCount == 2,
            "only nonzero-CdA cars produce applications");
        require(std::isfinite(telemetry.generalizedAerodynamicForceNewtons),
            "generated generalized force is finite");
        for (const auto& force : forces)
        {
            require(finite(force.worldForceNewtons),
                "generated force vector is finite");
            requireNear(force.worldForceNewtons, {}, 0.0,
                "zero air density produces zero drag");
        }
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

    std::fprintf(stdout, "Resistance Physics Tests\n");
    run("no explicit resistance and zero CdA", noExplicitResistanceAndZeroCda);
    run("analytic straight drag and integration", analyticStraightDragAndIntegration);
    run("zero reverse and speed squared", zeroReverseAndSpeedSquared);
    run("relative wind physics", relativeWindPhysics);
    run("invalid aerodynamics and environment", invalidAerodynamicsAndEnvironment);
    run("heterogeneous total and connector patterns", heterogeneousTotalAndConnectorPatterns);
    run("application center curved motion and energy", applicationCenterCurvedMotionAndEnergy);
    run("circuit seam boundary and determinism", circuitSeamBoundaryAndDeterminism);
    run("double count and legacy policies", doubleCountAndLegacyPolicies);
    run("aggregate static hold", aggregateStaticHoldRemainsIntact);
    run("finite generated output", finiteGeneratedOutput);

    std::fprintf(stdout, "%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
