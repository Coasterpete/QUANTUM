#include <quantum/physics/TrackFollower.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::coaster::TrackPhysicalSettings;
    using quantum::coaster::LayoutMode;
    using quantum::geometry::CurveFrame;
    using quantum::physics::CompiledPhysicsTrack;
    using quantum::physics::FixedStepSettings;
    using quantum::physics::FollowerRunState;
    using quantum::physics::PhysicsEnvironment;
    using quantum::physics::SingleFollowerDefinition;
    using quantum::physics::TrackFollowerState;
    using quantum::physics::TrackLocation;
    using quantum::physics::TravelDirection;
    using quantum::physics::primaryTrackPathId;
    using quantum::physics::stepTrackFollower;

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
                std::string(message)
                + ": expected " + std::to_string(expected)
                + ", got " + std::to_string(actual));
        }
    }

    template<typename Function>
    void requireInvalidArgument(
        Function&& function,
        const std::string_view message)
    {
        bool threw = false;
        try
        {
            function();
        }
        catch (const std::invalid_argument&)
        {
            threw = true;
        }
        require(threw, message);
    }

    [[nodiscard]] CurveFrame frameForTangent(const glm::dvec3& tangent)
    {
        const glm::dvec3 unitTangent = glm::normalize(tangent);
        const glm::dvec3 lateral{0.0, 1.0, 0.0};
        return {
            unitTangent,
            lateral,
            glm::normalize(glm::cross(unitTangent, lateral))
        };
    }

    [[nodiscard]] std::vector<TrackKinematicState> linearSamples(
        const double lengthCoordinateUnits,
        const glm::dvec3& tangent,
        const glm::dvec3& startPosition = glm::dvec3{0.0},
        const glm::dvec3& curvature = glm::dvec3{0.0})
    {
        const CurveFrame frame = frameForTangent(tangent);
        return {
            {0.0, startPosition, frame, curvature},
            {
                lengthCoordinateUnits,
                startPosition + lengthCoordinateUnits * frame.tangent,
                frame,
                curvature
            }
        };
    }

    [[nodiscard]] CompiledPhysicsTrack makeLinearTrack(
        const double lengthCoordinateUnits,
        const glm::dvec3& tangent = glm::dvec3{1.0, 0.0, 0.0},
        const double metersPerCoordinateUnit = 1.0,
        const TopologyKind topology = TopologyKind::OpenLinear,
        const glm::dvec3& curvature = glm::dvec3{0.0},
        const glm::dvec3& startPosition = glm::dvec3{0.0})
    {
        const auto samples = linearSamples(
            lengthCoordinateUnits, tangent, startPosition, curvature);
        return CompiledPhysicsTrack{
            samples, metersPerCoordinateUnit, topology};
    }

    [[nodiscard]] TrackFollowerState stateAt(
        const double stationMeters,
        const double signedVelocityMetersPerSecond)
    {
        TrackFollowerState state;
        state.location = TrackLocation{
            primaryTrackPathId,
            stationMeters,
            signedVelocityMetersPerSecond < 0.0
                ? TravelDirection::DecreasingStation
                : TravelDirection::IncreasingStation
        };
        state.signedVelocityMetersPerSecond =
            signedVelocityMetersPerSecond;
        return state;
    }

    [[nodiscard]] TrackFollowerState runSteps(
        const CompiledPhysicsTrack& track,
        const SingleFollowerDefinition& definition,
        const PhysicsEnvironment& environment,
        TrackFollowerState state,
        const FixedStepSettings& step,
        const int count)
    {
        for (int index = 0; index < count; ++index)
        {
            state = stepTrackFollower(
                track, definition, environment, state, step).state;
        }
        return state;
    }

    void flatTrackKeepsConstantVelocity()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        const SingleFollowerDefinition definition{};
        const PhysicsEnvironment environment{};
        const FixedStepSettings step{};

        const TrackFollowerState finalState = runSteps(
            track, definition, environment, stateAt(10.0, 3.0),
            step, 240);

        requireNear(finalState.signedVelocityMetersPerSecond, 3.0, 0.0,
            "flat-track velocity");
        requireNear(finalState.location.stationMeters, 13.0, 1.0e-12,
            "flat-track station");
        require(finalState.tick == 240, "flat-track tick count");
    }

    void downhillGravityAccelerates()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            200.0, {std::sqrt(0.75), 0.0, -0.5});
        const FixedStepSettings step{0.01};
        const PhysicsEnvironment environment{};

        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, environment,
            stateAt(0.0, 2.0), step);
        const double expectedAcceleration =
            0.5 * environment.gravityAccelerationMetersPerSecondSquared;

        requireNear(result.state.signedVelocityMetersPerSecond,
            2.0 + expectedAcceleration * step.deltaTimeSeconds, 1.0e-14,
            "downhill velocity");
        requireNear(result.telemetry.gravityForceNewtons,
            result.telemetry.massKilograms * expectedAcceleration, 1.0e-12,
            "downhill gravity force");
    }

    void uphillGravityDecelerates()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            200.0, {std::sqrt(0.75), 0.0, 0.5});
        const FixedStepSettings step{0.01};
        const PhysicsEnvironment environment{};

        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, environment,
            stateAt(0.0, 2.0), step);
        const double expectedAcceleration =
            -0.5 * environment.gravityAccelerationMetersPerSecondSquared;

        requireNear(result.state.signedVelocityMetersPerSecond,
            2.0 + expectedAcceleration * step.deltaTimeSeconds, 1.0e-14,
            "uphill velocity");
    }

    void reverseMotionUsesTheSameGravityProjection()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            200.0, {std::sqrt(0.75), 0.0, 0.5});
        const FixedStepSettings step{0.01};
        const PhysicsEnvironment environment{};

        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, environment,
            stateAt(100.0, -2.0), step);

        require(
            result.state.signedVelocityMetersPerSecond < -2.0,
            "reverse downhill motion must accelerate toward decreasing station");
        require(
            result.state.location.direction
                == TravelDirection::DecreasingStation,
            "reverse direction must be retained");
    }

    void positiveCircuitTravelWraps()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            10.0, {1.0, 0.0, 0.0}, 1.0,
            TopologyKind::ClosedCircuit);

        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(9.9, 4.0), FixedStepSettings{0.1});

        requireNear(result.state.location.stationMeters, 0.3, 1.0e-14,
            "positive circuit wrap");
        requireNear(result.state.signedVelocityMetersPerSecond, 4.0, 0.0,
            "positive wrap preserves speed");
        require(result.telemetry.wrapped, "positive wrap telemetry");
    }

    void negativeCircuitTravelWraps()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            10.0, {1.0, 0.0, 0.0}, 1.0,
            TopologyKind::ClosedCircuit);

        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(0.1, -4.0), FixedStepSettings{0.1});

        requireNear(result.state.location.stationMeters, 9.7, 1.0e-14,
            "negative circuit wrap");
        requireNear(result.state.signedVelocityMetersPerSecond, -4.0, 0.0,
            "negative wrap preserves speed");
        require(result.telemetry.wrapped, "negative wrap telemetry");
    }

    void openEndpointStopsAndHoldsOutwardMotion()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(10.0);
        const FixedStepSettings step{0.1};

        TrackFollowerState state = stepTrackFollower(
            track, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(9.9, 5.0), step).state;

        requireNear(state.location.stationMeters, 10.0, 0.0,
            "open endpoint station");
        requireNear(state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "open endpoint speed");
        require(state.runState == FollowerRunState::StoppedAtEnd,
            "open endpoint state");

        state = runSteps(
            track, SingleFollowerDefinition{}, PhysicsEnvironment{},
            state, step, 20);
        requireNear(state.location.stationMeters, 10.0, 0.0,
            "open endpoint held station");
        requireNear(state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "open endpoint held speed");
    }

    void constantMechanicalResistanceOpposesBothDirections()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.constantMechanicalForceNewtons = 100.0;
        const FixedStepSettings step{0.1};

        const auto positive = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 10.0), step);
        const auto negative = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, -10.0), step);

        requireNear(positive.telemetry.resistanceForceNewtons, -100.0, 0.0,
            "positive mechanical resistance");
        requireNear(negative.telemetry.resistanceForceNewtons, 100.0, 0.0,
            "negative mechanical resistance");
        require(positive.state.signedVelocityMetersPerSecond < 10.0,
            "mechanical resistance slows positive motion");
        require(negative.state.signedVelocityMetersPerSecond > -10.0,
            "mechanical resistance slows negative motion");
    }

    void linearResistanceHasCorrectSign()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            50.0;

        const auto positive = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 2.0),
            FixedStepSettings{0.01});
        const auto negative = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, -2.0),
            FixedStepSettings{0.01});

        requireNear(positive.telemetry.resistanceForceNewtons, -100.0, 1.0e-14,
            "positive linear resistance");
        requireNear(negative.telemetry.resistanceForceNewtons, 100.0, 1.0e-14,
            "negative linear resistance");
    }

    void aerodynamicResistanceHasCorrectSignAndQuadraticGrowth()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.airDensityKilogramsPerCubicMeter = 1.2;
        definition.resistance.dragAreaSquareMeters = 2.0;
        const FixedStepSettings step{0.001};

        const auto speedTwo = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 2.0), step);
        const auto speedFour = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 4.0), step);
        const auto reverse = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, -2.0), step);

        requireNear(speedTwo.telemetry.resistanceForceNewtons, -4.8, 1.0e-14,
            "aero force at two metres per second");
        requireNear(speedFour.telemetry.resistanceForceNewtons, -19.2, 1.0e-13,
            "aero force at four metres per second");
        requireNear(reverse.telemetry.resistanceForceNewtons, 4.8, 1.0e-14,
            "reverse aero force");
        requireNear(
            std::abs(speedFour.telemetry.resistanceForceNewtons)
                / std::abs(speedTwo.telemetry.resistanceForceNewtons),
            4.0, 1.0e-14, "quadratic aero growth");
    }

    void rollingResistanceRemovesEnergyButDoesNotCreateMotion()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.rollingResistanceCoefficient = 0.01;
        const FixedStepSettings step{0.1};

        const auto moving = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 1.0), step);
        const auto resting = stepTrackFollower(
            track, definition, PhysicsEnvironment{}, stateAt(50.0, 0.0), step);

        require(moving.state.signedVelocityMetersPerSecond < 1.0,
            "rolling resistance slows motion");
        require(moving.telemetry.resistanceForceNewtons < 0.0,
            "rolling resistance opposes positive speed");
        requireNear(resting.state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "rolling resistance does not create speed");
        requireNear(resting.state.location.stationMeters, 50.0, 0.0,
            "rolling resistance does not create displacement");
    }

    void resistanceStopsWithoutReversing()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(100.0);
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.constantMechanicalForceNewtons = 100.0;
        const FixedStepSettings step{0.1};

        TrackFollowerState state = stepTrackFollower(
            track, definition, PhysicsEnvironment{},
            stateAt(50.0, 0.05), step).state;
        requireNear(state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "resistance stop clamps at zero");

        state = runSteps(
            track, definition, PhysicsEnvironment{}, state, step, 20);
        requireNear(state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "resistance cannot reverse at rest");
        requireNear(state.location.stationMeters, 50.0, 0.0,
            "resistance stop has no overshoot displacement");
    }

    void staticResistanceHoldsThenAllowsRealGravityMotion()
    {
        SingleFollowerDefinition definition;
        definition.massKilograms = 100.0;
        definition.resistance.constantMechanicalForceNewtons = 20.0;
        const PhysicsEnvironment environment{};
        const FixedStepSettings step{0.1};

        const CompiledPhysicsTrack shallow = makeLinearTrack(
            100.0, {std::sqrt(1.0 - 0.01 * 0.01), 0.0, -0.01});
        const CompiledPhysicsTrack steep = makeLinearTrack(
            100.0, {std::sqrt(1.0 - 0.04 * 0.04), 0.0, -0.04});

        const auto held = stepTrackFollower(
            shallow, definition, environment, stateAt(50.0, 0.0), step);
        const auto released = stepTrackFollower(
            steep, definition, environment, stateAt(50.0, 0.0), step);

        requireNear(held.state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "static resistance holds a weak gravity force");
        requireNear(held.telemetry.totalLongitudinalForceNewtons, 0.0, 1.0e-12,
            "static hold balances gravity");
        require(released.state.signedVelocityMetersPerSecond > 0.0,
            "gravity exceeding static resistance initiates motion");
    }

    [[nodiscard]] double gravityEnergyPerKilogram(
        const CompiledPhysicsTrack& track,
        const TrackFollowerState& state,
        const PhysicsEnvironment& environment)
    {
        const auto sample = track.sample(state.location);
        return 0.5 * state.signedVelocityMetersPerSecond
                * state.signedVelocityMetersPerSecond
            + environment.gravityAccelerationMetersPerSecondSquared
                * sample.positionMeters.z;
    }

    void gravityEnergyErrorIsBoundedAndConverges()
    {
        const PhysicsEnvironment environment{};
        const CompiledPhysicsTrack track = makeLinearTrack(
            500.0,
            {std::sqrt(0.75), 0.0, -0.5},
            1.0,
            TopologyKind::OpenLinear,
            glm::dvec3{0.0},
            glm::dvec3{0.0, 0.0, 100.0});
        const TrackFollowerState initial = stateAt(0.0, 5.0);
        const double initialEnergy = gravityEnergyPerKilogram(
            track, initial, environment);

        const TrackFollowerState coarse = runSteps(
            track, SingleFollowerDefinition{}, environment, initial,
            FixedStepSettings{1.0 / 60.0}, 120);
        const TrackFollowerState fine = runSteps(
            track, SingleFollowerDefinition{}, environment, initial,
            FixedStepSettings{1.0 / 240.0}, 480);

        const double coarseError = std::abs(
            gravityEnergyPerKilogram(track, coarse, environment)
            - initialEnergy);
        const double fineError = std::abs(
            gravityEnergyPerKilogram(track, fine, environment)
            - initialEnergy);

        require(coarseError < 0.5,
            "60 Hz semi-implicit energy error remains bounded");
        require(fineError < coarseError * 0.3,
            "240 Hz energy error improves approximately linearly");
    }

    void exactTickRunsAreDeterministic()
    {
        const CompiledPhysicsTrack track = makeLinearTrack(
            1'000.0, {std::sqrt(0.91), 0.0, -0.3});
        SingleFollowerDefinition definition;
        definition.massKilograms = 500.0;
        definition.resistance.constantMechanicalForceNewtons = 15.0;
        definition.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            1.25;
        definition.resistance.airDensityKilogramsPerCubicMeter = 1.225;
        definition.resistance.dragAreaSquareMeters = 0.8;
        definition.resistance.rollingResistanceCoefficient = 0.002;

        const TrackFollowerState first = runSteps(
            track, definition, PhysicsEnvironment{}, stateAt(10.0, 4.0),
            FixedStepSettings{}, 1'000);
        const TrackFollowerState second = runSteps(
            track, definition, PhysicsEnvironment{}, stateAt(10.0, 4.0),
            FixedStepSettings{}, 1'000);

        require(first.location == second.location,
            "deterministic location");
        require(first.signedVelocityMetersPerSecond
                == second.signedVelocityMetersPerSecond,
            "deterministic velocity");
        require(first.longitudinalAccelerationMetersPerSecondSquared
                == second.longitudinalAccelerationMetersPerSecondSquared,
            "deterministic acceleration");
        require(first.tick == second.tick, "deterministic tick");
        require(first.runState == second.runState,
            "deterministic run state");
    }

    void curvatureDiagnosticsCannotAlterLongitudinalSpeed()
    {
        const CompiledPhysicsTrack straight = makeLinearTrack(100.0);
        const CompiledPhysicsTrack normalLoadVariant = makeLinearTrack(
            100.0, {1.0, 0.0, 0.0}, 1.0,
            TopologyKind::OpenLinear, {0.0, 0.0, 2.0});

        const auto baseline = stepTrackFollower(
            straight, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(10.0, 8.0), FixedStepSettings{0.1});
        const auto variant = stepTrackFollower(
            normalLoadVariant, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(10.0, 8.0), FixedStepSettings{0.1});

        requireNear(
            variant.state.signedVelocityMetersPerSecond,
            baseline.state.signedVelocityMetersPerSecond,
            0.0,
            "curvature/normal-load diagnostics do not inject speed");
        require(
            variant.telemetry.curvatureMagnitudePerMeter
                > baseline.telemetry.curvatureMagnitudePerMeter,
            "curvature remains available as telemetry");
    }

    void coordinateScaleIsConvertedAtTrackBoundary()
    {
        const auto samples = linearSamples(10.0, {1.0, 0.0, 0.0});
        TrackPhysicalSettings physicalSettings;
        physicalSettings.metersPerCoordinateUnit = 2.0;
        const CompiledPhysicsTrack track{
            samples,
            physicalSettings,
            LayoutMode::Shuttle,
            TopologyKind::ClosedCircuit
        };
        const auto result = stepTrackFollower(
            track, SingleFollowerDefinition{}, PhysicsEnvironment{},
            stateAt(0.0, 4.0), FixedStepSettings{0.5});

        requireNear(track.lengthMeters(), 20.0, 0.0,
            "scaled physical track length");
        require(track.topology() == TopologyKind::OpenLinear,
            "authored shuttle retains open endpoint semantics");
        requireNear(result.state.location.stationMeters, 2.0, 0.0,
            "SI station advancement");
        requireNear(result.telemetry.worldPositionMeters.x, 2.0, 1.0e-14,
            "scaled world position");
    }

    void invalidInputsAreRejectedAtBoundaries()
    {
        const auto samples = linearSamples(10.0, {1.0, 0.0, 0.0});
        requireInvalidArgument(
            [&]
            {
                (void)CompiledPhysicsTrack{
                    samples, 0.0, TopologyKind::OpenLinear};
            },
            "zero coordinate scale must be rejected");

        SingleFollowerDefinition invalidMass;
        invalidMass.massKilograms = 0.0;
        const CompiledPhysicsTrack track = makeLinearTrack(10.0);
        requireInvalidArgument(
            [&]
            {
                (void)stepTrackFollower(
                    track, invalidMass, PhysicsEnvironment{},
                    stateAt(1.0, 1.0), FixedStepSettings{});
            },
            "zero mass must be rejected");

        invalidMass.massKilograms =
            std::numeric_limits<double>::quiet_NaN();
        requireInvalidArgument(
            [&]
            {
                (void)stepTrackFollower(
                    track, invalidMass, PhysicsEnvironment{},
                    stateAt(1.0, 1.0), FixedStepSettings{});
            },
            "non-finite mass must be rejected");

        SingleFollowerDefinition invalidResistance;
        invalidResistance.resistance.dragAreaSquareMeters = -1.0;
        requireInvalidArgument(
            [&]
            {
                (void)stepTrackFollower(
                    track, invalidResistance, PhysicsEnvironment{},
                    stateAt(1.0, 1.0), FixedStepSettings{});
            },
            "negative resistance values must be rejected");

        PhysicsEnvironment invalidGravity;
        invalidGravity.gravityAccelerationMetersPerSecondSquared =
            std::numeric_limits<double>::infinity();
        requireInvalidArgument(
            [&]
            {
                (void)stepTrackFollower(
                    track, SingleFollowerDefinition{}, invalidGravity,
                    stateAt(1.0, 1.0), FixedStepSettings{});
            },
            "non-finite gravity must be rejected");

        TrackFollowerState invalidLocation = stateAt(11.0, 1.0);
        requireInvalidArgument(
            [&]
            {
                (void)stepTrackFollower(
                    track, SingleFollowerDefinition{}, PhysicsEnvironment{},
                    invalidLocation, FixedStepSettings{});
            },
            "out-of-range track location must be rejected");
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

    std::fprintf(stdout, "Track Follower Physics Tests\n");

    run("flat track keeps constant velocity", flatTrackKeepsConstantVelocity);
    run("downhill gravity accelerates", downhillGravityAccelerates);
    run("uphill gravity decelerates", uphillGravityDecelerates);
    run("reverse motion uses gravity projection", reverseMotionUsesTheSameGravityProjection);
    run("positive circuit travel wraps", positiveCircuitTravelWraps);
    run("negative circuit travel wraps", negativeCircuitTravelWraps);
    run("open endpoint stops and holds", openEndpointStopsAndHoldsOutwardMotion);
    run("mechanical resistance opposes both directions", constantMechanicalResistanceOpposesBothDirections);
    run("linear resistance has correct sign", linearResistanceHasCorrectSign);
    run("aerodynamic resistance is signed and quadratic", aerodynamicResistanceHasCorrectSignAndQuadraticGrowth);
    run("rolling resistance removes energy only", rollingResistanceRemovesEnergyButDoesNotCreateMotion);
    run("resistance stops without reversing", resistanceStopsWithoutReversing);
    run("static hold yields to real gravity", staticResistanceHoldsThenAllowsRealGravityMotion);
    run("gravity energy converges with timestep", gravityEnergyErrorIsBoundedAndConverges);
    run("exact tick runs are deterministic", exactTickRunsAreDeterministic);
    run("curvature diagnostics cannot alter speed", curvatureDiagnosticsCannotAlterLongitudinalSpeed);
    run("coordinate scale converts at boundary", coordinateScaleIsConvertedAtTrackBoundary);
    run("invalid boundaries are rejected", invalidInputsAreRejectedAtBoundaries);

    std::fprintf(stdout, "\nResults: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
