#include <quantum/physics/DynamicContactPhysics.hpp>

#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using namespace quantum::physics;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;

    void require(const bool value, const std::string_view message)
    {
        if (!value)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    void requireNear(const double actual, const double expected,
        const double tolerance, const std::string_view message)
    {
        if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] CurveFrame identityFrame()
    {
        return {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const CurveFrame& frame = identityFrame())
    {
        const std::vector<TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {200.0, {200.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}}};
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalCircuit()
    {
        constexpr double radius = 20.0;
        constexpr std::size_t count = 2000;
        std::vector<TrackKinematicState> samples;
        for (std::size_t index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi * index / count;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, std::sin(angle)};
            samples.push_back({
                radius * angle,
                {radius * std::sin(angle), 0.0,
                    radius * (1.0 - std::cos(angle))},
                {tangent, {0.0, 1.0, 0.0},
                    glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})},
                {-std::sin(angle) / radius, 0.0,
                    std::cos(angle) / radius}});
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] BogieContactDefinition contact(
        const BogieContactRole role, const double lateral,
        const double normal, const double clearance = 0.0)
    {
        return {role, {0.0, lateral, 0.0}, {0.0, 0.0, normal}, clearance};
    }

    [[nodiscard]] TrainDefinition singleCar(
        const bool running, const bool upstop = false)
    {
        CarDefinition car;
        car.dryMassKilograms = 1000.0;
        car.bodyDimensionsMeters = {4.0, 2.0, 1.5};
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.5};
        car.dryInertiaTensorBodyKgM2 = makeUniformBoxInertiaTensorBodyKgM2(
            car.dryMassKilograms, car.bodyDimensionsMeters);
        car.frontHitchPositionMeters = {2.2, 0.0, 0.0};
        car.rearHitchPositionMeters = {-2.2, 0.0, 0.0};
        for (const double x : {1.5, -1.5})
        {
            BogieDefinition bogie;
            bogie.referencePositionMeters = {x, 0.0, 0.0};
            if (running)
            {
                bogie.contacts.push_back(contact(
                    BogieContactRole::Running, -0.6, 1.0));
                bogie.contacts.push_back(contact(
                    BogieContactRole::Running, 0.6, 1.0));
            }
            if (upstop)
            {
                bogie.contacts.push_back(contact(
                    BogieContactRole::Upstop, -0.6, -1.0, 0.1));
                bogie.contacts.push_back(contact(
                    BogieContactRole::Upstop, 0.6, -1.0, 0.1));
            }
            car.bogies.push_back(std::move(bogie));
        }
        TrainDefinition result;
        result.cars.push_back({std::move(car), {}});
        return result;
    }

    [[nodiscard]] DynamicContactState stateAt(
        const double station = 100.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        DynamicContactState state;
        state.generalizedReferenceLocation = {
            primaryTrackPathId, station, direction};
        return state;
    }

    void testFreeFlight()
    {
        PhysicsEnvironment environment;
        environment.gravityAccelerationMetersPerSecondSquared = 0.0;
        DynamicContactState state = stateAt();
        state.frontVerticalVelocityMetersPerSecond = 1.25;
        state.rearVerticalVelocityMetersPerSecond = 1.25;
        const DynamicContactStepResult result = stepDynamicContact(
            straightTrack(), singleCar(false), environment, state);
        require(result.available(), "free flight available");
        requireNear(result.state.frontVerticalVelocityMetersPerSecond,
            1.25, 1.0e-8, "front free velocity constant");
        requireNear(result.state.rearVerticalVelocityMetersPerSecond,
            1.25, 1.0e-8, "rear free velocity constant");
        requireNear(result.state.frontVerticalOffsetMeters,
            1.25 * defaultFixedTimeStepSeconds, 1.0e-9,
            "front semi-implicit position");
        require(result.telemetry.candidateContactCount == 0
                && result.telemetry.substepCount == 1,
            "free flight has no contact solve or retry");
    }

    void testSymmetricGravity()
    {
        const PhysicsEnvironment environment;
        const DynamicContactStepResult result = stepDynamicContact(
            straightTrack(), singleCar(false), environment, stateAt());
        require(result.available(), "gravity free flight available");
        const double expected =
            -environment.gravityAccelerationMetersPerSecondSquared
                * defaultFixedTimeStepSeconds;
        requireNear(result.state.frontVerticalVelocityMetersPerSecond,
            expected, 2.0e-6, "front gravity acceleration");
        requireNear(result.state.rearVerticalVelocityMetersPerSecond,
            expected, 2.0e-6, "rear gravity acceleration");
        requireNear(result.state.frontVerticalOffsetMeters,
            result.state.rearVerticalOffsetMeters, 1.0e-10,
            "symmetric gravity retains zero pitch");
    }

    void testSupportAndRelease()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = singleCar(true, true);
        const DynamicContactStepResult supported = stepDynamicContact(
            straightTrack(), train, environment, stateAt());
        require(supported.available(), "supported state available");
        require(supported.telemetry.minimumSignedGapMeters
                >= -dynamicContactGapToleranceMeters,
            "supported state nonpenetrating");
        const double expectedImpulse = 1000.0
            * environment.gravityAccelerationMetersPerSecondSquared
            * defaultFixedTimeStepSeconds;
        requireNear(
            supported.telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds
                + supported.telemetry.aggregateRearBogieNormalImpulseNewtonSeconds,
            expectedImpulse, 2.0e-3, "aggregate support impulse equals weight");

        const ExternalForceApplication upward{
            0, {0.0, 0.0, 0.5},
            {0.0, 0.0, 2.0 * 1000.0
                * environment.gravityAccelerationMetersPerSecondSquared}};
        const DynamicContactStepResult released = stepDynamicContact(
            straightTrack(), train, environment, stateAt(), {},
            std::span<const ExternalForceApplication>{&upward, 1});
        require(released.available(), "release state available");
        require(released.telemetry.impulseCarryingContactCount == 0,
            "running contact cannot pull car downward");
        require(released.state.frontVerticalOffsetMeters > 0.0
                && released.telemetry.minimumSignedGapMeters > 0.0,
            "released car traverses clearance");

        const DynamicContactStepResult separated = stepDynamicContact(
            straightTrack(), train, environment, released.state, {},
            std::span<const ExternalForceApplication>{&upward, 1});
        require(separated.available()
                && separated.telemetry.candidateContactCount == 0
                && separated.telemetry.impulseCarryingContactCount == 0
                && separated.pose->frontContacts[2].signedGapMeters
                    < released.pose->frontContacts[2].signedGapMeters,
            "separated contacts carry no impulse while upstop gap closes");
    }

    void testPlasticUpstopImpact()
    {
        PhysicsEnvironment environment;
        environment.gravityAccelerationMetersPerSecondSquared = 0.0;
        DynamicContactState state = stateAt();
        state.frontVerticalOffsetMeters = 0.099;
        state.rearVerticalOffsetMeters = 0.099;
        state.frontVerticalVelocityMetersPerSecond = 1.0;
        state.rearVerticalVelocityMetersPerSecond = 1.0;
        const DynamicContactStepResult result = stepDynamicContact(
            straightTrack(), singleCar(true, true), environment, state);
        require(result.available(), "upstop impact available");
        require(result.telemetry.candidateContactCount == 2
                && result.telemetry.impulseCarryingContactCount == 2,
            "closing upstops are solved as two aggregate bogie constraints");
        require(result.telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds > 0.0
                && result.telemetry.aggregateRearBogieNormalImpulseNewtonSeconds > 0.0,
            "upstop produces positive compressive impulse");
        require(result.telemetry.minimumSignedGapMeters
                >= -dynamicContactGapToleranceMeters,
            "upstop impact is nonpenetrating");
        require(result.telemetry.kineticEnergyAfterContactJoules
                <= result.telemetry.kineticEnergyBeforeContactJoules + 1.0e-8,
            "plastic impact does not increase kinetic energy");
    }

    void testPitchAndReverseDeterminism()
    {
        PhysicsEnvironment environment;
        environment.gravityAccelerationMetersPerSecondSquared = 0.0;
        DynamicContactState pitched = stateAt();
        pitched.frontVerticalOffsetMeters = 0.03;
        const DynamicContactStepResult poseResult = stepDynamicContact(
            straightTrack(), singleCar(false), environment, pitched);
        require(poseResult.available() && poseResult.pose.has_value(),
            "pitched pose available");
        require(poseResult.pose->bodyFrame.tangent.z > 0.0,
            "front offset produces positive body pitch");
        require(poseResult.pose->frontHitchWorldPositionMeters.z
                > poseResult.pose->rearHitchWorldPositionMeters.z,
            "hitches follow the one rigid transform");
        require(poseResult.pose->frontContacts.empty()
                && poseResult.pose->rearContacts.empty(),
            "contact-free pose has no fabricated source contacts");

        DynamicContactState oneBogie = stateAt();
        oneBogie.frontVerticalOffsetMeters = 0.03;
        const DynamicContactStepResult oneBogieResult = stepDynamicContact(
            straightTrack(), singleCar(true), PhysicsEnvironment{}, oneBogie);
        require(oneBogieResult.available()
                && oneBogieResult.telemetry
                    .aggregateFrontBogieNormalImpulseNewtonSeconds == 0.0
                && oneBogieResult.telemetry
                    .aggregateRearBogieNormalImpulseNewtonSeconds > 0.0,
            "rear bogie can remain supported after front bogie releases");

        const TrainDefinition train = singleCar(true);
        const PhysicsEnvironment gravity;
        DynamicContactState forward = stateAt();
        DynamicContactState reverse = stateAt(
            100.0, TravelDirection::DecreasingStation);
        const DynamicContactStepResult first = stepDynamicContact(
            straightTrack(), train, gravity, forward);
        const DynamicContactStepResult second = stepDynamicContact(
            straightTrack(), train, gravity, reverse);
        const DynamicContactStepResult repeated = stepDynamicContact(
            straightTrack(), train, gravity, forward);
        require(first.available() && second.available() && repeated.available(),
            "forward reverse and repeat available");
        requireNear(first.telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds,
            second.telemetry.aggregateFrontBogieNormalImpulseNewtonSeconds,
            1.0e-10, "reverse front support equivalence");
        require(first.state.frontVerticalOffsetMeters
                    == repeated.state.frontVerticalOffsetMeters
                && first.telemetry.solverIterationCount
                    == repeated.telemetry.solverIterationCount,
            "identical run is deterministic");

        TrainDefinition coincident = singleCar(true, true);
        for (BogieDefinition& bogie : coincident.cars[0].car.bogies)
        {
            for (BogieContactDefinition& source : bogie.contacts)
            {
                source.clearanceMeters = 0.0;
            }
        }
        const DynamicContactStepResult noSelfStress = stepDynamicContact(
            straightTrack(), coincident, environment, stateAt());
        require(noSelfStress.available()
                && noSelfStress.telemetry
                    .aggregateFrontBogieNormalImpulseNewtonSeconds == 0.0
                && noSelfStress.telemetry
                    .aggregateRearBogieNormalImpulseNewtonSeconds == 0.0,
            "coincident opposing constraints do not invent self stress");
    }

    void testSeamAndRejections()
    {
        PhysicsEnvironment noGravity;
        noGravity.gravityAccelerationMetersPerSecondSquared = 0.0;
        const CompiledPhysicsTrack circuit = verticalCircuit();
        DynamicContactState seam = stateAt(circuit.lengthMeters() - 0.001);
        seam.signedLongitudinalVelocityMetersPerSecond = 1.0;
        const DynamicContactStepResult wrapped = stepDynamicContact(
            circuit, singleCar(false), noGravity, seam);
        require(wrapped.available()
                && wrapped.state.generalizedReferenceLocation.stationMeters < 0.01,
            "circuit seam uses canonical wrapping");
        require(std::abs(wrapped.state.frontVerticalVelocityMetersPerSecond)
                    < 1.0e-3
                && wrapped.telemetry.candidateContactCount == 0,
            "seam adds no artificial contact impulse or velocity jump");

        DynamicContactState penetrating = stateAt();
        penetrating.frontVerticalOffsetMeters = -0.01;
        require(stepDynamicContact(straightTrack(), singleCar(true),
                noGravity, penetrating).status
                == DynamicContactStepStatus::InvalidInitialPenetration,
            "material initial penetration is rejected");

        TrainDefinition guide = singleCar(false);
        for (BogieDefinition& bogie : guide.cars[0].car.bogies)
        {
            bogie.contacts.push_back({BogieContactRole::Guide,
                {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0});
        }
        require(stepDynamicContact(straightTrack(), guide, noGravity,
                stateAt()).status
                == DynamicContactStepStatus::UnsupportedGuideContact,
            "guide dynamics are rejected");

        const double bank = 0.1;
        const CurveFrame banked{{1.0, 0.0, 0.0},
            {0.0, std::cos(bank), std::sin(bank)},
            {0.0, -std::sin(bank), std::cos(bank)}};
        require(stepDynamicContact(straightTrack(banked), singleCar(false),
                noGravity, stateAt()).status
                == DynamicContactStepStatus::UnsupportedTrackGeometry,
            "banked track is rejected from compiled geometry");
    }

}

int main()
{
    try
    {
        testFreeFlight();
        testSymmetricGravity();
        testSupportAndRelease();
        testPlasticUpstopImpact();
        testPitchAndReverseDeterminism();
        testSeamAndRejections();
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "Dynamic contact physics test failure: %s\n",
            error.what());
        return 1;
    }
    return 0;
}
