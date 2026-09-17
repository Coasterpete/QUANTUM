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

    //=========================================================================
    // M1B Tests: Coupled Multi-Car Planar Vertical Dynamic Contact
    //=========================================================================

    [[nodiscard]] DynamicContactMultiCarState multiCarStateAt(
        const std::size_t carCount,
        const double station = 100.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        DynamicContactMultiCarState state;
        state.leadCarReferenceLocation = {
            primaryTrackPathId, station, direction};
        state.cars.resize(carCount);
        return state;
    }

    [[nodiscard]] TrainDefinition twoCarTrain(
        const bool running, const bool upstop = false)
    {
        TrainDefinition def;
        for (std::size_t i = 0; i < 2; ++i)
        {
            CarDefinition car;
            car.dryMassKilograms = 1000.0 + i * 500.0;
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
            TrainCarDefinition tcd;
            tcd.car = std::move(car);
            def.cars.push_back(std::move(tcd));
        }
        InterCarConnectionDefinition conn;
        conn.rigidLengthMeters = 5.0;
        def.connections.push_back(conn);
        return def;
    }

    [[nodiscard]] TrainDefinition fourCarTrain(
        const bool running, const bool upstop = false)
    {
        TrainDefinition def;
        for (std::size_t i = 0; i < 4; ++i)
        {
            CarDefinition car;
            car.dryMassKilograms = 1000.0 + i * 250.0;
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
            TrainCarDefinition tcd;
            tcd.car = std::move(car);
            def.cars.push_back(std::move(tcd));
            if (i + 1 < 4)
            {
                InterCarConnectionDefinition conn;
                conn.rigidLengthMeters = 5.0;
                def.connections.push_back(conn);
            }
        }
        return def;
    }

    void testMultiCarN1Regression()
    {
        // M1B with N=1 must produce identical results to M1A.
        const PhysicsEnvironment environment;
        const TrainDefinition train = singleCar(true, true);
        const DynamicContactMultiCarState mcState = multiCarStateAt(1);

        const DynamicContactMultiCarStepResult mcResult =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);

        DynamicContactState m1aState = stateAt();
        const DynamicContactStepResult m1aResult = stepDynamicContact(
            straightTrack(), train, environment, m1aState);

        require(mcResult.available() && m1aResult.available(),
            "N=1 regression both available");
        require(mcResult.status == m1aResult.status,
            "N=1 regression status match");
        require(mcResult.generalizedVelocity.size() == 3,
            "N=1 regression velocity size");
        require(mcResult.generalizedCoordinates.size() == 3,
            "N=1 regression coordinates size");
        requireNear(mcResult.generalizedVelocity[0],
            m1aResult.state.signedLongitudinalVelocityMetersPerSecond,
            1.0e-10, "N=1 longitudinal velocity match");
        requireNear(mcResult.generalizedVelocity[1],
            m1aResult.state.frontVerticalVelocityMetersPerSecond,
            1.0e-10, "N=1 front vertical velocity match");
        requireNear(mcResult.generalizedVelocity[2],
            m1aResult.state.rearVerticalVelocityMetersPerSecond,
            1.0e-10, "N=1 rear vertical velocity match");
    }

    void testTwoCarConnectorClosure()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(false);
        const DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "two-car closure available");
        require(result.carResults.size() == 2, "two cars in result");
        require(result.telemetry.maximumConnectorResidualMeters
                <= connectorLengthToleranceMeters,
            "two-car connector closure within tolerance");
    }

    void testFourCarConnectorClosure()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = fourCarTrain(false);
        const DynamicContactMultiCarState mcState = multiCarStateAt(4);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "four-car closure available");
        require(result.carResults.size() == 4, "four cars in result");
        require(result.telemetry.maximumConnectorResidualMeters
                <= connectorLengthToleranceMeters,
            "four-car connector closure within tolerance");
    }

    void testMultiCarFlatTrackGravity()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(false);
        DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "multi-car gravity available");
        const double expected = -environment.gravityAccelerationMetersPerSecondSquared
            * defaultFixedTimeStepSeconds;
        requireNear(result.generalizedVelocity[1], expected, 2.0e-6,
            "car 0 front gravity");
        requireNear(result.generalizedVelocity[2], expected, 2.0e-6,
            "car 0 rear gravity");
        requireNear(result.generalizedVelocity[3], expected, 2.0e-6,
            "car 1 front gravity");
        requireNear(result.generalizedVelocity[4], expected, 2.0e-6,
            "car 1 rear gravity");
    }

    void testMultiCarSupportAndRelease()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(true, true);
        const DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        if (!result.available())
        {
            std::fprintf(stderr,
                "M1B support FAIL: status=%d minGap=%e\n",
                static_cast<int>(result.status),
                result.telemetry.minimumSignedGapMeters);
        }
        require(result.available(), "multi-car support available");
        require(result.telemetry.minimumSignedGapMeters
                >= -dynamicContactGapToleranceMeters,
            "multi-car supported nonpenetrating");
    }

    void testMultiCarDeterministic()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(true);
        const DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult first =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        const DynamicContactMultiCarStepResult second =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(first.available() && second.available(),
            "deterministic both available");
        require(first.telemetry.solverIterationCount
                == second.telemetry.solverIterationCount,
            "deterministic solver iterations match");
        for (std::size_t i = 0; i < first.generalizedVelocity.size(); ++i)
        {
            requireNear(first.generalizedVelocity[i],
                second.generalizedVelocity[i], 1.0e-15,
                "deterministic velocity match");
        }
    }

    void testMultiCarDofLayout()
    {
        const DynamicContactDofLayout layout =
            DynamicContactDofLayout::create(4);
        require(layout.carCount == 4, "layout car count");
        require(layout.dofCount == 9, "layout dof count");
        require(layout.longitudinalDof() == 0, "longitudinal dof");
        require(layout.frontVerticalDof(0) == 1, "car 0 front dof");
        require(layout.rearVerticalDof(0) == 2, "car 0 rear dof");
        require(layout.frontVerticalDof(1) == 3, "car 1 front dof");
        require(layout.rearVerticalDof(1) == 4, "car 1 rear dof");
        require(layout.frontVerticalDof(3) == 7, "car 3 front dof");
        require(layout.rearVerticalDof(3) == 8, "car 3 rear dof");
        require(layout.carIndexForVerticalDof(0)
                == std::numeric_limits<std::size_t>::max(),
            "longitudinal not vertical");
        require(layout.carIndexForVerticalDof(1) == 0, "dof 1 -> car 0");
        require(layout.carIndexForVerticalDof(3) == 1, "dof 3 -> car 1");
        require(layout.carIndexForVerticalDof(7) == 3, "dof 7 -> car 3");
        require(layout.isLongitudinalDof(0), "dof 0 is longitudinal");
        require(!layout.isLongitudinalDof(1), "dof 1 not longitudinal");
    }

    void testMultiCarConnectorTelemetry()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(false);
        const DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "connector telemetry available");
        require(result.telemetry.connectorClosures.size() == 1,
            "one connector closure reported");
        require(std::abs(result.telemetry.connectorClosures[0].signedResidualMeters)
                <= connectorLengthToleranceMeters,
            "connector residual within tolerance");
    }

    void testMultiCarReverseTravel()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(false);
        DynamicContactMultiCarState mcState = multiCarStateAt(
            2, 100.0, TravelDirection::DecreasingStation);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "reverse travel available");
        require(result.telemetry.maximumConnectorResidualMeters
                <= connectorLengthToleranceMeters,
            "reverse connector closure");
    }

    void testMultiCarExternalForce()
    {
        PhysicsEnvironment environment;
        environment.gravityAccelerationMetersPerSecondSquared = 0.0;
        const TrainDefinition train = twoCarTrain(false);
        DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const ExternalForceApplication upward{
            0, {0.0, 0.0, 0.5},
            {0.0, 0.0, 2000.0}};
        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState, {},
                std::span<const ExternalForceApplication>{&upward, 1});
        require(result.available(), "external force available");
        require(result.generalizedVelocity[1] > 0.0,
            "car 0 front velocity upward from external force");
    }

    void testMultiCarHeterogeneousMass()
    {
        const PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(false);
        DynamicContactMultiCarState mcState = multiCarStateAt(2);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), train, environment, mcState);
        require(result.available(), "heterogeneous mass available");
        // With different masses, the gravitational accelerations should
        // still be the same (g) but inertias differ.
        const double expected = -environment.gravityAccelerationMetersPerSecondSquared
            * defaultFixedTimeStepSeconds;
        requireNear(result.generalizedVelocity[1], expected, 2.0e-6,
            "car 0 front gravity heterogeneous");
        requireNear(result.generalizedVelocity[3], expected, 2.0e-6,
            "car 1 front gravity heterogeneous");
    }

    void testMultiCarHeavyLightRatio()
    {
        // Deliberately awkward heavy/light mass ratio.
        PhysicsEnvironment environment;
        TrainDefinition def;
        for (std::size_t i = 0; i < 2; ++i)
        {
            CarDefinition car;
            car.dryMassKilograms = (i == 0) ? 10000.0 : 10.0;
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
                bogie.contacts.push_back(contact(
                    BogieContactRole::Running, -0.6, 1.0));
                bogie.contacts.push_back(contact(
                    BogieContactRole::Running, 0.6, 1.0));
                car.bogies.push_back(std::move(bogie));
            }
            TrainCarDefinition tcd;
            tcd.car = std::move(car);
            def.cars.push_back(std::move(tcd));
            if (i + 1 < 2)
            {
                InterCarConnectionDefinition conn;
                conn.rigidLengthMeters = 5.0;
                def.connections.push_back(conn);
            }
        }
        DynamicContactMultiCarState mcState = multiCarStateAt(2);
        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                straightTrack(), def, environment, mcState);
        require(result.available(), "heavy/light ratio available");
        require(result.telemetry.maximumConnectorResidualMeters
                <= connectorLengthToleranceMeters,
            "heavy/light connector closure");
    }

    void testMultiCar1000StepStability()
    {
        PhysicsEnvironment environment;
        const TrainDefinition train = twoCarTrain(true, true);
        DynamicContactMultiCarState state = multiCarStateAt(2);

        for (std::size_t step = 0; step < 1000; ++step)
        {
            const DynamicContactMultiCarStepResult result =
                stepDynamicContactMultiCar(
                    straightTrack(), train, environment, state);
            require(result.available(), "stability step available");
            require(result.telemetry.maximumConnectorResidualMeters
                    <= connectorLengthToleranceMeters,
                "stability connector closure");
            state.leadCarReferenceLocation = result.leadCarReferenceLocation;
            state.signedLongitudinalVelocityMetersPerSecond =
                result.generalizedVelocity[0];
            state.tick = result.tick;
            state.runState = result.runState;
            for (std::size_t i = 0; i < 2; ++i)
            {
                state.cars[i].frontVerticalOffsetMeters =
                    result.generalizedCoordinates[1 + 2 * i];
                state.cars[i].frontVerticalVelocityMetersPerSecond =
                    result.generalizedVelocity[1 + 2 * i];
                state.cars[i].rearVerticalOffsetMeters =
                    result.generalizedCoordinates[1 + 2 * i + 1];
                state.cars[i].rearVerticalVelocityMetersPerSecond =
                    result.generalizedVelocity[1 + 2 * i + 1];
            }
        }
    }

    void testMultiCarCircuitSeam()
    {
        PhysicsEnvironment noGravity;
        noGravity.gravityAccelerationMetersPerSecondSquared = 0.0;
        const CompiledPhysicsTrack circuit = verticalCircuit();
        const TrainDefinition train = twoCarTrain(false);
        DynamicContactMultiCarState mcState = multiCarStateAt(
            2, circuit.lengthMeters() - 0.001);

        const DynamicContactMultiCarStepResult result =
            stepDynamicContactMultiCar(
                circuit, train, noGravity, mcState);
        require(result.available(), "circuit seam available");
        require(result.leadCarReferenceLocation.stationMeters < 0.01
                || result.leadCarReferenceLocation.stationMeters
                    > circuit.lengthMeters() - 0.1,
            "circuit seam wrapping");
    }

}

int main()
{
    try
    {
        // M1A tests (existing).
        testFreeFlight();
        testSymmetricGravity();
        testSupportAndRelease();
        testPlasticUpstopImpact();
        testPitchAndReverseDeterminism();
        testSeamAndRejections();

        // M1B tests.
        testMultiCarN1Regression();
        testMultiCarDofLayout();
        testTwoCarConnectorClosure();
        testFourCarConnectorClosure();
        testMultiCarFlatTrackGravity();
        testMultiCarSupportAndRelease();
        testMultiCarDeterministic();
        testMultiCarConnectorTelemetry();
        testMultiCarReverseTravel();
        testMultiCarExternalForce();
        testMultiCarHeterogeneousMass();
        testMultiCarHeavyLightRatio();
        testMultiCar1000StepStability();
        testMultiCarCircuitSeam();
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "Dynamic contact physics test failure: %s\n",
            error.what());
        return 1;
    }
    return 0;
}
