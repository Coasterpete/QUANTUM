#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
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

    [[nodiscard]] bool finite(const glm::dquat& value) noexcept
    {
        return std::isfinite(value.w)
            && std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    [[nodiscard]] TrackLocation locationAt(
        const double station,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return {primaryTrackPathId, station, direction};
    }

    [[nodiscard]] CurveFrame frameForTangent(
        const glm::dvec3& tangent,
        const double bankRadians = 0.0)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        const CurveFrame base{
            unit,
            {0.0, 1.0, 0.0},
            glm::normalize(glm::cross(unit, glm::dvec3{0.0, 1.0, 0.0}))
        };
        return quantum::geometry::applyRoll(base, bankRadians);
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

    [[nodiscard]] CompiledPhysicsTrack horizontalCircleTrack(
        const double radius = 25.0,
        const bool varyingBank = false)
    {
        constexpr int count = 720;
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
            if (varyingBank)
            {
                frame = quantum::geometry::applyRoll(
                    frame, 0.35 * std::sin(2.0 * angle));
            }
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

    [[nodiscard]] CompiledPhysicsTrack verticalArcTrack(
        const bool crest,
        const bool varyingBank = false)
    {
        constexpr double radius = 24.0;
        constexpr double startAngle = -0.9;
        constexpr double endAngle = 0.9;
        constexpr int count = 600;
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = startAngle
                + (endAngle - startAngle) * static_cast<double>(index) / count;
            const double verticalSign = crest ? 1.0 : -1.0;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, -verticalSign * std::sin(angle)};
            CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            if (varyingBank)
            {
                frame = quantum::geometry::applyRoll(
                    frame, 0.3 * std::sin(3.0 * angle));
            }
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

    [[nodiscard]] CompiledPhysicsTrack straightBankTransitionTrack()
    {
        std::vector<TrackKinematicState> samples;
        for (int index = 0; index <= 200; ++index)
        {
            const double station = 0.25 * index;
            samples.push_back({
                station,
                {station, 0.0, 0.0},
                frameForTangent(
                    {1.0, 0.0, 0.0}, 0.5 * std::sin(station / 7.0)),
                {0.0, 0.0, 0.0}
            });
        }
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CarDefinition carDefinition(
        const double mass = 800.0,
        const double length = 4.0,
        const double bogieHalfSpacing = 1.15,
        const double hitchHeight = 0.25)
    {
        CarDefinition car;
        car.dryMassKilograms = mass;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.65};
        car.bodyDimensionsMeters = {length, 1.35, 1.4};
        car.frontHitchPositionMeters = {
            0.5 * length, 0.0, hitchHeight};
        car.rearHitchPositionMeters = {
            -0.5 * length, 0.0, hitchHeight};
        car.bogies = {
            BogieDefinition{{-bogieHalfSpacing, 0.0, 0.0}},
            BogieDefinition{{bogieHalfSpacing, 0.0, 0.0}}
        };
        return car;
    }

    [[nodiscard]] TrainCarDefinition loadedCar(
        const double dryMass = 800.0,
        const double loadMass = 200.0,
        const double length = 4.0,
        const double bogieHalfSpacing = 1.15)
    {
        return {
            carDefinition(dryMass, length, bogieHalfSpacing),
            CarLoadout{loadMass, {0.2, 0.0, 0.85}}
        };
    }

    [[nodiscard]] TrainDefinition trainOf(
        const std::size_t count,
        const double connectorLength = 0.5)
    {
        TrainDefinition train;
        for (std::size_t index = 0; index < count; ++index)
        {
            train.cars.push_back(loadedCar());
            if (index != 0)
            {
                train.connections.push_back({connectorLength});
            }
        }
        return train;
    }

    [[nodiscard]] double potentialEnergy(
        const TrainPose& pose,
        const PhysicsEnvironment& environment = {})
    {
        double result = 0.0;
        for (const TrainCarPose& car : pose.cars())
        {
            result += car.loadedMassKilograms()
                * environment.gravityAccelerationMetersPerSecondSquared
                * car.loadedWorldCenterOfGravityMeters().z;
        }
        return result;
    }

    [[nodiscard]] TrainDynamicsState dynamicsState(
        const double station,
        const double velocity,
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

    void oneCarConsistMatchesPhaseTwo()
    {
        const auto track = straightTrack();
        TrainDefinition train;
        train.cars.push_back(loadedCar());
        const TrackLocation location = locationAt(50.0);
        const TrainPose pose = solveTrainPose(track, train, location);
        const CarPose car = solveCarPose(
            track,
            train.cars[0].car,
            location,
            train.cars[0].loadout);
        require(pose.carCount() == 1 && pose.connectionCount() == 0,
            "one-car consist shape");
        requireNear(pose.cars()[0].carPose().bodyWorldPositionMeters(),
            car.bodyWorldPositionMeters(), 0.0, "Phase 2 pose parity");
    }

    void twoCarsCloseOnStraightTrack()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(), trainOf(2), locationAt(50.0));
        requireNear(pose.connections()[0].actualEndpointDistanceMeters(),
            0.5, connectorLengthToleranceMeters, "straight connector distance");
        require(pose.connections()[0].absoluteLengthErrorMeters()
                <= connectorLengthToleranceMeters,
            "straight connector closure");
    }

    void nonZeroConnectorEndpointsRemainDistinct()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(), trainOf(2, 0.75), locationAt(50.0));
        const auto& connection = pose.connections()[0];
        require(glm::length(
                connection.followingEndpointWorldPositionMeters()
                - connection.leadingEndpointWorldPositionMeters()) > 0.7,
            "non-zero connector endpoints distinct");
    }

    void zeroLengthConnectorIsSafe()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(), trainOf(2, 0.0), locationAt(50.0));
        const auto& connection = pose.connections()[0];
        requireNear(connection.actualEndpointDistanceMeters(), 0.0,
            connectorLengthToleranceMeters, "zero connector closure");
        require(!connection.worldDirection(),
            "zero connector direction must be absent");
    }

    void heterogeneousCarsUseTheirOwnGeometry()
    {
        TrainDefinition train;
        train.cars = {
            loadedCar(500.0, 0.0, 3.0, 0.8),
            loadedCar(900.0, 250.0, 4.8, 1.6),
            loadedCar(650.0, 80.0, 3.6, 1.0)
        };
        train.connections = {{0.2}, {0.9}};
        const TrainPose pose = solveTrainPose(
            horizontalCircleTrack(), train, locationAt(40.0));
        requireNear(pose.cars()[0].loadedMassKilograms(), 500.0, 0.0,
            "heterogeneous lead mass");
        requireNear(pose.cars()[1].loadedMassKilograms(), 1150.0, 0.0,
            "heterogeneous middle mass");
        require(pose.connections()[0].absoluteLengthErrorMeters()
                <= connectorLengthToleranceMeters
            && pose.connections()[1].absoluteLengthErrorMeters()
                <= connectorLengthToleranceMeters,
            "heterogeneous connection closure");
    }

    void nonPassengerLeadIsAnOrdinaryCar()
    {
        TrainDefinition train = trainOf(3);
        train.cars[0] = loadedCar(420.0, 0.0, 2.6, 0.75);
        const TrainPose pose = solveTrainPose(
            straightTrack(), train, locationAt(50.0));
        requireNear(pose.cars()[0].loadedMassKilograms(), 420.0, 0.0,
            "ordinary zero-load lead mass");
        require(pose.carCount() == 3, "ordinary zero-load lead car count");
    }

    void malformedConnectionCountIsRejected()
    {
        TrainDefinition train = trainOf(2);
        train.connections.clear();
        requireThrows([&] { validateTrainDefinition(train); },
            "missing connector rejection");
        TrainDefinition empty;
        requireThrows([&] { validateTrainDefinition(empty); },
            "empty consist rejection");
    }

    void invalidConnectorLengthsAreRejected()
    {
        requireThrows([] {
            validateInterCarConnectionDefinition({-0.1});
        }, "negative connector rejection");
        requireThrows([] {
            validateInterCarConnectionDefinition({
                std::numeric_limits<double>::infinity()});
        }, "infinite connector rejection");
    }

    void curvedTrackClosesConnector()
    {
        const TrainPose pose = solveTrainPose(
            horizontalCircleTrack(), trainOf(3), locationAt(30.0));
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "curved connector closure");
        require(std::abs(pose.connections()[0]
                .relativeYawPitchRollRadians().x) > 0.05,
            "curved articulation yaw");
    }

    void crestAndValleyArticulate()
    {
        for (const bool crest : {true, false})
        {
            const TrainPose pose = solveTrainPose(
                verticalArcTrack(crest), trainOf(3), locationAt(25.0));
            require(pose.maximumAbsoluteConnectorResidualMeters()
                    <= connectorLengthToleranceMeters,
                "vertical connector closure");
            require(std::abs(pose.connections()[0]
                    .relativeYawPitchRollRadians().y) > 0.04,
                "vertical articulation pitch");
        }
    }

    void bankingTransitionKeepsIndependentCarRoll()
    {
        const TrainPose pose = solveTrainPose(
            straightBankTransitionTrack(), trainOf(3), locationAt(30.0));
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "bank transition connector closure");
        require(std::abs(pose.connections()[0]
                .relativeYawPitchRollRadians().z) > 0.02,
            "adjacent cars retain distinct bogie-derived roll");
    }

    void sixCarClosureDoesNotAccumulate()
    {
        const TrainPose pose = solveTrainPose(
            horizontalCircleTrack(35.0), trainOf(6, 0.35), locationAt(55.0));
        require(pose.connectionCount() == 5, "six-car connection count");
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "six-car maximum closure error");
    }

    void connectorDirectionIsFiniteAndNormalized()
    {
        const TrainPose pose = solveTrainPose(
            horizontalCircleTrack(), trainOf(2), locationAt(30.0));
        const auto& connection = pose.connections()[0];
        require(connection.worldDirection().has_value(),
            "non-zero connector direction present");
        require(finite(*connection.worldDirection()),
            "connector direction finite");
        requireNear(glm::length(*connection.worldDirection()), 1.0, 1.0e-12,
            "connector direction normalized");
    }

    void closestLocalCircuitRootIsSelected()
    {
        const auto track = horizontalCircleTrack(12.0);
        const TrainPose pose = solveTrainPose(
            track, trainOf(2, 0.5), locationAt(2.0));
        const double lead = pose.cars()[0].referenceLocation().stationMeters;
        const double follow = pose.cars()[1].referenceLocation().stationMeters;
        double backward = lead - follow;
        if (backward < 0.0)
        {
            backward += track.lengthMeters();
        }
        require(backward > 3.0 && backward < 7.0,
            "solver selected adjacent circuit root");
    }

    void impossibleConnectorFailsExplicitly()
    {
        TrainDefinition train = trainOf(2, 0.0);
        train.cars[1].car.frontHitchPositionMeters.y = 1.0;
        requireThrows<std::domain_error>([&] {
            static_cast<void>(solveTrainPose(
                straightTrack(), train, locationAt(50.0)));
        }, "impossible connector rejection");
    }

    void consistCrossesCircuitSeamContinuously()
    {
        const auto track = horizontalCircleTrack();
        const TrainPose pose = solveTrainPose(
            track, trainOf(3), locationAt(1.0));
        require(pose.cars()[1].referenceLocation().stationMeters
                > track.lengthMeters() - 10.0,
            "following car wraps behind seam");
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "seam connector closure");
    }

    void reverseTravelPreservesConsistOrder()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(),
            trainOf(3),
            locationAt(50.0, TravelDirection::DecreasingStation));
        require(pose.cars()[0].carIndex() == 0
            && pose.cars()[1].carIndex() == 1
            && pose.cars()[2].carIndex() == 2,
            "reverse car order");
        require(pose.cars()[1].referenceLocation().stationMeters > 50.0,
            "following car remains behind reverse-facing lead");
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "reverse connector closure");
    }

    void openTrackRejectsIncompleteEnvelope()
    {
        requireThrows<std::domain_error>([] {
            static_cast<void>(solveTrainPose(
                straightTrack(40.0), trainOf(3), locationAt(5.0)));
        }, "open endpoint full-envelope rejection");
    }

    void openBoundaryStepStopsCompleteConsist()
    {
        const auto result = stepTrain(
            straightTrack(80.0),
            trainOf(3),
            PhysicsEnvironment{},
            dynamicsState(
                14.0, -25.0, TravelDirection::DecreasingStation),
            FixedStepSettings{1.0});
        requireNear(result.state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "open-envelope stop speed");
        require(result.telemetry.boundaryIntervention
            && result.telemetry.boundary == TrackBoundary::Start,
            "open-envelope boundary telemetry");
        require(result.telemetry.generalizedReferenceLocation
                == result.state.generalizedReferenceLocation
            && result.telemetry.carCount == 3
            && result.telemetry.connectionCount == 2,
            "committed train telemetry shape");
        require(result.telemetry.pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "stopped consist remains closed");
    }

    void heterogeneousLoadedMassesSum()
    {
        TrainDefinition train;
        train.cars = {
            loadedCar(400.0, 0.0, 3.0, 0.8),
            loadedCar(700.0, 100.0, 4.0, 1.1),
            loadedCar(900.0, 250.0, 4.5, 1.4)
        };
        train.connections = {{0.3}, {0.6}};
        const TrainPose pose = solveTrainPose(
            straightTrack(), train, locationAt(60.0));
        requireNear(pose.totalLoadedMassKilograms(), 2350.0, 0.0,
            "heterogeneous total loaded mass");
    }

    void aggregateCogIsMassWeighted()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(), trainOf(3), locationAt(60.0));
        glm::dvec3 expected{0.0};
        for (const TrainCarPose& car : pose.cars())
        {
            expected += car.loadedMassKilograms()
                * car.loadedWorldCenterOfGravityMeters();
        }
        expected /= pose.totalLoadedMassKilograms();
        requireNear(pose.aggregateWorldCenterOfGravityMeters(), expected,
            1.0e-12, "mass-weighted train COG");
    }

    void constantSlopeHasAnalyticalGravityAndMass()
    {
        const glm::dvec3 tangent{
            std::sqrt(1.0 - 0.3 * 0.3), 0.0, -0.3};
        const TrainDefinition train = trainOf(3);
        const auto evaluation = evaluateTrainKinematics(
            straightTrack(200.0, tangent),
            train,
            PhysicsEnvironment{},
            locationAt(60.0));
        const double expectedForce = train.cars.size()
            * 1000.0 * 9.80665 * 0.3;
        requireNear(evaluation.generalizedGravityForceNewtons,
            expectedForce, 2.0e-5, "constant-slope distributed gravity");
        requireNear(evaluation.effectiveGeneralizedMassKilograms,
            3000.0, 2.0e-6, "constant-slope effective mass");
    }

    void crestGravityDiffersFromLeadPointApproximation()
    {
        const auto track = verticalArcTrack(true);
        const TrainDefinition train = trainOf(4, 0.4);
        const auto evaluation = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(25.0));
        const double leadApproximation =
            evaluation.pose.totalLoadedMassKilograms()
            * glm::dot(glm::dvec3{0.0, 0.0, -9.80665},
                track.sample(locationAt(25.0)).frame.tangent);
        require(std::abs(evaluation.generalizedGravityForceNewtons
                - leadApproximation) > 500.0,
            "crest uses distributed gravity rather than lead tangent");
    }

    void valleyGravityReflectsMultipleSlopes()
    {
        const auto evaluation = evaluateTrainKinematics(
            verticalArcTrack(false),
            trainOf(4, 0.4),
            PhysicsEnvironment{},
            locationAt(25.0));
        double sum = 0.0;
        bool hasDifferentSigns = false;
        for (std::size_t index = 0; index < evaluation.cars.size(); ++index)
        {
            sum += evaluation.cars[index].generalizedGravityForceNewtons;
            if (index != 0
                && std::signbit(evaluation.cars[index]
                        .generalizedGravityForceNewtons)
                    != std::signbit(evaluation.cars[0]
                        .generalizedGravityForceNewtons))
            {
                hasDifferentSigns = true;
            }
        }
        requireNear(sum, evaluation.generalizedGravityForceNewtons,
            1.0e-9, "per-car valley gravity sum");
        require(hasDifferentSigns, "valley train spans different slopes");
    }

    void movingHeavyCarChangesDistributedGravity()
    {
        TrainDefinition frontHeavy = trainOf(3, 0.4);
        frontHeavy.cars[0] = loadedCar(2500.0, 0.0);
        TrainDefinition rearHeavy = trainOf(3, 0.4);
        rearHeavy.cars[2] = loadedCar(2500.0, 0.0);
        const auto track = verticalArcTrack(true);
        const double frontForce = evaluateTrainKinematics(
            track, frontHeavy, PhysicsEnvironment{}, locationAt(25.0))
            .generalizedGravityForceNewtons;
        const double rearForce = evaluateTrainKinematics(
            track, rearHeavy, PhysicsEnvironment{}, locationAt(25.0))
            .generalizedGravityForceNewtons;
        require(std::abs(frontForce - rearForce) > 1000.0,
            "mass placement changes distributed gravity");
    }

    void gravityMatchesPotentialDerivative()
    {
        const auto track = verticalArcTrack(true);
        const TrainDefinition train = trainOf(4, 0.4);
        const TrackLocation center = locationAt(25.0);
        const auto evaluation = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, center);
        constexpr double epsilon = trainKinematicJacobianStepMeters;
        TrackLocation before = track.advance(center, -epsilon).location;
        TrackLocation after = track.advance(center, epsilon).location;
        before.direction = center.direction;
        after.direction = center.direction;
        const double derivative =
            (potentialEnergy(solveTrainPose(track, train, after))
                - potentialEnergy(solveTrainPose(track, train, before)))
            / (2.0 * epsilon);
        requireNear(evaluation.generalizedGravityForceNewtons,
            -derivative, 1.0e-6,
            "gravity generalized force equals negative potential derivative");
    }

    void effectiveMassEqualsTotalOnStraightTrack()
    {
        const auto evaluation = evaluateTrainKinematics(
            straightTrack(),
            trainOf(6),
            PhysicsEnvironment{},
            locationAt(80.0));
        requireNear(evaluation.effectiveGeneralizedMassKilograms,
            evaluation.pose.totalLoadedMassKilograms(),
            5.0e-6,
            "straight-track translational effective mass");
    }

    void resistanceOpposesBothVelocitySigns()
    {
        TrainDefinition train = trainOf(2);
        train.resistance.constantMechanicalForceNewtons = 100.0;
        train.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            20.0;
        const auto track = straightTrack();
        const auto positive = stepTrain(
            track, train, PhysicsEnvironment{}, dynamicsState(50.0, 5.0),
            FixedStepSettings{0.01});
        const auto negative = stepTrain(
            track, train, PhysicsEnvironment{},
            dynamicsState(50.0, -5.0, TravelDirection::DecreasingStation),
            FixedStepSettings{0.01});
        require(positive.telemetry.resistanceForceNewtons < 0.0
            && negative.telemetry.resistanceForceNewtons > 0.0,
            "train resistance sign");
    }

    void staticResistanceHoldsWithoutReversal()
    {
        TrainDefinition train = trainOf(2);
        train.resistance.constantMechanicalForceNewtons = 10'000.0;
        const glm::dvec3 tangent{
            std::sqrt(1.0 - 0.1 * 0.1), 0.0, -0.1};
        TrainDynamicsState state = dynamicsState(50.0, 0.0);
        for (int index = 0; index < 10; ++index)
        {
            state = stepTrain(
                straightTrack(200.0, tangent),
                train,
                PhysicsEnvironment{},
                state,
                FixedStepSettings{0.01}).state;
        }
        requireNear(state.signedVelocityMetersPerSecond, 0.0, 0.0,
            "static train resistance hold");
        requireNear(state.generalizedReferenceLocation.stationMeters,
            50.0, 0.0, "static train resistance cannot reverse");
    }

    [[nodiscard]] double reducedEnergy(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrainDynamicsState& state)
    {
        const auto evaluation = evaluateTrainKinematics(
            track,
            train,
            PhysicsEnvironment{},
            state.generalizedReferenceLocation);
        return 0.5 * evaluation.effectiveGeneralizedMassKilograms
                * state.signedVelocityMetersPerSecond
                * state.signedVelocityMetersPerSecond
            + potentialEnergy(evaluation.pose);
    }

    [[nodiscard]] double energyErrorAfter(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        TrainDynamicsState state,
        const double timeStep,
        const int steps)
    {
        const double initialEnergy = reducedEnergy(track, train, state);
        for (int index = 0; index < steps; ++index)
        {
            state = stepTrain(
                track,
                train,
                PhysicsEnvironment{},
                state,
                FixedStepSettings{timeStep}).state;
        }
        return std::abs(reducedEnergy(track, train, state) - initialEnergy);
    }

    void gravityOnlyEnergyIsBoundedAndConverges()
    {
        const auto track = verticalArcTrack(false);
        const TrainDefinition train = trainOf(3, 0.4);
        const TrainDynamicsState initial = dynamicsState(24.0, 5.0);
        const double coarse = energyErrorAfter(
            track, train, initial, 1.0 / 120.0, 60);
        const double fine = energyErrorAfter(
            track, train, initial, 1.0 / 240.0, 120);
        require(std::isfinite(coarse) && coarse < 20'000.0,
            "gravity-only reduced energy bounded");
        require(fine < coarse * 0.8,
            "smaller fixed step improves reduced energy");
    }

    void bankingAloneCannotInjectLongitudinalGravity()
    {
        TrainDefinition train = trainOf(3);
        for (TrainCarDefinition& car : train.cars)
        {
            car.car.dryCenterOfGravityMeters = {0.0, 0.0, 0.0};
            car.car.frontHitchPositionMeters.z = 0.0;
            car.car.rearHitchPositionMeters.z = 0.0;
            car.loadout.centerOfMassMeters = {0.0, 0.0, 0.0};
        }
        const auto evaluation = evaluateTrainKinematics(
            straightBankTransitionTrack(),
            train,
            PhysicsEnvironment{},
            locationAt(30.0));
        requireNear(evaluation.generalizedGravityForceNewtons,
            0.0, 1.0e-6, "bank-only gravity rejection");
    }

    void repeatedSolvesAndRunsAreDeterministic()
    {
        const auto track = verticalArcTrack(false);
        const TrainDefinition train = trainOf(3, 0.4);
        const TrainPose first = solveTrainPose(track, train, locationAt(24.0));
        const TrainPose second = solveTrainPose(track, train, locationAt(24.0));
        requireNear(first.cars()[2].referenceLocation().stationMeters,
            second.cars()[2].referenceLocation().stationMeters,
            0.0, "deterministic train solve");
        auto run = [&]() {
            TrainDynamicsState state = dynamicsState(24.0, 4.0);
            for (int index = 0; index < 20; ++index)
            {
                state = stepTrain(
                    track,
                    train,
                    PhysicsEnvironment{},
                    state,
                    FixedStepSettings{}).state;
            }
            return state;
        };
        const TrainDynamicsState firstRun = run();
        const TrainDynamicsState secondRun = run();
        require(firstRun.generalizedReferenceLocation ==
                secondRun.generalizedReferenceLocation
            && firstRun.signedVelocityMetersPerSecond
                == secondRun.signedVelocityMetersPerSecond,
            "deterministic train integration");
    }

    void allRepresentativeOutputsRemainFinite()
    {
        const auto track = horizontalCircleTrack(25.0, true);
        const TrainDefinition train = trainOf(6, 0.4);
        for (const double station : {1.0, 20.0, 50.0, 100.0})
        {
            const auto evaluation = evaluateTrainKinematics(
                track,
                train,
                PhysicsEnvironment{},
                locationAt(station));
            require(std::isfinite(evaluation.effectiveGeneralizedMassKilograms)
                && std::isfinite(evaluation.generalizedGravityForceNewtons)
                && finite(evaluation.pose.aggregateWorldCenterOfGravityMeters()),
                "finite train-level output");
            for (const TrainCarPose& car : evaluation.pose.cars())
            {
                require(finite(car.loadedWorldCenterOfGravityMeters())
                    && finite(car.carPose().bodyOrientation()),
                    "finite car output");
            }
            for (const auto& connection : evaluation.pose.connections())
            {
                require(std::isfinite(connection.signedLengthResidualMeters())
                    && finite(connection.followingBodyRelativeOrientation()),
                    "finite connector output");
            }
        }
    }

    // Construction uses only TrackKinematicState -> CompiledPhysicsTrack;
    // rendered track styles, meshes, SDL, Vulkan, and editor data are absent.
    void canonicalTrackQueriesAreTheOnlyGeometryDependency()
    {
        const TrainPose pose = solveTrainPose(
            straightTrack(), trainOf(2), locationAt(50.0));
        require(pose.carCount() == 2,
            "canonical-only train physics construction");
    }

    void invalidDynamicsStateIsRejected()
    {
        TrainDynamicsState state = dynamicsState(50.0, 0.0);
        state.signedVelocityMetersPerSecond =
            std::numeric_limits<double>::quiet_NaN();
        requireThrows([&] {
            static_cast<void>(stepTrain(
                straightTrack(), trainOf(2), PhysicsEnvironment{}, state));
        }, "non-finite train state rejection");
    }

    void solverDiagnosticsArePopulated()
    {
        const TrainPose pose = solveTrainPose(
            horizontalCircleTrack(), trainOf(2), locationAt(30.0));
        const auto& connection = pose.connections()[0];
        require(connection.solverIterationCount() > 0,
            "connector iteration diagnostics");
        require(std::isfinite(connection.finalBracketSizeMeters())
            && connection.finalBracketSizeMeters() >= 0.0,
            "connector bracket diagnostics");
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

    std::fprintf(stdout, "Train Physics Tests\n");
    run("one-car Phase 2 parity", oneCarConsistMatchesPhaseTwo);
    run("two cars straight", twoCarsCloseOnStraightTrack);
    run("non-zero connector", nonZeroConnectorEndpointsRemainDistinct);
    run("zero-length connector", zeroLengthConnectorIsSafe);
    run("heterogeneous cars", heterogeneousCarsUseTheirOwnGeometry);
    run("ordinary non-passenger lead", nonPassengerLeadIsAnOrdinaryCar);
    run("connection count validation", malformedConnectionCountIsRejected);
    run("invalid connector length", invalidConnectorLengthsAreRejected);
    run("curved track", curvedTrackClosesConnector);
    run("crest and valley", crestAndValleyArticulate);
    run("banking transition", bankingTransitionKeepsIndependentCarRoll);
    run("six-car closure", sixCarClosureDoesNotAccumulate);
    run("connector direction", connectorDirectionIsFiniteAndNormalized);
    run("multiple-root protection", closestLocalCircuitRootIsSelected);
    run("impossible connection", impossibleConnectorFailsExplicitly);
    run("circuit seam", consistCrossesCircuitSeamContinuously);
    run("reverse travel", reverseTravelPreservesConsistOrder);
    run("open endpoint rejection", openTrackRejectsIncompleteEnvelope);
    run("open boundary stop", openBoundaryStepStopsCompleteConsist);
    run("total mass", heterogeneousLoadedMassesSum);
    run("aggregate COG", aggregateCogIsMassWeighted);
    run("constant-slope gravity", constantSlopeHasAnalyticalGravityAndMass);
    run("crest distributed gravity", crestGravityDiffersFromLeadPointApproximation);
    run("valley distributed gravity", valleyGravityReflectsMultipleSlopes);
    run("mass distribution", movingHeavyCarChangesDistributedGravity);
    run("potential consistency", gravityMatchesPotentialDerivative);
    run("effective mass", effectiveMassEqualsTotalOnStraightTrack);
    run("resistance sign", resistanceOpposesBothVelocitySigns);
    run("static hold", staticResistanceHoldsWithoutReversal);
    run("gravity-only energy", gravityOnlyEnergyIsBoundedAndConverges);
    run("bank energy rejection", bankingAloneCannotInjectLongitudinalGravity);
    run("determinism", repeatedSolvesAndRunsAreDeterministic);
    run("finite output", allRepresentativeOutputsRemainFinite);
    run("track-family independence", canonicalTrackQueriesAreTheOnlyGeometryDependency);
    run("invalid dynamics state", invalidDynamicsStateIsRejected);
    run("solver diagnostics", solverDiagnosticsArePopulated);

    std::fprintf(stdout, "%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
