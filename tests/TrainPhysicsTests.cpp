#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <chrono>
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

    void requireRigidBogiePivots(
        const TrainPose& pose,
        const TrainDefinition& definition,
        const std::string_view message)
    {
        constexpr double toleranceMeters = 1.0e-8;
        for (const TrainCarPose& trainCar : pose.cars())
        {
            const CarPose& carPose = trainCar.carPose();
            const CarDefinition& carDefinition =
                definition.cars[trainCar.carIndex()].car;
            for (const BogiePose* bogie : {
                &carPose.frontBogie(), &carPose.rearBogie()})
            {
                requireNear(
                    carPose.transformLocalPoint(
                        carDefinition.bogies[bogie->definitionIndex()]
                            .referencePositionMeters),
                    bogie->worldPositionMeters(),
                    toleranceMeters,
                    message);
            }
        }
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

    [[nodiscard]] CompiledPhysicsTrack shuttleSpikeTrack()
    {
        constexpr double runInLength = 40.0;
        constexpr double transitionRadius = 15.0;
        constexpr double transitionAngle = 1.2;
        constexpr double spikeLength = 80.0;
        constexpr int transitionSampleCount = 180;

        std::vector<TrackKinematicState> samples;
        samples.reserve(transitionSampleCount + 3);
        samples.push_back({
            0.0,
            {0.0, 0.0, 0.0},
            frameForTangent({1.0, 0.0, 0.0}),
            {0.0, 0.0, 0.0}
        });
        samples.push_back({
            runInLength,
            {runInLength, 0.0, 0.0},
            frameForTangent({1.0, 0.0, 0.0}),
            {0.0, 0.0, 1.0 / transitionRadius}
        });

        for (int index = 1; index <= transitionSampleCount; ++index)
        {
            const double angle = transitionAngle
                * static_cast<double>(index) / transitionSampleCount;
            const double station = runInLength
                + transitionRadius * angle;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, std::sin(angle)};
            samples.push_back({
                station,
                {
                    runInLength + transitionRadius * std::sin(angle),
                    0.0,
                    transitionRadius * (1.0 - std::cos(angle))
                },
                frameForTangent(tangent),
                {
                    -std::sin(angle) / transitionRadius,
                    0.0,
                    std::cos(angle) / transitionRadius
                }
            });
        }

        const double transitionEndStation = runInLength
            + transitionRadius * transitionAngle;
        const glm::dvec3 spikeTangent{
            std::cos(transitionAngle), 0.0, std::sin(transitionAngle)};
        const glm::dvec3 transitionEnd{
            runInLength + transitionRadius * std::sin(transitionAngle),
            0.0,
            transitionRadius * (1.0 - std::cos(transitionAngle))
        };
        samples.push_back({
            transitionEndStation + spikeLength,
            transitionEnd + spikeLength * spikeTangent,
            frameForTangent(spikeTangent),
            {0.0, 0.0, 0.0}
        });
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
        const CompiledPhysicsTrack track = horizontalCircleTrack();
        const TrainDefinition definition = trainOf(3);
        const TrainPose pose = solveTrainPose(
            track, definition, locationAt(30.0));
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "curved connector closure");
        require(std::abs(pose.connections()[0]
                .relativeYawPitchRollRadians().x) > 0.05,
            "curved articulation yaw");
        requireRigidBogiePivots(
            pose, definition, "curved multi-car rigid bogie pivots");
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
        const CompiledPhysicsTrack track = horizontalCircleTrack(35.0);
        const TrainDefinition definition = trainOf(6, 0.35);
        const TrainPose pose = solveTrainPose(
            track, definition, locationAt(55.0));
        require(pose.connectionCount() == 5, "six-car connection count");
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "six-car maximum closure error");
        requireRigidBogiePivots(
            pose, definition, "six-car rigid bogie pivots");
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
        const TrainDefinition definition = trainOf(3);
        const TrainPose pose = solveTrainPose(
            track, definition, locationAt(1.0));
        require(pose.cars()[1].referenceLocation().stationMeters
                > track.lengthMeters() - 10.0,
            "following car wraps behind seam");
        require(pose.maximumAbsoluteConnectorResidualMeters()
                <= connectorLengthToleranceMeters,
            "seam connector closure");
        requireRigidBogiePivots(
            pose, definition, "seam-crossing rigid bogie pivots");
    }

    void reverseTravelPreservesConsistOrder()
    {
        const TrainDefinition definition = trainOf(3);
        const TrainPose pose = solveTrainPose(
            straightTrack(),
            definition,
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
        requireRigidBogiePivots(
            pose, definition, "reverse multi-car rigid bogie pivots");
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

    void fourCarTrainNaturallyRollsBackFromSpike()
    {
        constexpr double fixedStepSeconds = 1.0 / 240.0;
        constexpr double initialStationMeters = 25.0;
        constexpr double initialVelocityMetersPerSecond = 24.0;
        constexpr double transitionBeginStationMeters = 40.0;
        constexpr double transitionEndStationMeters = 58.0;
        constexpr int maximumStepCount = 2'400;
        constexpr double maximumContinuousDisplacementMeters = 0.2;
        constexpr double minimumContinuousOrientationDot = 0.999;

        const CompiledPhysicsTrack track = shuttleSpikeTrack();
        const TrainDefinition train = trainOf(4, 0.5);
        TrainDynamicsState state = dynamicsState(
            initialStationMeters,
            initialVelocityMetersPerSecond,
            TravelDirection::IncreasingStation);
        TrainPose previousPose = solveTrainPose(
            track, train, state.generalizedReferenceLocation);
        double previousStation = initialStationMeters;
        double minimumAbsoluteVelocity = std::abs(
            initialVelocityMetersPerSecond);
        double maximumRigidPivotResidual = 0.0;
        double maximumConnectorResidual = 0.0;
        int reversalStep = -1;
        bool observedZeroNeighborhood = false;
        bool observedPositiveVelocity = false;
        bool observedNegativeVelocity = false;
        bool crossedTransitionForward = false;
        bool crossedTransitionEndBackward = false;
        bool crossedTransitionBeginBackward = false;
        bool usedExhaustiveConnectorFallback = false;

        const auto started = std::chrono::steady_clock::now();
        int completedSteps = 0;
        for (; completedSteps < maximumStepCount; ++completedSteps)
        {
            const double velocityBefore = state.signedVelocityMetersPerSecond;
            TrainStepResult result = stepTrain(
                track,
                train,
                PhysicsEnvironment{},
                state,
                FixedStepSettings{fixedStepSeconds});
            const TrainPose& pose = result.telemetry.pose;
            const double station = result.state.generalizedReferenceLocation
                .stationMeters;
            const double stationDelta = station - previousStation;

            require(result.state.generalizedReferenceLocation.direction
                    == TravelDirection::IncreasingStation,
                "rollback must not reverse the physical train orientation");
            require(!result.telemetry.boundaryIntervention
                    && result.telemetry.boundary == TrackBoundary::None,
                "rollback must remain inside the open-track envelope");
            require(std::isfinite(station)
                    && std::isfinite(
                        result.state.signedVelocityMetersPerSecond)
                    && std::isfinite(result.telemetry
                        .generalizedAccelerationMetersPerSecondSquared),
                "rollback dynamics must remain finite");
            if (result.state.signedVelocityMetersPerSecond > 0.0)
            {
                observedPositiveVelocity = true;
                require(stationDelta > 0.0,
                    "positive velocity must advance reference station");
            }
            else if (result.state.signedVelocityMetersPerSecond < 0.0)
            {
                observedNegativeVelocity = true;
                require(stationDelta < 0.0,
                    "negative velocity must reduce reference station");
                if (reversalStep < 0)
                {
                    reversalStep = completedSteps + 1;
                }
            }
            else
            {
                observedZeroNeighborhood = true;
                requireNear(stationDelta, 0.0, 0.0,
                    "zero-speed step must not move the train");
            }
            minimumAbsoluteVelocity = std::min(
                minimumAbsoluteVelocity,
                std::abs(result.state.signedVelocityMetersPerSecond));

            require(pose.carCount() == train.cars.size()
                    && pose.connectionCount() == train.connections.size(),
                "rollback committed pose shape");
            require(pose.maximumAbsoluteConnectorResidualMeters()
                    <= connectorLengthToleranceMeters,
                "rollback connector closure");
            maximumConnectorResidual = std::max(
                maximumConnectorResidual,
                pose.maximumAbsoluteConnectorResidualMeters());
            for (const InterCarConnectionPose& connection
                : pose.connections())
            {
                usedExhaustiveConnectorFallback |=
                    connection.usedExhaustiveSearchFallback();
            }

            for (std::size_t carIndex = 0;
                carIndex < pose.cars().size(); ++carIndex)
            {
                const TrainCarPose& car = pose.cars()[carIndex];
                const CarPose& carPose = car.carPose();
                const CarPose& previousCarPose =
                    previousPose.cars()[carIndex].carPose();
                require(car.carIndex() == carIndex,
                    "rollback must preserve authored car order");
                if (carIndex != 0)
                {
                    require(car.referenceLocation().stationMeters
                            < pose.cars()[carIndex - 1]
                                .referenceLocation().stationMeters,
                        "following car must remain behind its leading car");
                }
                require(glm::length(carPose.bodyWorldPositionMeters()
                            - previousCarPose.bodyWorldPositionMeters())
                        <= maximumContinuousDisplacementMeters,
                    "rollback car body position discontinuity");
                require(std::abs(glm::dot(
                            carPose.bodyOrientation(),
                            previousCarPose.bodyOrientation()))
                        >= minimumContinuousOrientationDot,
                    "rollback car body orientation discontinuity");
                require(finite(carPose.bodyWorldPositionMeters())
                        && finite(carPose.bodyOrientation()),
                    "rollback car pose must remain finite");

                const CarDefinition& carDefinition =
                    train.cars[carIndex].car;
                for (const BogiePose* bogie : {
                    &carPose.frontBogie(), &carPose.rearBogie()})
                {
                    const double pivotResidual = glm::length(
                        carPose.transformLocalPoint(
                            carDefinition.bogies[bogie->definitionIndex()]
                                .referencePositionMeters)
                        - bogie->worldPositionMeters());
                    maximumRigidPivotResidual = std::max(
                        maximumRigidPivotResidual, pivotResidual);
                    require(std::isfinite(pivotResidual)
                            && pivotResidual <= 1.0e-8,
                        "rollback rigid bogie pivot closure");
                    const double previousBogieStation =
                        bogie->definitionIndex()
                            == previousCarPose.frontBogie().definitionIndex()
                        ? previousCarPose.frontBogie().location().stationMeters
                        : previousCarPose.rearBogie().location().stationMeters;
                    require(std::abs(bogie->location().stationMeters
                                - previousBogieStation)
                            <= maximumContinuousDisplacementMeters,
                        "rollback bogie local-root discontinuity");
                    require(bogie->location().stationMeters > 0.0
                            && bogie->location().stationMeters
                                < track.lengthMeters(),
                        "rollback bogie must not clamp to an open endpoint");
                }
            }

            crossedTransitionForward |= previousStation
                < transitionEndStationMeters
                && station >= transitionEndStationMeters;
            crossedTransitionEndBackward |= observedNegativeVelocity
                && previousStation > transitionEndStationMeters
                && station <= transitionEndStationMeters;
            crossedTransitionBeginBackward |= observedNegativeVelocity
                && previousStation > transitionBeginStationMeters
                && station <= transitionBeginStationMeters;

            previousPose = pose;
            previousStation = station;
            state = result.state;
            if (crossedTransitionBeginBackward)
            {
                ++completedSteps;
                break;
            }
            require(!(velocityBefore == 0.0
                    && result.state.signedVelocityMetersPerSecond == 0.0),
                "gravity must restart the train backward after the zero step");
        }
        const auto elapsed = std::chrono::steady_clock::now() - started;
        const double elapsedMilliseconds = std::chrono::duration<double,
            std::milli>(elapsed).count();

        require(completedSteps < maximumStepCount,
            "rollback must complete within the deterministic step budget");
        require(observedPositiveVelocity && observedZeroNeighborhood
                && observedNegativeVelocity && reversalStep > 0,
            "rollback must naturally cross from positive through zero to negative velocity");
        require(crossedTransitionForward && crossedTransitionEndBackward
                && crossedTransitionBeginBackward,
            "rollback must traverse the complete transition in both directions");

        std::fprintf(stdout,
            "        spike metrics: steps=%d reversal_step=%d min_abs_velocity=%.12g "
            "max_pivot_residual=%.12g max_connector_residual=%.12g "
            "physics_ms=%.3f exhaustive_fallback=%s\n",
            completedSteps,
            reversalStep,
            minimumAbsoluteVelocity,
            maximumRigidPivotResidual,
            maximumConnectorResidual,
            elapsedMilliseconds,
            usedExhaustiveConnectorFallback ? "yes" : "no");
    }

    // Independent exact-pose evaluation of the original exhaustive grid. This
    // checks which root is selected, rather than only checking hitch closure.
    void adaptiveSearchMatchesExhaustiveCircuitRoot()
    {
        const TrainDefinition train = trainOf(2);
        constexpr double expectedOffset = 4.5;
        constexpr double searchHalfExtent = 1.1 * 4.0;
        constexpr double searchBegin = expectedOffset - searchHalfExtent;
        constexpr double searchEnd = expectedOffset + searchHalfExtent;
        // 3 m and 6 m need expansion; 2 m exceeds the local budget and must
        // still find the same adjacent root using the safety fallback.
        for (const double radius : {2.0, 3.0, 6.0, 12.0, 25.0})
        {
            const auto track = horizontalCircleTrack(radius);
            for (const auto direction : {
                TravelDirection::IncreasingStation,
                TravelDirection::DecreasingStation})
            {
                for (const double station : {0.1, track.lengthMeters() - 0.1})
                {
                    const TrackLocation location = locationAt(station, direction);
                    const CarPose lead = solveCarPose(
                        track, train.cars[0].car, location,
                        train.cars[0].loadout);
                    const double sign = direction
                        == TravelDirection::IncreasingStation ? 1.0 : -1.0;
                    const auto residual = [&](const double offset)
                    {
                        auto following = track.advance(
                            location, -sign * offset).location;
                        following.direction = direction;
                        const CarPose car = solveCarPose(
                            track, train.cars[1].car, following,
                            train.cars[1].loadout);
                        return glm::length(car.frontHitchWorldPositionMeters()
                            - lead.rearHitchWorldPositionMeters()) - 0.5;
                    };
                    const auto offsetAt = [](const std::size_t index)
                    {
                        return index == 160 ? searchEnd
                            : std::lerp(searchBegin, searchEnd,
                                static_cast<double>(index) / 160.0);
                    };
                    double lower = 0.0;
                    double upper = 0.0;
                    double nearest = std::numeric_limits<double>::infinity();
                    double previous = residual(offsetAt(0));
                    for (std::size_t index = 1; index <= 160; ++index)
                    {
                        const double current = residual(offsetAt(index));
                        const double distance = std::abs(0.5
                            * (offsetAt(index - 1) + offsetAt(index))
                            - expectedOffset);
                        if (std::signbit(previous) != std::signbit(current)
                            && distance < nearest)
                        {
                            lower = offsetAt(index - 1);
                            upper = offsetAt(index);
                            nearest = distance;
                        }
                        previous = current;
                    }
                    require(std::isfinite(nearest), "reference root exists");
                    double lowerResidual = residual(lower);
                    double upperResidual = residual(upper);
                    for (int iteration = 0; iteration < 80
                        && std::abs(lowerResidual) > connectorLengthToleranceMeters
                        && std::abs(upperResidual) > connectorLengthToleranceMeters;
                        ++iteration)
                    {
                        const double middle = 0.5 * (lower + upper);
                        const double middleResidual = residual(middle);
                        if (std::signbit(lowerResidual) == std::signbit(middleResidual))
                        {
                            lower = middle;
                            lowerResidual = middleResidual;
                        }
                        else
                        {
                            upper = middle;
                            upperResidual = middleResidual;
                        }
                    }
                    const double expected = std::abs(lowerResidual)
                        <= std::abs(upperResidual) ? lower : upper;
                    const TrainPose pose = solveTrainPose(track, train, location);
                    const auto expectedLocation = track.advance(
                        location, -sign * expected).location;
                    requireNear(pose.cars()[1].referenceLocation().stationMeters,
                        expectedLocation.stationMeters, 1.0e-12,
                        "adaptive/fallback selects exhaustive adjacent root");
                    require(pose.maximumAbsoluteConnectorResidualMeters()
                        <= connectorLengthToleranceMeters, "adaptive closure");
                    requireRigidBogiePivots(pose, train, "adaptive rigid pivots");
                }
            }
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

    std::fprintf(stdout, "Train Physics Tests\n");
    run("adaptive/exhaustive root parity", adaptiveSearchMatchesExhaustiveCircuitRoot);
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
    run("four-car spike rollback", fourCarTrainNaturallyRollsBackFromSpike);

    std::fprintf(stdout, "%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
