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

    template<typename Function>
    void requireInvalid(Function&& function, const std::string_view message)
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

    [[nodiscard]] glm::dmat3 diagonalInertia(
        const double x,
        const double y,
        const double z)
    {
        glm::dmat3 result{0.0};
        result[0][0] = x;
        result[1][1] = y;
        result[2][2] = z;
        return result;
    }

    [[nodiscard]] CurveFrame rolledStraightFrame(const double rollRadians)
    {
        return quantum::geometry::applyRoll(
            CurveFrame{
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}},
            rollRadians);
    }

    template<typename RollFunction>
    [[nodiscard]] CompiledPhysicsTrack straightRollTrack(
        RollFunction&& roll,
        const double lengthMeters = 60.0)
    {
        constexpr double spacing = 0.01;
        const std::size_t count = static_cast<std::size_t>(
            std::round(lengthMeters / spacing));
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (std::size_t index = 0; index <= count; ++index)
        {
            const double station = index == count
                ? lengthMeters
                : spacing * static_cast<double>(index);
            samples.push_back({
                station,
                {station, 0.0, 0.0},
                rolledStraightFrame(roll(station)),
                {0.0, 0.0, 0.0}
            });
        }
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalCircleTrack(
        const double radiusMeters = 25.0,
        const bool varyingBank = false)
    {
        const std::size_t count = static_cast<std::size_t>(std::ceil(
            2.0 * std::numbers::pi * radiusMeters / 0.005));
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (std::size_t index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / static_cast<double>(count);
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
                radiusMeters * angle,
                {
                    radiusMeters * std::sin(angle),
                    radiusMeters * (1.0 - std::cos(angle)),
                    0.0
                },
                frame,
                {
                    -std::sin(angle) / radiusMeters,
                    std::cos(angle) / radiusMeters,
                    0.0
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalLoopTrack(
        const double radiusMeters = 18.0)
    {
        const std::size_t count = static_cast<std::size_t>(std::ceil(
            2.0 * std::numbers::pi * radiusMeters / 0.005));
        std::vector<TrackKinematicState> samples;
        samples.reserve(count + 1);
        for (std::size_t index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / static_cast<double>(count);
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, std::sin(angle)};
            samples.push_back({
                radiusMeters * angle,
                {
                    radiusMeters * std::sin(angle),
                    0.0,
                    radiusMeters * (1.0 - std::cos(angle))
                },
                {
                    tangent,
                    {0.0, 1.0, 0.0},
                    glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
                },
                {
                    -std::sin(angle) / radiusMeters,
                    0.0,
                    std::cos(angle) / radiusMeters
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CarDefinition carDefinition(
        const glm::dmat3& inertia = diagonalInertia(260.0, 900.0, 980.0))
    {
        CarDefinition car;
        car.dryMassKilograms = 800.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.0};
        car.dryInertiaTensorBodyKgM2 = inertia;
        car.bodyDimensionsMeters = {4.0, 1.4, 1.5};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.0};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.0};
        car.bogies = {
            BogieDefinition{{-1.0, 0.0, 0.0}},
            BogieDefinition{{1.0, 0.0, 0.0}}
        };
        return car;
    }

    [[nodiscard]] TrainDefinition singleCarTrain(
        const glm::dmat3& inertia = diagonalInertia(260.0, 900.0, 980.0))
    {
        TrainDefinition train;
        train.cars.push_back({carDefinition(inertia), {}});
        return train;
    }

    [[nodiscard]] TrainKinematicEvaluation evaluateAt(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const double station,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return evaluateTrainKinematics(
            track,
            train,
            PhysicsEnvironment{},
            locationAt(station, direction));
    }

    [[nodiscard]] TrainAngularKinematicEvaluation angularAt(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const double station,
        const double velocity,
        const double acceleration = 0.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        const TrainKinematicEvaluation kinematics = evaluateAt(
            track, train, station, direction);
        return evaluateTrainAngularKinematics(
            kinematics, velocity, acceleration);
    }

    void straightTrackHasZeroRotation()
    {
        const auto track = straightRollTrack([](double) { return 0.0; });
        const auto evaluation = evaluateAt(track, singleCarTrain(), 20.0);
        const auto angular = evaluateTrainAngularKinematics(
            evaluation, 12.0, 3.0);
        requireNear(angular.cars[0].worldAngularVelocityRadiansPerSecond(),
            {0.0, 0.0, 0.0}, 1.0e-12, "straight omega");
        requireNear(angular.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            {0.0, 0.0, 0.0}, 1.0e-12, "straight alpha");
        requireNear(angular.totalRotationalKineticEnergyJoules,
            0.0, 1.0e-12, "straight rotational energy");
        requireNear(evaluation.rotationalEffectiveGeneralizedMassKilograms,
            0.0, 1.0e-12, "straight rotational mass");
        requireNear(evaluation.effectiveGeneralizedMassKilograms,
            evaluation.translationalEffectiveGeneralizedMassKilograms,
            1.0e-12, "straight Phase 7 dynamics");
    }

    void constantRollMatchesAnalyticRateAndScaling()
    {
        constexpr double rollRate = 0.025;
        const auto track = straightRollTrack(
            [](const double station) { return rollRate * station; });
        const TrainDefinition train = singleCarTrain();
        const auto evaluation = evaluateAt(track, train, 25.0);
        const auto slow = evaluateTrainAngularKinematics(
            evaluation, 4.0, 0.0);
        const auto fast = evaluateTrainAngularKinematics(
            evaluation, 8.0, 0.0);
        requireNear(slow.cars[0].worldAngularVelocityRadiansPerSecond(),
            {rollRate * 4.0, 0.0, 0.0}, 2.0e-7,
            "constant-roll analytic omega");
        requireNear(slow.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            {0.0, 0.0, 0.0}, 2.0e-7,
            "constant-roll zero geometry alpha");
        requireNear(fast.cars[0].worldAngularVelocityRadiansPerSecond(),
            2.0 * slow.cars[0].worldAngularVelocityRadiansPerSecond(),
            2.0e-7, "speed doubling omega");
        requireNear(fast.totalRotationalKineticEnergyJoules,
            4.0 * slow.totalRotationalKineticEnergyJoules,
            2.0e-7, "speed doubling rotational energy");
        requireNear(evaluation.rotationalEffectiveGeneralizedMassKilograms,
            260.0 * rollRate * rollRate, 2.0e-7,
            "constant-roll effective mass");
    }

    void variableRollIncludesGeometryAndQddTerms()
    {
        constexpr double coefficient = 0.001;
        constexpr double station = 20.0;
        constexpr double speed = 5.0;
        const auto track = straightRollTrack(
            [](const double q) { return coefficient * q * q; });
        const TrainDefinition train = singleCarTrain();
        const auto noQdd = angularAt(track, train, station, speed);
        const auto withQdd = angularAt(track, train, station, speed, 2.0);
        requireNear(noQdd.cars[0].worldAngularVelocityRadiansPerSecond(),
            {2.0 * coefficient * station * speed, 0.0, 0.0},
            3.0e-6, "variable-roll omega");
        requireNear(noQdd.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            {2.0 * coefficient * speed * speed, 0.0, 0.0},
            5.0e-5, "variable-roll qdot-squared alpha");
        requireNear(
            withQdd.cars[0].worldAngularAccelerationRadiansPerSecondSquared()
                - noQdd.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            {2.0 * coefficient * station * 2.0, 0.0, 0.0},
            5.0e-6, "qdd angular-acceleration contribution");
    }

    void curvatureProducesPitchYawAndCompoundRotation()
    {
        constexpr double horizontalRadius = 25.0;
        const auto circle = horizontalCircleTrack(horizontalRadius);
        const auto yaw = angularAt(
            circle, singleCarTrain(), 15.0, 10.0);
        requireNear(yaw.cars[0].worldAngularVelocityRadiansPerSecond().z,
            10.0 / horizontalRadius, 2.0e-5,
            "horizontal curvature yaw rate");

        constexpr double loopRadius = 18.0;
        const auto loop = verticalLoopTrack(loopRadius);
        const double top = std::numbers::pi * loopRadius;
        const auto pitch = angularAt(
            loop, singleCarTrain(), top, 9.0);
        require(std::abs(pitch.cars[0]
                .worldAngularVelocityRadiansPerSecond().y) > 0.45,
            "inverted vertical curvature pitch rate");
        require(pitch.cars[0].available()
                && finite(pitch.cars[0]
                    .worldAngularAccelerationRadiansPerSecondSquared()),
            "inverted angular result finite and available");

        const auto compound = angularAt(
            horizontalCircleTrack(horizontalRadius, true),
            singleCarTrain(), 12.0, 11.0);
        require(std::abs(compound.cars[0]
                .bodyAngularVelocityRadiansPerSecond().x) > 1.0e-3
                && std::abs(compound.cars[0]
                    .worldAngularVelocityRadiansPerSecond().z) > 0.3,
            "banked compound rotation has roll and yaw");
    }

    void heterogeneousAndOffDiagonalInertiaAreCoherent()
    {
        constexpr double rollRate = 0.02;
        const auto track = straightRollTrack(
            [](const double station) { return rollRate * station; });
        TrainDefinition heterogeneous;
        heterogeneous.cars.push_back({
            carDefinition(diagonalInertia(100.0, 500.0, 550.0)), {}});
        heterogeneous.cars.push_back({
            carDefinition(diagonalInertia(400.0, 500.0, 550.0)), {}});
        heterogeneous.connections.push_back({0.5});
        const auto result = angularAt(track, heterogeneous, 30.0, 6.0);
        requireNear(result.cars[0].worldAngularVelocityRadiansPerSecond(),
            result.cars[1].worldAngularVelocityRadiansPerSecond(),
            2.0e-7, "heterogeneous kinematics independent of inertia");
        requireNear(result.cars[1].rotationalEffectiveMassKilograms(),
            4.0 * result.cars[0].rotationalEffectiveMassKilograms(),
            2.0e-7, "heterogeneous rotational mass");
        requireNear(result.cars[1].rotationalKineticEnergyJoules(),
            4.0 * result.cars[0].rotationalKineticEnergyJoules(),
            2.0e-7, "heterogeneous rotational energy");

        glm::dmat3 offDiagonal = diagonalInertia(500.0, 600.0, 700.0);
        offDiagonal[0][1] = offDiagonal[1][0] = 50.0;
        offDiagonal[0][2] = offDiagonal[2][0] = 20.0;
        offDiagonal[1][2] = offDiagonal[2][1] = 30.0;
        const TrainDefinition coupled = singleCarTrain(offDiagonal);
        const auto compoundTrack = horizontalCircleTrack(25.0, true);
        const auto kinematics = evaluateAt(compoundTrack, coupled, 12.0);
        const auto angular = evaluateTrainAngularKinematics(
            kinematics, 10.0, 1.0);
        const glm::dmat3 worldInertia = worldCarInertiaTensorKgM2(
            kinematics.pose.cars()[0].carPose(), offDiagonal);
        const double worldEnergy = 0.5 * glm::dot(
            angular.cars[0].worldAngularVelocityRadiansPerSecond(),
            worldInertia
                * angular.cars[0].worldAngularVelocityRadiansPerSecond());
        require(worldEnergy > 0.0, "off-diagonal energy positive");
        requireNear(worldEnergy,
            angular.cars[0].rotationalKineticEnergyJoules(),
            1.0e-9, "body/world rotational energy equivalence");
    }

    void inertiaValidationRejectsMalformedValues()
    {
        CarDefinition car = carDefinition();
        car.dryInertiaTensorBodyKgM2[0][0] =
            std::numeric_limits<double>::quiet_NaN();
        requireInvalid([&] { validateCarDefinition(car); },
            "non-finite inertia rejected");

        car = carDefinition();
        car.dryInertiaTensorBodyKgM2[0][1] = 10.0;
        requireInvalid([&] { validateCarDefinition(car); },
            "asymmetric inertia rejected");

        car = carDefinition();
        car.dryInertiaTensorBodyKgM2 = diagonalInertia(-1.0, 2.0, 2.0);
        requireInvalid([&] { validateCarDefinition(car); },
            "negative inertia rejected");

        car = carDefinition();
        car.dryInertiaTensorBodyKgM2 = diagonalInertia(0.0, 2.0, 2.0);
        requireInvalid([&] { validateCarDefinition(car); },
            "singular inertia rejected");

        car = carDefinition();
        car.dryInertiaTensorBodyKgM2 = diagonalInertia(1.0, 1.0, 3.0);
        requireInvalid([&] { validateCarDefinition(car); },
            "principal-moment triangle violation rejected");

        car = carDefinition(diagonalInertia(1.0e-200, 1.0e-200, 1.0e-200));
        validateCarDefinition(car);
        car = carDefinition(diagonalInertia(1.0e200, 1.0e200, 1.0e200));
        validateCarDefinition(car);
    }

    void loadoutUsesParallelAxisTheorem()
    {
        CarDefinition car = carDefinition(
            diagonalInertia(10.0, 20.0, 25.0));
        car.dryMassKilograms = 100.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.0};
        const CarLoadout load{100.0, {0.0, 1.0, 0.0}};
        requireNear(loadedCarCenterOfGravityMeters(car, load),
            {0.0, 0.5, 0.0}, 1.0e-12, "loaded COG");
        const glm::dmat3 loaded = loadedCarInertiaTensorBodyKgM2(car, load);
        requireNear(loaded[0][0], 60.0, 1.0e-12,
            "parallel-axis Ixx");
        requireNear(loaded[1][1], 20.0, 1.0e-12,
            "parallel-axis Iyy");
        requireNear(loaded[2][2], 75.0, 1.0e-12,
            "parallel-axis Izz");

        constexpr double rollRate = 0.02;
        const auto track = straightRollTrack(
            [](const double station) { return rollRate * station; });
        TrainDefinition dry;
        dry.cars.push_back({car, {}});
        TrainDefinition loadedTrain;
        loadedTrain.cars.push_back({car, load});
        const auto dryAngular = angularAt(track, dry, 20.0, 5.0);
        const auto loadedAngular = angularAt(
            track, loadedTrain, 20.0, 5.0);
        require(loadedAngular.totalRotationalKineticEnergyJoules
                > dryAngular.totalRotationalKineticEnergyJoules,
            "parallel-axis load increases roll energy");
    }

    void seamsBoundariesAndReverseAreStable()
    {
        constexpr double radius = 25.0;
        const auto circuit = horizontalCircleTrack(radius, true);
        const TrainDefinition train = singleCarTrain();
        const auto seam = angularAt(circuit, train, 0.005, 8.0);
        const auto interior = angularAt(circuit, train, 20.0, 8.0);
        require(finite(seam.cars[0].worldAngularVelocityRadiansPerSecond())
                && glm::length(seam.cars[0]
                    .worldAngularVelocityRadiansPerSecond()) < 2.0,
            "circuit seam has no angular spike");
        requireNear(seam.cars[0].worldAngularVelocityRadiansPerSecond().z,
            8.0 / radius, 3.0e-4, "seam yaw continuity");
        require(std::abs(interior.cars[0]
                .worldAngularVelocityRadiansPerSecond().z) > 0.3,
            "interior yaw finite");

        const auto reverse = angularAt(
            circuit, train, 20.0, -8.0, 0.0,
            TravelDirection::DecreasingStation);
        requireNear(reverse.cars[0].worldAngularVelocityRadiansPerSecond().z,
            -8.0 / radius, 3.0e-4, "reverse angular velocity sign");

        const auto open = straightRollTrack(
            [](const double q) { return 0.02 * q; });
        const auto front = evaluateAt(open, train, 1.0);
        const auto back = evaluateAt(open, train, 59.0);
        require(front.finiteDifferenceKind == TrainFiniteDifferenceKind::Forward,
            "open start uses forward angular derivative");
        require(back.finiteDifferenceKind == TrainFiniteDifferenceKind::Backward,
            "open end uses backward angular derivative");
        requireNear(front.cars[0]
                .worldAngularRateJacobianRadiansPerGeneralizedMeter.x,
            0.02, 5.0e-6, "open forward angular derivative");
        requireNear(back.cars[0]
                .worldAngularRateJacobianRadiansPerGeneralizedMeter.x,
            0.02, 5.0e-6, "open backward angular derivative");
    }

    void massDerivativeAndDynamicsAccountingAreConsistent()
    {
        constexpr double coefficient = 0.001;
        constexpr double station = 20.0;
        const auto track = straightRollTrack(
            [](const double q) { return coefficient * q * q; });
        const TrainDefinition train = singleCarTrain();
        const auto center = evaluateAt(track, train, station);
        constexpr double epsilon = trainKinematicJacobianStepMeters;
        const double numericDerivative =
            (evaluateAt(track, train, station + epsilon)
                    .rotationalEffectiveGeneralizedMassKilograms
                - evaluateAt(track, train, station - epsilon)
                    .rotationalEffectiveGeneralizedMassKilograms)
            / (2.0 * epsilon);
        require(center.rotationalEffectiveGeneralizedMassKilograms >= 0.0,
            "rotational effective mass non-negative");
        requireNear(
            center.rotationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter,
            numericDerivative, 2.0e-5, "rotational mass derivative");
        requireNear(center.effectiveGeneralizedMassKilograms,
            center.translationalEffectiveGeneralizedMassKilograms
                + center.rotationalEffectiveGeneralizedMassKilograms,
            1.0e-12, "total mass decomposition");
        requireNear(
            center.effectiveGeneralizedMassDerivativeKilogramsPerMeter,
            center.translationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter
                + center.rotationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter,
            1.0e-12, "total mass derivative decomposition");

        TrainDynamicsState state;
        state.generalizedReferenceLocation = locationAt(station);
        state.signedVelocityMetersPerSecond = 7.0;
        state.runState = FollowerRunState::Running;
        const TrainStepResult step = stepTrain(
            track, train, PhysicsEnvironment{}, state,
            FixedStepSettings{1.0e-4});
        const double expectedAcceleration = -0.5
            * center.effectiveGeneralizedMassDerivativeKilogramsPerMeter
            * 7.0 * 7.0 / center.effectiveGeneralizedMassKilograms;
        requireNear(step.state.generalizedAccelerationMetersPerSecondSquared,
            expectedAcceleration, 2.0e-7,
            "rotational mass gradient enters train equation");
        requireNear(step.telemetry.rotationalEffectiveGeneralizedMassKilograms,
            center.rotationalEffectiveGeneralizedMassKilograms,
            1.0e-12, "rotational telemetry mass");
        require(step.telemetry.totalRotationalKineticEnergyJoules > 0.0,
            "rotational telemetry energy");
    }

    [[nodiscard]] double reducedEnergy(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrainDynamicsState& state)
    {
        const auto evaluation = evaluateAt(
            track,
            train,
            state.generalizedReferenceLocation.stationMeters,
            state.generalizedReferenceLocation.direction);
        double potential = 0.0;
        for (const TrainCarPose& car : evaluation.pose.cars())
        {
            potential += car.loadedMassKilograms() * 9.80665
                * car.loadedWorldCenterOfGravityMeters().z;
        }
        return 0.5 * evaluation.effectiveGeneralizedMassKilograms
                * state.signedVelocityMetersPerSecond
                * state.signedVelocityMetersPerSecond
            + potential;
    }

    [[nodiscard]] double energyErrorAfter(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        TrainDynamicsState state,
        const double timeStep,
        const std::size_t stepCount)
    {
        const double initial = reducedEnergy(track, train, state);
        for (std::size_t index = 0; index < stepCount; ++index)
        {
            state = stepTrain(
                track, train, PhysicsEnvironment{}, state,
                FixedStepSettings{timeStep}).state;
        }
        return std::abs(reducedEnergy(track, train, state) - initial);
    }

    void totalEnergyIsBoundedAndConverges()
    {
        const auto bank = straightRollTrack(
            [](const double q) { return 0.001 * q * q; });
        const TrainDefinition train = singleCarTrain(
            diagonalInertia(1200.0, 1300.0, 1400.0));
        TrainDynamicsState state;
        state.generalizedReferenceLocation = locationAt(18.0);
        state.signedVelocityMetersPerSecond = 6.0;
        state.runState = FollowerRunState::Running;
        const double coarse = energyErrorAfter(bank, train, state, 1.0e-3, 200);
        const double fine = energyErrorAfter(bank, train, state, 5.0e-4, 400);
        require(std::isfinite(coarse) && coarse < 5.0,
            "orientation-only energy remains bounded");
        require(fine < coarse * 0.75,
            "orientation-only energy converges with timestep");

        const auto loop = verticalLoopTrack();
        state.generalizedReferenceLocation = locationAt(20.0);
        state.signedVelocityMetersPerSecond = 12.0;
        const double gravityCoarse = energyErrorAfter(
            loop, train, state, 1.0 / 500.0, 100);
        const double gravityFine = energyErrorAfter(
            loop, train, state, 1.0 / 1000.0, 200);
        require(std::isfinite(gravityCoarse) && gravityCoarse < 500.0,
            "gravity total energy with rotation bounded");
        require(gravityFine < gravityCoarse * 0.8,
            "gravity rotational energy converges with timestep");
    }

    void reactionConnectorAndDeterminismRemainCompatible()
    {
        const auto circle = horizontalCircleTrack(25.0, true);
        const TrainDefinition oneCar = singleCarTrain(
            diagonalInertia(700.0, 900.0, 1000.0));
        const auto evaluation = evaluateAt(circle, oneCar, 15.0);
        TrainDynamicsState reactionState;
        reactionState.generalizedReferenceLocation = locationAt(15.0);
        reactionState.signedVelocityMetersPerSecond = 8.0;
        reactionState.generalizedAccelerationMetersPerSecondSquared =
            (evaluation.generalizedGravityForceNewtons
                - 0.5
                    * evaluation.effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * 8.0 * 8.0)
            / evaluation.effectiveGeneralizedMassKilograms;
        reactionState.runState = FollowerRunState::Running;
        const BogieReactionAnalysis reactions = evaluateBogieReactions(
            circle, oneCar, PhysicsEnvironment{}, reactionState);
        require(reactions.exactAggregateRecoveryAvailable()
                && reactions.cars.size() == 1,
            "aggregate track reaction remains available");

        constexpr double rollRate = 0.03;
        const auto straight = straightRollTrack(
            [](const double q) { return rollRate * q; });
        TrainDefinition twoCars;
        twoCars.cars.push_back({carDefinition(), {}});
        twoCars.cars.push_back({carDefinition(), {}});
        twoCars.connections.push_back({0.5});
        TrainDynamicsState connectorState;
        connectorState.generalizedReferenceLocation = locationAt(25.0);
        const RigidConnectorLoadAnalysis loads = evaluateRigidConnectorLoads(
            straight, twoCars, PhysicsEnvironment{}, connectorState);
        require(loads.exactRecoveryAvailable()
                && loads.connectorLoads().size() == 1,
            "axial connector recovery remains available with rotation");

        TrainDefinition curvedCars;
        curvedCars.cars.push_back({carDefinition(
            diagonalInertia(700.0, 900.0, 1000.0)), {}});
        curvedCars.cars.push_back({carDefinition(
            diagonalInertia(1200.0, 1400.0, 1500.0)), {}});
        curvedCars.connections.push_back({0.5});
        const auto curvedEvaluation = evaluateAt(circle, curvedCars, 15.0);
        connectorState.generalizedReferenceLocation = locationAt(15.0);
        connectorState.signedVelocityMetersPerSecond = 8.0;
        connectorState.generalizedAccelerationMetersPerSecondSquared =
            (curvedEvaluation.generalizedGravityForceNewtons
                - 0.5
                    * curvedEvaluation.effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * 8.0 * 8.0)
            / curvedEvaluation.effectiveGeneralizedMassKilograms;
        connectorState.runState = FollowerRunState::Running;
        const RigidConnectorLoadAnalysis curvedLoads =
            evaluateRigidConnectorLoads(
                circle, curvedCars, PhysicsEnvironment{}, connectorState);
        require(curvedLoads.exactRecoveryAvailable(),
            "curved heterogeneous connector recovery includes rotation");

        const auto firstEvaluation = evaluateAt(circle, oneCar, 15.0);
        const auto secondEvaluation = evaluateAt(circle, oneCar, 15.0);
        const auto first = evaluateTrainAngularKinematics(
            firstEvaluation, 8.0, 1.5);
        const auto second = evaluateTrainAngularKinematics(
            secondEvaluation, 8.0, 1.5);
        requireNear(first.cars[0].worldAngularVelocityRadiansPerSecond(),
            second.cars[0].worldAngularVelocityRadiansPerSecond(),
            0.0, "angular velocity deterministic");
        requireNear(first.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            second.cars[0].worldAngularAccelerationRadiansPerSecondSquared(),
            0.0, "angular acceleration deterministic");
        requireNear(first.totalRotationalKineticEnergyJoules,
            second.totalRotationalKineticEnergyJoules,
            0.0, "rotational energy deterministic");
    }

    template<typename Function>
    void run(
        const std::string_view name,
        Function&& function,
        int& passed,
        int& failed)
    {
        try
        {
            function();
            ++passed;
        }
        catch (const std::exception& error)
        {
            ++failed;
            std::fprintf(stderr, "  FAIL  %.*s: %s\n",
                static_cast<int>(name.size()), name.data(), error.what());
        }
    }
}

int main()
{
    int passed = 0;
    int failed = 0;
    run("straight zero rotation", straightTrackHasZeroRotation, passed, failed);
    run("constant roll and speed scaling",
        constantRollMatchesAnalyticRateAndScaling, passed, failed);
    run("variable roll and qdd", variableRollIncludesGeometryAndQddTerms,
        passed, failed);
    run("pitch yaw compound geometry",
        curvatureProducesPitchYawAndCompoundRotation, passed, failed);
    run("heterogeneous off-diagonal inertia",
        heterogeneousAndOffDiagonalInertiaAreCoherent, passed, failed);
    run("inertia validation", inertiaValidationRejectsMalformedValues,
        passed, failed);
    run("loadout parallel axis", loadoutUsesParallelAxisTheorem,
        passed, failed);
    run("seams boundaries reverse", seamsBoundariesAndReverseAreStable,
        passed, failed);
    run("mass derivative and dynamics",
        massDerivativeAndDynamicsAccountingAreConsistent, passed, failed);
    run("energy consistency", totalEnergyIsBoundedAndConverges,
        passed, failed);
    run("reaction connector determinism",
        reactionConnectorAndDeterminismRemainCompatible, passed, failed);

    std::printf("Rotational Physics Tests\n%d passed, %d failed\n",
        passed, failed);
    return failed == 0 ? 0 : 1;
}
