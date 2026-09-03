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

    [[nodiscard]] CompiledPhysicsTrack horizontalCircuit(
        const bool varyingBank = false)
    {
        constexpr double radius = 25.0;
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
                    frame, 0.25 * std::sin(2.0 * angle));
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

    [[nodiscard]] CarDefinition carDefinition(
        const double mass = 800.0,
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

    [[nodiscard]] TrainDynamicsState gravityConsistentState(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const double station,
        const double velocity = 0.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        const auto kinematics = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, locationAt(station, direction));
        TrainDynamicsState state;
        state.generalizedReferenceLocation = locationAt(station, direction);
        state.signedVelocityMetersPerSecond = velocity;
        state.generalizedAccelerationMetersPerSecondSquared =
            (kinematics.generalizedGravityForceNewtons
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
        const double station,
        const double velocity = 0.0,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return evaluateRigidConnectorLoads(
            track,
            train,
            PhysicsEnvironment{},
            gravityConsistentState(track, train, station, velocity, direction));
    }

    void requireAvailable(
        const RigidConnectorLoadAnalysis& result,
        const std::string_view message)
    {
        require(result.exactRecoveryAvailable(), message);
        require(result.status() == RigidConnectorLoadRecoveryStatus::Available,
            message);
        require(result.balanceResidualNewtons().has_value(), message);
        require(std::abs(*result.balanceResidualNewtons())
                <= result.balanceToleranceNewtons(),
            message);
    }

    [[nodiscard]] double forceAt(
        const RigidConnectorLoadAnalysis& result,
        const std::size_t connectionIndex)
    {
        require(result.connectorLoads()[connectionIndex]
                .axialForceNewtons().has_value(),
            "available force value");
        return *result.connectorLoads()[connectionIndex].axialForceNewtons();
    }

    void singleCarIsTriviallyAvailable()
    {
        TrainDefinition train;
        train.cars.push_back(loadedCar());
        const auto result = analyze(straightTrack(), train, 50.0);
        require(result.exactRecoveryAvailable()
                && result.connectorLoads().empty()
                && !result.maximumAbsoluteLoadNewtons(),
            "single-car empty load recovery");
    }

    void straightLevelAndConstantSlopeAreAnalyticallyUnloaded()
    {
        TrainDefinition train = trainOf(2);
        train.cars[0] = loadedCar(450.0, 0.0);
        train.cars[1] = loadedCar(1'900.0, 350.0);
        const auto level = analyze(straightTrack(), train, 50.0);
        requireAvailable(level, "straight level recovery");
        requireNear(forceAt(level, 0), 0.0, 1.0e-4,
            "straight level analytic force");

        const glm::dvec3 slope{
            std::sqrt(1.0 - 0.3 * 0.3), 0.0, -0.3};
        const auto inclined = analyze(straightTrack(200.0, slope), train, 50.0);
        requireAvailable(inclined, "constant slope recovery");
        // Every car has the same free g*sin(theta) acceleration, independent
        // of mass, so heterogeneous masses do not require an internal force.
        requireNear(forceAt(inclined, 0), 0.0, 2.0e-3,
            "constant-slope heterogeneous analytic force");
        require(inclined.connectorLoads()[0].classification()
                == RigidConnectorLoadClassification::NearZero,
            "near-zero classification dead-band");
    }

    void crestAndValleyRecoverMultipleLoads()
    {
        TrainDefinition train = trainOf(4, 0.4);
        train.cars[0] = loadedCar(1'800.0, 300.0);
        train.cars[1] = loadedCar(650.0, 50.0);
        train.cars[2] = loadedCar(1'100.0, 200.0);
        train.cars[3] = loadedCar(500.0, 0.0);
        for (const bool crest : {true, false})
        {
            const auto result = analyze(verticalArcTrack(crest), train, 25.0);
            requireAvailable(result, crest ? "crest recovery" : "valley recovery");
            require(result.connectorLoads().size() == 3,
                "multi-connector recovery count");
            require(result.maximumAbsoluteLoadNewtons().has_value()
                    && *result.maximumAbsoluteLoadNewtons() > 100.0,
                "vertical arc produces differential axial load");
        }
    }

    void speedChangesGeometricInertialLoad()
    {
        TrainDefinition train = trainOf(3, 0.4);
        train.cars[1] = loadedCar(2'400.0, 0.0);
        const auto track = verticalArcTrack(false);
        const auto stationary = analyze(track, train, 25.0, 0.0);
        const auto fast = analyze(track, train, 25.0, 18.0);
        requireAvailable(stationary, "stationary valley recovery");
        requireAvailable(fast, "moving valley recovery");
        require(std::abs(forceAt(stationary, 0) - forceAt(fast, 0)) > 10.0
                || std::abs(forceAt(stationary, 1) - forceAt(fast, 1)) > 10.0,
            "velocity-squared acceleration changes connector load");
    }

    void heterogeneousMassPlacementChangesBothSides()
    {
        const auto track = verticalArcTrack(true);
        TrainDefinition baseline = trainOf(3, 0.4);
        TrainDefinition leadHeavy = baseline;
        TrainDefinition middleHeavy = baseline;
        TrainDefinition rearHeavy = baseline;
        leadHeavy.cars[0] = loadedCar(3'000.0, 0.0);
        middleHeavy.cars[1] = loadedCar(3'000.0, 0.0);
        rearHeavy.cars[2] = loadedCar(3'000.0, 0.0);
        const auto lead = analyze(track, leadHeavy, 25.0);
        const auto middle = analyze(track, middleHeavy, 25.0);
        const auto rear = analyze(track, rearHeavy, 25.0);
        requireAvailable(lead, "heavy lead recovery");
        requireAvailable(middle, "heavy middle recovery");
        requireAvailable(rear, "heavy rear recovery");
        require(std::abs(forceAt(lead, 0) - forceAt(rear, 0)) > 100.0,
            "lead/rear mass placement changes front connector");
        require(std::abs(forceAt(middle, 0) - forceAt(lead, 0)) > 100.0
                && std::abs(forceAt(middle, 1) - forceAt(rear, 1)) > 100.0,
            "heavy middle changes both adjacent loads");

        TrainDefinition zeroLoadLead = baseline;
        zeroLoadLead.cars[0] = loadedCar(420.0, 0.0, 2.6, 0.75);
        const auto ordinaryLead = analyze(track, zeroLoadLead, 25.0);
        requireAvailable(ordinaryLead, "zero-load lead remains ordinary");
    }

    void forceVectorsAndClassificationFollowSignedForce()
    {
        TrainDefinition train = trainOf(4, 0.4);
        train.cars[0] = loadedCar(2'200.0, 0.0);
        const auto result = analyze(verticalArcTrack(true), train, 25.0);
        requireAvailable(result, "force vector recovery");
        for (const RigidConnectorLoad& load : result.connectorLoads())
        {
            require(load.worldDirection().has_value()
                    && load.worldForceOnLeadingCarNewtons().has_value()
                    && load.worldForceOnFollowingCarNewtons().has_value(),
                "world connector force fields");
            requireNear(
                *load.worldForceOnLeadingCarNewtons(),
                -*load.worldForceOnFollowingCarNewtons(),
                1.0e-10,
                "connector forces equal and opposite");
            requireNear(
                *load.worldForceOnLeadingCarNewtons(),
                *load.axialForceNewtons() * *load.worldDirection(),
                1.0e-10,
                "signed force direction convention");
            const double force = *load.axialForceNewtons();
            if (force > connectorLoadClassificationToleranceNewtons)
            {
                require(load.classification()
                        == RigidConnectorLoadClassification::Tension,
                    "positive force is tension");
            }
            else if (force < -connectorLoadClassificationToleranceNewtons)
            {
                require(load.classification()
                        == RigidConnectorLoadClassification::Compression,
                    "negative force is compression");
            }
        }
    }

    void resistanceAndUndefinedAxesAreUnavailable()
    {
        TrainDefinition resisted = trainOf(2);
        resisted.resistance.linearResistanceCoefficientNewtonSecondsPerMeter =
            20.0;
        const auto resistanceResult = evaluateRigidConnectorLoads(
            straightTrack(),
            resisted,
            PhysicsEnvironment{},
            gravityConsistentState(straightTrack(), resisted, 50.0, 5.0));
        require(!resistanceResult.exactRecoveryAvailable()
                && resistanceResult.status()
                    == RigidConnectorLoadRecoveryStatus::
                        AggregateResistanceUnderdetermined
                && !resistanceResult.connectorLoads()[0].axialForceNewtons(),
            "aggregate resistance is explicitly underdetermined");

        TrainDefinition zeroLength = trainOf(2, 0.0);
        const auto zeroResult = analyze(straightTrack(), zeroLength, 50.0);
        require(!zeroResult.exactRecoveryAvailable()
                && zeroResult.status()
                    == RigidConnectorLoadRecoveryStatus::UndefinedConnectorAxis
                && !zeroResult.connectorLoads()[0].worldDirection(),
            "zero-length connector axis unavailable");
    }

    void perpendicularConnectorIsIllConditioned()
    {
        TrainDefinition train = trainOf(2, 1.0);
        train.cars[1].car.frontHitchPositionMeters.y = 1.0;
        const auto result = analyze(straightTrack(), train, 50.0);
        require(!result.exactRecoveryAvailable()
                && result.status()
                    == RigidConnectorLoadRecoveryStatus::IllConditioned,
            "perpendicular hitch motion is rejected");
    }

    void inconsistentAccelerationInvalidatesRecovery()
    {
        const auto track = verticalArcTrack(true);
        const TrainDefinition train = trainOf(3, 0.4);
        TrainDynamicsState state = gravityConsistentState(track, train, 25.0);
        state.generalizedAccelerationMetersPerSecondSquared += 3.0;
        const auto result = evaluateRigidConnectorLoads(
            track, train, PhysicsEnvironment{}, state);
        require(!result.exactRecoveryAvailable()
                && result.status()
                    == RigidConnectorLoadRecoveryStatus::InconsistentBalance
                && result.balanceResidualNewtons().has_value()
                && std::abs(*result.balanceResidualNewtons())
                    > result.balanceToleranceNewtons(),
            "redundant global balance equation invalidates bad state");
    }

    void circuitSeamAndReverseRemainFinite()
    {
        const auto circuit = horizontalCircuit(true);
        const TrainDefinition train = trainOf(3, 0.4);
        const auto seam = analyze(circuit, train, 0.005, 14.0);
        requireAvailable(seam, "circuit seam recovery");
        for (const RigidConnectorLoad& load : seam.connectorLoads())
        {
            require(std::isfinite(*load.axialForceNewtons()),
                "finite seam force");
        }

        const auto reverse = analyze(
            verticalArcTrack(false),
            train,
            18.0,
            -9.0,
            TravelDirection::DecreasingStation);
        requireAvailable(reverse, "reverse recovery");
        for (std::size_t index = 0;
            index < reverse.connectorLoads().size();
            ++index)
        {
            require(reverse.connectorLoads()[index].leadingCarIndex() == index
                    && reverse.connectorLoads()[index].followingCarIndex()
                        == index + 1,
                "reverse motion preserves consist order");
        }
    }

    void openBoundaryUsesOneSidedDerivatives()
    {
        const auto track = straightTrack(80.0);
        const TrainDefinition train = trainOf(2);
        const auto result = analyze(track, train, 78.849);
        requireAvailable(result, "open boundary recovery");
        require(result.constrainedDerivativeKind()
                    == TrainFiniteDifferenceKind::Backward
                || result.localDerivativeKinds()[0]
                    == TrainFiniteDifferenceKind::Backward,
            "open boundary uses backward samples");
    }

    void compoundBankGeometryIsFinite()
    {
        TrainDefinition train = trainOf(3, 0.4);
        const auto result = analyze(
            verticalArcTrack(false, true), train, 25.0, 12.0);
        requireAvailable(result, "banked compound recovery");
        for (const RigidConnectorLoad& load : result.connectorLoads())
        {
            require(std::isfinite(*load.axialForceNewtons()),
                "finite banked connector load");
        }
    }

    void internalConnectorForcesCancelInGeneralizedMotion()
    {
        const auto track = verticalArcTrack(true);
        const TrainDefinition train = trainOf(4, 0.4);
        constexpr double station = 25.0;
        constexpr double epsilon = trainKinematicJacobianStepMeters;
        const auto result = analyze(track, train, station, 10.0);
        requireAvailable(result, "internal-force cancellation recovery");

        TrackLocation beforeLocation = track.advance(
            locationAt(station), -epsilon).location;
        TrackLocation afterLocation = track.advance(
            locationAt(station), epsilon).location;
        beforeLocation.direction = TravelDirection::IncreasingStation;
        afterLocation.direction = TravelDirection::IncreasingStation;
        const TrainPose before = solveTrainPose(track, train, beforeLocation);
        const TrainPose after = solveTrainPose(track, train, afterLocation);

        double generalizedInternalForce = 0.0;
        for (std::size_t index = 0;
            index < result.connectorLoads().size();
            ++index)
        {
            const glm::dvec3 rearHitchDerivative =
                (after.cars()[index].carPose()
                        .rearHitchWorldPositionMeters()
                    - before.cars()[index].carPose()
                        .rearHitchWorldPositionMeters())
                / (2.0 * epsilon);
            const glm::dvec3 frontHitchDerivative =
                (after.cars()[index + 1].carPose()
                        .frontHitchWorldPositionMeters()
                    - before.cars()[index + 1].carPose()
                        .frontHitchWorldPositionMeters())
                / (2.0 * epsilon);
            const RigidConnectorLoad& load = result.connectorLoads()[index];
            generalizedInternalForce += glm::dot(
                *load.worldForceOnLeadingCarNewtons(), rearHitchDerivative);
            generalizedInternalForce += glm::dot(
                *load.worldForceOnFollowingCarNewtons(), frontHitchDerivative);
        }
        requireNear(generalizedInternalForce, 0.0, 0.1,
            "internal connector forces cannot propel the train");
    }

    void analysisIsDeterministicAndDoesNotFeedBack()
    {
        const auto track = verticalArcTrack(false);
        const TrainDefinition train = trainOf(3, 0.4);
        const TrainDynamicsState state = gravityConsistentState(
            track, train, 25.0, 8.0);
        const auto first = evaluateRigidConnectorLoads(
            track, train, PhysicsEnvironment{}, state);
        const auto second = evaluateRigidConnectorLoads(
            track, train, PhysicsEnvironment{}, state);
        requireAvailable(first, "first deterministic recovery");
        requireAvailable(second, "second deterministic recovery");
        for (std::size_t index = 0; index < first.connectorLoads().size(); ++index)
        {
            requireNear(forceAt(first, index), forceAt(second, index), 0.0,
                "bit-stable connector recovery");
        }

        const auto stepBefore = stepTrain(
            track, train, PhysicsEnvironment{}, state, FixedStepSettings{});
        static_cast<void>(evaluateRigidConnectorLoads(
            track, train, PhysicsEnvironment{}, state));
        const auto stepAfter = stepTrain(
            track, train, PhysicsEnvironment{}, state, FixedStepSettings{});
        require(stepBefore.state.generalizedReferenceLocation
                    == stepAfter.state.generalizedReferenceLocation
                && stepBefore.state.signedVelocityMetersPerSecond
                    == stepAfter.state.signedVelocityMetersPerSecond,
            "derived connector loads do not affect integration");
    }

    void nonFiniteInputIsRejected()
    {
        const auto track = straightTrack();
        const TrainDefinition train = trainOf(2);
        TrainDynamicsState state = gravityConsistentState(track, train, 50.0);
        state.generalizedAccelerationMetersPerSecondSquared =
            std::numeric_limits<double>::quiet_NaN();
        requireThrows([&] {
            static_cast<void>(evaluateRigidConnectorLoads(
                track, train, PhysicsEnvironment{}, state));
        }, "non-finite load-analysis state rejection");
    }

    // Construction uses only canonical physics samples. TrackStylePreset,
    // rendered rail meshes, SDL, Vulkan, and Editor types are absent.
    void canonicalTrackDataIsTheOnlyGeometryDependency()
    {
        const auto result = analyze(horizontalCircuit(), trainOf(2), 20.0, 5.0);
        requireAvailable(result, "canonical track-only load recovery");
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

    std::fprintf(stdout, "Rigid Connector Load Physics Tests\n");
    run("single car", singleCarIsTriviallyAvailable);
    run("straight analytic loads", straightLevelAndConstantSlopeAreAnalyticallyUnloaded);
    run("crest and valley", crestAndValleyRecoverMultipleLoads);
    run("speed dependence", speedChangesGeometricInertialLoad);
    run("heterogeneous mass", heterogeneousMassPlacementChangesBothSides);
    run("force vectors and sign", forceVectorsAndClassificationFollowSignedForce);
    run("aggregate resistance and zero length", resistanceAndUndefinedAxesAreUnavailable);
    run("ill-conditioned projection", perpendicularConnectorIsIllConditioned);
    run("global balance residual", inconsistentAccelerationInvalidatesRecovery);
    run("circuit seam and reverse", circuitSeamAndReverseRemainFinite);
    run("open boundary", openBoundaryUsesOneSidedDerivatives);
    run("banked compound geometry", compoundBankGeometryIsFinite);
    run("internal force cancellation", internalConnectorForcesCancelInGeneralizedMotion);
    run("determinism and no feedback", analysisIsDeterministicAndDoesNotFeedBack);
    run("non-finite input", nonFiniteInputIsRejected);
    run("track-family independence", canonicalTrackDataIsTheOnlyGeometryDependency);

    std::fprintf(stdout, "%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
