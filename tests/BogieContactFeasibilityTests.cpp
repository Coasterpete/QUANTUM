#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using namespace quantum::physics;
    using quantum::coaster::TrackKinematicState;
    using quantum::coaster::TopologyKind;
    using quantum::geometry::CurveFrame;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
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
            throw std::runtime_error(std::string(message));
        }
    }

    void requireNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const std::string_view message)
    {
        if (glm::length(actual - expected) > tolerance)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Function>
    void requireThrows(Function&& function, const std::string_view message)
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

    [[nodiscard]] CurveFrame identityFrame()
    {
        return {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
    }

    [[nodiscard]] BogieReaction syntheticReaction(
        const glm::dvec3& reactionNewtons,
        const CurveFrame& bogieFrame = identityFrame(),
        const glm::dvec3& referenceWorldPositionMeters = {0.0, 0.0, 0.0},
        const BogieReactionRecoveryStatus status =
            BogieReactionRecoveryStatus::Available)
    {
        BogieReaction result;
        result.carIndex = 0;
        result.bogieDefinitionIndex = 0;
        result.role = BogieRole::Front;
        result.worldPositionMeters = referenceWorldPositionMeters;
        result.trackFrame = bogieFrame;
        result.bogieFrame = bogieFrame;
        result.status = status;
        if (status == BogieReactionRecoveryStatus::Available)
        {
            result.worldReactionNewtons = reactionNewtons;
        }
        return result;
    }

    [[nodiscard]] BogieContactDefinition contact(
        const BogieContactRole role,
        const glm::dvec3& position,
        const glm::dvec3& normal)
    {
        return {role, position, normal};
    }

    [[nodiscard]] std::vector<BogieContactDefinition> runningPair()
    {
        return {
            contact(BogieContactRole::Running,
                {0.0, -0.55, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 0.55, 0.0}, {0.0, 0.0, 1.0})
        };
    }

    [[nodiscard]] std::vector<BogieContactDefinition> guidePair()
    {
        return {
            contact(BogieContactRole::Guide,
                {0.0, 0.0, -0.3}, {0.0, 1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.3}, {0.0, 1.0, 0.0})
        };
    }

    [[nodiscard]] std::vector<BogieContactDefinition> upstopPair()
    {
        return {
            contact(BogieContactRole::Upstop,
                {0.0, -0.55, 0.0}, {0.0, 0.0, -1.0}),
            contact(BogieContactRole::Upstop,
                {0.0, 0.55, 0.0}, {0.0, 0.0, -1.0})
        };
    }

    [[nodiscard]] std::vector<BogieContactDefinition>
        conventionalContacts()
    {
        std::vector<BogieContactDefinition> result = runningPair();
        const std::vector<BogieContactDefinition> guides = guidePair();
        const std::vector<BogieContactDefinition> upstops = upstopPair();
        result.insert(result.end(), guides.begin(), guides.end());
        result.insert(result.end(), upstops.begin(), upstops.end());
        return result;
    }

    [[nodiscard]] BogieDefinition bogieWith(
        std::vector<BogieContactDefinition> contacts)
    {
        BogieDefinition result;
        result.contacts = std::move(contacts);
        return result;
    }

    [[nodiscard]] BogieContactFeasibilityResult analyzeSynthetic(
        const std::vector<BogieContactDefinition>& contacts,
        const glm::dvec3& reactionNewtons)
    {
        return analyzeBogieContactFeasibility(
            bogieWith(contacts), syntheticReaction(reactionNewtons));
    }

    void requireAvailable(
        const BogieContactFeasibilityResult& result,
        const std::string_view context)
    {
        require(result.status == BogieContactFeasibilityStatus::Available,
            std::string(context) + " available (status "
                + std::to_string(static_cast<int>(result.status)) + ")");
        require(result.forceSpanFeasible && result.wrenchSpanFeasible,
            std::string(context) + " force and wrench feasible");
        require(result.forceSpanResidualNewtons.has_value()
                && result.wrenchForceResidualNewtons.has_value()
                && result.wrenchMomentResidualNewtonMeters.has_value(),
            std::string(context) + " residuals published");
        require(glm::length(*result.forceSpanResidualNewtons)
                    <= result.forceToleranceNewtons
                && glm::length(*result.wrenchForceResidualNewtons)
                    <= result.forceToleranceNewtons
                && glm::length(*result.wrenchMomentResidualNewtonMeters)
                    <= result.momentToleranceNewtonMeters,
            std::string(context) + " residuals within tolerance");
    }

    [[nodiscard]] TrackLocation locationAt(
        const double stationMeters,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return {primaryTrackPathId, stationMeters, direction};
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const double lengthMeters = 200.0,
        const bool inverted = false)
    {
        const CurveFrame frame = inverted
            ? CurveFrame{
                {1.0, 0.0, 0.0},
                {0.0, -1.0, 0.0},
                {0.0, 0.0, -1.0}}
            : identityFrame();
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
        const std::vector<BogieContactDefinition>& contacts =
            conventionalContacts())
    {
        CarDefinition car;
        car.dryMassKilograms = massKilograms;
        car.dryCenterOfGravityMeters = {centerOfGravityX, 0.0, 0.65};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.25};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.25};
        car.bogies = {
            BogieDefinition{{-bogieHalfSpacingMeters, 0.0, 0.0}, contacts},
            BogieDefinition{{bogieHalfSpacingMeters, 0.0, 0.0}, contacts}
        };
        return car;
    }

    [[nodiscard]] TrainDefinition singleCarTrain(
        const CarDefinition& car = carDefinition())
    {
        TrainDefinition train;
        train.cars.push_back({car, {}});
        return train;
    }

    [[nodiscard]] TrainDynamicsState consistentState(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const double velocityMetersPerSecond = 0.0)
    {
        const TrainKinematicEvaluation kinematics = evaluateTrainKinematics(
            track, train, PhysicsEnvironment{}, location);
        TrainDynamicsState state;
        state.generalizedReferenceLocation = location;
        state.signedVelocityMetersPerSecond = velocityMetersPerSecond;
        state.generalizedAccelerationMetersPerSecondSquared =
            (kinematics.generalizedGravityForceNewtons
                - 0.5
                    * kinematics
                        .effectiveGeneralizedMassDerivativeKilogramsPerMeter
                    * velocityMetersPerSecond * velocityMetersPerSecond)
            / kinematics.effectiveGeneralizedMassKilograms;
        state.runState = velocityMetersPerSecond == 0.0
            ? FollowerRunState::Resting
            : FollowerRunState::Running;
        return state;
    }

    [[nodiscard]] BogieContactFeasibilityAnalysis analyzeTrain(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrackLocation& location,
        const double velocityMetersPerSecond = 0.0)
    {
        return evaluateBogieContactFeasibility(
            track,
            train,
            PhysicsEnvironment{},
            consistentState(track, train, location, velocityMetersPerSecond));
    }

    void validationAndEmptyGeometry()
    {
        const BogieContactFeasibilityResult empty = analyzeSynthetic(
            {}, {0.0, 0.0, 1'000.0});
        require(empty.status == BogieContactFeasibilityStatus::NoContacts
                && empty.contacts.empty()
                && !empty.forceSpanFeasible
                && !empty.wrenchSpanFeasible,
            "no contacts are cleanly unavailable");

        BogieDefinition invalid;
        invalid.contacts.push_back(contact(
            BogieContactRole::Running,
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}));
        requireThrows([&] { validateBogieDefinition(invalid); },
            "zero normal rejected");
        invalid.contacts[0].contactNormalLocal = {
            0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
        requireThrows([&] { validateBogieDefinition(invalid); },
            "non-finite normal rejected");
        invalid.contacts[0].contactNormalLocal = {0.0, 0.0, 1.0};
        invalid.contacts[0].localPositionMeters.x =
            std::numeric_limits<double>::infinity();
        requireThrows([&] { validateBogieDefinition(invalid); },
            "non-finite position rejected");
        invalid.contacts[0].localPositionMeters = {0.0, 0.0, 0.0};
        invalid.contacts[0].contactNormalLocal = {0.0, 0.0, 2.0};
        requireThrows([&] { validateBogieDefinition(invalid); },
            "non-unit normal rejected");
        invalid.contacts[0].contactNormalLocal =
            glm::normalize(glm::dvec3{1.0e-4, 0.0, 1.0});
        requireThrows([&] { validateBogieDefinition(invalid); },
            "tangent-contaminated normal rejected");
        invalid.contacts[0].contactNormalLocal = {0.0, 0.0, 1.0};
        invalid.contacts[0].role = static_cast<BogieContactRole>(255);
        requireThrows([&] { validateBogieDefinition(invalid); },
            "invalid role rejected");

        invalid.contacts.assign(maximumBogieContactCount + 1,
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
        requireThrows([&] { validateBogieDefinition(invalid); },
            "unreasonable contact count rejected");
    }

    void forceRoleCoverage()
    {
        requireAvailable(analyzeSynthetic(runningPair(), {0.0, 0.0, 900.0}),
            "running only upright");
        requireAvailable(analyzeSynthetic(guidePair(), {0.0, 700.0, 0.0}),
            "guide only lateral");
        requireAvailable(analyzeSynthetic(upstopPair(), {0.0, 0.0, -500.0}),
            "upstop only negative-up");

        std::vector<BogieContactDefinition> runningGuide = runningPair();
        const auto guides = guidePair();
        runningGuide.insert(runningGuide.end(), guides.begin(), guides.end());
        requireAvailable(analyzeSynthetic(
            runningGuide, {0.0, 350.0, 800.0}),
            "running plus guide combined");

        std::vector<BogieContactDefinition> runningUpstop = runningPair();
        const auto upstops = upstopPair();
        runningUpstop.insert(
            runningUpstop.end(), upstops.begin(), upstops.end());
        requireAvailable(analyzeSynthetic(
            runningUpstop, {0.0, 0.0, -600.0}),
            "running plus upstop");
        requireAvailable(analyzeSynthetic(
            conventionalContacts(), {0.0, -275.0, 625.0}),
            "running guide upstop coverage");
        requireAvailable(analyzeSynthetic({
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0})},
            {0.0, 400.0, 0.0}),
            "role does not override authored normal");

        const BogieContactFeasibilityResult lateralWithoutGuide =
            analyzeSynthetic(runningPair(), {0.0, 500.0, 0.0});
        require(lateralWithoutGuide.status
                    == BogieContactFeasibilityStatus::ForceNotRepresentable
                && !lateralWithoutGuide.forceSpanFeasible,
            "running-only lateral force is outside the span");
    }

    void unilateralSemanticsAreDeferred()
    {
        const BogieContactFeasibilityResult runningNegative =
            analyzeSynthetic(runningPair(), {0.0, 0.0, -600.0});
        requireAvailable(runningNegative,
            "negative-up unconstrained running span");
        require(runningNegative.unilateralFeasibilityDeferred
                && runningNegative.diagnosticWrenchSpanCoefficients
                && (*runningNegative.diagnosticWrenchSpanCoefficients)[0]
                    < 0.0,
            "negative running coefficient is diagnostic, not engagement");

        const BogieContactFeasibilityResult upstopNegative =
            analyzeSynthetic(upstopPair(), {0.0, 0.0, -600.0});
        requireAvailable(upstopNegative,
            "oppositely oriented upstop capability");
        require(upstopNegative.diagnosticWrenchSpanCoefficients
                && (*upstopNegative.diagnosticWrenchSpanCoefficients)[0]
                    > 0.0,
            "upstop direction supports negative-up with a positive diagnostic");
    }

    void forceAndWrenchDistinction()
    {
        const std::vector<BogieContactDefinition> offset{
            contact(BogieContactRole::Running,
                {0.0, 0.5, 0.0}, {0.0, 0.0, 1.0})
        };
        const BogieContactFeasibilityResult result = analyzeSynthetic(
            offset, {0.0, 0.0, 1'000.0});
        require(result.forceSpanFeasible
                && !result.wrenchSpanFeasible
                && result.status
                    == BogieContactFeasibilityStatus::WrenchNotRepresentable,
            "single offset contact passes force but fails zero-moment wrench");
        require(result.forceSpanResidualNewtons
                && result.wrenchMomentResidualNewtonMeters
                && glm::length(*result.forceSpanResidualNewtons)
                    <= result.forceToleranceNewtons
                && glm::length(*result.wrenchMomentResidualNewtonMeters)
                    > result.momentToleranceNewtonMeters,
            "force and moment residuals diagnose offset contact");

        const BogieContactFeasibilityResult symmetric = analyzeSynthetic(
            runningPair(), {0.0, 0.0, 1'000.0});
        requireAvailable(symmetric, "symmetric running pair");
        requireNear(*symmetric.wrenchMomentResidualNewtonMeters,
            {0.0, 0.0, 0.0}, 1.0e-9,
            "symmetric running contacts cancel roll moment");

        const std::vector<BogieContactDefinition> wideRunningPair{
            contact(BogieContactRole::Running,
                {0.0, -2.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 2.0, 0.0}, {0.0, 0.0, 1.0})
        };
        const auto wide = analyzeSynthetic(
            wideRunningPair, {0.0, 0.0, 1'000.0});
        requireAvailable(wide, "wide symmetric running pair");
        requireNear(wide.momentRowScalePerMeter, 0.5, 1.0e-12,
            "wrench rows use inverse characteristic contact radius");

        const std::vector<BogieContactDefinition> mirroredGuides{
            contact(BogieContactRole::Guide,
                {0.0, -0.5, 0.0}, {0.0, 1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.5, 0.0}, {0.0, -1.0, 0.0})
        };
        requireAvailable(analyzeSynthetic(
            mirroredGuides, {0.0, 450.0, 0.0}),
            "mirrored guide pair");
    }

    void rankRedundancyAndConditioning()
    {
        const std::vector<BogieContactDefinition> duplicates{
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0})
        };
        const BogieContactFeasibilityResult redundant = analyzeSynthetic(
            duplicates, {0.0, 0.0, 900.0});
        requireAvailable(redundant, "redundant contacts");
        require(redundant.forceRank == 1
                && redundant.wrenchRank == 1
                && !redundant.forceRepresentativeUnique
                && !redundant.wrenchRepresentativeUnique,
            "rank-deficient feasible system is explicitly nonunique");

        const std::vector<BogieContactDefinition> nearCoincident{
            contact(BogieContactRole::Running,
                {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 1.0e-6, 0.0}, {0.0, 0.0, 1.0})
        };
        const BogieContactFeasibilityResult ill = analyzeSynthetic(
            nearCoincident, {0.0, 0.0, 900.0});
        require(ill.forceSpanFeasible
                && ill.wrenchSpanFeasible
                && ill.status == BogieContactFeasibilityStatus::IllConditioned
                && ill.wrenchConditionEstimate
                && *ill.wrenchConditionEstimate
                    > bogieContactMaximumConditionEstimate
                && !ill.diagnosticWrenchSpanCoefficients,
            "ill-conditioned geometry suppresses unstable wrench coefficients");
        requireNear(ill.momentRowScalePerMeter, 1.0, 0.0,
            "minimum one-metre wrench scale");
    }

    void bogieWorldTransforms()
    {
        const CurveFrame rotated{
            {0.0, 1.0, 0.0},
            {-1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        const BogieDefinition definition = bogieWith({
            contact(BogieContactRole::Guide,
                {1.0, 2.0, 3.0}, {0.0, 1.0, 0.0})
        });
        const BogieContactFeasibilityResult result =
            analyzeBogieContactFeasibility(
                definition,
                syntheticReaction(
                    {-250.0, 0.0, 0.0}, rotated, {10.0, 20.0, 30.0}));
        require(result.contacts.size() == 1
                && result.contacts[0].sourceContactIndex == 0
                && result.contacts[0].role == BogieContactRole::Guide,
            "world contact preserves source index and role");
        requireNear(result.contacts[0].worldPositionMeters,
            {8.0, 21.0, 33.0}, 1.0e-12,
            "bogie-local point transforms through oriented frame");
        requireNear(result.contacts[0].worldNormal,
            {-1.0, 0.0, 0.0}, 1.0e-12,
            "bogie-local normal transforms through oriented frame");

        const CarDefinition car = carDefinition();
        const CarPose pose = solveCarPose(
            straightTrack(), car, locationAt(50.0));
        requireNear(pose.frontBogie().transformLocalPoint({0.0, 0.4, 0.2}),
            pose.frontBogie().worldPositionMeters()
                + 0.4 * pose.frontBogie().orientedFrame().lateral
                + 0.2 * pose.frontBogie().orientedFrame().up,
            1.0e-12,
            "Phase 2 bogie point helper");
        requireNear(pose.frontBogie().transformLocalDirection({0.0, 0.0, 1.0}),
            pose.frontBogie().orientedFrame().up,
            1.0e-12,
            "Phase 2 bogie direction helper");
    }

    void phase9AvailabilityAndRequiredWrench()
    {
        const BogieContactFeasibilityResult unavailable =
            analyzeBogieContactFeasibility(
                bogieWith(runningPair()),
                syntheticReaction(
                    {0.0, 0.0, 0.0},
                    identityFrame(),
                    {0.0, 0.0, 0.0},
                    BogieReactionRecoveryStatus::IllConditioned));
        require(unavailable.status
                    == BogieContactFeasibilityStatus::Phase9ReactionUnavailable
                && !unavailable.requiredWorldReactionNewtons,
            "unavailable Phase 9 reaction remains unavailable");

        const auto available = analyzeSynthetic(
            runningPair(), {0.0, 0.0, 500.0});
        requireNear(available.requiredMomentNewtonMeters,
            {0.0, 0.0, 0.0}, 0.0,
            "Phase 9 point resultant requires zero bogie-reference moment");
    }

    void staticFrontRearAndOffCenterIntegration()
    {
        const CompiledPhysicsTrack track = straightTrack();
        const TrainDefinition centeredTrain = singleCarTrain();
        const BogieContactFeasibilityAnalysis centered = analyzeTrain(
            track, centeredTrain, locationAt(50.0));
        require(centered.phase9Reactions.exactAggregateRecoveryAvailable()
                && centered.bogies.size() == 2,
            "centered static Phase 9 feeds two bogies");
        require(centered.bogies[0].role == BogieRole::Front
                && centered.bogies[1].role == BogieRole::Rear,
            "front/rear result ordering is named");
        requireAvailable(centered.bogies[0], "centered static front");
        requireAvailable(centered.bogies[1], "centered static rear");
        require(centered.bogies[0].requiredWorldReactionNewtons->z > 0.0
                && centered.bogies[1].requiredWorldReactionNewtons->z > 0.0,
            "static upright reactions point upward");

        const TrainDefinition offCenter = singleCarTrain(
            carDefinition(1'000.0, 1.15, 0.45));
        const BogieContactFeasibilityAnalysis shifted = analyzeTrain(
            track, offCenter, locationAt(50.0));
        requireAvailable(shifted.bogies[0], "off-center COG front");
        requireAvailable(shifted.bogies[1], "off-center COG rear");
        require(std::abs(shifted.bogies[0]
                    .requiredWorldReactionNewtons->z
                - shifted.bogies[1].requiredWorldReactionNewtons->z)
                > 1.0,
            "off-center COG retains the Phase 9 load split");
    }

    void crestValleyBankAndInversionIntegration()
    {
        constexpr double arcMiddle = 24.0 * 0.9;
        const TrainDefinition train = singleCarTrain();
        const CompiledPhysicsTrack crest = verticalArcTrack(true);
        const CompiledPhysicsTrack valley = verticalArcTrack(false);
        const BogieContactFeasibilityAnalysis crestResult = analyzeTrain(
            crest, train, locationAt(arcMiddle), 14.0);
        const BogieContactFeasibilityAnalysis valleyResult = analyzeTrain(
            valley, train, locationAt(arcMiddle), 14.0);
        requireAvailable(crestResult.bogies[0], "crest front");
        requireAvailable(crestResult.bogies[1], "crest rear");
        requireAvailable(valleyResult.bogies[0], "valley front");
        requireAvailable(valleyResult.bogies[1], "valley rear");

        CarDefinition bankedCar = carDefinition();
        bankedCar.dryCenterOfGravityMeters.z = 0.0;
        const TrainDefinition bankedTrain = singleCarTrain(bankedCar);
        const CompiledPhysicsTrack banked = horizontalCircuit(0.55);
        const BogieContactFeasibilityAnalysis bankedResult = analyzeTrain(
            banked, bankedTrain, locationAt(12.0), 14.0);
        requireAvailable(bankedResult.bogies[0], "banked front");
        requireAvailable(bankedResult.bogies[1], "banked rear");
        const glm::dvec3 bankedComponents =
            bankedResult.phase9Reactions.cars[0]
                .frontBogie.trackFrameComponentsNewtons.value();
        require(std::abs(bankedComponents.y) > 1.0
                && std::abs(bankedComponents.z) > 1.0,
            "banked reaction combines lateral and up components");

        const CompiledPhysicsTrack loop = verticalLoopCircuit();
        constexpr double radius = 20.0;
        for (const double station : {
            0.0,
            0.5 * std::numbers::pi * radius,
            std::numbers::pi * radius})
        {
            const BogieContactFeasibilityAnalysis atLoop = analyzeTrain(
                loop, train, locationAt(station), 30.0);
            requireAvailable(atLoop.bogies[0], "loop front");
            requireAvailable(atLoop.bogies[1], "loop rear");
        }
        const BogieContactFeasibilityAnalysis top = analyzeTrain(
            loop, train, locationAt(std::numbers::pi * radius), 30.0);
        require(top.phase9Reactions.cars[0]
                    .aggregateWorldBogieReactionNewtons->z < 0.0
                && top.bogies[0].unilateralFeasibilityDeferred
                && top.bogies[1].unilateralFeasibilityDeferred,
            "loop top remains feasible with deferred unilateral semantics");

        const auto inverted = analyzeTrain(
            straightTrack(200.0, true), train, locationAt(50.0));
        requireAvailable(inverted.bogies[0], "inverted front");
        requireAvailable(inverted.bogies[1], "inverted rear");
        require(inverted.phase9Reactions.cars[0]
                    .frontBogie.trackFrameComponentsNewtons->z < 0.0
                && inverted.phase9Reactions.cars[0]
                    .rearBogie.trackFrameComponentsNewtons->z < 0.0,
            "inverted static reactions require negative local up capability");
    }

    void reverseSeamEndpointAndDeterminism()
    {
        const TrainDefinition train = singleCarTrain();
        const CompiledPhysicsTrack open = straightTrack();
        const auto reverse = analyzeTrain(
            open,
            train,
            locationAt(100.0, TravelDirection::DecreasingStation),
            -5.0);
        requireAvailable(reverse.bogies[0], "reverse front");
        requireAvailable(reverse.bogies[1], "reverse rear");
        require(glm::dot(reverse.bogies[0].contacts[0].worldNormal,
                    reverse.phase9Reactions.cars[0]
                        .frontBogie.bogieFrame.up)
                > 1.0 - 1.0e-12,
            "reverse contact uses travel-oriented bogie frame");

        const auto atStart = analyzeTrain(open, train, locationAt(1.15));
        const auto atEnd = analyzeTrain(open, train, locationAt(198.85));
        require(atStart.phase9Reactions.finiteDifferenceKind
                    == TrainFiniteDifferenceKind::Forward
                && atEnd.phase9Reactions.finiteDifferenceKind
                    == TrainFiniteDifferenceKind::Backward,
            "open endpoints retain legal one-sided Phase 9 derivatives");
        requireAvailable(atStart.bogies[0], "open start front");
        requireAvailable(atEnd.bogies[1], "open end rear");

        const CompiledPhysicsTrack circuit = horizontalCircuit();
        CarDefinition circuitCar = carDefinition();
        circuitCar.dryCenterOfGravityMeters.z = 0.0;
        const TrainDefinition circuitTrain = singleCarTrain(circuitCar);
        const auto seam = analyzeTrain(
            circuit, circuitTrain, locationAt(0.005), 12.0);
        requireAvailable(seam.bogies[0], "circuit seam front");
        requireAvailable(seam.bogies[1], "circuit seam rear");

        const auto first = analyzeTrain(
            circuit, circuitTrain, locationAt(41.0), 11.0);
        const auto second = analyzeTrain(
            circuit, circuitTrain, locationAt(41.0), 11.0);
        for (std::size_t index = 0; index < first.bogies.size(); ++index)
        {
            require(first.bogies[index].status == second.bogies[index].status
                    && first.bogies[index].forceRank
                        == second.bogies[index].forceRank
                    && first.bogies[index].wrenchRank
                        == second.bogies[index].wrenchRank,
                "deterministic status and rank");
            requireNear(
                *first.bogies[index].wrenchForceResidualNewtons,
                *second.bogies[index].wrenchForceResidualNewtons,
                0.0,
                "deterministic force residual");
            requireNear(
                *first.bogies[index].wrenchMomentResidualNewtonMeters,
                *second.bogies[index].wrenchMomentResidualNewtonMeters,
                0.0,
                "deterministic moment residual");
        }
    }

    void heterogeneousAndGenericGeometryIntegration()
    {
        std::vector<BogieContactDefinition> planar = conventionalContacts();
        planar[0].localPositionMeters = {0.0, -0.8, 0.1};
        planar[1].localPositionMeters = {0.0, 0.8, 0.1};

        TrainDefinition train;
        train.cars.push_back({carDefinition(700.0), {}});
        train.cars.push_back({carDefinition(1'400.0, 1.35, -0.2, planar), {}});
        train.connections.push_back({0.5});
        const CompiledPhysicsTrack track = straightTrack(300.0);
        const auto heterogeneous = analyzeTrain(
            track, train, locationAt(180.0));
        require(heterogeneous.bogies.size() == 4,
            "heterogeneous train returns two bogies per car");
        for (const BogieContactFeasibilityResult& bogie
            : heterogeneous.bogies)
        {
            requireAvailable(bogie, "heterogeneous contact geometry");
            require(bogie.contacts.size() == 6,
                "each car retains its authored contact collection");
        }

        const std::vector<BogieContactDefinition> singleRailLike{
            contact(BogieContactRole::Running,
                {0.0, -0.35, 0.25}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 0.35, 0.25}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, -0.25}, {0.0, 1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.25}, {0.0, 1.0, 0.0})
        };
        requireAvailable(analyzeSynthetic(
            singleRailLike, {0.0, 220.0, 800.0}),
            "central-spine SingleRail-like geometry");

        const std::vector<BogieContactDefinition> planarHybridLike{
            contact(BogieContactRole::Running,
                {0.0, -0.7, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running,
                {0.0, 0.7, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, -0.4}, {0.0, -1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.4}, {0.0, -1.0, 0.0})
        };
        requireAvailable(analyzeSynthetic(
            planarHybridLike, {0.0, -180.0, 750.0}),
            "planar/hybrid-like geometry");
    }

    void finiteDiagnosticsAndNoAllocationState()
    {
        const BogieContactFeasibilityResult result = analyzeSynthetic(
            conventionalContacts(), {0.0, 345.0, 987.0});
        requireAvailable(result, "finite representative output");
        require(result.contactCount() == 6
                && result.requiredWorldReactionNewtons
                && finite(*result.requiredWorldReactionNewtons)
                && result.forceConditionEstimate
                && std::isfinite(*result.forceConditionEstimate)
                && result.wrenchConditionEstimate
                && std::isfinite(*result.wrenchConditionEstimate)
                && result.diagnosticForceSpanCoefficients
                && result.diagnosticWrenchSpanCoefficients
                && result.diagnosticForceSpanCoefficients->size()
                    == result.contacts.size()
                && result.diagnosticWrenchSpanCoefficients->size()
                    == result.contacts.size(),
            "all diagnostic outputs are finite and dimensioned by contacts");
        require(result.unilateralFeasibilityDeferred,
            "no engagement or unilateral active-set state is claimed");
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
    run("validation and empty geometry", validationAndEmptyGeometry);
    run("contact role and force coverage", forceRoleCoverage);
    run("unilateral semantics deferred", unilateralSemanticsAreDeferred);
    run("force and wrench distinction", forceAndWrenchDistinction);
    run("rank redundancy and conditioning", rankRedundancyAndConditioning);
    run("bogie world transforms", bogieWorldTransforms);
    run("Phase 9 availability and required wrench",
        phase9AvailabilityAndRequiredWrench);
    run("static front rear and off-center integration",
        staticFrontRearAndOffCenterIntegration);
    run("crest valley bank and inversion integration",
        crestValleyBankAndInversionIntegration);
    run("reverse seam endpoint and determinism",
        reverseSeamEndpointAndDeterminism);
    run("heterogeneous and generic geometry integration",
        heterogeneousAndGenericGeometryIntegration);
    run("finite diagnostics and no allocation state",
        finiteDiagnosticsAndNoAllocationState);
    return 0;
}
