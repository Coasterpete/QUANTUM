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
                {0.0, -0.3, 0.0}, {0.0, 1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.3, 0.0}, {0.0, -1.0, 0.0})
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

    void requireAllocationAvailable(
        const BogieContactFeasibilityResult& result,
        const std::string_view context)
    {
        require(result.allocation.status
                == BogieContactAllocationStatus::Available
                && result.allocation.unilateralFeasible(),
            std::string(context) + " unilateral allocation available");
        require(result.allocation.representativeContacts.size()
                == result.contacts.size()
                && result.allocation.reconstructedForceNewtons
                && result.allocation.reconstructedMomentNewtonMeters
                && result.allocation.forceResidualNewtons
                && result.allocation.momentResidualNewtonMeters,
            std::string(context) + " representative allocation published");
        require(glm::length(*result.allocation.forceResidualNewtons)
                    <= result.forceToleranceNewtons
                && glm::length(*result.allocation.momentResidualNewtonMeters)
                    <= result.momentToleranceNewtonMeters,
            std::string(context) + " physical residuals within tolerance");

        glm::dvec3 reconstructedForce{0.0};
        glm::dvec3 reconstructedMoment{0.0};
        double runningTotal = 0.0;
        double guideTotal = 0.0;
        double upstopTotal = 0.0;
        std::size_t reportingActiveCount = 0;
        for (const ContactAllocation& allocation
            : result.allocation.representativeContacts)
        {
            require(allocation.sourceContactIndex < result.contacts.size(),
                std::string(context) + " source contact index valid");
            const WorldBogieContact& contact =
                result.contacts[allocation.sourceContactIndex];
            require(allocation.role == contact.role,
                std::string(context) + " contact role preserved");
            require(allocation.normalForceNewtons >= 0.0,
                std::string(context) + " coefficient is nonnegative");
            requireNear(allocation.worldForceNewtons,
                allocation.normalForceNewtons * contact.worldNormal,
                1.0e-10,
                std::string(context) + " world force uses authored normal");
            require(allocation.reportingActive
                    == (allocation.normalForceNewtons
                        > result.allocation
                            .reportingActiveToleranceNewtons),
                std::string(context) + " reporting activity threshold");
            reportingActiveCount += allocation.reportingActive ? 1 : 0;
            reconstructedForce += allocation.worldForceNewtons;
            reconstructedMoment += glm::cross(
                contact.worldPositionMeters
                    - result.bogieReferenceWorldPositionMeters,
                allocation.worldForceNewtons);
            switch (allocation.role)
            {
            case BogieContactRole::Running:
                runningTotal += allocation.normalForceNewtons;
                break;
            case BogieContactRole::Guide:
                guideTotal += allocation.normalForceNewtons;
                break;
            case BogieContactRole::Upstop:
                upstopTotal += allocation.normalForceNewtons;
                break;
            }
        }

        requireNear(reconstructedForce,
            *result.allocation.reconstructedForceNewtons,
            1.0e-9,
            std::string(context) + " force reconstruction uses all coefficients");
        requireNear(reconstructedMoment,
            *result.allocation.reconstructedMomentNewtonMeters,
            1.0e-9,
            std::string(context) + " moment reconstruction uses all coefficients");
        requireNear(runningTotal, result.allocation.runningTotalNewtons,
            1.0e-9, std::string(context) + " running role total");
        requireNear(guideTotal, result.allocation.guideTotalNewtons,
            1.0e-9, std::string(context) + " guide role total");
        requireNear(upstopTotal, result.allocation.upstopTotalNewtons,
            1.0e-9, std::string(context) + " upstop role total");
        require(reportingActiveCount
                == result.allocation.reportingActiveContactCount,
            std::string(context) + " reporting active count");
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

    void phase10DiagnosticsPreserved()
    {
        // Phase 10 unconstrained diagnostics must remain available.
        const BogieContactFeasibilityResult runningNegative =
            analyzeSynthetic(runningPair(), {0.0, 0.0, -600.0});
        requireAvailable(runningNegative,
            "negative-up unconstrained running span (Phase 10)");
        require(runningNegative.diagnosticWrenchSpanCoefficients
                && (*runningNegative.diagnosticWrenchSpanCoefficients)[0]
                    < 0.0,
            "Phase 10 diagnostic: negative running coefficient preserved");

        const BogieContactFeasibilityResult upstopNegative =
            analyzeSynthetic(upstopPair(), {0.0, 0.0, -600.0});
        requireAvailable(upstopNegative,
            "Phase 10: oppositely oriented upstop capability");
        require(upstopNegative.diagnosticWrenchSpanCoefficients
                && (*upstopNegative.diagnosticWrenchSpanCoefficients)[0]
                    > 0.0,
            "Phase 10 diagnostic: upstop direction supports negative-up with positive diagnostic");

        require(runningNegative.allocation.status
                == BogieContactAllocationStatus::UnilaterallyInfeasible,
            "Phase 11: running-only negative-up is unilaterally infeasible");
        requireAllocationAvailable(upstopNegative,
            "Phase 11: upstop supports negative-up");
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
                && !ill.diagnosticWrenchSpanCoefficients
                && ill.allocation.status
                    == BogieContactAllocationStatus::Unavailable
                && ill.allocation.representativeContacts.empty(),
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
                && !unavailable.requiredWorldReactionNewtons
                && unavailable.allocation.status
                    == BogieContactAllocationStatus::Unavailable
                && unavailable.allocation.representativeContacts.empty(),
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
        requireAllocationAvailable(
            centered.bogies[0], "centered static front");
        requireAllocationAvailable(
            centered.bogies[1], "centered static rear");
        require(centered.bogies[0].requiredWorldReactionNewtons->z > 0.0
                && centered.bogies[1].requiredWorldReactionNewtons->z > 0.0,
            "static upright reactions point upward");

        const TrainDefinition offCenter = singleCarTrain(
            carDefinition(1'000.0, 1.15, 0.45));
        const BogieContactFeasibilityAnalysis shifted = analyzeTrain(
            track, offCenter, locationAt(50.0));
        requireAvailable(shifted.bogies[0], "off-center COG front");
        requireAvailable(shifted.bogies[1], "off-center COG rear");
        requireAllocationAvailable(
            shifted.bogies[0], "off-center COG front");
        requireAllocationAvailable(
            shifted.bogies[1], "off-center COG rear");
        require(std::abs(shifted.bogies[0]
                    .requiredWorldReactionNewtons->z
                - shifted.bogies[1].requiredWorldReactionNewtons->z)
                > 1.0,
            "off-center COG retains the Phase 9 load split");
        require(std::abs(
                    shifted.bogies[0].allocation.runningTotalNewtons
                    - shifted.bogies[1].allocation.runningTotalNewtons)
                > 1.0,
            "front and rear allocations retain their distinct load split");
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
        requireAllocationAvailable(crestResult.bogies[0], "crest front");
        requireAllocationAvailable(crestResult.bogies[1], "crest rear");
        requireAllocationAvailable(valleyResult.bogies[0], "valley front");
        requireAllocationAvailable(valleyResult.bogies[1], "valley rear");

        CarDefinition bankedCar = carDefinition();
        bankedCar.dryCenterOfGravityMeters.z = 0.0;
        const TrainDefinition bankedTrain = singleCarTrain(bankedCar);
        const CompiledPhysicsTrack banked = horizontalCircuit(0.55);
        const BogieContactFeasibilityAnalysis bankedResult = analyzeTrain(
            banked, bankedTrain, locationAt(12.0), 14.0);
        requireAvailable(bankedResult.bogies[0], "banked front");
        requireAvailable(bankedResult.bogies[1], "banked rear");
        requireAllocationAvailable(bankedResult.bogies[0], "banked front");
        requireAllocationAvailable(bankedResult.bogies[1], "banked rear");
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
            requireAllocationAvailable(atLoop.bogies[0], "loop front");
            requireAllocationAvailable(atLoop.bogies[1], "loop rear");
        }
        const BogieContactFeasibilityAnalysis top = analyzeTrain(
            loop, train, locationAt(std::numbers::pi * radius), 30.0);
        require(top.phase9Reactions.cars[0]
                    .aggregateWorldBogieReactionNewtons->z < 0.0,
            "loop top has negative local-up reaction");
        // Phase 11: at loop top, bogie is inverted. Running contacts (now on
        // track side) have world normals pointing -Z, matching the required
        // downward reaction. Upstops point +Z (opposite direction).
        require(top.bogies[0].allocation.unilateralFeasible()
                && top.bogies[1].allocation.unilateralFeasible(),
            "loop top feasible with running contacts (inverted)");
        require(top.bogies[0].allocation.runningTotalNewtons > 0.0
                && top.bogies[1].allocation.runningTotalNewtons > 0.0,
            "running contacts carry the negative-up load when inverted");
        require(top.bogies[0].allocation.upstopTotalNewtons == 0.0
                && top.bogies[1].allocation.upstopTotalNewtons == 0.0,
            "upstops inactive (point opposite to required load)");
        require(top.bogies[0].status == BogieContactFeasibilityStatus::Available
                && top.bogies[1].status == BogieContactFeasibilityStatus::Available,
            "Phase 10 still available");

        const auto inverted = analyzeTrain(
            straightTrack(200.0, true), train, locationAt(50.0));
        requireAvailable(inverted.bogies[0], "inverted front");
        requireAvailable(inverted.bogies[1], "inverted rear");
        requireAllocationAvailable(inverted.bogies[0], "inverted front");
        requireAllocationAvailable(inverted.bogies[1], "inverted rear");
        require(inverted.phase9Reactions.cars[0]
                    .frontBogie.trackFrameComponentsNewtons->z < 0.0
                && inverted.phase9Reactions.cars[0]
                    .rearBogie.trackFrameComponentsNewtons->z < 0.0,
            "inverted static reactions require negative local up capability");
        require(inverted.bogies[0].allocation.upstopTotalNewtons > 0.0
                && inverted.bogies[1].allocation.upstopTotalNewtons > 0.0
                && inverted.bogies[0].allocation.runningTotalNewtons == 0.0
                && inverted.bogies[1].allocation.runningTotalNewtons == 0.0,
            "inverted static train is supported by authored upstop normals");
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
        requireAllocationAvailable(atStart.bogies[0], "open start front");
        requireAllocationAvailable(atEnd.bogies[1], "open end rear");

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
            requireAllocationAvailable(
                bogie, "heterogeneous contact geometry");
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
        const auto singleRail = analyzeSynthetic(
            singleRailLike, {0.0, 220.0, 800.0});
        requireAvailable(singleRail,
            "central-spine SingleRail-like geometry");
        requireAllocationAvailable(singleRail,
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
        const auto planarHybrid = analyzeSynthetic(
            planarHybridLike, {0.0, -180.0, 750.0});
        requireAvailable(planarHybrid,
            "planar/hybrid-like geometry");
        requireAllocationAvailable(planarHybrid,
            "planar/hybrid-like geometry");
    }

    void finiteDiagnosticsAndAllocationState()
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
        requireAllocationAvailable(result,
            "finite conventional mixed allocation");
    }

    void unilateralAllocationBasic()
    {
        const auto result = analyzeSynthetic(runningPair(), {0.0, 0.0, 1'000.0});
        requireAvailable(result, "upright running allocation");
        requireAllocationAvailable(result, "upright running allocation");
        require(result.allocation.reportingActiveContactCount == 2,
            "both running contacts active");
        require(result.allocation.uniqueness
                == BogieContactAllocationUniqueness::Unique,
            "independent running pair allocation is unique");
        require(result.allocation.runningTotalNewtons > 990.0
            && result.allocation.runningTotalNewtons < 1'010.0,
            "running total matches required reaction");
        require(result.allocation.guideTotalNewtons == 0.0,
            "no guide load");
        require(result.allocation.upstopTotalNewtons == 0.0,
            "no upstop load");
        requireNear(result.allocation.representativeContacts[0]
                .normalForceNewtons,
            500.0, 1.0e-9, "left running representative");
        requireNear(result.allocation.representativeContacts[1]
                .normalForceNewtons,
            500.0, 1.0e-9, "right running representative");
    }

    void unilateralInvertedInfeasible()
    {
        const auto result = analyzeSynthetic(runningPair(), {0.0, 0.0, -600.0});
        require(result.status == BogieContactFeasibilityStatus::Available,
            "Phase 10 linearly feasible");
        require(result.wrenchSpanFeasible
                && result.diagnosticWrenchSpanCoefficients
                && (*result.diagnosticWrenchSpanCoefficients)[0] < -100.0,
            "Phase 10 preserves its negative unconstrained diagnostic");
        require(result.allocation.status
                == BogieContactAllocationStatus::UnilaterallyInfeasible
                && !result.allocation.unilateralFeasible(),
            "Phase 11 separately reports unilateral infeasibility");
        require(result.allocation.representativeContacts.empty()
                && !result.allocation.reconstructedForceNewtons
                && !result.allocation.forceResidualNewtons,
            "infeasible NNLS best fit is not published as an allocation");
    }

    void unilateralUpstopFeasible()
    {
        const auto result = analyzeSynthetic(upstopPair(), {0.0, 0.0, -600.0});
        requireAvailable(result, "negative-up upstop allocation");
        requireAllocationAvailable(result, "negative-up upstop allocation");
        require(result.allocation.upstopTotalNewtons > 590.0
            && result.allocation.upstopTotalNewtons < 610.0,
            "upstop total matches required reaction");
        require(result.allocation.runningTotalNewtons == 0.0,
            "no running load");
    }

    void unilateralGuideAllocation()
    {
        const auto runningOnly = analyzeSynthetic(runningPair(), {0.0, 500.0, 0.0});
        require(runningOnly.status
                == BogieContactFeasibilityStatus::ForceNotRepresentable
                && runningOnly.allocation.status
                    == BogieContactAllocationStatus::UnilaterallyInfeasible,
            "unconstrained span failure implies unilateral infeasibility");

        const auto withGuides = analyzeSynthetic(
            guidePair(), {0.0, 700.0, 0.0});
        requireAvailable(withGuides, "pure lateral guide allocation");
        requireAllocationAvailable(withGuides,
            "pure lateral guide allocation");
        require(withGuides.allocation.guideTotalNewtons > 690.0
            && withGuides.allocation.guideTotalNewtons < 710.0,
            "guide total matches required lateral reaction");
    }

    void unilateralCombinedRunningGuide()
    {
        std::vector<BogieContactDefinition> runningGuide = runningPair();
        const std::vector<BogieContactDefinition> guides = guidePair();
        runningGuide.insert(runningGuide.end(), guides.begin(), guides.end());
        const auto result = analyzeSynthetic(runningGuide, {0.0, 350.0, 800.0});
        requireAllocationAvailable(result, "combined running and guide");
        require(result.allocation.runningTotalNewtons > 790.0
            && result.allocation.runningTotalNewtons < 810.0,
            "running total correct");
        require(result.allocation.guideTotalNewtons > 340.0
            && result.allocation.guideTotalNewtons < 360.0,
            "guide total correct");
    }

    void unilateralZeroWrench()
    {
        const auto result = analyzeSynthetic(conventionalContacts(), {0.0, 0.0, 0.0});
        requireAllocationAvailable(result, "zero required wrench");
        require(result.allocation.reportingActiveContactCount == 0,
            "no active contacts");
        for (const auto& allocation
            : result.allocation.representativeContacts)
        {
            require(allocation.normalForceNewtons == 0.0,
                "all lambdas zero");
        }
        require(result.allocation.runningTotalNewtons == 0.0
            && result.allocation.guideTotalNewtons == 0.0
            && result.allocation.upstopTotalNewtons == 0.0,
            "all role totals zero");
        require(glm::length(*result.allocation.forceResidualNewtons) <= 1.0e-9,
            "zero force residual");
        require(glm::length(*result.allocation.momentResidualNewtonMeters) <= 1.0e-9,
            "zero moment residual");
    }

    void unilateralActiveSetRepairsNegativeDiagnostic()
    {
        const double diagonal = std::sqrt(3.0) / 2.0;
        const std::vector<BogieContactDefinition> contacts{
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.0}, {0.0, -0.5, diagonal}),
            contact(BogieContactRole::Guide,
                {0.0, 0.0, 0.0}, {0.0, -0.5, -diagonal})
        };
        const auto result = analyzeSynthetic(
            contacts, {0.0, 500.0, -1'000.0 * diagonal});
        requireAvailable(result, "negative basic diagnostic case");
        require(result.diagnosticWrenchSpanCoefficients
                && (*result.diagnosticWrenchSpanCoefficients)[1] < -900.0,
            "unconstrained basic representative is materially negative");
        requireAllocationAvailable(result,
            "active-set repaired negative diagnostic case");
        requireNear(result.allocation.representativeContacts[0]
                .normalForceNewtons,
            1'000.0, 1.0e-8, "first cone generator selected");
        requireNear(result.allocation.representativeContacts[1]
                .normalForceNewtons,
            0.0, 1.0e-10, "negative unconstrained coefficient not clamped");
        requireNear(result.allocation.representativeContacts[2]
                .normalForceNewtons,
            1'000.0, 1.0e-8, "alternate cone generator re-solved");
    }

    void unilateralPassiveRemovalResolvesAgain()
    {
        const double decoyAngle = -7.0 * std::numbers::pi / 9.0;
        const glm::dvec3 decoyNormal{
            0.0, std::cos(decoyAngle), std::sin(decoyAngle)};
        const glm::dvec3 supportNormal{
            0.0, 0.5, std::sqrt(3.0) / 2.0};
        const glm::dvec3 supportOnePosition{0.0, 0.25, 0.0};
        const glm::dvec3 supportTwoPosition{0.0, 0.75, 0.5};
        const std::vector<BogieContactDefinition> contacts{
            contact(BogieContactRole::Guide,
                {0.0, 0.75, -1.0}, decoyNormal),
            contact(BogieContactRole::Guide,
                supportOnePosition, supportNormal),
            contact(BogieContactRole::Guide,
                supportTwoPosition, decoyNormal)
        };

        constexpr double supportOneCoefficient = 1'000.0;
        const double supportTwoCoefficient = -supportOneCoefficient
            * glm::cross(supportOnePosition, supportNormal).x
            / glm::cross(supportTwoPosition, decoyNormal).x;
        const glm::dvec3 requiredForce = supportOneCoefficient
                * supportNormal
            + supportTwoCoefficient * decoyNormal;
        const auto result = analyzeSynthetic(contacts, requiredForce);

        requireAllocationAvailable(result,
            "passive removal and reduced re-solve");
        require(result.allocation.solverIterationCount >= 4,
            "blocked coefficient triggers a reduced passive re-solve");
        requireNear(result.allocation.representativeContacts[0]
                .normalForceNewtons,
            0.0, 1.0e-9, "decoy contact leaves passive set");
        requireNear(result.allocation.representativeContacts[1]
                .normalForceNewtons,
            supportOneCoefficient, 1.0e-8,
            "first support coefficient recovered after re-solve");
        requireNear(result.allocation.representativeContacts[2]
                .normalForceNewtons,
            supportTwoCoefficient, 1.0e-8,
            "second support coefficient recovered after re-solve");
    }

    void unilateralToleranceSeparationAndCleanup()
    {
        std::vector<BogieContactDefinition> contacts = runningPair();
        contacts.push_back(contact(BogieContactRole::Guide,
            {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}));
        constexpr double smallGuideLoad = 5.0e-4;
        const auto result = analyzeSynthetic(
            contacts, {0.0, smallGuideLoad, 1'000.0});
        requireAllocationAvailable(result, "tolerance separation");
        require(result.allocation.nonnegativeToleranceNewtons
                < result.allocation.reportingActiveToleranceNewtons,
            "mathematical and reporting tolerances are distinct");
        const ContactAllocation& guide =
            result.allocation.representativeContacts[2];
        requireNear(guide.normalForceNewtons,
            smallGuideLoad, 1.0e-12,
            "sub-reporting coefficient remains mathematical allocation");
        require(!guide.reportingActive,
            "small coefficient is inactive only for telemetry");
        requireNear(result.allocation.guideTotalNewtons,
            smallGuideLoad, 1.0e-12,
            "role total retains sub-reporting coefficient");
        for (const ContactAllocation& allocation
            : result.allocation.representativeContacts)
        {
            require(allocation.normalForceNewtons >= 0.0,
                "tiny boundary cleanup never publishes a negative coefficient");
        }
    }

    void unilateralUniqueAllocation()
    {
        const std::vector<BogieContactDefinition> contacts{
            contact(BogieContactRole::Running, {0.0, -0.5, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running, {0.0, 0.5, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Guide, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0})
        };
        const auto result = analyzeSynthetic(contacts, {0.0, 200.0, 1'000.0});
        requireAllocationAvailable(result, "unique three-contact allocation");
        require(result.wrenchRank == contacts.size()
                && result.allocation.uniqueness
                    == BogieContactAllocationUniqueness::Unique
                && result.allocation.representativeUnique(),
            "full-column-rank wrench allocation is proven unique");
    }

    void unilateralNonuniqueAndTieBreaking()
    {
        const std::vector<BogieContactDefinition> contacts{
            contact(BogieContactRole::Running, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0})
        };
        const auto result = analyzeSynthetic(contacts, {0.0, 0.0, 900.0});
        requireAllocationAvailable(result, "redundant deterministic allocation");
        require(result.allocation.uniqueness
                == BogieContactAllocationUniqueness::NonUnique,
            "redundant contacts are explicitly nonunique");
        requireNear(result.allocation.representativeContacts[0]
                .normalForceNewtons,
            900.0, 1.0e-9, "lowest source index wins equal-gradient tie");
        require(result.allocation.representativeContacts[1]
                    .normalForceNewtons == 0.0
                && result.allocation.representativeContacts[2]
                    .normalForceNewtons == 0.0,
            "deterministic basic representative leaves duplicates at zero");
    }

    void unilateralRankDeficientCases()
    {
        const std::vector<BogieContactDefinition> contacts{
            contact(BogieContactRole::Running, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}),
            contact(BogieContactRole::Running, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0})
        };
        const auto feasible = analyzeSynthetic(
            contacts, {0.0, 0.0, 1'000.0});
        requireAvailable(feasible, "rank-deficient feasible span");
        requireAllocationAvailable(feasible,
            "rank-deficient feasible allocation");
        require(feasible.wrenchRank == 1
                && feasible.allocation.uniqueness
                    == BogieContactAllocationUniqueness::NonUnique,
            "rank deficiency is feasible and demonstrably nonunique");

        const auto infeasible = analyzeSynthetic(
            contacts, {0.0, 500.0, 0.0});
        require(infeasible.status
                == BogieContactFeasibilityStatus::ForceNotRepresentable
                && infeasible.allocation.status
                    == BogieContactAllocationStatus::UnilaterallyInfeasible,
            "rank-deficient out-of-span wrench is infeasible, not singular");
    }

    void unilateralActiveInactiveFlags()
    {
        std::vector<BogieContactDefinition> contacts = runningPair();
        const std::vector<BogieContactDefinition> upstops = upstopPair();
        contacts.insert(contacts.end(), upstops.begin(), upstops.end());
        const auto result = analyzeSynthetic(contacts, {0.0, 0.0, 1'000.0});
        requireAllocationAvailable(result, "running/upstop activity");
        std::size_t runningActive = 0;
        std::size_t upstopActive = 0;
        for (const ContactAllocation& allocation
            : result.allocation.representativeContacts)
        {
            if (allocation.role == BogieContactRole::Running
                && allocation.reportingActive)
            {
                ++runningActive;
            }
            if (allocation.role == BogieContactRole::Upstop
                && allocation.reportingActive)
            {
                ++upstopActive;
            }
        }
        require(runningActive == 2, "running contacts active");
        require(upstopActive == 0, "upstop contacts inactive");
        require(result.allocation.uniqueness
                == BogieContactAllocationUniqueness::NonUnique,
            "opposed running/upstop self-stress is nonunique");
    }

    void unilateralPhase10Consistency()
    {
        // Phase 10 feasible -> Phase 11 feasible OR Phase 11 infeasible
        // (if exact support requires negative lambda).
        // Phase 11 feasible -> Phase 10 must be feasible.
        const auto result = analyzeSynthetic(conventionalContacts(), {0.0, 200.0, 800.0});
        requireAllocationAvailable(result, "Phase 10/11 consistency");
        require(result.status == BogieContactFeasibilityStatus::Available
                && result.wrenchSpanFeasible,
            "Phase 11 feasibility preserves Phase 10 status and diagnostics");
    }

    void unilateralDeterminism()
    {
        const auto r1 = analyzeSynthetic(conventionalContacts(), {0.0, 150.0, 950.0});
        const auto r2 = analyzeSynthetic(conventionalContacts(), {0.0, 150.0, 950.0});
        requireAllocationAvailable(r1, "first deterministic allocation");
        requireAllocationAvailable(r2, "second deterministic allocation");
        require(r1.allocation.status == r2.allocation.status
                && r1.allocation.uniqueness == r2.allocation.uniqueness
                && r1.allocation.reportingActiveContactCount
                    == r2.allocation.reportingActiveContactCount
                && r1.allocation.solverIterationCount
                    == r2.allocation.solverIterationCount,
            "deterministic allocation metadata");
        for (std::size_t index = 0;
            index < r1.allocation.representativeContacts.size();
            ++index)
        {
            requireNear(r1.allocation.representativeContacts[index]
                    .normalForceNewtons,
                r2.allocation.representativeContacts[index]
                    .normalForceNewtons,
                0.0, "deterministic representative coefficients");
        }
    }

    void unilateralCrestValley()
    {
        constexpr double arcMiddle = 24.0 * 0.9;
        const TrainDefinition train = singleCarTrain();
        const CompiledPhysicsTrack crest = verticalArcTrack(true);
        const CompiledPhysicsTrack valley = verticalArcTrack(false);

        const auto crestResult = analyzeTrain(crest, train, locationAt(arcMiddle), 14.0);
        const auto valleyResult = analyzeTrain(valley, train, locationAt(arcMiddle), 14.0);

        requireAllocationAvailable(crestResult.bogies[0], "crest front");
        requireAllocationAvailable(crestResult.bogies[1], "crest rear");
        requireAllocationAvailable(valleyResult.bogies[0], "valley front");
        requireAllocationAvailable(valleyResult.bogies[1], "valley rear");

        const double crestRunning = crestResult.bogies[0].allocation.runningTotalNewtons
            + crestResult.bogies[1].allocation.runningTotalNewtons;
        const double valleyRunning = valleyResult.bogies[0].allocation.runningTotalNewtons
            + valleyResult.bogies[1].allocation.runningTotalNewtons;

        require(valleyRunning > crestRunning,
            "valley support > crest support");
    }

    void unilateralLoopTop()
    {
        const auto runningOnly = analyzeSynthetic(runningPair(), {0.0, 0.0, -500.0});
        require(runningOnly.status == BogieContactFeasibilityStatus::Available
                && runningOnly.allocation.status
                    == BogieContactAllocationStatus::UnilaterallyInfeasible,
            "negative local-up is outside the running-only cone");

        std::vector<BogieContactDefinition> withUpstops = runningPair();
        const std::vector<BogieContactDefinition> upstops = upstopPair();
        withUpstops.insert(
            withUpstops.end(), upstops.begin(), upstops.end());
        const auto withUpstopsResult = analyzeSynthetic(withUpstops, {0.0, 0.0, -500.0});
        requireAllocationAvailable(withUpstopsResult,
            "negative local-up with upstops");
        require(withUpstopsResult.allocation.upstopTotalNewtons > 490.0,
            "upstops carry the load");
    }

    void unilateralReverseTravel()
    {
        const CompiledPhysicsTrack open = straightTrack();
        const TrainDefinition train = singleCarTrain();
        const auto reverse = analyzeTrain(
            open, train, locationAt(100.0, TravelDirection::DecreasingStation), -5.0);
        requireAllocationAvailable(reverse.bogies[0], "reverse front");
        requireAllocationAvailable(reverse.bogies[1], "reverse rear");
        for (const auto& bogie : reverse.bogies)
        {
            for (const ContactAllocation& allocation
                : bogie.allocation.representativeContacts)
            {
                require(allocation.role
                        == bogie.contacts[allocation.sourceContactIndex].role,
                    "role preserved in reverse");
            }
        }
    }

    void unilateralCircuitSeam()
    {
        const CompiledPhysicsTrack circuit = horizontalCircuit();
        CarDefinition circuitCar = carDefinition();
        circuitCar.dryCenterOfGravityMeters.z = 0.0;
        const TrainDefinition circuitTrain = singleCarTrain(circuitCar);
        const auto seam1 = analyzeTrain(
            circuit, circuitTrain, locationAt(0.005), 12.0);
        const auto seam2 = analyzeTrain(
            circuit, circuitTrain, locationAt(0.005), 12.0);
        for (std::size_t i = 0; i < seam1.bogies.size(); ++i)
        {
            require(seam1.bogies[i].allocation.status
                    == seam2.bogies[i].allocation.status,
                "deterministic feasibility at seam");
            require(seam1.bogies[i].allocation.reportingActiveContactCount
                    == seam2.bogies[i].allocation
                        .reportingActiveContactCount,
                "deterministic active count at seam");
            requireAllocationAvailable(seam1.bogies[i], "first seam bogie");
            requireAllocationAvailable(seam2.bogies[i], "second seam bogie");
        }
    }

    void unilateralNoContacts()
    {
        const auto result = analyzeSynthetic({}, {0.0, 0.0, 1'000.0});
        require(result.status == BogieContactFeasibilityStatus::NoContacts,
            "no contacts status");
        require(result.allocation.status
                    == BogieContactAllocationStatus::Unavailable
                && !result.allocation.unilateralFeasible()
                && result.allocation.representativeContacts.empty(),
            "missing authored geometry leaves allocation unavailable");
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
    run("Phase 10 diagnostics preserved", phase10DiagnosticsPreserved);
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
    run("finite diagnostics and allocation state",
        finiteDiagnosticsAndAllocationState);

    // Phase 11 tests
    run("unilateral allocation basic", unilateralAllocationBasic);
    run("unilateral inverted infeasible", unilateralInvertedInfeasible);
    run("unilateral upstop feasible", unilateralUpstopFeasible);
    run("unilateral guide allocation", unilateralGuideAllocation);
    run("unilateral combined running+guide", unilateralCombinedRunningGuide);
    run("unilateral zero wrench", unilateralZeroWrench);
    run("unilateral active-set repairs negative diagnostic",
        unilateralActiveSetRepairsNegativeDiagnostic);
    run("unilateral passive removal resolves again",
        unilateralPassiveRemovalResolvesAgain);
    run("unilateral tolerance separation and cleanup",
        unilateralToleranceSeparationAndCleanup);
    run("unilateral unique allocation", unilateralUniqueAllocation);
    run("unilateral nonunique and tie breaking",
        unilateralNonuniqueAndTieBreaking);
    run("unilateral rank deficient cases", unilateralRankDeficientCases);
    run("unilateral active/inactive flags", unilateralActiveInactiveFlags);
    run("unilateral Phase 10 consistency", unilateralPhase10Consistency);
    run("unilateral determinism", unilateralDeterminism);
    run("unilateral crest valley", unilateralCrestValley);
    run("unilateral loop top", unilateralLoopTop);
    run("unilateral reverse travel", unilateralReverseTravel);
    run("unilateral circuit seam", unilateralCircuitSeam);
    run("unilateral no contacts", unilateralNoContacts);

    return 0;
}
