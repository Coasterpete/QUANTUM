#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/RiderLoads.hpp>

#include <nlohmann/json.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
    using namespace quantum::coaster;
    using quantum::math::TransitionType;
    constexpr double pi = std::numbers::pi;

    void require(const bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    void near(const double actual, const double expected, const double tolerance,
        const std::string& message)
    {
        require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + " (actual " + std::to_string(actual) + ", expected " + std::to_string(expected) + ")");
    }

    void near(const glm::dvec3& actual, const glm::dvec3& expected,
        const double tolerance, const std::string& message)
    {
        near(glm::length(actual - expected), 0.0, tolerance, message);
    }

    ChannelProfile channel(const double length, const double begin, const double end,
        const TransitionType type = TransitionType::Linear)
    {
        return {{{7, {0.0, length, begin, end, type}}}, 19};
    }

    ForceDrivenRegion& force(AuthoredTrack& track, const std::size_t index = 0)
    {
        return std::get<ForceDrivenRegion>(std::get<GeometryRegion>(track.section(index).region).construction);
    }

    const ForceDrivenRegion& force(const AuthoredTrack& track, const std::size_t index = 0)
    {
        return std::get<ForceDrivenRegion>(std::get<GeometryRegion>(track.section(index).region).construction);
    }

    AuthoredTrack forceTrack(const double length = 30.0)
    {
        auto track = createNewDocument();
        track.section(0) = createForceDrivenSection(length);
        return track;
    }

    std::vector<TrackKinematicState> solve(const AuthoredTrack& track,
        const double spacing = 0.25, const double tolerance = 1.0e-10)
    {
        auto result = generateAuthoredTrackKinematics(track, spacing, {tolerance, 24});
        require(result.has_value(), result ? "" : result.error().message);
        return std::move(*result);
    }

    void checkFrames(const std::vector<TrackKinematicState>& states)
    {
        for (const auto& state : states)
        {
            near(glm::length(state.frame.tangent), 1.0, 2.0e-14, "unit tangent");
            near(glm::length(state.frame.lateral), 1.0, 2.0e-14, "unit lateral");
            near(glm::length(state.frame.up), 1.0, 2.0e-14, "unit up");
            near(glm::cross(state.frame.tangent, state.frame.lateral), state.frame.up,
                2.0e-14, "right-handed frame");
        }
    }

    void checkTargets(const AuthoredTrack& track, const std::vector<TrackKinematicState>& states)
    {
        const auto loads = evaluateRiderLoads(states, riderLoadEvaluationSettings(track.physicalSettings()));
        require(loads.completed() && loads.states.size() == states.size(), "complete actual loads");
        const auto& targets = force(track);
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            near(loads.states[i].normalG, evaluateChannelProfile(targets.targetNormalG, states[i].distance),
                2.0e-12, "actual normal agrees with target");
            near(loads.states[i].lateralG, evaluateChannelProfile(targets.targetLateralG, states[i].distance),
                2.0e-12, "actual lateral agrees with target");
            near(loads.states[i].longitudinalG, 0.0, 2.0e-12, "no longitudinal specific force");
        }
    }

    void flatAndSignedLateral()
    {
        auto track = forceTrack();
        auto states = solve(track);
        for (const auto& state : states)
        {
            near(state.position, {state.distance, 0, 0}, 2.0e-12, "flat +1G straight geometry");
            near(state.centerlineCurvature, {0, 0, 0}, 1.0e-15, "zero curvature");
        }
        checkTargets(track, states);
        for (const double lateral : {-0.7, 0.7})
        {
            force(track).targetLateralG = channel(30, lateral, lateral);
            states = solve(track);
            const double k = standardGravityAcceleration * lateral / 400.0;
            for (const auto& state : states)
            {
                const double angle = k * state.distance;
                near(state.position, {std::sin(angle) / k, (1.0 - std::cos(angle)) / k, 0},
                    1.0e-8, "independent circular position including lateral sign");
                near(state.frame.tangent, {std::cos(angle), std::sin(angle), 0},
                    1.0e-9, "independent circular tangent");
                near(glm::length(state.centerlineCurvature), std::abs(k), 1.0e-14, "physical curvature");
            }
            checkFrames(states);
            checkTargets(track, states);
        }
    }

    void entryFrameAndRoll()
    {
        auto track = forceTrack(12.0);
        const double bank = 0.6;
        track.setStartPose({{3, 4, 5}, glm::angleAxis(bank, glm::dvec3{1, 0, 0})});
        force(track).targetNormalG = channel(12, 1.3, 1.3);
        force(track).targetLateralG = channel(12, -0.2, -0.2);
        auto states = solve(track);
        const auto& entry = states.front();
        const double expectedYaw = standardGravityAcceleration * (-0.2 - std::sin(bank)) / 400.0;
        const double expectedPitch = -standardGravityAcceleration * (1.3 - std::cos(bank)) / 400.0;
        near(glm::dot(entry.centerlineCurvature, entry.frame.lateral), expectedYaw,
            1.0e-15, "bank projects gravity into lateral with correct sign");
        near(glm::dot(entry.centerlineCurvature, entry.frame.up), -expectedPitch,
            1.0e-15, "bank projects gravity into normal with correct sign");
        checkTargets(track, states);

        // With no authored roll, frame derivatives must have no rotation
        // about T even though force targets bend the track.
        const auto fine = solve(track, 0.001);
        const auto dL = (fine[2].frame.lateral - fine[0].frame.lateral) / 0.002;
        near(glm::dot(dL, fine[1].frame.up), 0.0, 1.0e-9, "no automatic banking");

        force(track).rollRate = channel(12, 0.06, 0.06);
        states = solve(track, 0.002);
        for (std::size_t i = 1; i + 1 < states.size(); ++i)
        {
            const double h = states[i + 1].distance - states[i - 1].distance;
            const auto lateralDerivative = (states[i + 1].frame.lateral - states[i - 1].frame.lateral) / h;
            near(glm::dot(lateralDerivative, states[i].frame.up), 0.06, 2.0e-8,
                "authored roll independently recovered from frame derivative");
        }
        checkFrames(states);
        checkTargets(track, states);
    }

    AuthoredTrack nonlinearTrack()
    {
        auto track = forceTrack(36.0);
        force(track).targetNormalG = channel(36, 1.5, 0.7, TransitionType::Smootherstep);
        force(track).targetLateralG = channel(36, -0.35, 0.5, TransitionType::CosineEaseInOut);
        force(track).rollRate = channel(36, 0.03, -0.04, TransitionType::CubicEaseInOut);
        return track;
    }

    void independentKinematicsAndConvergence()
    {
        const auto track = nonlinearTrack();
        const auto states = solve(track, 0.01);
        for (std::size_t i = 1; i + 1 < states.size(); ++i)
        {
            const double h = states[i + 1].distance - states[i - 1].distance;
            near((states[i + 1].position - states[i - 1].position) / h,
                states[i].frame.tangent, 1.0e-7, "independent P'=T");
            near((states[i + 1].frame.tangent - states[i - 1].frame.tangent) / h,
                states[i].centerlineCurvature, 2.0e-8, "independent T'=curvature");
            const auto& frame = states[i].frame;
            const double y = glm::dot(states[i].centerlineCurvature, frame.lateral);
            const double p = -glm::dot(states[i].centerlineCurvature, frame.up);
            const double r = evaluateChannelProfile(force(track).rollRate, states[i].distance);
            near((states[i + 1].frame.lateral - states[i - 1].frame.lateral) / h,
                -y * frame.tangent + r * frame.up, 2.0e-8, "independent L'=-yT+rU");
            near((states[i + 1].frame.up - states[i - 1].frame.up) / h,
                p * frame.tangent - r * frame.lateral, 2.0e-8, "independent U'=pT-rL");
        }
        checkTargets(track, states);
        const auto reference = solve(track, 36.0, 1.0e-12).back();
        const auto loose = solve(track, 36.0, 1.0e-5).back();
        const auto tight = solve(track, 36.0, 1.0e-9).back();
        const double loosePosition = glm::length(loose.position - reference.position);
        const double tightPosition = glm::length(tight.position - reference.position);
        const double looseFrame = glm::length(loose.frame.up - reference.frame.up);
        const double tightFrame = glm::length(tight.frame.up - reference.frame.up);
        require(tightPosition < loosePosition / 20.0 && tightFrame < looseFrame / 20.0,
            "position and frame converge independently under refinement");
        near(tightPosition, 0.0, 2.0e-7, "converged endpoint");
        near(tightFrame, 0.0, 2.0e-8, "converged frame");
        const auto again = solve(track, 0.01);
        require(again.size() == states.size(), "deterministic output count");
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            require(again[i].distance == states[i].distance && again[i].position == states[i].position
                && again[i].frame.tangent == states[i].frame.tangent
                && again[i].frame.lateral == states[i].frame.lateral && again[i].frame.up == states[i].frame.up
                && again[i].centerlineCurvature == states[i].centerlineCurvature, "bitwise deterministic fields");
        }
    }

    void nonlinearStaggeredBreakpoints()
    {
        auto track = nonlinearTrack();
        // Insert mathematically inert breakpoints only into the constant
        // lateral channel, leaving the nonlinear normal and roll intact.
        force(track).targetLateralG = channel(36, 0.2, 0.2);
        const auto baseline = solve(track, 0.5);
        force(track).targetLateralG = {{{3, {0, 5.3, 0.2, 0.2, TransitionType::Linear}},
            {8, {5.3, 23.7, 0.2, 0.2, TransitionType::SineEaseIn}},
            {12, {23.7, 36, 0.2, 0.2, TransitionType::Smootherstep}}}, 25};
        const auto split = solve(track, 0.5);
        checkTargets(track, split);
        for (const auto& before : baseline)
        {
            const auto found = std::find_if(split.begin(), split.end(), [&](const auto& state)
                { return state.distance == before.distance; });
            require(found != split.end(), "original output coordinate retained");
            near(found->position, before.position, 3.0e-9, "other channel breakpoints do not re-ease normal");
            near(found->frame.up, before.frame.up, 3.0e-10, "other channel breakpoints do not re-ease roll");
        }
    }

    void scaleAndStartPose()
    {
        auto track = nonlinearTrack();
        const auto baseline = solve(track, 0.5);
        auto scaled = track;
        auto settings = scaled.physicalSettings();
        settings.metersPerCoordinateUnit = 0.2;
        scaled.setPhysicalSettings(settings);
        setSectionLength(scaled.section(0), 180.0);
        for (auto& segment : force(scaled).rollRate.segments)
        {
            segment.transition.valueBegin *= 0.2;
            segment.transition.valueEnd *= 0.2;
        }
        const auto scaledStates = solve(scaled, 2.5);
        require(scaledStates.size() == baseline.size(), "physical scale sample correspondence");
        for (std::size_t i = 0; i < baseline.size(); ++i)
        {
            near(0.2 * scaledStates[i].position, baseline[i].position, 1.0e-9, "equivalent physical position");
            near(scaledStates[i].frame.up, baseline[i].frame.up, 1.0e-10, "equivalent frame");
            near(scaledStates[i].centerlineCurvature / 0.2, baseline[i].centerlineCurvature,
                1.0e-10, "equivalent physical curvature");
        }
        checkTargets(scaled, scaledStates);
        auto translated = track;
        const glm::dvec3 offset{25, -40, 70};
        translated.setStartPose({offset, {1, 0, 0, 0}});
        const auto translatedStates = solve(translated, 0.5);
        for (std::size_t i = 0; i < baseline.size(); ++i)
            near(translatedStates[i].position - offset, baseline[i].position, 1.0e-10, "translation preserves energy geometry");

        auto rotated = forceTrack();
        const auto upright = solve(rotated);
        const auto q = glm::angleAxis(0.4, glm::dvec3{0, 1, 0});
        rotated.setStartPose({{}, q});
        const auto tilted = solve(rotated);
        near(tilted.front().frame.tangent, q * glm::dvec3{1, 0, 0}, 1.0e-14, "authored entry pose used");
        require(glm::length(tilted.back().position - q * upright.back().position) > 0.1,
            "rotation relative to gravity regenerates geometry, not a rigid transform");
        checkTargets(rotated, tilted);
    }

    void mixedBoundaryAndGlobalEnergy()
    {
        auto track = createNewDocument();
        setSectionLength(track.section(0), 12);
        track.section(0).rateProfileRegion().rateProfiles.pitch = channel(12, 0.03, 0.03);
        track.insertSectionAfter(0, createForceDrivenSection(15));
        force(track, 1).targetNormalG = channel(15, 1.2, 0.9, TransitionType::Smoothstep);
        force(track, 1).targetLateralG = channel(15, -0.2, 0.2);
        track.appendSection();
        convertSectionToPlanarArc(track.section(2));
        setSectionLength(track.section(2), 10);
        const auto states = solve(track, 0.1);
        const auto loads = evaluateRiderLoads(states, riderLoadEvaluationSettings(track.physicalSettings()));
        require(loads.completed(), "mixed loads complete");
        for (const double boundary : {12.0, 27.0})
        {
            require(std::count_if(states.begin(), states.end(), [&](const auto& state)
                { return state.distance == boundary; }) == 1, "one shared boundary sample");
        }
        const auto entryIt = std::find_if(states.begin(), states.end(), [](const auto& state)
            { return state.distance == 12.0; });
        const std::size_t entryIndex = static_cast<std::size_t>(entryIt - states.begin());
        const auto& entry = *entryIt;
        const double entrySpeedSquared = 400.0 - 2.0 * standardGravityAcceleration * entry.position.z;
        require(entrySpeedSquared > 400.0, "descending legacy lead-in increases speed");
        near(loads.states[entryIndex].vehicleSpeed, std::sqrt(entrySpeedSquared), 1.0e-12, "global energy at later force entry");
        near(loads.states[entryIndex].normalG, 1.2, 1.0e-12, "right-owned force entry load");
        near(loads.states[entryIndex].lateralG, -0.2, 1.0e-12, "right-owned force lateral load");
        const auto direct = integrateForceDrivenRegion(entry.position, entry.frame, force(track, 1), 15,
            track.physicalSettings(), track.startPose().position, 0.1);
        const auto arcEntry = std::find_if(states.begin(), states.end(), [](const auto& state)
            { return state.distance == 27.0; });
        near(arcEntry->position, direct.back().position, 1.0e-12, "continuous force-to-arc position");
        near(arcEntry->frame.up, direct.back().frame.up, 1.0e-12, "continuous force-to-arc frame");
        near(glm::dot(arcEntry->centerlineCurvature, arcEntry->frame.lateral), 1.0 / 25.0,
            1.0e-12, "following arc owns boundary curvature");
        const auto standaloneLoads = evaluateRiderLoads(direct, {
            std::sqrt(entrySpeedSquared), 1.0, standardGravityAcceleration});
        const auto arcIndex = static_cast<std::size_t>(arcEntry - states.begin());
        near(loads.states[arcIndex].vehicleSpeed, standaloneLoads.states.back().vehicleSpeed,
            1.0e-12, "speed continuous at force exit");
        checkFrames(states);
        // Exercise all six construction orderings with the same data.
        for (const auto& order : {std::array{0, 1, 2}, std::array{0, 2, 1}, std::array{1, 0, 2},
            std::array{1, 2, 0}, std::array{2, 0, 1}, std::array{2, 1, 0}})
        {
            auto permutation = createNewDocument();
            permutation.section(0) = track.section(order[0]);
            permutation.insertSectionAfter(0, track.section(order[1]));
            permutation.insertSectionAfter(1, track.section(order[2]));
            checkFrames(solve(permutation, 0.3));
        }
    }

    void failuresAndValidation()
    {
        auto track = forceTrack();
        track.setPhysicalSettings({0.0, 1.0, standardGravityAcceleration});
        auto result = generateAuthoredTrackKinematics(track, 30);
        require(!result && result.error().reason == TrackGenerationFailureReason::InsufficientSpeed,
            "zero speed explicitly fails, including a 0/0 force numerator");
        require(result.error().sectionIndex == 0 && result.error().localDistance == 0.0
            && result.error().cumulativeDistance == 0.0 && result.error().speedSquared == 0.0,
            "failure includes section, distances, and energy");
        track.setPhysicalSettings({1.0e-9, 1, standardGravityAcceleration});
        result = generateAuthoredTrackKinematics(track, 30);
        require(!result && result.error().reason == TrackGenerationFailureReason::InsufficientSpeed,
            "positive unresolved speed fails without a physical minimum-speed clamp");
        track.setPhysicalSettings({1.0e-6, 1, standardGravityAcceleration});
        require(generateAuthoredTrackKinematics(track, 30).has_value(),
            "resolved positive speed is accepted without an arbitrary minimum speed");

        track = forceTrack();
        force(track).targetNormalG = channel(30, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        result = generateAuthoredTrackKinematics(track, 30);
        require(!result && result.error().reason == TrackGenerationFailureReason::NonfiniteDerivedRates,
            "nonfinite derived rates distinct from speed failure");

        track = forceTrack(1.0e308);
        track.setStartPose({{std::numeric_limits<double>::max(), 0, 0}, {1, 0, 0, 0}});
        result = generateAuthoredTrackKinematics(track, 1.0e308);
        require(!result && result.error().reason == TrackGenerationFailureReason::IntegrationFailure,
            "nonfinite world position must never escape as canonical output");

        track = nonlinearTrack();
        result = generateAuthoredTrackKinematics(track, 36, {1.0e-18, 0});
        require(!result && result.error().reason == TrackGenerationFailureReason::IntegrationFailure,
            "exhausted refinement is explicit");
        result = generateAuthoredTrackKinematics(track, 0);
        require(!result && result.error().reason == TrackGenerationFailureReason::InvalidInput,
            "invalid spacing distinguished");
        result = generateAuthoredTrackKinematics(track, 1, {-1, 24});
        require(!result && result.error().reason == TrackGenerationFailureReason::InvalidInput,
            "invalid integrator controls distinguished");
        force(track).targetNormalG.segments[0].transition.domainEnd = 20;
        result = generateAuthoredTrackKinematics(track, 1);
        require(!result && result.error().reason == TrackGenerationFailureReason::InvalidInput
            && result.error().sectionIndex == 0, "invalid force profile domain");
        for (const auto& physical : {TrackPhysicalSettings{-1, 1, 9.8}, {20, 0, 9.8},
            {20, 1, 0}, {std::numeric_limits<double>::infinity(), 1, 9.8},
            {20, std::numeric_limits<double>::quiet_NaN(), 9.8}})
        {
            const auto before = track.physicalSettings();
            bool threw = false;
            try { track.setPhysicalSettings(physical); }
            catch (const std::invalid_argument&) { threw = true; }
            require(threw && track.physicalSettings() == before, "invalid physical settings never replace document state");
        }
    }

    void interiorEnergyBarrier()
    {
        // A vertical circular path has a lower final endpoint but an
        // unreachable summit. The analytic signed normal target that would
        // produce this circle is n(s)=v0^2/(gR)-2+3cos(s/R). A cosine-eased
        // half circle represents it exactly, even where energy changes sign.
        const double radius = 10.0;
        const double half = pi * radius;
        const double length = 2.0 * half;
        auto track = forceTrack(length);
        track.setPhysicalSettings({std::sqrt(3.0 * standardGravityAcceleration * radius), 1, standardGravityAcceleration});
        force(track).targetNormalG = {{{1, {0, half, 4, -2, TransitionType::CosineEaseInOut}},
            {2, {half, length, -2, 4, TransitionType::CosineEaseInOut}}}, 3};
        const double barrier = radius * std::acos(-0.5);
        const auto result = generateAuthoredTrackKinematics(track, length);
        require(!result, "internal stages detect barrier despite a reachable circle endpoint");
        // The circle ODE becomes singular at the barrier. Numerical error
        // control may exhaust refinement just before a negative stage; that
        // is a valid failure, but its location must still resolve the barrier.
        require(result.error().reason == TrackGenerationFailureReason::EnergeticallyUnreachable
            || result.error().reason == TrackGenerationFailureReason::InsufficientSpeed
            || result.error().reason == TrackGenerationFailureReason::IntegrationFailure,
            "interior barrier explicitly stops generation");
        require(result.error().localDistance && *result.error().localDistance > 0
            && *result.error().localDistance < half, "failure before the summit, not just at endpoints");
        near(*result.error().localDistance, barrier, 0.01, "failure locates the actual energy barrier");
        require(result.error().speedSquared && std::isfinite(*result.error().speedSquared),
            "barrier failure carries the available speed squared");

        auto vertical = forceTrack(40);
        vertical.setStartPose({{}, glm::angleAxis(-pi / 2.0, glm::dvec3{0, 1, 0})});
        force(vertical).targetNormalG = channel(40, 0, 0);
        const auto crossing = generateAuthoredTrackKinematics(vertical, 40);
        require(!crossing, "straight uphill generation cannot continue past the barrier");
        near(*crossing.error().localDistance, 400.0 / (2.0 * standardGravityAcceleration), 0.001,
            "straight uphill internal barrier location");
        setSectionLength(vertical.section(0), 100);
        const auto negativeStage = generateAuthoredTrackKinematics(vertical, 100, {1.0e-10, 0});
        require(!negativeStage && negativeStage.error().reason == TrackGenerationFailureReason::EnergeticallyUnreachable
            && negativeStage.error().speedSquared < 0.0 && negativeStage.error().localDistance < 100.0,
            "a negative internal stage reports unreachable when refinement is exhausted");

        // Later force entry still uses the whole-track reference.
        auto later = createNewDocument();
        setSectionLength(later.section(0), 30);
        later.setStartPose({{}, glm::angleAxis(-pi / 2.0, glm::dvec3{0, 1, 0})});
        later.insertSectionAfter(0, createForceDrivenSection(1));
        const auto unreachable = generateAuthoredTrackKinematics(later, 1);
        require(!unreachable && unreachable.error().reason == TrackGenerationFailureReason::EnergeticallyUnreachable
            && unreachable.error().sectionIndex == 1 && unreachable.error().cumulativeDistance == 30,
            "later region reports globally unreachable entry");
    }

    void serializationAndLength()
    {
        auto track = nonlinearTrack();
        track.setPhysicalSettings({31, 0.4, 8.5});
        track.setStartPose({{2, -7, 18}, glm::angleAxis(0.2, glm::dvec3{0, 0, 1})});
        force(track).targetLateralG = {{{4, {0, 9, -0.2, 0.1, TransitionType::CubicEaseIn}},
            {9, {9, 36, 0.1, 0.6, TransitionType::Smootherstep}}}, 28};
        const auto original = force(track);
        setSectionLength(track.section(0), 72);
        const auto& resized = force(track);
        for (const auto pair : {std::pair{&original.targetNormalG, &resized.targetNormalG},
            std::pair{&original.targetLateralG, &resized.targetLateralG}, std::pair{&original.rollRate, &resized.rollRate}})
        {
            require(pair.first->nextSegmentId == pair.second->nextSegmentId, "rescale retains ID allocator");
            for (std::size_t i = 0; i < pair.first->segments.size(); ++i)
            {
                const auto& before = pair.first->segments[i];
                const auto& after = pair.second->segments[i];
                require(before.id == after.id && before.transition.transitionType == after.transition.transitionType
                    && before.transition.valueBegin == after.transition.valueBegin && before.transition.valueEnd == after.transition.valueEnd,
                    "rescale retains values, easing, IDs, and order");
                near(after.transition.domainBegin, 2 * before.transition.domainBegin, 0, "rescaled begin");
                near(after.transition.domainEnd, 2 * before.transition.domainEnd, 0, "rescaled end");
            }
        }
        track.prependSection();
        track.appendSection();
        convertSectionToPlanarArc(track.section(2));
        const std::string serialized = serializeCoasterDocument(track);
        auto loaded = deserializeCoasterDocument(serialized);
        require(loaded.has_value(), "force document loads");
        require(serializeCoasterDocument(*loaded) == serialized, "deterministic exact serialization including IDs and ordering");
        require(loaded->physicalSettings() == track.physicalSettings() && loaded->startPose() == track.startPose(),
            "physical settings and start pose roundtrip");
        require(isForceDrivenSection(loaded->section(1)), "construction position retained");
        const auto root = nlohmann::json::parse(serialized);
        require(root["formatVersion"] == 1 && root["sections"][1]["forceDriven"].size() == 3,
            "additive v1 with only authored force channels");
        for (int defect = 0; defect < 11; ++defect)
        {
            auto broken = root;
            auto& section = broken["sections"][1];
            switch (defect)
            {
            case 0: section["planarArc"] = root["sections"][2]["planarArc"]; break;
            case 1: section.erase("forceDriven"); break;
            case 2: section["forceDriven"].erase("rollRate"); break;
            case 3: section["forceDriven"]["speedSamples"] = nlohmann::json::array(); break;
            case 4: broken["physicalSettings"]["metersPerCoordinateUnit"] = -1; break;
            case 5: broken["physicalSettings"].erase("initialSpeed"); break;
            case 6: section["forceDriven"]["targetLateralG"]["nextSegmentId"] = 9; break;
            case 7: section["forceDriven"]["targetNormalG"] = nullptr; break;
            case 8: section["forceDriven"]["targetNormalG"]["segments"][0]["id"] = -1; break;
            case 9: section["forceDriven"]["targetNormalG"]["nextSegmentId"] = -1; break;
            case 10: section["forceDriven"]["targetNormalG"]["nextSegmentId"] = 4294967315ULL; break;
            }
            require(!deserializeCoasterDocument(broken.dump()), "malformed/ambiguous force payload rejected " + std::to_string(defect));
        }
        auto legacy = nlohmann::json::parse(serializeCoasterDocument(createNewDocument()));
        legacy.erase("physicalSettings");
        legacy.erase("startPose");
        loaded = deserializeCoasterDocument(legacy.dump());
        require(loaded && loaded->physicalSettings() == TrackPhysicalSettings{}, "legacy physical defaults");
        const auto geometry = integrateAuthoredTrack(*loaded, 1);
        near(geometry.back().position, {60, 0, 0}, 0, "legacy geometry unchanged");
        auto converted = createForceDrivenSection(23);
        convertSectionToPlanarArc(converted);
        require(!isForceDrivenSection(converted) && sectionLength(converted) == 23, "force to planar conversion is not a no-op");
        converted = createForceDrivenSection(23);
        bool threw = false;
        try { convertSectionToRateProfiles(converted); }
        catch (const std::invalid_argument&) { threw = true; }
        require(threw && isForceDrivenSection(converted), "unsupported inverse conversion rejected without mutation");

        auto tiny = createForceDrivenSection(1.0e-300);
        setSectionLength(tiny, 1.0e300);
        near(sectionLength(tiny), 1.0e300, 0.0, "finite force length edits avoid intermediate ratio overflow");
    }
}

int main()
{
    const std::pair<const char*, void(*)()> tests[]{
        {"flat and signed lateral", flatAndSignedLateral},
        {"entry frame and authored roll", entryFrameAndRoll},
        {"independent kinematics, convergence, determinism", independentKinematicsAndConvergence},
        {"nonlinear staggered breakpoints", nonlinearStaggeredBreakpoints},
        {"physical scale and start pose", scaleAndStartPose},
        {"mixed boundaries and global energy", mixedBoundaryAndGlobalEnergy},
        {"failures and validation", failuresAndValidation},
        {"interior energy barrier", interiorEnergyBarrier},
        {"serialization and length", serializationAndLength}};
    int failed = 0;
    for (const auto& [name, test] : tests)
    {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failed; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    return failed == 0 ? 0 : 1;
}
