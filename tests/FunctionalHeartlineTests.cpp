#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/RiderLoads.hpp>
#include <quantum/coaster/TrackGeneration.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    template <typename Exception, typename Callable>
    void requireThrows(Callable&& callable, const std::string& message)
    {
        try
        {
            callable();
        }
        catch (const Exception&)
        {
            return;
        }
        throw std::runtime_error(message);
    }

    void near(const double actual, const double expected,
        const double tolerance, const std::string& message)
    {
        require(std::isfinite(actual)
                && std::abs(actual - expected) <= tolerance,
            message + " (actual " + std::to_string(actual)
                + ", expected " + std::to_string(expected) + ")");
    }

    void near(const glm::dvec3& actual, const glm::dvec3& expected,
        const double tolerance, const std::string& message)
    {
        near(glm::length(actual - expected), 0.0, tolerance, message);
    }

    ChannelProfile channel(const double length, const double begin,
        const double end)
    {
        return {{{1, {0.0, length, begin, end,
            quantum::math::TransitionType::Linear}}}, 2};
    }

    void setHeartline(AuthoredTrack& track, const double offsetMeters)
    {
        CoasterSetup setup = track.coasterSetup();
        setup.heartline.enabled = true;
        setup.heartline.offsetMeters = offsetMeters;
        track.setCoasterSetup(setup);
    }

    void zeroOffsetPreservesConstructionAndLoads()
    {
        AuthoredTrack disabled = createNewDocument();
        setSectionLength(disabled.section(0), 20.0);
        auto& rates = disabled.section(0).rateProfileRegion().rateProfiles;
        rates.roll = channel(20.0, 0.01, 0.04);
        rates.pitch = channel(20.0, -0.03, 0.02);
        rates.yaw = channel(20.0, 0.015, -0.01);
        CoasterSetup disabledSetup = disabled.coasterSetup();
        disabledSetup.heartline.enabled = false;
        disabled.setCoasterSetup(disabledSetup);

        AuthoredTrack zero = disabled;
        setHeartline(zero, 0.0);
        const auto disabledStates = integrateAuthoredTrackKinematics(
            disabled, 0.25);
        const auto zeroStates = integrateAuthoredTrackKinematics(zero, 0.25);
        require(disabledStates.size() == zeroStates.size(),
            "zero offset retains the generated sample grid");
        for (std::size_t index = 0; index < zeroStates.size(); ++index)
        {
            near(zeroStates[index].position,
                disabledStates[index].position, 0.0,
                "zero offset retains construction positions exactly");
            near(zeroStates[index].centerlineCurvature,
                disabledStates[index].centerlineCurvature, 0.0,
                "zero offset retains construction curvature exactly");
        }
        const auto disabledLoads = evaluateRiderLoads(disabledStates,
            riderLoadEvaluationSettings(
                disabled.physicalSettings(), disabled.coasterSetup().heartline));
        const auto zeroLoads = evaluateRiderLoads(zeroStates,
            riderLoadEvaluationSettings(
                zero.physicalSettings(), zero.coasterSetup().heartline));
        require(disabledLoads.states == zeroLoads.states,
            "enabled zero offset retains legacy load results exactly");
    }

    void straightRollMovesAndAcceleratesRiderReference()
    {
        AuthoredTrack track = createNewDocument();
        setSectionLength(track.section(0), 20.0);
        constexpr double rollRate = 0.1;
        track.section(0).rateProfileRegion().rateProfiles.roll =
            channel(20.0, rollRate, rollRate);
        setHeartline(track, 1.4);

        const auto states = integrateAuthoredTrackKinematics(track, 0.25);
        const auto loads = evaluateRiderLoads(states,
            riderLoadEvaluationSettings(
                track.physicalSettings(), track.coasterSetup().heartline));
        require(loads.completed() && loads.states.size() == states.size(),
            "straight roll rider-reference loads complete");
        near(loads.states.front().riderReferencePosition,
            {0.0, 0.0, 1.4}, 1.0e-14,
            "heartline begins at C+hU");
        near(loads.states.back().riderReferencePosition,
            riderReferencePosition(states.back(), 1.4), 1.0e-14,
            "heartline follows the rolled local up axis");
        require(std::abs(loads.states.back().riderReferencePosition.y) > 1.0,
            "roll moves an offset rider laterally while construction stays straight");

        const double expectedNormal = 1.0
            - track.physicalSettings().initialSpeed
                * track.physicalSettings().initialSpeed
                * 1.4 * rollRate * rollRate
                / standardGravityAcceleration;
        near(loads.states.front().normalG, expectedNormal, 1.0e-12,
            "roll-axis offset contributes centripetal rider load");
    }

    void bankedArcUsesConstructionAndRiderReferences()
    {
        AuthoredTrack track = createNewDocument();
        convertSectionToPlanarArc(track.section(0));
        setPlanarArcRadius(track.section(0), 25.0);
        setPlanarArcSweptAngle(
            track.section(0), std::numbers::pi_v<double> / 2.0);
        setPlanarArcBankChange(
            track.section(0), std::numbers::pi_v<double> / 2.0);
        setHeartline(track, 1.4);

        const auto states = integrateAuthoredTrackKinematics(track, 0.2);
        const auto offsetLoads = evaluateRiderLoads(states,
            riderLoadEvaluationSettings(
                track.physicalSettings(), track.coasterSetup().heartline));
        const auto constructionLoads = evaluateRiderLoads(states,
            riderLoadEvaluationSettings(track.physicalSettings()));
        near(offsetLoads.states.back().riderReferencePosition,
            riderReferencePosition(states.back(), 1.4), 1.0e-12,
            "banked arc rider reference follows the banked up axis");
        near(states.back().position.z, 0.0, 1.0e-11,
            "bank does not move the planar construction reference");
        require(std::abs(offsetLoads.states.back().normalG
                - constructionLoads.states.back().normalG) > 1.0e-3,
            "bank motion changes loads at the authored rider reference");
    }

    void forceTargetsApplyAtOffsetRiderReference()
    {
        AuthoredTrack track = createNewDocument();
        track.section(0) = createForceDrivenSection(20.0);
        setHeartline(track, 1.4);
        auto& force = std::get<ForceDrivenRegion>(
            std::get<GeometryRegion>(track.section(0).region).construction);
        force.targetNormalG = channel(20.0, 1.15, 1.35);
        force.targetLateralG = channel(20.0, -0.1, 0.2);
        force.rollRate = channel(20.0, 0.015, 0.04);

        const auto states = integrateAuthoredTrackKinematics(track, 0.1);
        const auto loads = evaluateRiderLoads(states,
            riderLoadEvaluationSettings(
                track.physicalSettings(), track.coasterSetup().heartline));
        require(loads.completed() && loads.states.size() == states.size(),
            "force-based heartline solve completes");
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            near(loads.states[index].normalG,
                evaluateChannelProfile(
                    force.targetNormalG, states[index].distance),
                5.0e-11, "normal target is enforced at rider reference");
            near(loads.states[index].lateralG,
                evaluateChannelProfile(
                    force.targetLateralG, states[index].distance),
                5.0e-11, "lateral target is enforced at rider reference");
        }

        AuthoredTrack zero = track;
        setHeartline(zero, 0.0);
        const auto zeroStates = integrateAuthoredTrackKinematics(zero, 0.1);
        require(glm::length(states.back().position - zeroStates.back().position)
                > 1.0e-3,
            "functional offset participates in force-generated construction geometry");
    }

    void singularOffsetIsRejectedRatherThanClamped()
    {
        // A fully inverted frame on a slow force-driven region asks for more
        // vertical load than a rider 1.4 m from the roll axis can supply, so
        // the offset-aware pitch solve has no real root. The combination must
        // be reported as a generation failure, never silently clamped to a
        // weaker normal G, and the construction reference must still generate
        // at zero offset.
        AuthoredTrack track = createNewDocument();
        track.section(0) = createForceDrivenSection(6.0);
        track.setStartPose({{0.0, 0.0, 0.0}, glm::angleAxis(
            std::numbers::pi_v<double>, glm::dvec3{1.0, 0.0, 0.0})});
        track.setPhysicalSettings(
            {10.0, 1.0, standardGravityAcceleration});

        setHeartline(track, 0.0);
        const auto construction =
            integrateAuthoredTrackKinematics(track, 0.1);
        require(!construction.empty()
                && construction.front().frame.up.z < -0.99,
            "zero offset generates the inverted force-driven region");

        setHeartline(track, 1.4);
        requireThrows<TrackGenerationError>(
            [&track]
            {
                static_cast<void>(
                    integrateAuthoredTrackKinematics(track, 0.1));
            },
            "a singular rider-reference offset fails track generation");
    }

    void mixedRegionsAndInversionRemainContinuous()
    {
        AuthoredTrack track = createNewDocument();
        setSectionLength(track.section(0), 20.0);
        auto& profile = track.section(0).rateProfileRegion().rateProfiles;
        profile.yaw = channel(20.0, 0.02, 0.02);
        profile.roll = channel(20.0, 0.0, 0.03);

        AuthoredTrackSection arc = createRateProfileSection(10.0);
        convertSectionToPlanarArc(arc);
        setPlanarArcRadius(arc, 20.0);
        setPlanarArcSweptAngle(arc, 0.5);
        setPlanarArcPlaneTilt(arc, 0.0);
        setPlanarArcBankChange(arc, 0.2);
        track.insertSectionAfter(0, arc);

        AuthoredTrackSection forceSection = createForceDrivenSection(10.0);
        track.insertSectionAfter(1, forceSection);
        auto& force = std::get<ForceDrivenRegion>(
            std::get<GeometryRegion>(track.section(2).region).construction);
        force.targetNormalG = channel(10.0, 1.0, 1.0);
        force.targetLateralG = channel(10.0, 0.0, 0.0);
        force.rollRate = channel(10.0, 0.0, 0.0);
        setHeartline(track, 1.4);

        const auto states = integrateAuthoredTrackKinematics(track, 0.1);
        const auto loads = evaluateRiderLoads(states,
            riderLoadEvaluationSettings(
                track.physicalSettings(), track.coasterSetup().heartline));
        require(loads.completed() && loads.states.size() == states.size(),
            "mixed Profile/Arc/Force rider solve completes");
        for (const double boundary : {20.0, 30.0})
        {
            std::size_t matches = 0;
            for (const auto& state : states)
            {
                if (std::abs(state.distance - boundary) < 1.0e-12)
                    ++matches;
            }
            require(matches == 1,
                "mixed-region boundary has one shared construction/rider state");
        }
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            near(loads.states[index].riderReferencePosition,
                riderReferencePosition(states[index], 1.4), 1.0e-12,
                "all region kinds use the same rider-reference convention");
        }

        AuthoredTrack inversion = createNewDocument();
        const double radius = 12.0;
        const double length = 2.0 * std::numbers::pi_v<double> * radius;
        setSectionLength(inversion.section(0), length);
        inversion.section(0).rateProfileRegion().rateProfiles.pitch =
            channel(length, -1.0 / radius, -1.0 / radius);
        setHeartline(inversion, 1.4);
        const auto inversionStates = integrateAuthoredTrackKinematics(
            inversion, 0.2);
        const auto top = inversionStates[inversionStates.size() / 2];
        require(top.frame.up.z < -0.99,
            "profile fixture reaches an inverted rider frame");
        near(riderReferencePosition(top, 1.4),
            top.position + 1.4 * top.frame.up, 1.0e-14,
            "inversion keeps the signed local-up heartline convention");
    }
}

int main()
{
    const auto run = [](const char* name, void (*test)())
    {
        try
        {
            test();
            std::cout << "  PASS: " << name << '\n';
            return true;
        }
        catch (const std::exception& error)
        {
            std::cerr << "  FAIL: " << name << ": "
                << error.what() << '\n';
            return false;
        }
    };
    const bool passed =
        run("zero offset", zeroOffsetPreservesConstructionAndLoads)
        && run("straight roll", straightRollMovesAndAcceleratesRiderReference)
        && run("banked arc", bankedArcUsesConstructionAndRiderReferences)
        && run("force targets", forceTargetsApplyAtOffsetRiderReference)
        && run("singular offset", singularOffsetIsRejectedRatherThanClamped)
        && run("mixed regions and inversion", mixedRegionsAndInversionRemainContinuous);
    if (!passed) return 1;
    std::cout << "FunctionalHeartlineTests passed\n";
    return 0;
}
