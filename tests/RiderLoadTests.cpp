#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/RiderLoads.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::GeometryRegion;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::RegionKind;
    using quantum::coaster::RiderLoadEvaluationSettings;
    using quantum::coaster::RiderLoadHistory;
    using quantum::coaster::RiderLoadState;
    using quantum::coaster::TrackKinematicState;
    using quantum::coaster::evaluateRiderLoads;
    using quantum::coaster::integrateAuthoredTrack;
    using quantum::coaster::integrateAuthoredTrackKinematics;
    using quantum::coaster::planarArcLength;
    using quantum::coaster::setSectionLength;
    using quantum::coaster::standardGravityAcceleration;
    using quantum::geometry::CurveFrame;

    constexpr double pi = 3.14159265358979323846;

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
        if (std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(message) + ": expected "
                + std::to_string(expected) + ", got "
                + std::to_string(actual)
            );
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
            throw TestFailure(std::string(message));
        }
    }

    template<typename ExpectedException, typename Function>
    void requireThrows(Function&& function, const std::string_view message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const ExpectedException&)
        {
            return;
        }

        throw TestFailure(std::string(message));
    }

    [[nodiscard]] constexpr CurveFrame identityFrame()
    {
        return {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
    }

    void setConstantRate(
        quantum::coaster::ChannelProfile& channel,
        const double value)
    {
        require(channel.segments.size() == 1,
            "default test channel has one segment");
        channel.segments.front().transition.valueBegin = value;
        channel.segments.front().transition.valueEnd = value;
    }

    void setConstantRates(
        AuthoredTrackSection& section,
        const double pitch,
        const double yaw,
        const double roll = 0.0)
    {
        auto& profiles = section.rateProfileRegion().rateProfiles;
        setConstantRate(profiles.pitch, pitch);
        setConstantRate(profiles.yaw, yaw);
        setConstantRate(profiles.roll, roll);
    }

    void setPlanarArc(
        AuthoredTrackSection& section,
        const PlanarArcRegion& arc)
    {
        section.kind = RegionKind::Geometry;
        section.region = GeometryRegion{arc};
        section.length = planarArcLength(arc);
    }

    [[nodiscard]] std::size_t findDistance(
        const std::vector<TrackKinematicState>& states,
        const double distance)
    {
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            if (states[index].distance == distance)
            {
                return index;
            }
        }

        throw TestFailure("expected exact distance sample is missing");
    }

    void testStationaryUprightReference()
    {
        const std::vector<TrackKinematicState> kinematics{
            {0.0, {0.0, 0.0, 0.0}, identityFrame(), {0.0, 0.0, 0.0}}
        };

        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics,
            RiderLoadEvaluationSettings{}
        );

        require(history.completed(), "stationary reference completes");
        require(history.states.size() == 1,
            "stationary reference returns one state");
        const RiderLoadState& state = history.states.front();
        requireNear(state.vehicleSpeed, 0.0, 0.0,
            "stationary reference speed");
        requireNear(state.normalG, 1.0, 1e-15,
            "stationary reference normal G");
        requireNear(state.lateralG, 0.0, 1e-15,
            "stationary reference lateral G");
        requireNear(state.longitudinalG, 0.0, 1e-15,
            "stationary reference longitudinal G");
    }

    void testFlatStraightGravityOnlyMotion()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 20.0);

        const auto kinematics = integrateAuthoredTrackKinematics(track, 3.0);
        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = 12.0;
        settings.metersPerCoordinateUnit = 2.5;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);

        require(history.completed(), "flat straight completes");
        for (const RiderLoadState& state : history.states)
        {
            requireNear(state.vehicleSpeed, 12.0, 1e-13,
                "flat straight speed stays constant");
            requireNear(state.normalG, 1.0, 1e-13,
                "flat straight normal G");
            requireNear(state.lateralG, 0.0, 1e-13,
                "flat straight lateral G");
            requireNear(state.longitudinalG, 0.0, 1e-13,
                "flat straight longitudinal G");
        }
    }

    void testKnownConstantCurvatureLoad()
    {
        constexpr double radius = 20.0;
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), radius * pi / 4.0);
        setConstantRates(track.section(0), -1.0 / radius, 0.0);

        const auto kinematics = integrateAuthoredTrackKinematics(track, 1.0);
        requireNear(kinematics.front().centerlineCurvature,
            glm::dvec3{0.0, 0.0, 1.0 / radius}, 1e-15,
            "rate/profile dT/ds follows yaw L minus pitch U");

        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = 20.0;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);
        const double expectedNormalG = 1.0
            + settings.initialSpeed * settings.initialSpeed
                / (radius * standardGravityAcceleration);

        requireNear(history.states.front().normalG, expectedNormalG, 1e-13,
            "upward curvature adds positive v squared over r normal load");
        requireNear(history.states.front().lateralG, 0.0, 1e-13,
            "vertical curvature has no lateral load at entry");
    }

    void testBankedFrameProjection()
    {
        constexpr double radius = 25.0;
        constexpr double speed = 15.0;
        AuthoredTrack track;
        track.appendSection();
        setPlanarArc(track.section(0), PlanarArcRegion{
            radius, pi / 2.0, 0.0, pi / 2.0});

        const auto kinematics = integrateAuthoredTrackKinematics(track, 1.0);
        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = speed;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);
        const RiderLoadState& exit = history.states.back();

        requireNear(kinematics.back().centerlineCurvature,
            glm::dvec3{-1.0 / radius, 0.0, 0.0}, 2e-11,
            "banking does not rotate planar-arc centerline curvature");
        requireNear(exit.vehicleSpeed, speed, 1e-10,
            "horizontal banked arc preserves speed");
        requireNear(exit.normalG,
            -speed * speed / (radius * standardGravityAcceleration),
            2e-10,
            "banked frame projects curvature onto negative normal");
        requireNear(exit.lateralG, 1.0, 2e-10,
            "banked frame projects gravity onto positive lateral");
        requireNear(exit.longitudinalG, 0.0, 2e-10,
            "banked arc has no longitudinal specific force");
    }

    void testMixedRegionsShareOneSpeedHistory()
    {
        constexpr double radius = 20.0;
        const double firstLength = radius * pi / 2.0;

        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), firstLength);
        setConstantRates(track.section(0), 1.0 / radius, 0.0);

        track.appendSection();
        setPlanarArc(track.section(1), PlanarArcRegion{
            15.0, pi / 4.0, 0.0, 0.25});

        const auto kinematics = integrateAuthoredTrackKinematics(track, 1.0);
        const std::size_t boundary = findDistance(kinematics, firstLength);

        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = 5.0;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);
        require(history.completed(), "mixed authored history completes");

        const double expectedBoundarySpeed = std::sqrt(
            settings.initialSpeed * settings.initialSpeed
            - 2.0 * settings.gravityAcceleration
                * kinematics[boundary].position.z
                * settings.metersPerCoordinateUnit);
        requireNear(history.states[boundary].vehicleSpeed,
            expectedBoundarySpeed, 1e-11,
            "mixed-region boundary uses whole-track energy");
        require(history.states[boundary].vehicleSpeed
                > settings.initialSpeed + 1.0,
            "speed does not reset at the authored-region boundary");
    }

    void testPhysicalScaleChangesCurvatureLoad()
    {
        constexpr double radius = 10.0;
        constexpr double speed = 10.0;
        AuthoredTrack track;
        track.appendSection();
        setPlanarArc(track.section(0), PlanarArcRegion{
            radius, pi / 4.0, 0.0, 0.0});
        const auto kinematics = integrateAuthoredTrackKinematics(track, 1.0);

        RiderLoadEvaluationSettings unitScale;
        unitScale.initialSpeed = speed;
        const RiderLoadHistory loadsAtUnitScale = evaluateRiderLoads(
            kinematics, unitScale);

        RiderLoadEvaluationSettings doubleScale = unitScale;
        doubleScale.metersPerCoordinateUnit = 2.0;
        const RiderLoadHistory loadsAtDoubleScale = evaluateRiderLoads(
            kinematics, doubleScale);

        const double unitLateral = loadsAtUnitScale.states.front().lateralG;
        const double doubleLateral =
            loadsAtDoubleScale.states.front().lateralG;
        requireNear(unitLateral, 2.0 * doubleLateral, 1e-13,
            "doubling metres per coordinate unit halves physical curvature");

        doubleScale.metersPerCoordinateUnit = 0.0;
        requireThrows<std::invalid_argument>(
            [&]
            {
                static_cast<void>(evaluateRiderLoads(
                    kinematics, doubleScale));
            },
            "non-positive physical scale is rejected"
        );
    }

    void testUnreachableUphillTrackStops()
    {
        constexpr double radius = 20.0;
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), radius * pi / 2.0);
        setConstantRates(track.section(0), -1.0 / radius, 0.0);

        const auto kinematics = integrateAuthoredTrackKinematics(track, 0.5);
        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = 3.0;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);

        require(!history.completed(), "unreachable uphill is reported");
        require(history.unreachable->speedSquared < 0.0,
            "unreachable state retains negative energy evidence");
        require(history.unreachable->distance > 0.0,
            "unreachable state occurs after track start");
        require(history.states.size() < kinematics.size(),
            "unreachable history stops instead of continuing");
        for (const RiderLoadState& state : history.states)
        {
            require(std::isfinite(state.vehicleSpeed),
                "reported reachable speeds remain physical");
        }
    }

    void testRightContinuousBoundaryCurvatureAndLoad()
    {
        constexpr double firstLength = 10.0;
        constexpr double radius = 20.0;
        constexpr double speed = 10.0;

        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), firstLength);

        track.appendSection();
        setPlanarArc(track.section(1), PlanarArcRegion{
            radius, pi / 4.0, pi / 2.0, 0.0});

        const auto kinematics = integrateAuthoredTrackKinematics(track, 3.0);
        const std::size_t boundary = findDistance(kinematics, firstLength);
        require(boundary > 0, "boundary has a preceding sample");
        requireNear(kinematics[boundary - 1].centerlineCurvature,
            glm::dvec3{0.0, 0.0, 0.0}, 1e-15,
            "straight side retains zero curvature");
        requireNear(kinematics[boundary].centerlineCurvature,
            glm::dvec3{0.0, 0.0, 1.0 / radius}, 1e-14,
            "shared boundary belongs to following planar arc");
        require(glm::length(kinematics.back().centerlineCurvature) > 0.0,
            "final endpoint belongs to the final region");

        RiderLoadEvaluationSettings settings;
        settings.initialSpeed = speed;
        const RiderLoadHistory history = evaluateRiderLoads(
            kinematics, settings);
        requireNear(history.states[boundary - 1].normalG, 1.0, 1e-13,
            "load before curvature step is one G");
        requireNear(history.states[boundary].normalG,
            1.0 + speed * speed
                / (radius * standardGravityAcceleration),
            1e-12,
            "boundary load uses following region curvature without averaging");
    }

    void testExistingGeometryOutputIsUnchanged()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 17.0);
        setConstantRates(track.section(0), -0.015, 0.02, 0.01);

        track.appendSection();
        setPlanarArc(track.section(1), PlanarArcRegion{
            18.0, pi / 3.0, 0.35, 0.4});

        const auto geometry = integrateAuthoredTrack(track, 1.75);
        const auto kinematics = integrateAuthoredTrackKinematics(track, 1.75);
        require(geometry.size() == kinematics.size(),
            "kinematic output preserves generated sample count");

        for (std::size_t index = 0; index < geometry.size(); ++index)
        {
            requireNear(kinematics[index].distance,
                geometry[index].distance, 0.0,
                "kinematic output preserves sample distances");
            requireNear(kinematics[index].position,
                geometry[index].position, 0.0,
                "kinematic output preserves centerline positions");
            requireNear(kinematics[index].frame.tangent,
                geometry[index].frame.tangent, 0.0,
                "kinematic output preserves tangents");
            requireNear(kinematics[index].frame.lateral,
                geometry[index].frame.lateral, 0.0,
                "kinematic output preserves lateral axes");
            requireNear(kinematics[index].frame.up,
                geometry[index].frame.up, 0.0,
                "kinematic output preserves up axes");
        }
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"stationary upright reference", testStationaryUprightReference},
        {"flat straight gravity-only motion",
            testFlatStraightGravityOnlyMotion},
        {"known constant-curvature load", testKnownConstantCurvatureLoad},
        {"banked rider-frame projection", testBankedFrameProjection},
        {"mixed regions share one speed history",
            testMixedRegionsShareOneSpeedHistory},
        {"physical scale changes curvature load",
            testPhysicalScaleChangesCurvatureLoad},
        {"unreachable uphill track stops", testUnreachableUphillTrackStops},
        {"right-continuous boundary curvature and load",
            testRightContinuousBoundaryCurvatureAndLoad},
        {"existing generated geometry is unchanged",
            testExistingGeometryOutputIsUnchanged}
    };

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[FAIL] " << name << ": "
                << exception.what() << '\n';
            return 1;
        }
    }

    std::cout << "All RiderLoad tests passed.\n";
    return 0;
}
