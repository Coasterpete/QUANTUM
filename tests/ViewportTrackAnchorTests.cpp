#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::TrackKinematicState;
    using quantum::editor::ViewportCamera;
    using quantum::editor::ViewportCameraPose;
    using quantum::editor::ViewportTrackAnchor;
    using quantum::editor::ViewportTrackAnchorKind;

    constexpr double pi = 3.14159265358979323846;

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
        if (std::abs(actual - expected) > tolerance)
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

    void setChannel(
        quantum::coaster::ChannelProfile& channel,
        const double begin,
        const double end)
    {
        quantum::math::ScalarTransition& transition =
            channel.segments.front().transition;
        transition.valueBegin = begin;
        transition.valueEnd = end;
        transition.transitionType =
            quantum::math::TransitionType::Smoothstep;
    }

    [[nodiscard]] AuthoredTrack createMixedTrack()
    {
        AuthoredTrack track;
        track.appendSection();
        quantum::coaster::setSectionLength(track.section(0), 17.0);
        auto& first = track.section(0).rateProfileRegion().rateProfiles;
        setChannel(first.roll, 0.0, 0.008);
        setChannel(first.pitch, -0.003, 0.011);
        setChannel(first.yaw, 0.004, 0.009);

        track.appendSection();
        quantum::coaster::convertSectionToPlanarArc(track.section(1));
        quantum::coaster::setPlanarArcRadius(track.section(1), 28.0);
        quantum::coaster::setPlanarArcSweptAngle(
            track.section(1),
            62.0 * pi / 180.0
        );
        quantum::coaster::setPlanarArcPlaneTilt(
            track.section(1),
            14.0 * pi / 180.0
        );
        quantum::coaster::setPlanarArcBankChange(
            track.section(1),
            19.0 * pi / 180.0
        );

        track.appendSection();
        quantum::coaster::setSectionLength(track.section(2), 23.0);
        auto& final = track.section(2).rateProfileRegion().rateProfiles;
        setChannel(final.roll, 0.006, -0.002);
        setChannel(final.pitch, 0.005, -0.007);
        setChannel(final.yaw, -0.004, 0.003);
        return track;
    }

    [[nodiscard]] const TrackKinematicState& stateAtDistance(
        const std::vector<TrackKinematicState>& states,
        const double distance)
    {
        for (const TrackKinematicState& state : states)
        {
            const double tolerance = 1.0e-9
                * std::max(1.0, std::abs(distance));
            if (std::abs(state.distance - distance) <= tolerance)
            {
                return state;
            }
        }

        throw std::runtime_error("Expected kinematic boundary was absent.");
    }

    void countClassificationAndNeighbors()
    {
        const AuthoredTrack track = createMixedTrack();
        const auto anchors =
            quantum::editor::createViewportTrackAnchors(track);

        require(anchors.size() == track.sectionCount() + 1,
            "N regions must expose N+1 anchors");
        require(anchors.front().kind == ViewportTrackAnchorKind::Start,
            "anchor zero must be the start anchor");
        require(!anchors.front().previousRegionIndex.has_value()
                && anchors.front().nextRegionIndex == 0,
            "start neighbors must name only region zero");
        require(anchors[1].kind == ViewportTrackAnchorKind::Interior
                && anchors[1].previousRegionIndex == 0
                && anchors[1].nextRegionIndex == 1,
            "an interior anchor must be one shared boundary");
        require(anchors.back().kind == ViewportTrackAnchorKind::End,
            "the final anchor must be the end anchor");
        require(anchors.back().previousRegionIndex == 2
                && !anchors.back().nextRegionIndex.has_value(),
            "end neighbors must name only the final region");
    }

    void initialPoseIsExact()
    {
        const auto anchors = quantum::editor::createViewportTrackAnchors(
            createMixedTrack());
        const ViewportTrackAnchor& start = anchors.front();

        requireNear(start.distance, 0.0, 0.0,
            "start distance must be exactly zero");
        requireNear(start.position, {0.0, 0.0, 0.0}, 0.0,
            "start position must use the exact authored-track seed");
        requireNear(start.forward, {1.0, 0.0, 0.0}, 0.0,
            "start forward must use the exact authored-track seed");
        requireNear(start.lateral, {0.0, 1.0, 0.0}, 0.0,
            "start lateral must use the exact authored-track seed");
        requireNear(start.up, {0.0, 0.0, 1.0}, 0.0,
            "start up must use the exact authored-track seed");
    }

    void mixedRegionBoundariesUseExactKinematics()
    {
        const AuthoredTrack track = createMixedTrack();
        const auto anchors =
            quantum::editor::createViewportTrackAnchors(track);
        const auto fineStates =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 0.19);

        double distance = 0.0;
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            distance += quantum::coaster::sectionLength(track.section(index));
            const TrackKinematicState& endpoint = stateAtDistance(
                fineStates,
                distance
            );
            const ViewportTrackAnchor& anchor = anchors[index + 1];

            requireNear(anchor.distance, distance, 1.0e-9,
                "anchor distance must equal the authored prefix length");
            requireNear(anchor.position, endpoint.position, 2.0e-7,
                "anchor position must equal the exact region endpoint");
            requireNear(anchor.forward, endpoint.frame.tangent, 2.0e-10,
                "anchor forward must equal the exact endpoint frame");
            requireNear(anchor.lateral, endpoint.frame.lateral, 2.0e-10,
                "anchor lateral must equal the exact endpoint frame");
            requireNear(anchor.up, endpoint.frame.up, 2.0e-10,
                "anchor up must equal the exact endpoint frame");
        }
    }

    void extractionDoesNotUseVisualizationSpacing()
    {
        const AuthoredTrack track = createMixedTrack();
        const auto anchors =
            quantum::editor::createViewportTrackAnchors(track);
        const auto coarse =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 50.0);
        const auto fine =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 0.07);

        for (const ViewportTrackAnchor& anchor : anchors)
        {
            const TrackKinematicState& coarseBoundary = stateAtDistance(
                coarse,
                anchor.distance
            );
            const TrackKinematicState& fineBoundary = stateAtDistance(
                fine,
                anchor.distance
            );
            requireNear(coarseBoundary.position, fineBoundary.position,
                2.0e-7,
                "boundary pose must be independent of output sample spacing");
            requireNear(anchor.position, fineBoundary.position, 2.0e-7,
                "anchor extraction must match spacing-independent endpoint");
        }
    }

    void rightContinuousSelectionAndStaleReplacement()
    {
        const auto start =
            quantum::editor::selectionForViewportTrackAnchor(0, 3);
        const auto interior =
            quantum::editor::selectionForViewportTrackAnchor(1, 3);
        const auto final =
            quantum::editor::selectionForViewportTrackAnchor(3, 3);
        require(start.regionIndex == 0 && start.anchorIndex == 0,
            "start anchor must select region zero");
        require(interior.regionIndex == 1 && interior.anchorIndex == 1,
            "interior anchor must select the following region");
        require(final.regionIndex == 2 && final.anchorIndex == 3,
            "final anchor must select the final region");

        const auto regionSelection =
            quantum::editor::selectionForViewportTrackRegion(2, 3);
        require(regionSelection.regionIndex == 2
                && regionSelection.anchorIndex == 2,
            "region selection must replace a stale final-anchor highlight");
    }

    [[nodiscard]] ViewportCamera pickingCamera()
    {
        ViewportCamera camera;
        camera.setBounds({-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0});
        camera.setPose(ViewportCameraPose{
            .focus = {0.0, 0.0, 0.0},
            .yaw = 0.0,
            .pitch = 0.0,
            .distance = 10.0
        });
        return camera;
    }

    [[nodiscard]] ViewportTrackAnchor anchor(
        const std::size_t index,
        const glm::dvec3& position)
    {
        ViewportTrackAnchor value;
        value.anchorIndex = index;
        value.position = position;
        return value;
    }

    void pickingIsDeterministicAndPrefersPointerProximity()
    {
        const ViewportCamera camera = pickingCamera();
        const std::vector<ViewportTrackAnchor> overlap{
            anchor(4, {0.0, 0.0, 0.0}),
            anchor(1, {0.0, 0.0, 0.0})
        };
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            const auto hit = quantum::editor::pickViewportTrackAnchor(
                overlap,
                camera,
                {0.5, 0.5},
                1000,
                1000
            );
            require(hit.has_value() && hit->anchorIndex == 1,
                "exactly overlapping anchors must choose the lower index");
        }

        const std::vector<ViewportTrackAnchor> depthOverlap{
            anchor(0, {0.0, 0.0, 0.0}),
            anchor(1, {2.0, 0.0, 0.0})
        };
        const auto front = quantum::editor::pickViewportTrackAnchor(
            depthOverlap,
            camera,
            {0.5, 0.5},
            1000,
            1000
        );
        require(front.has_value() && front->anchorIndex == 1,
            "equal screen positions must choose the frontmost anchor");

        const std::vector<ViewportTrackAnchor> nearby{
            anchor(0, {0.0, 0.0, 0.0}),
            anchor(1, {0.0, 0.15, 0.0})
        };
        const auto projected = quantum::editor::projectViewportPoint(
            camera,
            nearby[1].position,
            1.0
        );
        require(projected.has_value(),
            "nearby picking fixture must project into the viewport");
        const auto closest = quantum::editor::pickViewportTrackAnchor(
            nearby,
            camera,
            projected->normalizedPosition,
            1000,
            1000,
            20.0
        );
        require(closest.has_value() && closest->anchorIndex == 1,
            "the marker nearest the pointer must win before tie breaking");
    }
}

int main()
{
    try
    {
        countClassificationAndNeighbors();
        initialPoseIsExact();
        mixedRegionBoundariesUseExactKinematics();
        extractionDoesNotUseVisualizationSpacing();
        rightContinuousSelectionAndStaleReplacement();
        pickingIsDeterministicAndPrefersPointerProximity();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Viewport track anchor test failure: "
                  << exception.what() << '\n';
        return 1;
    }

    std::cout << "Viewport track anchor tests passed.\n";
    return 0;
}
