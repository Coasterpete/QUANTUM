#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <nlohmann/json.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredStartPose;
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::coaster::TrackKinematicState;
    using quantum::editor::AuthoredTrackEditTransaction;
    using quantum::editor::StartPoseTransformAxis;
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
        if (!std::isfinite(glm::length(actual - expected))
            || glm::length(actual - expected) > tolerance)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Exception, typename Function>
    void requireThrows(Function&& function, const std::string_view message)
    {
        try
        {
            function();
        }
        catch (const Exception&)
        {
            return;
        }
        throw std::runtime_error(std::string(message));
    }

    void requireFrame(
        const quantum::geometry::CurveFrame& frame,
        const double tolerance,
        const std::string_view message)
    {
        requireNear(glm::length(frame.tangent), 1.0, tolerance, message);
        requireNear(glm::length(frame.lateral), 1.0, tolerance, message);
        requireNear(glm::length(frame.up), 1.0, tolerance, message);
        requireNear(glm::dot(frame.tangent, frame.lateral), 0.0,
            tolerance, message);
        requireNear(glm::dot(frame.tangent, frame.up), 0.0,
            tolerance, message);
        requireNear(glm::dot(frame.lateral, frame.up), 0.0,
            tolerance, message);
        requireNear(glm::cross(frame.tangent, frame.lateral), frame.up,
            tolerance, message);
    }

    [[nodiscard]] bool exactlyEqual(
        const RiderLocalGeometryState& left,
        const RiderLocalGeometryState& right) noexcept
    {
        return left.distance == right.distance
            && glm::length(left.position - right.position) == 0.0
            && left.frame == right.frame;
    }

    [[nodiscard]] bool exactlyEqual(
        const std::vector<RiderLocalGeometryState>& left,
        const std::vector<RiderLocalGeometryState>& right) noexcept
    {
        if (left.size() != right.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < left.size(); ++index)
        {
            if (!exactlyEqual(left[index], right[index]))
            {
                return false;
            }
        }
        return true;
    }

    void defaultPoseReproducesLegacyGeometryExactly()
    {
        const AuthoredTrack track =
            quantum::coaster::createDefaultAuthoredTrack();
        const auto& section = track.section(0);
        const auto& profiles = section.rateProfileRegion().rateProfiles;
        constexpr quantum::geometry::CurveFrame legacyFrame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        const std::vector<RiderLocalGeometryState> legacy =
            quantum::coaster::integrateLocalRollPitchYawRateProfiles(
                {0.0, 0.0, 0.0},
                legacyFrame,
                profiles.roll,
                profiles.pitch,
                profiles.yaw,
                section.length,
                0.75
            );
        const std::vector<RiderLocalGeometryState> generated =
            quantum::coaster::integrateAuthoredTrack(track, 0.75);

        require(generated.size() == legacy.size(),
            "default start pose changed the legacy sample grid");
        for (std::size_t index = 0; index < legacy.size(); ++index)
        {
            require(exactlyEqual(generated[index], legacy[index]),
                "default start pose must reproduce legacy geometry exactly");
        }
    }

    void translationAndRotationTransformTheCompleteTrack()
    {
        AuthoredTrack track = quantum::coaster::createDefaultAuthoredTrack();
        track.appendSection();
        const std::vector<TrackKinematicState> original =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 0.75);

        constexpr glm::dvec3 offset{12.0, -8.0, 4.5};
        track.setStartPose({offset, {1.0, 0.0, 0.0, 0.0}});
        const std::vector<TrackKinematicState> translated =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 0.75);
        require(translated.size() == original.size(),
            "start translation changed the sample grid");
        for (std::size_t index = 0; index < original.size(); ++index)
        {
            requireNear(translated[index].position,
                original[index].position + offset, 3.0e-10,
                "start translation did not move every position equally");
            requireNear(translated[index].frame.tangent,
                original[index].frame.tangent, 3.0e-12,
                "translation changed the rider frame");
        }

        const glm::dquat rotation = glm::angleAxis(
            67.0 * pi / 180.0,
            glm::normalize(glm::dvec3{0.4, -0.2, 0.9})
        );
        track.setStartPose({offset, rotation});
        const std::vector<TrackKinematicState> rotated =
            quantum::coaster::integrateAuthoredTrackKinematics(track, 0.75);
        for (std::size_t index = 0; index < original.size(); ++index)
        {
            requireNear(rotated[index].position,
                offset + rotation * original[index].position, 2.0e-8,
                "start rotation did not rotate the complete centerline");
            requireNear(rotated[index].frame.tangent,
                rotation * original[index].frame.tangent, 2.0e-10,
                "start rotation did not rotate tangent consistently");
            requireNear(rotated[index].frame.lateral,
                rotation * original[index].frame.lateral, 2.0e-10,
                "start rotation did not rotate lateral consistently");
            requireNear(rotated[index].frame.up,
                rotation * original[index].frame.up, 2.0e-10,
                "start rotation did not rotate up consistently");
            requireNear(rotated[index].centerlineCurvature,
                rotation * original[index].centerlineCurvature, 2.0e-10,
                "start rotation did not rotate curvature consistently");
        }
    }

    void storedOrientationStaysOrthonormalAndRightHanded()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        const glm::dquat rotation = glm::angleAxis(
            1.2,
            glm::normalize(glm::dvec3{1.0, 2.0, -0.5})
        );
        track.setStartPose({{3.0, 4.0, 5.0}, 9.0 * rotation});
        requireNear(glm::length(track.startPose().orientation), 1.0,
            2.0e-15, "stored start quaternion must be normalized");
        requireFrame(quantum::coaster::startPoseRiderFrame(
            track.startPose()), 3.0e-15,
            "authored start rider frame must stay orthonormal");

        const AuthoredStartPose rotated = quantum::editor::rotateStartPose(
            track.startPose(), StartPoseTransformAxis::Z, 0.7);
        track.setStartPose(rotated);
        requireFrame(quantum::coaster::startPoseRiderFrame(
            track.startPose()), 3.0e-15,
            "edited start rider frame must stay right-handed");
    }

    void semanticAnchorZeroIsTheEditableStartPose()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.appendSection();
        track.appendSection();
        const AuthoredStartPose pose{
            {7.0, -3.0, 11.0},
            glm::angleAxis(0.8, glm::normalize(glm::dvec3{1.0, 1.0, 0.5}))
        };
        track.setStartPose(pose);
        const auto anchors =
            quantum::editor::createViewportTrackAnchors(track);
        const auto frame = quantum::coaster::startPoseRiderFrame(
            track.startPose());

        require(anchors.size() == track.sectionCount() + 1,
            "N authored regions must still expose N+1 anchors");
        require(anchors.front().kind == ViewportTrackAnchorKind::Start,
            "anchor zero must remain the semantic start");
        requireNear(anchors.front().position, track.startPose().position, 0.0,
            "anchor zero position must exactly match the authored pose");
        requireNear(anchors.front().forward, frame.tangent, 0.0,
            "anchor zero forward must exactly match the authored pose");
        requireNear(anchors.front().lateral, frame.lateral, 0.0,
            "anchor zero lateral must exactly match the authored pose");
        requireNear(anchors.front().up, frame.up, 0.0,
            "anchor zero up must exactly match the authored pose");
        require(quantum::editor::isViewportTrackAnchorEditable(
                anchors.front().kind),
            "start anchor must be editable");
        require(!quantum::editor::isViewportTrackAnchorEditable(
                anchors[1].kind),
            "interior anchor must remain read-only");
        require(!quantum::editor::isViewportTrackAnchorEditable(
                anchors.back().kind),
            "final anchor must remain read-only");

        const auto selection =
            quantum::editor::selectionForViewportTrackAnchor(0, 3);
        require(selection.regionIndex == 0 && selection.anchorIndex == 0,
            "start-anchor editing must keep Region 0 selection synchronized");
    }

    void rejectedCandidateLeavesCommittedStateUnchanged()
    {
        AuthoredTrack committed = quantum::coaster::createDefaultAuthoredTrack();
        const AuthoredStartPose before = committed.startPose();
        const auto geometryBefore =
            quantum::coaster::integrateAuthoredTrack(committed, 1.0);
        AuthoredTrackEditTransaction transaction{committed};
        AuthoredStartPose invalid = before;
        invalid.position.x = std::numeric_limits<double>::infinity();

        requireThrows<std::invalid_argument>(
            [&transaction, &invalid]
            {
                transaction.candidate().setStartPose(invalid);
            },
            "non-finite start-pose candidate must be rejected");
        require(!transaction.committed(),
            "rejected start-pose candidate must not commit");
        require(committed.startPose() == before,
            "rejected start-pose candidate changed the document");
        require(exactlyEqual(
                quantum::coaster::integrateAuthoredTrack(committed, 1.0),
                geometryBefore),
            "rejected start-pose candidate changed generated geometry");
    }

    void transformedBoundsFollowOnlyTheCenterline()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        const glm::dquat rotation = glm::angleAxis(
            0.5 * pi,
            glm::dvec3{0.0, 0.0, 1.0}
        );
        track.setStartPose({{5.0, -4.0, 3.0}, rotation});
        const auto visualization =
            quantum::editor::createCenterlineVisualization(track);

        requireNear(visualization.minimumPosition, {5.0, -4.0, 3.0},
            2.0e-10, "transformed camera minimum did not follow centerline");
        requireNear(visualization.maximumPosition, {5.0, 56.0, 3.0},
            2.0e-10, "transformed camera maximum did not follow centerline");
        quantum::editor::ViewportCamera camera;
        camera.setBounds(
            visualization.minimumPosition,
            visualization.maximumPosition
        );
        requireNear(camera.boundsCenter(), {5.0, 26.0, 3.0}, 2.0e-10,
            "camera bounds did not consume transformed authored geometry");

        double largestReferenceZ = -std::numeric_limits<double>::infinity();
        for (const auto& vertex : visualization.vertices)
        {
            largestReferenceZ = std::max(
                largestReferenceZ,
                static_cast<double>(vertex.z)
            );
        }
        require(largestReferenceZ > visualization.maximumPosition.z + 1.0,
            "test fixture must contain display geometry outside centerline bounds");
        requireNear(visualization.maximumPosition.z, 3.0, 2.0e-10,
            "reference curves or semantic overlays enlarged camera bounds");
    }

    void serializationPersistsPoseAndDefaultsLegacyDocuments()
    {
        AuthoredTrack track = quantum::coaster::createDefaultAuthoredTrack();
        track.setStartPose({
            {1.25, -9.5, 6.75},
            glm::angleAxis(0.63,
                glm::normalize(glm::dvec3{0.2, 0.7, -0.4}))
        });
        const std::string serialized =
            quantum::coaster::serializeCoasterDocument(track);
        const nlohmann::json saved = nlohmann::json::parse(serialized);
        require(saved.contains("startPose"),
            "saved documents must persist the authored start pose");
        auto restored = quantum::coaster::deserializeCoasterDocument(
            serialized);
        require(restored.has_value(), "start-pose document must load");
        requireNear(restored->startPose().position,
            track.startPose().position, 0.0,
            "start position did not survive save/load");
        requireNear(restored->startPose().orientation.w,
            track.startPose().orientation.w, 0.0,
            "start orientation did not survive save/load");
        requireNear(restored->startPose().orientation.x,
            track.startPose().orientation.x, 2.0e-15,
            "start orientation X did not survive save/load");
        requireNear(restored->startPose().orientation.y,
            track.startPose().orientation.y, 2.0e-15,
            "start orientation Y did not survive save/load");
        requireNear(restored->startPose().orientation.z,
            track.startPose().orientation.z, 2.0e-15,
            "start orientation Z did not survive save/load");
        const auto restoredAgain =
            quantum::coaster::deserializeCoasterDocument(serialized);
        require(restoredAgain.has_value()
                && restoredAgain->startPose() == restored->startPose(),
            "loading the same start pose must be deterministic");
        require(nlohmann::json::parse(
                quantum::coaster::serializeCoasterDocument(*restored))
                    ["sections"] == saved["sections"],
            "start-pose round-trip changed authored region values");

        nlohmann::json legacy = saved;
        legacy.erase("startPose");
        auto legacyResult = quantum::coaster::deserializeCoasterDocument(
            legacy.dump());
        require(legacyResult.has_value(),
            "legacy document without start pose must load");
        require(legacyResult->startPose() == AuthoredStartPose{},
            "legacy document must receive the canonical default start pose");
    }

    void forceDiagnosticsRegenerateWithPhysicsSemantics()
    {
        AuthoredTrack base = quantum::coaster::createNewDocument();
        const auto baseLoads =
            quantum::editor::evaluateRiderLoadDiagnostics(base);

        AuthoredTrack translated = base;
        translated.setStartPose({{100.0, -45.0, 28.0},
            {1.0, 0.0, 0.0, 0.0}});
        const auto translatedLoads =
            quantum::editor::evaluateRiderLoadDiagnostics(translated);
        require(translatedLoads.states.size() == baseLoads.states.size(),
            "uniform translation changed force sample count");
        for (std::size_t index = 0; index < baseLoads.states.size(); ++index)
        {
            requireNear(translatedLoads.states[index].vehicleSpeed,
                baseLoads.states[index].vehicleSpeed, 2.0e-13,
                "uniform XYZ translation changed relative-height speed");
            requireNear(translatedLoads.states[index].normalG,
                baseLoads.states[index].normalG, 2.0e-13,
                "uniform translation changed rider loads");
        }

        AuthoredTrack rotated = base;
        rotated.setStartPose({{0.0, 0.0, 0.0},
            glm::angleAxis(30.0 * pi / 180.0,
                glm::dvec3{0.0, 1.0, 0.0})});
        const auto rotatedLoads =
            quantum::editor::evaluateRiderLoadDiagnostics(rotated);
        require(rotatedLoads.completed(),
            "downhill rotated fixture should remain physically reachable");
        require(rotatedLoads.states.size() == baseLoads.states.size(),
            "rotation changed force diagnostic sample grid");
        require(rotatedLoads.states.back().vehicleSpeed
                > baseLoads.states.back().vehicleSpeed,
            "world-space rotation must regenerate gravity-derived speed");
    }
}

int main()
{
    using Test = std::pair<std::string_view, void (*)()>;
    const std::vector<Test> tests{
        {"default pose reproduces legacy geometry",
            defaultPoseReproducesLegacyGeometryExactly},
        {"translation and rotation transform complete track",
            translationAndRotationTransformTheCompleteTrack},
        {"orientation stays right-handed and orthonormal",
            storedOrientationStaysOrthonormalAndRightHanded},
        {"Anchor 0 is editable authored start pose",
            semanticAnchorZeroIsTheEditableStartPose},
        {"rejected candidate preserves committed state",
            rejectedCandidateLeavesCommittedStateUnchanged},
        {"transformed centerline owns camera bounds",
            transformedBoundsFollowOnlyTheCenterline},
        {"start pose serialization and legacy default",
            serializationPersistsPoseAndDefaultsLegacyDocuments},
        {"force diagnostics regenerate from canonical pose",
            forceDiagnosticsRegenerateWithPhysicsSemantics}
    };

    try
    {
        for (const auto& [name, test] : tests)
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] " << exception.what() << '\n';
        return 1;
    }

    std::cout << "All authored start-pose tests passed.\n";
    return 0;
}
