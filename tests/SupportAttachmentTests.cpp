#include <quantum/coaster/AuthoredTrack.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Function>
    void requireInvalid(Function&& function, const std::string_view message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw std::runtime_error(std::string(message));
    }

    struct NodeFixture
    {
        AuthoredTrack track = createNewDocument();
        SupportStructureId structureId = invalidSupportStructureId;
        SupportElementId nodeId = invalidSupportElementId;

        NodeFixture()
        {
            structureId = track.createSupportStructure("Attachment fixture");
            nodeId = track.createSupportNode(
                structureId, {-100.0, -100.0, -100.0});
        }
    };

    [[nodiscard]] bool near(
        const glm::dvec3& first,
        const glm::dvec3& second,
        const double tolerance = 1.0e-11)
    {
        return glm::length(first - second) <= tolerance;
    }

    void modelValidationAndMutation()
    {
        validateSupportCollection({});

        NodeFixture fixture;
        fixture.track.setSupportTrackAttachment(
            fixture.structureId, fixture.nodeId, {10.0, 2.0, 3.0});
        validateSupportAnchors(fixture.track);
        const SupportNode& attached =
            fixture.track.supports().structures.front().nodes.front();
        require(attached.trackAttachment == TrackAttachment{10.0, 2.0, 3.0},
            "attachment mutation must target the stable node identity");
        require(!attached.foundation.has_value(),
            "an attached node must not acquire foundation metadata");

        requireInvalid([&] {
            fixture.track.setSupportTrackAttachment(
                fixture.structureId, 999, {10.0, 0.0, 0.0});
        }, "missing attachment target node must be rejected");
        requireInvalid([&] {
            fixture.track.setSupportTrackAttachment(
                fixture.structureId, fixture.nodeId,
                {10.0, std::numeric_limits<double>::infinity(), 0.0});
        }, "non-finite attachment offsets must be rejected");
        requireInvalid([&] {
            fixture.track.setSupportTrackAttachment(
                fixture.structureId, fixture.nodeId,
                {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        }, "non-finite attachment stations must be rejected");
        require(attached.trackAttachment
                    == TrackAttachment{10.0, 2.0, 3.0},
            "rejected attachment edits must preserve prior metadata");

        fixture.track.clearSupportTrackAttachment(
            fixture.structureId, fixture.nodeId);
        fixture.track.setSupportFoundation(
            fixture.structureId, fixture.nodeId);
        const SupportNode& foundation =
            fixture.track.supports().structures.front().nodes.front();
        require(foundation.foundation.has_value()
                && !foundation.trackAttachment.has_value(),
            "foundation mutation must preserve exclusive node metadata");
        require(foundation.position == glm::dvec3{-100.0, -100.0, -100.0},
            "foundation intent must retain the explicit authored position");
        requireInvalid([&] {
            fixture.track.setSupportTrackAttachment(
                fixture.structureId, fixture.nodeId, {10.0, 0.0, 0.0});
        }, "foundation and attachment metadata must be mutually exclusive");
    }

    void resolutionUsesTrackFrameAndCoordinateUnits()
    {
        NodeFixture fixture;
        const TrackAttachment attachment{10.0, 2.0, 3.0};
        fixture.track.setSupportTrackAttachment(
            fixture.structureId, fixture.nodeId, attachment);
        const auto states = integrateAuthoredTrackKinematics(fixture.track, 1.0);
        const ResolvedTrackAttachment resolved =
            resolveSupportTrackAttachment(states, attachment);
        require(near(resolved.position, {10.0, 2.0, 3.0}),
            "station and local L/U offsets must resolve in the track frame");
        require(near(resolved.frame.tangent, {1.0, 0.0, 0.0})
                && near(resolved.frame.lateral, {0.0, 1.0, 0.0})
                && near(resolved.frame.up, {0.0, 0.0, 1.0}),
            "resolution must retain the canonical double-precision frame");

        TrackPhysicalSettings physical = fixture.track.physicalSettings();
        physical.metersPerCoordinateUnit = 4.0;
        fixture.track.setPhysicalSettings(physical);
        const auto rescaledStates =
            integrateAuthoredTrackKinematics(fixture.track, 1.0);
        require(near(resolveSupportTrackAttachment(
                rescaledStates, attachment).position, resolved.position),
            "physical SI scale must not alter authored station semantics");

        AuthoredStartPose moved = fixture.track.startPose();
        moved.position = {5.0, 7.0, 11.0};
        fixture.track.setStartPose(moved);
        const auto movedStates =
            integrateAuthoredTrackKinematics(fixture.track, 1.0);
        require(near(resolveSupportTrackAttachment(
                movedStates, attachment).position, {15.0, 9.0, 14.0}),
            "geometry edits must change only the derived attachment position");
    }

    void topologyDomainsAreCanonical()
    {
        NodeFixture shuttle;
        shuttle.track.setLayoutMode(LayoutMode::Shuttle);
        shuttle.track.setSupportTrackAttachment(
            shuttle.structureId, shuttle.nodeId, {60.0, 0.0, 0.0});
        validateSupportAnchors(shuttle.track);

        requireInvalid([&] {
            shuttle.track.setLayoutMode(LayoutMode::Circuit);
        }, "a circuit must reject the duplicate end-station representation");

        NodeFixture circuit;
        circuit.track.setSupportTrackAttachment(
            circuit.structureId, circuit.nodeId, {59.5, 0.0, 0.0});
        validateSupportAnchors(circuit.track);
        requireInvalid([&] {
            circuit.track.setSupportTrackAttachment(
                circuit.structureId, circuit.nodeId, {60.0, 0.0, 0.0});
        }, "circuit stations must use the half-open canonical domain");
        requireInvalid([&] {
            circuit.track.setSupportTrackAttachment(
                circuit.structureId, circuit.nodeId, {-0.01, 0.0, 0.0});
        }, "negative stations must be rejected");
    }

    void shorteningInvalidationRejectsGeneration()
    {
        NodeFixture fixture;
        fixture.track.setLayoutMode(LayoutMode::Shuttle);
        fixture.track.setSupportTrackAttachment(
            fixture.structureId, fixture.nodeId, {40.0, 0.0, 0.0});

        setSectionLength(fixture.track.section(0), 50.0);
        static_cast<void>(integrateAuthoredTrackKinematics(fixture.track, 1.0));

        setSectionLength(fixture.track.section(0), 30.0);
        requireInvalid([&] {
            static_cast<void>(integrateAuthoredTrackKinematics(
                fixture.track, 1.0));
        }, "a shortening edit must reject an invalid committed attachment");
        require(fixture.track.supports().structures.front().nodes.front()
                .trackAttachment->station == 40.0,
            "rejected generation must not clamp or detach the attachment");
    }
}

int main()
{
    try
    {
        modelValidationAndMutation();
        resolutionUsesTrackFrameAndCoordinateUnits();
        topologyDomainsAreCanonical();
        shorteningInvalidationRejectsGeneration();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Support attachment test failure: "
                  << exception.what() << '\n';
        return 1;
    }
}
