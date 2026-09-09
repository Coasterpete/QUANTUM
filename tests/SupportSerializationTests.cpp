#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/Supports.hpp>

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <limits>
#include <string>
#include <string_view>

namespace
{
    using namespace quantum::coaster;
    using json = nlohmann::json;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] SupportCollection supportFixture()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            allocateSupportStructureId(collection);
        collection.structures.push_back({structureId, "Bent A"});
        SupportStructure& structure = collection.structures.back();

        const SupportElementId left = allocateSupportElementId(structure);
        structure.nodes.push_back({left, {-2.0, 1.0, 0.0}});
        const SupportElementId right = allocateSupportElementId(structure);
        structure.nodes.push_back({right, {2.0, 1.0, 0.0}});
        const SupportElementId top = allocateSupportElementId(structure);
        structure.nodes.push_back({top, {0.0, 1.0, 5.0}});

        const SupportMemberProfile tube{
            SupportMemberProfileShape::Circular, {0.25, 0.25}, 0.02};
        structure.members.push_back({
            allocateSupportElementId(structure), left, top, tube});
        structure.members.push_back({
            allocateSupportElementId(structure), right, top, tube});
        return collection;
    }

    void oldV1WithoutSupportsLoadsEmpty()
    {
        json legacy = json::parse(serializeCoasterDocument(
            createNewDocument()));
        legacy.erase("supports");

        const auto restored = deserializeCoasterDocument(legacy.dump());
        require(restored.has_value(),
            "v1 document without supports must load successfully");
        require(restored->supports().empty(),
            "missing supports must produce an empty collection");
        require(restored->supports().nextStructureId == 1,
            "missing supports must retain the initial allocator state");
    }

    void supportsRoundTripDeterministically()
    {
        AuthoredTrack track = createNewDocument();
        const SupportCollection supports = supportFixture();
        track.setSupports(supports);

        const std::string serialized = serializeCoasterDocument(track);
        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(), "support document must deserialize");
        require(restored->supports() == supports,
            "support IDs, counters, references, and order must round-trip");
        require(serializeCoasterDocument(*restored) == serialized,
            "support serialization must be deterministic");
    }

    void malformedReferencesAreRejected()
    {
        AuthoredTrack track = createNewDocument();
        track.setSupports(supportFixture());
        json malformed = json::parse(serializeCoasterDocument(track));
        malformed["supports"]["structures"][0]["members"][0]
            ["endNodeId"] = 999;

        const auto restored = deserializeCoasterDocument(malformed.dump());
        require(!restored.has_value(),
            "dangling serialized member reference must be rejected");
    }

    void malformedIdsAndCountersAreRejected()
    {
        AuthoredTrack track = createNewDocument();
        track.setSupports(supportFixture());
        const json serialized = json::parse(serializeCoasterDocument(track));

        json zeroId = serialized;
        zeroId["supports"]["structures"][0]["nodes"][0]["id"] = 0;
        require(!deserializeCoasterDocument(zeroId.dump()).has_value(),
            "zero serialized support ID must be rejected");

        json reusedCounter = serialized;
        reusedCounter["supports"]["structures"][0]["nextElementId"] = 1;
        require(!deserializeCoasterDocument(reusedCounter.dump()).has_value(),
            "serialized allocator counter capable of reuse must be rejected");
    }

    void unknownSupportFieldsAreRejected()
    {
        AuthoredTrack track = createNewDocument();
        track.setSupports(supportFixture());
        json malformed = json::parse(serializeCoasterDocument(track));
        malformed["supports"]["structures"][0]["members"][0]
            ["futureField"] = true;

        const auto restored = deserializeCoasterDocument(malformed.dump());
        require(!restored.has_value(),
            "unknown support fields must be rejected");
        require(restored.error().find("futureField") != std::string::npos,
            "unknown support field error must identify the field");
    }

    void manuallyAuthoredGraphRoundTripsExactly()
    {
        SupportCollection supports;
        const SupportStructureId frameId =
            createSupportStructure(supports, "Manual steel frame");
        const SupportElementId base = createSupportNode(
            supports, frameId, {0.0, 0.0, 0.0});
        const SupportElementId top = createSupportNode(
            supports, frameId, {0.0, 0.0, 7.0});
        createSupportMember(supports, frameId, base, top,
            {SupportMemberProfileShape::Circular, {0.28, 0.28}, 0.018});
        const SupportElementId loose = createSupportNode(
            supports, frameId, {1.5, 0.0, 3.5});
        const SupportStructureId bentId =
            createSupportStructure(supports);
        createSupportNode(supports, bentId, {-2.0, 0.0, 0.0});
        createSupportNode(supports, bentId, {2.0, 0.0, 0.0});
        // Delete one node and the structure it left behind so the round-trip
        // must preserve the allocator past an erase.
        removeSupportStructure(supports, frameId);

        AuthoredTrack track = createNewDocument();
        track.setSupports(supports);

        const std::string serialized = serializeCoasterDocument(track);
        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(),
            "a manually authored graph must deserialize");
        require(restored->supports() == track.supports(),
            "manual graph IDs, counters, profiles, and erase history must "
            "round-trip");
        require(serializeCoasterDocument(*restored) == serialized,
            "manual graph serialization must be deterministic");
    }

    void attachmentAndFoundationRoundTrip()
    {
        AuthoredTrack track = createNewDocument();
        track.setLayoutMode(LayoutMode::Shuttle);
        const SupportStructureId structureId =
            track.createSupportStructure("Anchored support");
        const SupportElementId attached = track.createSupportNode(
            structureId, {91.0, 92.0, 93.0});
        const SupportElementId foundation = track.createSupportNode(
            structureId, {4.0, 5.0, 6.0});
        track.setSupportTrackAttachment(
            structureId, attached, {12.5, -1.25, 2.75});
        track.setSupportFoundation(structureId, foundation);

        const std::string serialized = serializeCoasterDocument(track);
        const json document = json::parse(serialized);
        const json& attachedJson =
            document["supports"]["structures"][0]["nodes"][0];
        require(attachedJson["position"]["x"] == 91.0,
            "the authored fallback position must remain serialized");
        require(attachedJson["trackAttachment"]["station"] == 12.5,
            "attachment station must be serialized in authored units");
        require(!attachedJson.contains("resolvedPosition")
                && !attachedJson["trackAttachment"].contains("position"),
            "derived world position must not be serialized");
        require(document["supports"]["structures"][0]["nodes"][1]
                ["foundation"].empty(),
            "foundation v1 metadata must be an empty additive object");

        const auto restored = deserializeCoasterDocument(serialized);
        require(restored.has_value(),
            "attachment/foundation document must deserialize");
        require(restored->supports() == track.supports(),
            "attachment and foundation metadata must round-trip exactly");
        require(serializeCoasterDocument(*restored) == serialized,
            "attachment/foundation serialization must be deterministic");
    }

    void malformedAnchorFieldsAreRejected()
    {
        AuthoredTrack track = createNewDocument();
        const SupportStructureId structureId =
            track.createSupportStructure("Attached");
        const SupportElementId nodeId = track.createSupportNode(
            structureId, {0.0, 0.0, 0.0});
        track.setSupportTrackAttachment(
            structureId, nodeId, {10.0, 0.0, 0.0});
        const json valid = json::parse(serializeCoasterDocument(track));

        json unknown = valid;
        unknown["supports"]["structures"][0]["nodes"][0]
            ["trackAttachment"]["pathId"] = 0;
        require(!deserializeCoasterDocument(unknown.dump()).has_value(),
            "unknown attachment fields must be rejected");

        json malformed = valid;
        malformed["supports"]["structures"][0]["nodes"][0]
            ["trackAttachment"]["lateralOffset"] = "NaN";
        require(!deserializeCoasterDocument(malformed.dump()).has_value(),
            "non-numeric attachment values must be rejected");

        json conflicting = valid;
        conflicting["supports"]["structures"][0]["nodes"][0]
            ["foundation"] = json::object();
        require(!deserializeCoasterDocument(conflicting.dump()).has_value(),
            "serialized attachment/foundation conflicts must be rejected");

        json invalidStation = valid;
        invalidStation["supports"]["structures"][0]["nodes"][0]
            ["trackAttachment"]["station"] = 60.0;
        require(!deserializeCoasterDocument(invalidStation.dump()).has_value(),
            "non-canonical circuit end stations must be rejected");
    }
}

int main()
{
    oldV1WithoutSupportsLoadsEmpty();
    supportsRoundTripDeterministically();
    malformedReferencesAreRejected();
    malformedIdsAndCountersAreRejected();
    unknownSupportFieldsAreRejected();
    manuallyAuthoredGraphRoundTripsExactly();
    attachmentAndFoundationRoundTrip();
    malformedAnchorFieldsAreRejected();
}
