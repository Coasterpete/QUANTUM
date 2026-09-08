#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/Supports.hpp>

#include <nlohmann/json.hpp>

#include <stdexcept>
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
}

int main()
{
    oldV1WithoutSupportsLoadsEmpty();
    supportsRoundTripDeterministically();
    malformedReferencesAreRejected();
    malformedIdsAndCountersAreRejected();
    unknownSupportFieldsAreRejected();
}
