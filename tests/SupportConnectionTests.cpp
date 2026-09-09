#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/Supports.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
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

    [[nodiscard]] SupportMemberProfile roundTube()
    {
        return {SupportMemberProfileShape::Circular, {0.3, 0.3}, 0.025};
    }

    [[nodiscard]] SupportMemberProfile timber()
    {
        return {SupportMemberProfileShape::Rectangular, {0.2, 0.3}, 0.0};
    }

    struct Fixture
    {
        SupportCollection collection;
        SupportStructureId structureId;
        SupportElementId base;
        SupportElementId mid;
        SupportElementId top;
        // lowerLeg connects base -> mid; upperLeg connects mid -> top.
        SupportElementId lowerLeg;
        SupportElementId upperLeg;
    };

    [[nodiscard]] Fixture createFixture()
    {
        SupportCollection collection;
        const SupportStructureId structureId =
            createSupportStructure(collection, "Steel column");
        const SupportElementId base = createSupportNode(
            collection, structureId, {0.0, 0.0, 0.0});
        const SupportElementId mid = createSupportNode(
            collection, structureId, {0.0, 0.0, 4.0});
        const SupportElementId top = createSupportNode(
            collection, structureId, {0.0, 0.0, 8.0});
        const SupportElementId lowerLeg = createSupportMember(
            collection, structureId, base, mid, roundTube());
        const SupportElementId upperLeg = createSupportMember(
            collection, structureId, mid, top, roundTube());
        setSupportFoundation(collection, structureId, base);
        setSupportTrackAttachment(
            collection, structureId, top, {12.5, 0.0, 0.0});
        return {std::move(collection), structureId,
            base, mid, top, lowerLeg, upperLeg};
    }

    [[nodiscard]] SupportMember& findMember(
        SupportCollection& collection,
        const SupportElementId memberId)
    {
        SupportMember* result = nullptr;
        for (SupportStructure& structure : collection.structures)
        {
            for (SupportMember& member : structure.members)
            {
                if (member.id == memberId)
                {
                    result = &member;
                }
            }
        }
        require(result != nullptr, "test fixture member must exist");
        return *result;
    }

    [[nodiscard]] const SupportMember& findMember(
        const SupportCollection& collection,
        const SupportElementId memberId)
    {
        const SupportMember* result = nullptr;
        for (const SupportStructure& structure : collection.structures)
        {
            for (const SupportMember& member : structure.members)
            {
                if (member.id == memberId)
                {
                    result = &member;
                }
            }
        }
        require(result != nullptr, "test fixture member must exist");
        return *result;
    }

    void connectionEditsUseStableMemberIdentity()
    {
        Fixture fixture = createFixture();
        const std::uint32_t nextStructureId =
            fixture.collection.nextStructureId;
        const std::uint32_t nextElementId =
            fixture.collection.structures.front().nextElementId;

        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::Start,
            {SupportMemberEndTreatment::EndCap});
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle,
             StaticMeshAssetReference{
                 "assets://support/connectors/upper-saddle.glb", false},
             SupportMemberEndPlacement{{0.0, 0.0, 0.2}}});
        validateSupportCollection(fixture.collection);

        const SupportMember& member =
            findMember(fixture.collection, fixture.upperLeg);
        require(member.startConnection.has_value()
                && member.startConnection->treatment
                    == SupportMemberEndTreatment::EndCap,
            "the start connection must be authored independently");
        require(!member.startConnection->asset.has_value(),
            "the start connection must not inherit the end connection asset");
        require(member.endConnection.has_value()
                && member.endConnection->treatment
                    == SupportMemberEndTreatment::Saddle,
            "the end connection must be authored independently");
        require(member.endConnection->asset.has_value()
                && member.endConnection->asset->placeholder == false,
            "the end connection must retain its asset reference");
        require(fixture.collection.nextStructureId == nextStructureId
                && fixture.collection.structures.front().nextElementId
                    == nextElementId,
            "connection edits must not touch the ID allocators");

        clearSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End);
        const SupportMember& afterClear =
            findMember(fixture.collection, fixture.upperLeg);
        require(!afterClear.endConnection.has_value(),
            "clearing one end must not affect the other end");
        require(afterClear.startConnection.has_value(),
            "the start connection must survive an unrelated clear");

        clearSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::Start);
        const SupportMember& afterBoth =
            findMember(fixture.collection, fixture.upperLeg);
        require(!afterBoth.startConnection.has_value()
                && !afterBoth.endConnection.has_value(),
            "clearing both ends must leave no connection metadata");
    }

    void rejectedEndEditsLeaveStateUnchanged()
    {
        Fixture fixture = createFixture();
        const SupportMemberEndConnection committed{
            SupportMemberEndTreatment::EndCap};
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::Start, committed);
        const SupportCollection before = fixture.collection;

        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, 999, fixture.upperLeg,
                SupportMemberEnd::End, committed);
        }, "unknown structure ID must be rejected");
        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, 999,
                SupportMemberEnd::End, committed);
        }, "unknown member ID must be rejected");

        const SupportMemberEndConnection malformedTreatment{
            static_cast<SupportMemberEndTreatment>(255)};
        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End, malformedTreatment);
        }, "malformed treatment must be rejected");

        const SupportMemberEndConnection nonFinitePlacement{
            SupportMemberEndTreatment::EndCap,
            std::nullopt,
            SupportMemberEndPlacement{
                {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}};
        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End, nonFinitePlacement);
        }, "non-finite placement position must be rejected");

        const SupportMemberEndConnection nonPositiveScale{
            SupportMemberEndTreatment::EndCap,
            std::nullopt,
            SupportMemberEndPlacement{{0.0, 0.0, 0.0}, {}, {0.0, 1.0, 1.0}}};
        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End, nonPositiveScale);
        }, "non-positive placement scale must be rejected");

        const SupportMemberEndConnection saddleAtPlainNode{
            SupportMemberEndTreatment::Saddle};
        requireInvalid([&] {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.lowerLeg,
                SupportMemberEnd::End, saddleAtPlainNode);
        }, "a Saddle at an unattached node must be rejected");

        require(fixture.collection == before,
            "every rejected edit must leave the collection unchanged");
    }

    void deletingMemberRemovesOwnedConnections()
    {
        Fixture fixture = createFixture();
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::Start,
            {SupportMemberEndTreatment::MiteredCut});
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle});

        removeSupportMember(
            fixture.collection, fixture.structureId, fixture.upperLeg);
        validateSupportCollection(fixture.collection);
        for (const SupportStructure& structure : fixture.collection.structures)
        {
            for (const SupportMember& member : structure.members)
            {
                require(!member.startConnection.has_value()
                        && !member.endConnection.has_value(),
                    "deleting a member must remove both end connections");
            }
        }
    }

    void noConnectionOwningIdsAreAllocated()
    {
        Fixture fixture = createFixture();
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle});
        require(fixture.collection.structures.front().nextElementId
                == 6,
            "connection authorship must never allocate an element ID");
    }

    void nodeContextRulesForTrackOnlyTreatments()
    {
        Fixture fixture = createFixture();

        for (const SupportMemberEndTreatment treatment :
            {SupportMemberEndTreatment::Saddle,
             SupportMemberEndTreatment::Clamp})
        {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End, {treatment});
            validateSupportCollection(fixture.collection);

            clearSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End);
            requireInvalid([&] {
                setSupportMemberEndConnection(
                    fixture.collection, fixture.structureId, fixture.lowerLeg,
                    SupportMemberEnd::End, {treatment});
            }, "a track-only treatment at a plain node must be rejected");
            requireInvalid([&] {
                setSupportMemberEndConnection(
                    fixture.collection, fixture.structureId, fixture.lowerLeg,
                    SupportMemberEnd::Start, {treatment});
            }, "a track-only treatment at a foundation node must be rejected");
            requireInvalid([&] {
                setSupportMemberEndConnection(
                    fixture.collection, fixture.structureId, fixture.upperLeg,
                    SupportMemberEnd::Start, {treatment});
            }, "a track-only treatment at the opposite plain end must be rejected");
        }

        Fixture direct = createFixture();
        direct.collection.structures.front().members[1].endConnection =
            SupportMemberEndConnection{SupportMemberEndTreatment::Saddle};
        validateSupportCollection(direct.collection);

        Fixture conflicting = createFixture();
        conflicting.collection.structures.front().members[0].startConnection =
            SupportMemberEndConnection{SupportMemberEndTreatment::Clamp};
        requireInvalid([&] {
            validateSupportCollection(conflicting.collection);
        }, "hand-authored track-only metadata at a foundation node must be rejected");
    }

    void nodeContextRulesForFoundationTreatments()
    {
        Fixture fixture = createFixture();

        for (const SupportMemberEndTreatment treatment :
            {SupportMemberEndTreatment::Base,
             SupportMemberEndTreatment::Footing})
        {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.lowerLeg,
                SupportMemberEnd::Start, {treatment});
            validateSupportCollection(fixture.collection);

            clearSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.lowerLeg,
                SupportMemberEnd::Start);
            requireInvalid([&] {
                setSupportMemberEndConnection(
                    fixture.collection, fixture.structureId, fixture.lowerLeg,
                    SupportMemberEnd::End, {treatment});
            }, "a foundation treatment at a plain node must be rejected");
            requireInvalid([&] {
                setSupportMemberEndConnection(
                    fixture.collection, fixture.structureId, fixture.upperLeg,
                    SupportMemberEnd::End, {treatment});
            }, "a foundation treatment at a track-attached node must be rejected");
        }
    }

    void connectionMayCoexistWithEitherAnchorKind()
    {
        Fixture fixture = createFixture();

        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle});
        const SupportNode* top = nullptr;
        for (const SupportNode& node : fixture.collection.structures.front().nodes)
        {
            if (node.id == fixture.top)
            {
                top = &node;
            }
        }
        require(top != nullptr && top->trackAttachment.has_value(),
            "a track-attached node must keep its attachment beside a Saddle");
        validateSupportCollection(fixture.collection);

        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.lowerLeg,
            SupportMemberEnd::Start,
            {SupportMemberEndTreatment::EndCap});
        const SupportNode& base = fixture.collection.structures.front().nodes[0];
        require(base.foundation.has_value(),
            "a foundation node must keep its anchor metadata beside a "
            "generic member-end connection");
        validateSupportCollection(fixture.collection);
    }

    void genericTreatmentsAreUnconstrained()
    {
        Fixture fixture = createFixture();
        for (const SupportMemberEndTreatment treatment :
            {SupportMemberEndTreatment::MiteredCut,
             SupportMemberEndTreatment::EndCap,
             SupportMemberEndTreatment::Plate,
             SupportMemberEndTreatment::Flange,
             SupportMemberEndTreatment::Splice})
        {
            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.upperLeg,
                SupportMemberEnd::End, {treatment});
            validateSupportCollection(fixture.collection);

            setSupportMemberEndConnection(
                fixture.collection, fixture.structureId, fixture.lowerLeg,
                SupportMemberEnd::Start, {treatment});
            validateSupportCollection(fixture.collection);
        }
    }

    void connectorAssetIdentifiersNormalize()
    {
        require(normalizeSupportConnectorAssetIdentifier(
                    "assets://support/connectors/steel-foot.glb")
                == "assets://support/connectors/steel-foot.glb",
            "a canonical support asset must normalize unchanged");
        require(normalizeSupportConnectorAssetIdentifier(
                    "assets://support/connectors//./steel-foot.glb")
                == "assets://support/connectors/steel-foot.glb",
            "redundant journey components must be normalized");
        require(normalizeSupportConnectorAssetIdentifier(
                    "assets://support\\connectors\\steel-foot.glb")
                == "assets://support/connectors/steel-foot.glb",
            "backslash identifiers must be normalized");

        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier("");
        }, "an empty connector asset identifier must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier("support/foot.glb");
        }, "a missing assets:// scheme must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier("assets://track/foot.glb");
        }, "a connector asset outside assets://support/ must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier(
                "assets://support/../../outside.glb");
        }, "escaping the asset root must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier("/absolute/foot.glb");
        }, "an absolute connector asset path must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier(
                "assets://support/connectors/foot.obj");
        }, "a non-GLB connector asset must be rejected");
        requireInvalid([&] {
            normalizeSupportConnectorAssetIdentifier(
                "assets://support/connectors/foot");
        }, "an extensionless connector asset must be rejected");
    }

    void placementNormalizesOnMutation()
    {
        Fixture fixture = createFixture();
        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Plate,
             std::nullopt,
             SupportMemberEndPlacement{
                 {0.1, -0.2, 0.3},
                 {2.0, 0.0, 0.0, 0.0},
                 {1.5, 1.0, 1.0}}});

        const SupportMemberEndConnection& stored =
            *findMember(fixture.collection, fixture.upperLeg).endConnection;
        require(stored.localPlacement->position == glm::dvec3{0.1, -0.2, 0.3},
            "the placement position must round-trip");
        require(stored.localPlacement->orientation
                == glm::dquat{1.0, 0.0, 0.0, 0.0},
            "a positive scalar orientation must canonicalize to identity");
        require(stored.localPlacement->scale == glm::dvec3{1.5, 1.0, 1.0},
            "the placement scale must round-trip");

        setSupportMemberEndConnection(
            fixture.collection, fixture.structureId, fixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Plate,
             std::nullopt,
             SupportMemberEndPlacement{
                 {0.0, 0.0, 0.0},
                 {0.0, -1.0, 0.0, 0.0},
                 {1.0, 1.0, 1.0}}});
        const SupportMemberEndConnection& flipped =
            *findMember(fixture.collection, fixture.upperLeg).endConnection;
        require(flipped.localPlacement->orientation
                == glm::dquat{0.0, 1.0, 0.0, 0.0},
            "a negative X orientation must canonicalize to positive X");
    }

    void collectionValidationRequiresCanonicalStoredState()
    {
        const glm::dquat zeroOrientation{0.0, 0.0, 0.0, 0.0};
        const glm::dquat nonUnitOrientation{2.0, 0.0, 0.0, 0.0};

        const auto expectStoredStateRejected =
            [](Fixture fixture, const SupportMemberEndConnection& connection,
                const std::string_view message)
        {
            fixture.collection.structures.front()
                .members[1].endConnection = connection;
            requireInvalid([&] {
                validateSupportCollection(fixture.collection);
            }, message);
            requireInvalid([&] {
                validateSupportMemberEndConnection(connection);
            }, message);
        };

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle, std::nullopt,
             SupportMemberEndPlacement{{}, zeroOrientation}},
            "a zero stored orientation must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle, std::nullopt,
             SupportMemberEndPlacement{{}, nonUnitOrientation}},
            "a non-unit stored orientation must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle, std::nullopt,
             SupportMemberEndPlacement{{}, {0.0, 0.0, 0.0, -1.0}}},
            "a stored orientation with the non-canonical sign must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle, std::nullopt,
             SupportMemberEndPlacement{
                 {0.0, std::numeric_limits<double>::infinity(), 0.0}}},
            "a non-finite stored position must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle, std::nullopt,
             SupportMemberEndPlacement{{}, {}, {0.0, 1.0, 1.0}}},
            "a non-positive stored scale must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {static_cast<SupportMemberEndTreatment>(255)},
            "a malformed stored treatment must be rejected");

        expectStoredStateRejected(
            createFixture(),
            {SupportMemberEndTreatment::Saddle,
             StaticMeshAssetReference{
                 "assets://track/out-of-family.glb", false}},
            "a stored connector asset outside assets://support/ must be rejected");
    }

    void familyNeutralModelSuitsSteelAndTimber()
    {
        Fixture steel = createFixture();
        setSupportMemberEndConnection(
            steel.collection, steel.structureId, steel.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle,
             StaticMeshAssetReference{
                 "assets://support/connectors/steel-saddle.glb", false}});
        setSupportMemberEndConnection(
            steel.collection, steel.structureId, steel.lowerLeg,
            SupportMemberEnd::Start,
            {SupportMemberEndTreatment::Footing});
        validateSupportCollection(steel.collection);

        Fixture woodFixture = createFixture();
        for (SupportMember& member : woodFixture.collection.structures.front().members)
        {
            member.profile = timber();
        }
        setSupportMemberEndConnection(
            woodFixture.collection, woodFixture.structureId, woodFixture.upperLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::Plate,
             StaticMeshAssetReference{
                 "assets://support/connectors/timber-plate.glb", false},
             SupportMemberEndPlacement{{0.0, 0.0, 0.1}}});
        setSupportMemberEndConnection(
            woodFixture.collection, woodFixture.structureId, woodFixture.lowerLeg,
            SupportMemberEnd::End,
            {SupportMemberEndTreatment::MiteredCut, std::nullopt,
             SupportMemberEndPlacement{{0.0, 0.0, 0.05}}});
        setSupportMemberEndConnection(
            woodFixture.collection, woodFixture.structureId, woodFixture.lowerLeg,
            SupportMemberEnd::Start,
            {SupportMemberEndTreatment::Base});
        validateSupportCollection(woodFixture.collection);
        require(woodFixture.collection == woodFixture.collection,
            "both families must validate through the same model");
    }

    void authoredTrackWrappersPublishAndClearConnections()
    {
        const Fixture fixture = createFixture();
        AuthoredTrack track = createNewDocument();
        track.setSupports(fixture.collection);

        track.setSupportMemberEndConnection(
            fixture.structureId, fixture.upperLeg, SupportMemberEnd::End,
            {SupportMemberEndTreatment::Saddle,
             StaticMeshAssetReference{
                 "assets://support/connectors/authored-saddle.glb", false}});
        const SupportMember& member =
            findMember(track.supports(), fixture.upperLeg);
        require(member.endConnection.has_value()
                && member.endConnection->treatment
                    == SupportMemberEndTreatment::Saddle,
            "the AuthoredTrack wrapper must publish the connection");

        track.clearSupportMemberEndConnection(
            fixture.structureId, fixture.upperLeg, SupportMemberEnd::End);
        const SupportMember& cleared =
            findMember(track.supports(), fixture.upperLeg);
        require(!cleared.endConnection.has_value(),
            "the AuthoredTrack wrapper must clear the connection");
    }
}

int main()
{
    try
    {
        connectionEditsUseStableMemberIdentity();
        rejectedEndEditsLeaveStateUnchanged();
        deletingMemberRemovesOwnedConnections();
        noConnectionOwningIdsAreAllocated();
        nodeContextRulesForTrackOnlyTreatments();
        nodeContextRulesForFoundationTreatments();
        connectionMayCoexistWithEitherAnchorKind();
        genericTreatmentsAreUnconstrained();
        connectorAssetIdentifiersNormalize();
        placementNormalizesOnMutation();
        collectionValidationRequiresCanonicalStoredState();
        familyNeutralModelSuitsSteelAndTimber();
        authoredTrackWrappersPublishAndClearConnections();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Support connection test failure: " << exception.what()
                  << '\n';
        return 1;
    }
}