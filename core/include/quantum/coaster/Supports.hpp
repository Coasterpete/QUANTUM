#pragma once

#include <quantum/coaster/StaticMeshAsset.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace quantum::coaster
{
    // Structure identity is document-wide. Node and member identities share
    // one structure-local element namespace so every support element remains
    // unambiguous when paired with its owning structure ID.
    using SupportStructureId = std::uint32_t;
    using SupportElementId = std::uint32_t;

    inline constexpr SupportStructureId invalidSupportStructureId = 0;
    inline constexpr SupportElementId invalidSupportElementId = 0;

    enum class SupportMemberProfileShape : std::uint8_t
    {
        Circular,
        Rectangular
    };

    // The two directed ends of a member. Connections are authored per end and
    // remain independent of the node at that end.
    enum class SupportMemberEnd : std::uint8_t
    {
        Start,
        End
    };

    // The fixed set of member-end treatments. Serialization matches these
    // exact names (never numeric codes) so moving JSON between documents
    // cannot change treatment semantics.
    enum class SupportMemberEndTreatment : std::uint8_t
    {
        MiteredCut,
        EndCap,
        Plate,
        Flange,
        // M2B records an unpaired splice marker only; splice partners and
        // splice pairing are a later milestone.
        Splice,
        Saddle,
        Clamp,
        Base,
        Footing
    };

    // Optional logical placement of the member-end connector mesh relative to
    // the member's local frame at the affected end. When absent, the renderer
    // chooses the connector pose from support modeling conventions.
    struct SupportMemberEndPlacement
    {
        glm::dvec3 position{0.0};
        // Canonical unit quaternion (w, x, y, z), same convention as
        // AuthoredStartPose::orientation.
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 scale{1.0};

        [[nodiscard]] friend bool operator==(
            const SupportMemberEndPlacement&,
            const SupportMemberEndPlacement&) = default;
    };

    // Persistent authored connection metadata for one member end. No separate
    // connection ID exists: identity derives from (structureId, memberId,
    // SupportMemberEnd), so deleting a member removes both end connections.
    struct SupportMemberEndConnection
    {
        SupportMemberEndTreatment treatment =
            SupportMemberEndTreatment::MiteredCut;
        std::optional<StaticMeshAssetReference> asset;
        std::optional<SupportMemberEndPlacement> localPlacement;

        [[nodiscard]] friend bool operator==(
            const SupportMemberEndConnection&,
            const SupportMemberEndConnection&) = default;
    };

    // Cross-section geometry only. A zero wall thickness denotes a solid
    // member; a positive thickness denotes a hollow tube or box section.
    struct SupportMemberProfile
    {
        SupportMemberProfileShape shape =
            SupportMemberProfileShape::Circular;
        glm::dvec2 outerDimensions{0.1, 0.1};
        double wallThickness = 0.0;

        [[nodiscard]] friend bool operator==(
            const SupportMemberProfile&,
            const SupportMemberProfile&) = default;
    };

    // Authored placement on the document's single ordered track path.
    // Station and offsets use Core coordinate units, not SI metres. The
    // resolved world position/frame are derived from current track geometry
    // and are deliberately not persisted here.
    struct TrackAttachment
    {
        double station = 0.0;
        double lateralOffset = 0.0;
        double verticalOffset = 0.0;

        [[nodiscard]] friend bool operator==(
            const TrackAttachment&,
            const TrackAttachment&) = default;
    };

    // Marks a node as an explicitly authored foundation/ground anchor. Its
    // SupportNode::position remains authoritative; terrain projection and
    // terrain-follow metadata are future additive concerns.
    struct Foundation
    {
        [[nodiscard]] friend bool operator==(
            const Foundation&,
            const Foundation&) = default;
    };

    struct SupportNode
    {
        SupportElementId id = invalidSupportElementId;
        glm::dvec3 position{0.0, 0.0, 0.0};
        std::optional<TrackAttachment> trackAttachment;
        std::optional<Foundation> foundation;

        [[nodiscard]] friend bool operator==(
            const SupportNode&, const SupportNode&) = default;
    };

    struct SupportMember
    {
        SupportElementId id = invalidSupportElementId;
        SupportElementId startNodeId = invalidSupportElementId;
        SupportElementId endNodeId = invalidSupportElementId;
        SupportMemberProfile profile;
        std::optional<SupportMemberEndConnection> startConnection;
        std::optional<SupportMemberEndConnection> endConnection;

        [[nodiscard]] friend bool operator==(
            const SupportMember&, const SupportMember&) = default;
    };

    struct SupportStructure
    {
        SupportStructureId id = invalidSupportStructureId;
        std::string name;
        std::vector<SupportNode> nodes;
        std::vector<SupportMember> members;
        SupportElementId nextElementId = 1;

        [[nodiscard]] friend bool operator==(
            const SupportStructure&, const SupportStructure&) = default;
    };

    struct SupportCollection
    {
        std::vector<SupportStructure> structures;
        SupportStructureId nextStructureId = 1;

        [[nodiscard]] bool empty() const noexcept
        {
            return structures.empty();
        }

        [[nodiscard]] friend bool operator==(
            const SupportCollection&, const SupportCollection&) = default;
    };

    // The profile created for manually connected members. A simple valid
    // solid round section; the M1A manual toolset authoring does not pick
    // materials or manufacturer-specific sections yet.
    [[nodiscard]] SupportMemberProfile defaultSupportMemberProfile() noexcept;

    // Allocation advances the persisted counter and never searches for or
    // reuses gaps. Malformed counters and exhausted ID spaces are rejected.
    [[nodiscard]] SupportStructureId allocateSupportStructureId(
        SupportCollection& collection);
    [[nodiscard]] SupportElementId allocateSupportElementId(
        SupportStructure& structure);

    // Invariant-preserving topology editing. Every mutation operates on a
    // live SupportCollection, owns allocator behavior (IDs are never
    // caller-supplied), validates its inputs before changing anything, and
    // finishes by validating the complete collection. On a rejected mutation
    // the collection is left exactly unchanged; deleted IDs are never reused
    // and allocator counters never roll back.
    //
    // Creates an empty structure, allocating its nextSupportStructureId and a
    // deterministic default name ("Support N") when name is empty.
    // Throws std::invalid_argument when the structure ID space is exhausted.
    [[nodiscard]] SupportStructureId createSupportStructure(
        SupportCollection& collection,
        std::string name = {});
    // Removes one structure and its entire owned node/member graph.
    // Throws std::invalid_argument for an unknown structure ID.
    void removeSupportStructure(
        SupportCollection& collection,
        SupportStructureId structureId);
    // Adds one node to the target structure, allocating its stable element ID.
    // Throws std::invalid_argument for an unknown structure or a non-finite
    // position.
    [[nodiscard]] SupportElementId createSupportNode(
        SupportCollection& collection,
        SupportStructureId structureId,
        const glm::dvec3& position);
    // Refuses to remove a node referenced by any member: the connected members
    // must be deleted first. Throws std::invalid_argument for an unknown
    // node or a node still referenced by a member.
    void removeSupportNode(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    // Connects two distinct nodes of the same structure. Rejects another
    // member across the same unordered endpoint pair, so A->B and B->A count
    // as the same connection and coincident duplicate members cannot be
    // authored. Uses defaultSupportMemberProfile() when no profile is given.
    // Throws std::invalid_argument for missing endpoints, a self connection,
    // or an existing member across the same pair.
    [[nodiscard]] SupportElementId createSupportMember(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId startNodeId,
        SupportElementId endNodeId,
        const SupportMemberProfile& profile = defaultSupportMemberProfile());
    // Removes exactly one member, preserving every node. Throws
    // std::invalid_argument for an unknown member ID.
    void removeSupportMember(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId);

    // Node anchor metadata uses the node's existing stable identity. Setting
    // one anchor kind replaces no other state and rejects a node already
    // carrying the mutually exclusive anchor kind.
    void setSupportTrackAttachment(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId,
        const TrackAttachment& attachment);
    void clearSupportTrackAttachment(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    void setSupportFoundation(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);
    void clearSupportFoundation(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId nodeId);

    // Member-end connection metadata uses the member's existing stable
    // identity; no separate connection IDs are allocated. Deleting the member
    // deletes both end connections. The two ends are independent and never
    // merge with the node's anchor metadata. The supplied connection is
    // normalized (including its placement quaternion) before publication.
    // Throws std::invalid_argument for unknown members, malformed connection
    // values, or a Saddle/Clamp treatment at a node without a track
    // attachment / Base/Footing treatment at a node without a foundation.
    void setSupportMemberEndConnection(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId,
        SupportMemberEnd end,
        const SupportMemberEndConnection& connection);
    void clearSupportMemberEndConnection(
        SupportCollection& collection,
        SupportStructureId structureId,
        SupportElementId memberId,
        SupportMemberEnd end);

    // Returns a canonical package-relative identifier for a connector asset
    // below assets://support/. File-backed connectors must use the .glb
    // extension.
    [[nodiscard]] std::string normalizeSupportConnectorAssetIdentifier(
        std::string_view identifier);

    // Normalizes member-end connection state so that a stored placement
    // quaternion is finite, unit, and sign-canonical per Core conventions.
    // Throws std::invalid_argument for malformed values.
    [[nodiscard]] SupportMemberEndConnection normalizeSupportMemberEndConnection(
        const SupportMemberEndConnection& connection);
    void validateSupportMemberEndConnection(
        const SupportMemberEndConnection& connection);

    // Geometric/document consistency only. This does not perform loads,
    // stress, buckling, foundation, or code-compliance analysis.
    void validateSupportMemberProfile(const SupportMemberProfile& profile);
    void validateSupportCollection(const SupportCollection& collection);
}
