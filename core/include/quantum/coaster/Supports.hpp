#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
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

    struct SupportNode
    {
        SupportElementId id = invalidSupportElementId;
        glm::dvec3 position{0.0, 0.0, 0.0};

        [[nodiscard]] friend bool operator==(
            const SupportNode&, const SupportNode&) = default;
    };

    struct SupportMember
    {
        SupportElementId id = invalidSupportElementId;
        SupportElementId startNodeId = invalidSupportElementId;
        SupportElementId endNodeId = invalidSupportElementId;
        SupportMemberProfile profile;

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

    // Geometric/document consistency only. This does not perform loads,
    // stress, buckling, foundation, or code-compliance analysis.
    void validateSupportMemberProfile(const SupportMemberProfile& profile);
    void validateSupportCollection(const SupportCollection& collection);
}
