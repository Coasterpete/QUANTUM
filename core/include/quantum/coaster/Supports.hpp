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

    // Allocation advances the persisted counter and never searches for or
    // reuses gaps. Malformed counters and exhausted ID spaces are rejected.
    [[nodiscard]] SupportStructureId allocateSupportStructureId(
        SupportCollection& collection);
    [[nodiscard]] SupportElementId allocateSupportElementId(
        SupportStructure& structure);

    // Geometric/document consistency only. This does not perform loads,
    // stress, buckling, foundation, or code-compliance analysis.
    void validateSupportMemberProfile(const SupportMemberProfile& profile);
    void validateSupportCollection(const SupportCollection& collection);
}
