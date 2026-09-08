#pragma once

#include <quantum/coaster/Supports.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace quantum::editor
{
    enum class SupportSelectionKind
    {
        Structure,
        Node,
        Member
    };

    struct SupportSelection
    {
        coaster::SupportStructureId structureId =
            coaster::invalidSupportStructureId;
        SupportSelectionKind kind = SupportSelectionKind::Node;
        coaster::SupportElementId elementId =
            coaster::invalidSupportElementId;

        [[nodiscard]] friend bool operator==(
            const SupportSelection&,
            const SupportSelection&) = default;
    };

    struct SupportVisualizationNode
    {
        SupportSelection selection;
        glm::dvec3 position{0.0};
    };

    struct SupportVisualizationMember
    {
        SupportSelection selection;
        coaster::SupportElementId startNodeId =
            coaster::invalidSupportElementId;
        coaster::SupportElementId endNodeId =
            coaster::invalidSupportElementId;
        glm::dvec3 startPosition{0.0};
        glm::dvec3 endPosition{0.0};
    };

    // Editor-owned projection of committed support graph data. Double-
    // precision metadata remains authoritative for picking; float vertices
    // are only the renderer upload stream.
    struct SupportVisualization
    {
        std::vector<renderer::LineVertex> memberVertices;
        std::vector<SupportVisualizationNode> nodes;
        std::vector<SupportVisualizationMember> members;
    };

    [[nodiscard]] SupportVisualization createSupportVisualization(
        const coaster::SupportCollection& supports);

    [[nodiscard]] bool supportSelectionExists(
        const coaster::SupportCollection& supports,
        const SupportSelection& selection) noexcept;

    // UI-side readiness check for the Connect Nodes workflow. Core
    // createSupportMember remains the authoritative gate; this query gives
    // the editor an exception-free reason to present before a commit.
    enum class SupportConnectCheck : std::uint8_t
    {
        Ok,
        UnknownStructure,
        UnknownNode,
        SameNode,
        DifferentStructure,
        AlreadyConnected
    };

    [[nodiscard]] std::string_view supportConnectCheckMessage(
        SupportConnectCheck check) noexcept;

    // Returns Ok when a member between the two nodes may be authored, and a
    // concrete reason otherwise. Both endpoints must exist in the same target
    // structure, be distinct, and not already be connected by an existing
    // member across the unordered pair.
    [[nodiscard]] SupportConnectCheck checkSupportNodeConnection(
        const coaster::SupportCollection& supports,
        coaster::SupportStructureId structureId,
        coaster::SupportElementId firstNodeId,
        coaster::SupportElementId secondNodeId) noexcept;
}
