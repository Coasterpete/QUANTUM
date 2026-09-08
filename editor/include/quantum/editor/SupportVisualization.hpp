#pragma once

#include <quantum/coaster/Supports.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace quantum::editor
{
    enum class SupportSelectionKind
    {
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
}
