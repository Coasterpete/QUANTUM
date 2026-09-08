#include <quantum/editor/SupportVisualization.hpp>

#include <algorithm>
#include <array>
#include <ranges>
#include <stdexcept>

namespace quantum::editor
{
    SupportVisualization createSupportVisualization(
        const coaster::SupportCollection& supports)
    {
        coaster::validateSupportCollection(supports);

        SupportVisualization visualization;
        std::size_t nodeCount = 0;
        std::size_t memberCount = 0;
        for (const coaster::SupportStructure& structure : supports.structures)
        {
            nodeCount += structure.nodes.size();
            memberCount += structure.members.size();
        }
        visualization.nodes.reserve(nodeCount);
        visualization.members.reserve(memberCount);
        visualization.memberVertices.reserve(memberCount * 2);

        constexpr std::array<float, 4> memberColor{
            0.32F, 0.66F, 0.78F, 1.0F};
        for (const coaster::SupportStructure& structure : supports.structures)
        {
            for (const coaster::SupportNode& node : structure.nodes)
            {
                visualization.nodes.push_back({
                    {structure.id, SupportSelectionKind::Node, node.id},
                    node.position});
            }

            for (const coaster::SupportMember& member : structure.members)
            {
                const auto start = std::find_if(
                    structure.nodes.begin(), structure.nodes.end(),
                    [&member](const coaster::SupportNode& node)
                    {
                        return node.id == member.startNodeId;
                    });
                const auto end = std::find_if(
                    structure.nodes.begin(), structure.nodes.end(),
                    [&member](const coaster::SupportNode& node)
                    {
                        return node.id == member.endNodeId;
                    });
                if (start == structure.nodes.end()
                    || end == structure.nodes.end())
                {
                    throw std::logic_error(
                        "Validated support member endpoints were not found.");
                }

                visualization.members.push_back({
                    {structure.id, SupportSelectionKind::Member, member.id},
                    member.startNodeId,
                    member.endNodeId,
                    start->position,
                    end->position});
                visualization.memberVertices.push_back({
                    static_cast<float>(start->position.x),
                    static_cast<float>(start->position.y),
                    static_cast<float>(start->position.z),
                    memberColor});
                visualization.memberVertices.push_back({
                    static_cast<float>(end->position.x),
                    static_cast<float>(end->position.y),
                    static_cast<float>(end->position.z),
                    memberColor});
            }
        }

        return visualization;
    }

    bool supportSelectionExists(
        const coaster::SupportCollection& supports,
        const SupportSelection& selection) noexcept
    {
        const auto structure = std::find_if(
            supports.structures.begin(), supports.structures.end(),
            [&selection](const coaster::SupportStructure& value)
            {
                return value.id == selection.structureId;
            });
        if (structure == supports.structures.end())
        {
            return false;
        }

        if (selection.kind == SupportSelectionKind::Node)
        {
            return std::ranges::any_of(
                structure->nodes,
                [&selection](const coaster::SupportNode& node)
                {
                    return node.id == selection.elementId;
                });
        }
        return std::ranges::any_of(
            structure->members,
            [&selection](const coaster::SupportMember& member)
            {
                return member.id == selection.elementId;
            });
    }
}
