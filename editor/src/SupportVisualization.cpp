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

        if (selection.kind == SupportSelectionKind::Structure)
        {
            return true;
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

    std::string_view supportConnectCheckMessage(
        const SupportConnectCheck check) noexcept
    {
        switch (check)
        {
        case SupportConnectCheck::Ok:
            return "Nodes can be connected.";
        case SupportConnectCheck::UnknownStructure:
            return "Select a valid support structure.";
        case SupportConnectCheck::UnknownNode:
            return "The target node no longer exists.";
        case SupportConnectCheck::SameNode:
            return "A member must connect two different nodes.";
        case SupportConnectCheck::DifferentStructure:
            return "A member must connect nodes in the same structure.";
        case SupportConnectCheck::AlreadyConnected:
            return "A member already connects this pair of nodes.";
        }
        return "Cannot connect these nodes.";
    }

    SupportConnectCheck checkSupportNodeConnection(
        const coaster::SupportCollection& supports,
        const coaster::SupportStructureId structureId,
        const coaster::SupportElementId firstNodeId,
        const coaster::SupportElementId secondNodeId) noexcept
    {
        const auto structure = std::find_if(
            supports.structures.begin(), supports.structures.end(),
            [structureId](const coaster::SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == supports.structures.end())
        {
            return SupportConnectCheck::UnknownStructure;
        }

        const auto nodeInStructure =
            [](const coaster::SupportStructure& target,
                const coaster::SupportElementId nodeId)
        {
            return std::ranges::any_of(
                target.nodes,
                [nodeId](const coaster::SupportNode& node)
                {
                    return node.id == nodeId;
                });
        };
        const auto nodeInOtherStructure =
            [&supports, structure, nodeInStructure](
                const coaster::SupportElementId nodeId)
        {
            return std::ranges::any_of(
                supports.structures,
                [structure, nodeId, nodeInStructure](
                    const coaster::SupportStructure& value)
                {
                    return &value != &*structure
                        && nodeInStructure(value, nodeId);
                });
        };

        if (!nodeInStructure(*structure, firstNodeId))
        {
            return nodeInOtherStructure(firstNodeId)
                ? SupportConnectCheck::DifferentStructure
                : SupportConnectCheck::UnknownNode;
        }
        if (!nodeInStructure(*structure, secondNodeId))
        {
            return nodeInOtherStructure(secondNodeId)
                ? SupportConnectCheck::DifferentStructure
                : SupportConnectCheck::UnknownNode;
        }
        if (firstNodeId == secondNodeId)
        {
            return SupportConnectCheck::SameNode;
        }

        const auto connectedPair = std::ranges::any_of(
            structure->members,
            [firstNodeId, secondNodeId](
                const coaster::SupportMember& member)
            {
                return std::min(member.startNodeId, member.endNodeId)
                        == std::min(firstNodeId, secondNodeId)
                    && std::max(member.startNodeId, member.endNodeId)
                        == std::max(firstNodeId, secondNodeId);
            });
        return connectedPair
            ? SupportConnectCheck::AlreadyConnected
            : SupportConnectCheck::Ok;
    }
}
