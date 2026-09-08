#include <quantum/editor/SupportNodeManipulation.hpp>

#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    [[nodiscard]] double comparisonTolerance(
        const double left, const double right) noexcept
    {
        return 1.0e-10 * std::max({1.0, std::abs(left), std::abs(right)});
    }

    [[nodiscard]] bool samePosition(
        const glm::dvec3& left, const glm::dvec3& right) noexcept
    {
        return left.x == right.x && left.y == right.y && left.z == right.z;
    }

    [[nodiscard]] const quantum::editor::SupportVisualizationNode*
    findNode(
        const quantum::editor::SupportVisualization& visualization,
        const quantum::editor::SupportSelection& selection) noexcept
    {
        const auto node = std::find_if(
            visualization.nodes.begin(), visualization.nodes.end(),
            [&selection](
                const quantum::editor::SupportVisualizationNode& value)
            {
                return value.selection == selection;
            });
        return node == visualization.nodes.end() ? nullptr : &*node;
    }

    [[nodiscard]] bool collapsesIncidentMember(
        const quantum::editor::SupportVisualization& visualization,
        const quantum::editor::SupportSelection& draggedNode,
        const glm::dvec3& candidatePosition) noexcept
    {
        for (const auto& member : visualization.members)
        {
            if (member.selection.structureId != draggedNode.structureId)
            {
                continue;
            }

            quantum::coaster::SupportElementId otherNodeId =
                quantum::coaster::invalidSupportElementId;
            if (member.startNodeId == draggedNode.elementId)
            {
                otherNodeId = member.endNodeId;
            }
            else if (member.endNodeId == draggedNode.elementId)
            {
                otherNodeId = member.startNodeId;
            }
            if (otherNodeId == quantum::coaster::invalidSupportElementId)
            {
                continue;
            }

            const quantum::editor::SupportSelection otherSelection{
                draggedNode.structureId,
                quantum::editor::SupportSelectionKind::Node,
                otherNodeId};
            const auto* const otherNode = findNode(
                visualization, otherSelection);
            if (otherNode != nullptr
                && samePosition(otherNode->position, candidatePosition))
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool better(
        const quantum::editor::SupportNodeSnapTarget& candidate,
        const quantum::editor::SupportNodeSnapTarget& current) noexcept
    {
        const double distanceTolerance = comparisonTolerance(
            candidate.distancePixels, current.distancePixels);
        if (candidate.distancePixels
            < current.distancePixels - distanceTolerance)
        {
            return true;
        }
        if (std::abs(candidate.distancePixels - current.distancePixels)
            > distanceTolerance)
        {
            return false;
        }

        const double depthTolerance = comparisonTolerance(
            candidate.depth, current.depth);
        if (candidate.depth < current.depth - depthTolerance)
        {
            return true;
        }
        if (std::abs(candidate.depth - current.depth) > depthTolerance)
        {
            return false;
        }

        return candidate.selection.structureId
                < current.selection.structureId
            || (candidate.selection.structureId
                    == current.selection.structureId
                && candidate.selection.elementId
                    < current.selection.elementId);
    }
}

namespace quantum::editor
{
    glm::dvec3 supportMoveWorldAxis(const SupportMoveAxis axis) noexcept
    {
        switch (axis)
        {
        case SupportMoveAxis::X: return {1.0, 0.0, 0.0};
        case SupportMoveAxis::Y: return {0.0, 1.0, 0.0};
        case SupportMoveAxis::Z: return {0.0, 0.0, 1.0};
        }
        return {1.0, 0.0, 0.0};
    }

    glm::dvec3 translateSupportNode(
        const glm::dvec3& initialPosition,
        const SupportMoveAxis axis,
        const double distance)
    {
        if (!std::isfinite(distance))
        {
            throw std::invalid_argument(
                "A support-node translation distance must be finite.");
        }
        return initialPosition + distance * supportMoveWorldAxis(axis);
    }

    bool supportNodeManipulationAvailable(
        const SupportVisualization& visualization,
        const std::optional<SupportSelection>& selection,
        const bool connectWaiting) noexcept
    {
        return !connectWaiting
            && selection.has_value()
            && selection->kind == SupportSelectionKind::Node
            && findNode(visualization, *selection) != nullptr;
    }

    std::optional<SupportNodeSnapTarget> pickSupportNodeSnapTarget(
        const SupportVisualization& visualization,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        const std::uint32_t viewportPixelWidth,
        const std::uint32_t viewportPixelHeight,
        const SupportSelection& draggedNode,
        const double radiusPixels)
    {
        if (draggedNode.kind != SupportSelectionKind::Node
            || viewportPixelWidth == 0 || viewportPixelHeight == 0
            || !std::isfinite(normalizedPointer.x)
            || !std::isfinite(normalizedPointer.y)
            || normalizedPointer.x < 0.0 || normalizedPointer.x > 1.0
            || normalizedPointer.y < 0.0 || normalizedPointer.y > 1.0
            || !std::isfinite(radiusPixels) || radiusPixels <= 0.0)
        {
            throw std::invalid_argument(
                "Support-node snapping requires a valid node and viewport.");
        }

        const double aspectRatio = static_cast<double>(viewportPixelWidth)
            / static_cast<double>(viewportPixelHeight);
        const glm::dvec2 pointerPixels{
            normalizedPointer.x * viewportPixelWidth,
            normalizedPointer.y * viewportPixelHeight};

        std::optional<SupportNodeSnapTarget> best;
        for (const SupportVisualizationNode& node : visualization.nodes)
        {
            if (node.selection == draggedNode
                || collapsesIncidentMember(
                    visualization, draggedNode, node.position))
            {
                continue;
            }

            const auto projected = projectViewportPoint(
                camera, node.position, aspectRatio);
            if (!projected.has_value()
                || projected->normalizedPosition.x < 0.0
                || projected->normalizedPosition.x > 1.0
                || projected->normalizedPosition.y < 0.0
                || projected->normalizedPosition.y > 1.0)
            {
                continue;
            }

            const glm::dvec2 nodePixels{
                projected->normalizedPosition.x * viewportPixelWidth,
                projected->normalizedPosition.y * viewportPixelHeight};
            const glm::dvec2 difference = nodePixels - pointerPixels;
            const double distance = std::sqrt(
                difference.x * difference.x + difference.y * difference.y);
            if (distance > radiusPixels)
            {
                continue;
            }

            const SupportNodeSnapTarget candidate{
                node.selection, node.position, distance, projected->depth};
            if (!best.has_value() || better(candidate, *best))
            {
                best = candidate;
            }
        }
        return best;
    }

    SupportPositionSnapResult resolveSupportPositionSnap(
        const glm::dvec3& unsnappedPosition,
        const SupportMoveAxis axis,
        const std::optional<SupportNodeSnapTarget>& nodeTarget,
        const bool nodeSnapEnabled,
        const bool groundSnapEnabled,
        const double groundThresholdWorldUnits)
    {
        if (!std::isfinite(unsnappedPosition.x)
            || !std::isfinite(unsnappedPosition.y)
            || !std::isfinite(unsnappedPosition.z)
            || !std::isfinite(groundThresholdWorldUnits)
            || groundThresholdWorldUnits < 0.0)
        {
            throw std::invalid_argument(
                "Support-node snap inputs must be finite.");
        }

        if (nodeSnapEnabled && nodeTarget.has_value())
        {
            return {
                nodeTarget->position,
                SupportPositionSnapKind::Node,
                nodeTarget->selection};
        }

        if (groundSnapEnabled && axis == SupportMoveAxis::Z
            && std::abs(unsnappedPosition.z) <= groundThresholdWorldUnits)
        {
            glm::dvec3 grounded = unsnappedPosition;
            grounded.z = 0.0;
            return {grounded, SupportPositionSnapKind::Ground, std::nullopt};
        }

        return {unsnappedPosition, SupportPositionSnapKind::None, std::nullopt};
    }
}
