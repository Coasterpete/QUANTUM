#include <quantum/editor/SupportPicking.hpp>

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

    [[nodiscard]] bool better(
        const quantum::editor::SupportPickResult& candidate,
        const quantum::editor::SupportPickResult& current) noexcept
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

    struct SegmentDistance
    {
        double distance = 0.0;
        double parameter = 0.0;
    };

    [[nodiscard]] SegmentDistance pointSegmentDistance(
        const glm::dvec2& point,
        const glm::dvec2& begin,
        const glm::dvec2& end) noexcept
    {
        const glm::dvec2 segment = end - begin;
        const double lengthSquared = segment.x * segment.x
            + segment.y * segment.y;
        const double parameter = lengthSquared
                > std::numeric_limits<double>::epsilon()
            ? std::clamp(
                ((point.x - begin.x) * segment.x
                    + (point.y - begin.y) * segment.y) / lengthSquared,
                0.0,
                1.0)
            : 0.0;
        const glm::dvec2 closest = begin + parameter * segment;
        const glm::dvec2 difference = point - closest;
        return {
            std::sqrt(difference.x * difference.x
                + difference.y * difference.y),
            parameter};
    }
}

namespace quantum::editor
{
    std::optional<SupportPickResult> pickSupport(
        const SupportVisualization& visualization,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        const std::uint32_t viewportPixelWidth,
        const std::uint32_t viewportPixelHeight,
        const double nodeRadiusPixels,
        const double memberTolerancePixels)
    {
        if (viewportPixelWidth == 0 || viewportPixelHeight == 0
            || !std::isfinite(normalizedPointer.x)
            || !std::isfinite(normalizedPointer.y)
            || normalizedPointer.x < 0.0 || normalizedPointer.x > 1.0
            || normalizedPointer.y < 0.0 || normalizedPointer.y > 1.0
            || !std::isfinite(nodeRadiusPixels) || nodeRadiusPixels <= 0.0
            || !std::isfinite(memberTolerancePixels)
            || memberTolerancePixels <= 0.0)
        {
            throw std::invalid_argument(
                "Support picking requires a valid viewport and tolerance.");
        }

        const double aspectRatio = static_cast<double>(viewportPixelWidth)
            / static_cast<double>(viewportPixelHeight);
        const glm::dvec2 pointerPixels{
            normalizedPointer.x * viewportPixelWidth,
            normalizedPointer.y * viewportPixelHeight};

        std::optional<SupportPickResult> bestNode;
        for (const SupportVisualizationNode& node : visualization.nodes)
        {
            const auto projected = projectViewportPoint(
                camera, node.position, aspectRatio);
            if (!projected.has_value())
            {
                continue;
            }
            const glm::dvec2 nodePixels{
                projected->normalizedPosition.x * viewportPixelWidth,
                projected->normalizedPosition.y * viewportPixelHeight};
            const glm::dvec2 difference = nodePixels - pointerPixels;
            const double distance = std::sqrt(
                difference.x * difference.x + difference.y * difference.y);
            if (distance > nodeRadiusPixels)
            {
                continue;
            }
            const SupportPickResult candidate{
                node.selection, distance, projected->depth};
            if (!bestNode.has_value() || better(candidate, *bestNode))
            {
                bestNode = candidate;
            }
        }
        if (bestNode.has_value())
        {
            return bestNode;
        }

        std::optional<SupportPickResult> bestMember;
        for (const SupportVisualizationMember& member : visualization.members)
        {
            const auto start = projectViewportPoint(
                camera, member.startPosition, aspectRatio);
            const auto end = projectViewportPoint(
                camera, member.endPosition, aspectRatio);
            if (!start.has_value() || !end.has_value())
            {
                continue;
            }
            const glm::dvec2 startPixels{
                start->normalizedPosition.x * viewportPixelWidth,
                start->normalizedPosition.y * viewportPixelHeight};
            const glm::dvec2 endPixels{
                end->normalizedPosition.x * viewportPixelWidth,
                end->normalizedPosition.y * viewportPixelHeight};
            const SegmentDistance proximity = pointSegmentDistance(
                pointerPixels, startPixels, endPixels);
            if (proximity.distance > memberTolerancePixels)
            {
                continue;
            }
            const SupportPickResult candidate{
                member.selection,
                proximity.distance,
                start->depth + proximity.parameter * (end->depth - start->depth)};
            if (!bestMember.has_value() || better(candidate, *bestMember))
            {
                bestMember = candidate;
            }
        }
        return bestMember;
    }
}
