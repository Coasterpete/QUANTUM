#include <quantum/editor/SupportPicking.hpp>

#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <glm/geometric.hpp>

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

    [[nodiscard]] std::optional<double> solidHit(
        const quantum::editor::SupportVisualizationMember& member,
        const quantum::editor::ViewportRay& ray)
    {
        const auto& frame = member.frame;
        if (frame.length <= quantum::coaster::minimumSupportMemberLength) return std::nullopt;
        const glm::dvec3 offset = ray.origin - frame.origin;
        const glm::dvec3 origin{glm::dot(offset, frame.axisX),
            glm::dot(offset, frame.axisY), glm::dot(offset, frame.axisZ)};
        const glm::dvec3 direction{glm::dot(ray.direction, frame.axisX),
            glm::dot(ray.direction, frame.axisY), glm::dot(ray.direction, frame.axisZ)};
        double near = 0.0;
        double far = std::numeric_limits<double>::infinity();
        const auto slab = [&](const int axis, const double half) -> bool
        {
            if (std::abs(direction[axis]) <= 1e-12) return std::abs(origin[axis]) <= half;
            const double a = (-half - origin[axis]) / direction[axis];
            const double b = (half - origin[axis]) / direction[axis];
            near = std::max(near, std::min(a, b));
            far = std::min(far, std::max(a, b));
            return near <= far;
        };
        if (!slab(0, frame.length * 0.5)) return std::nullopt;
        if (member.profile.shape == quantum::coaster::SupportMemberProfileShape::Rectangular)
        {
            if (!slab(1, member.profile.outerDimensions.x * 0.5)
                || !slab(2, member.profile.outerDimensions.y * 0.5)) return std::nullopt;
        }
        else
        {
            const double radius = member.profile.outerDimensions.x * 0.5;
            const double a = direction.y * direction.y + direction.z * direction.z;
            const double b = origin.y * direction.y + origin.z * direction.z;
            const double c = origin.y * origin.y + origin.z * origin.z - radius * radius;
            if (a <= 1e-24)
            {
                if (c > 0.0) return std::nullopt;
            }
            else
            {
                const double discriminant = b * b - a * c;
                if (discriminant < 0.0) return std::nullopt;
                near = std::max(near, (-b - std::sqrt(discriminant)) / a);
                far = std::min(far, (-b + std::sqrt(discriminant)) / a);
                if (near > far) return std::nullopt;
            }
        }
        return near;
    }

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
        const auto ray = camera.viewportRay(normalizedPointer.x, normalizedPointer.y, aspectRatio);
        for (const SupportVisualizationMember& member : visualization.members)
        {
            if (const auto hit = solidHit(member, ray))
            {
                const auto point = projectViewportPoint(camera, ray.origin + *hit * ray.direction, aspectRatio);
                if (point)
                {
                    const SupportPickResult candidate{member.selection, 0.0, point->depth};
                    if (!bestMember || better(candidate, *bestMember)) bestMember = candidate;
                    continue;
                }
            }
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
