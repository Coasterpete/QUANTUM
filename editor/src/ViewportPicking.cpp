#include <quantum/editor/ViewportPicking.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    struct RaySegmentProximity
    {
        double rayDistance = 0.0;
        double separation = 0.0;
    };

    [[nodiscard]] bool finiteVector(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    [[nodiscard]] RaySegmentProximity closestRaySegmentPoints(
        const quantum::editor::ViewportRay& ray,
        const glm::dvec3& segmentBegin,
        const glm::dvec3& segmentEnd)
    {
        const glm::dvec3 segmentDirection = segmentEnd - segmentBegin;
        const glm::dvec3 originOffset = ray.origin - segmentBegin;
        const double rayLengthSquared = glm::dot(
            ray.direction,
            ray.direction
        );
        const double segmentLengthSquared = glm::dot(
            segmentDirection,
            segmentDirection
        );
        const double directionsDot = glm::dot(
            ray.direction,
            segmentDirection
        );
        const double rayOffsetDot = glm::dot(
            ray.direction,
            originOffset
        );

        double segmentParameter = 0.0;
        if (segmentLengthSquared > std::numeric_limits<double>::epsilon())
        {
            const double segmentOffsetDot = glm::dot(
                segmentDirection,
                originOffset
            );
            const double denominator = rayLengthSquared
                * segmentLengthSquared
                - directionsDot * directionsDot;

            if (denominator > std::numeric_limits<double>::epsilon())
            {
                segmentParameter = std::clamp(
                    (rayLengthSquared * segmentOffsetDot
                        - directionsDot * rayOffsetDot)
                        / denominator,
                    0.0,
                    1.0
                );
            }
            else
            {
                segmentParameter = std::clamp(
                    segmentOffsetDot / segmentLengthSquared,
                    0.0,
                    1.0
                );
            }
        }

        double rayParameter = (
            directionsDot * segmentParameter - rayOffsetDot)
            / rayLengthSquared;

        if (rayParameter < 0.0)
        {
            rayParameter = 0.0;
            segmentParameter = segmentLengthSquared
                    > std::numeric_limits<double>::epsilon()
                ? std::clamp(
                    glm::dot(segmentDirection, originOffset)
                        / segmentLengthSquared,
                    0.0,
                    1.0
                )
                : 0.0;
        }

        const glm::dvec3 rayPoint = ray.origin
            + rayParameter * ray.direction;
        const glm::dvec3 segmentPoint = segmentBegin
            + segmentParameter * segmentDirection;
        return {rayParameter, glm::length(rayPoint - segmentPoint)};
    }

    [[nodiscard]] glm::dvec3 vertexPosition(
        const quantum::renderer::LineVertex& vertex) noexcept
    {
        return {vertex.x, vertex.y, vertex.z};
    }

    [[nodiscard]] double comparisonTolerance(
        const double left,
        const double right) noexcept
    {
        return 1.0e-10 * std::max({1.0, std::abs(left), std::abs(right)});
    }
}

namespace quantum::editor
{
    std::optional<ViewportPickResult> pickViewportSection(
        const CenterlineVisualization& visualization,
        const ViewportCamera& camera,
        const ViewportRay& ray,
        const std::uint32_t viewportPixelHeight,
        const std::uint32_t visibleCurveMask,
        const double tolerancePixels)
    {
        if (viewportPixelHeight == 0
            || !std::isfinite(tolerancePixels)
            || tolerancePixels <= 0.0
            || !finiteVector(ray.origin)
            || !finiteVector(ray.direction)
            || glm::dot(ray.direction, ray.direction)
                <= std::numeric_limits<double>::epsilon())
        {
            throw std::invalid_argument(
                "Viewport picking requires a valid ray, height, and tolerance."
            );
        }

        if (visualization.verticesPerCurve == 0
            || visualization.sectionSlices.empty()
            || visibleCurveMask == 0)
        {
            return std::nullopt;
        }

        const std::size_t expectedVertexCount =
            static_cast<std::size_t>(visualization.verticesPerCurve)
            * renderer::viewportCurveCount;
        if (visualization.vertices.size() != expectedVertexCount)
        {
            throw std::invalid_argument(
                "Viewport picking requires four complete reference-curve runs."
            );
        }

        const ViewportRay normalizedRay{
            ray.origin,
            glm::normalize(ray.direction)
        };
        std::optional<ViewportPickResult> best;

        for (std::size_t sectionIndex = 0;
            sectionIndex < visualization.sectionSlices.size();
            ++sectionIndex)
        {
            const CenterlineSectionSlice& slice =
                visualization.sectionSlices[sectionIndex];
            if (slice.vertexCount < 2
                || slice.firstVertex > visualization.verticesPerCurve
                || slice.vertexCount
                    > visualization.verticesPerCurve - slice.firstVertex)
            {
                throw std::invalid_argument(
                    "Viewport picking encountered an invalid section slice."
                );
            }

            const std::uint32_t sectionEnd = slice.firstVertex
                + slice.vertexCount;
            for (std::uint32_t curve = 0;
                curve < renderer::viewportCurveCount;
                ++curve)
            {
                if ((visibleCurveMask & (1u << curve)) == 0)
                {
                    continue;
                }

                const std::uint32_t curveOffset = curve
                    * visualization.verticesPerCurve;
                for (std::uint32_t firstVertex = slice.firstVertex;
                    firstVertex + 1 < sectionEnd;
                    firstVertex += 2)
                {
                    const RaySegmentProximity proximity =
                        closestRaySegmentPoints(
                            normalizedRay,
                            vertexPosition(visualization.vertices[
                                curveOffset + firstVertex]),
                            vertexPosition(visualization.vertices[
                                curveOffset + firstVertex + 1])
                        );

                    const double scaleDistance =
                        camera.projection() == ViewportProjection::Perspective
                        ? proximity.rayDistance
                        : camera.distance();
                    const double worldTolerance = tolerancePixels
                        * (2.0 * scaleDistance
                            * std::tan(0.5
                                * camera.verticalFieldOfView()))
                        / static_cast<double>(viewportPixelHeight);

                    if (proximity.rayDistance <= 0.0
                        || proximity.separation > worldTolerance)
                    {
                        continue;
                    }

                    const ViewportPickResult candidate{
                        sectionIndex,
                        curve,
                        firstVertex,
                        proximity.rayDistance,
                        proximity.separation
                    };

                    if (!best.has_value())
                    {
                        best = candidate;
                        continue;
                    }

                    const double depthTolerance = comparisonTolerance(
                        candidate.rayDistance,
                        best->rayDistance
                    );
                    const double distanceTolerance = comparisonTolerance(
                        candidate.distanceToSegment,
                        best->distanceToSegment
                    );
                    const bool nearer = candidate.rayDistance
                        < best->rayDistance - depthTolerance;
                    const bool sameDepth = std::abs(
                        candidate.rayDistance - best->rayDistance)
                        <= depthTolerance;
                    const bool closerAtSameDepth = sameDepth
                        && candidate.distanceToSegment
                            < best->distanceToSegment - distanceTolerance;
                    const bool exactGeometryTie = sameDepth
                        && std::abs(candidate.distanceToSegment
                            - best->distanceToSegment)
                            <= distanceTolerance;
                    const bool deterministicTieWinner = exactGeometryTie
                        && (candidate.sectionIndex < best->sectionIndex
                            || (candidate.sectionIndex == best->sectionIndex
                                && (candidate.curveIndex < best->curveIndex
                                    || (candidate.curveIndex
                                            == best->curveIndex
                                        && candidate.segmentFirstVertex
                                            < best->segmentFirstVertex))));

                    if (nearer || closerAtSameDepth
                        || deterministicTieWinner)
                    {
                        best = candidate;
                    }
                }
            }
        }

        return best;
    }
}
