#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    [[nodiscard]] double comparisonTolerance(
        const double left,
        const double right) noexcept
    {
        return 1.0e-10 * std::max({1.0, std::abs(left), std::abs(right)});
    }

    [[nodiscard]] bool finiteVector(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }
}

namespace quantum::editor
{
    ViewportTrackSelection selectionForViewportTrackRegion(
        const std::size_t regionIndex,
        const std::size_t regionCount)
    {
        if (regionCount == 0 || regionIndex >= regionCount)
        {
            throw std::out_of_range(
                "A viewport region selection must name an authored region."
            );
        }

        return {regionIndex, regionIndex};
    }

    ViewportTrackSelection selectionForViewportTrackAnchor(
        const std::size_t anchorIndex,
        const std::size_t regionCount)
    {
        if (regionCount == 0 || anchorIndex > regionCount)
        {
            throw std::out_of_range(
                "A viewport anchor selection must name a track boundary."
            );
        }

        return {
            std::min(anchorIndex, regionCount - 1),
            anchorIndex
        };
    }

    std::vector<ViewportTrackAnchor> createViewportTrackAnchors(
        const coaster::AuthoredTrack& track)
    {
        if (track.sectionCount() == 0)
        {
            return {};
        }

        std::vector<double> boundaryDistances;
        boundaryDistances.reserve(track.sectionCount() + 1);
        boundaryDistances.push_back(0.0);

        double distance = 0.0;
        double largestSectionLength = 0.0;
        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            const double length = coaster::sectionLength(track.section(index));
            distance += length;
            if (!std::isfinite(distance))
            {
                throw std::overflow_error(
                    "Authored track boundary distance is not finite."
                );
            }

            boundaryDistances.push_back(distance);
            largestSectionLength = std::max(largestSectionLength, length);
        }

        // Spacing controls only returned output density in the Core solve.
        // Using the largest region length avoids visualization-density work;
        // each exact section start/end is still emitted by the kinematic path.
        const std::vector<coaster::TrackKinematicState> states =
            coaster::integrateAuthoredTrackKinematics(
                track,
                largestSectionLength
            );

        std::vector<ViewportTrackAnchor> anchors;
        anchors.reserve(boundaryDistances.size());
        std::size_t searchIndex = 0;

        for (std::size_t anchorIndex = 0;
            anchorIndex < boundaryDistances.size();
            ++anchorIndex)
        {
            const double targetDistance = boundaryDistances[anchorIndex];
            const double tolerance = comparisonTolerance(
                targetDistance,
                targetDistance
            );

            while (searchIndex + 1 < states.size()
                && states[searchIndex].distance
                    < targetDistance - tolerance)
            {
                ++searchIndex;
            }

            if (searchIndex >= states.size()
                || std::abs(states[searchIndex].distance - targetDistance)
                    > tolerance)
            {
                throw std::runtime_error(
                    "Core kinematics did not emit an authored track boundary."
                );
            }

            const coaster::TrackKinematicState& state = states[searchIndex];
            if (!finiteVector(state.position)
                || !finiteVector(state.frame.tangent)
                || !finiteVector(state.frame.lateral)
                || !finiteVector(state.frame.up))
            {
                throw std::runtime_error(
                    "Core kinematics emitted a non-finite track boundary pose."
                );
            }

            const bool isStart = anchorIndex == 0;
            const bool isEnd = anchorIndex == track.sectionCount();
            anchors.push_back(ViewportTrackAnchor{
                .anchorIndex = anchorIndex,
                .distance = targetDistance,
                .position = state.position,
                .forward = state.frame.tangent,
                .lateral = state.frame.lateral,
                .up = state.frame.up,
                .previousRegionIndex = isStart
                    ? std::nullopt
                    : std::optional<std::size_t>{anchorIndex - 1},
                .nextRegionIndex = isEnd
                    ? std::nullopt
                    : std::optional<std::size_t>{anchorIndex},
                .kind = isStart
                    ? ViewportTrackAnchorKind::Start
                    : isEnd
                        ? ViewportTrackAnchorKind::End
                        : ViewportTrackAnchorKind::Interior
            });
        }

        return anchors;
    }

    std::optional<ViewportProjectedPoint> projectViewportPoint(
        const ViewportCamera& camera,
        const glm::dvec3& worldPosition,
        const double aspectRatio)
    {
        if (!finiteVector(worldPosition))
        {
            throw std::invalid_argument(
                "Viewport projection requires a finite world position."
            );
        }

        const std::array<float, 16> matrix =
            camera.viewProjection(aspectRatio);
        const std::array<double, 4> point{
            worldPosition.x,
            worldPosition.y,
            worldPosition.z,
            1.0
        };
        std::array<double, 4> clip{};

        for (std::size_t row = 0; row < 4; ++row)
        {
            for (std::size_t column = 0; column < 4; ++column)
            {
                clip[row] += static_cast<double>(
                    matrix[column * 4 + row]) * point[column];
            }
        }

        if (!std::isfinite(clip[0])
            || !std::isfinite(clip[1])
            || !std::isfinite(clip[2])
            || !std::isfinite(clip[3])
            || clip[3] <= std::numeric_limits<double>::epsilon())
        {
            return std::nullopt;
        }

        const double inverseW = 1.0 / clip[3];
        const glm::dvec3 normalizedDevice{
            clip[0] * inverseW,
            clip[1] * inverseW,
            clip[2] * inverseW
        };
        if (!finiteVector(normalizedDevice)
            || normalizedDevice.z < 0.0
            || normalizedDevice.z > 1.0)
        {
            return std::nullopt;
        }

        return ViewportProjectedPoint{
            .normalizedPosition = {
                0.5 * (normalizedDevice.x + 1.0),
                0.5 * (normalizedDevice.y + 1.0)
            },
            .depth = normalizedDevice.z
        };
    }

    std::optional<ViewportTrackAnchorPickResult> pickViewportTrackAnchor(
        const std::span<const ViewportTrackAnchor> anchors,
        const ViewportCamera& camera,
        const glm::dvec2& normalizedPointer,
        const std::uint32_t viewportPixelWidth,
        const std::uint32_t viewportPixelHeight,
        const double hitRadiusPixels)
    {
        if (viewportPixelWidth == 0
            || viewportPixelHeight == 0
            || !std::isfinite(normalizedPointer.x)
            || !std::isfinite(normalizedPointer.y)
            || normalizedPointer.x < 0.0
            || normalizedPointer.x > 1.0
            || normalizedPointer.y < 0.0
            || normalizedPointer.y > 1.0
            || !std::isfinite(hitRadiusPixels)
            || hitRadiusPixels <= 0.0)
        {
            throw std::invalid_argument(
                "Anchor picking requires a valid viewport, pointer, and radius."
            );
        }

        const double aspectRatio = static_cast<double>(viewportPixelWidth)
            / static_cast<double>(viewportPixelHeight);
        const glm::dvec2 pointerPixels{
            normalizedPointer.x * viewportPixelWidth,
            normalizedPointer.y * viewportPixelHeight
        };
        const double hitRadiusSquared = hitRadiusPixels * hitRadiusPixels;
        std::optional<ViewportTrackAnchorPickResult> best;

        for (const ViewportTrackAnchor& anchor : anchors)
        {
            const auto projected = projectViewportPoint(
                camera,
                anchor.position,
                aspectRatio
            );
            if (!projected.has_value())
            {
                continue;
            }

            const glm::dvec2 anchorPixels{
                projected->normalizedPosition.x * viewportPixelWidth,
                projected->normalizedPosition.y * viewportPixelHeight
            };
            const glm::dvec2 offset = anchorPixels - pointerPixels;
            const double distanceSquared = offset.x * offset.x
                + offset.y * offset.y;
            if (distanceSquared > hitRadiusSquared)
            {
                continue;
            }

            const ViewportTrackAnchorPickResult candidate{
                anchor.anchorIndex,
                std::sqrt(distanceSquared),
                projected->depth
            };
            if (!best.has_value())
            {
                best = candidate;
                continue;
            }

            const double distanceTolerance = comparisonTolerance(
                candidate.distancePixels,
                best->distancePixels
            );
            const double depthTolerance = comparisonTolerance(
                candidate.depth,
                best->depth
            );
            const bool closer = candidate.distancePixels
                < best->distancePixels - distanceTolerance;
            const bool sameDistance = std::abs(
                candidate.distancePixels - best->distancePixels)
                <= distanceTolerance;
            const bool frontmost = sameDistance
                && candidate.depth < best->depth - depthTolerance;
            const bool sameDepth = sameDistance
                && std::abs(candidate.depth - best->depth) <= depthTolerance;
            const bool deterministicTie = sameDepth
                && candidate.anchorIndex < best->anchorIndex;

            if (closer || frontmost || deterministicTie)
            {
                best = candidate;
            }
        }

        return best;
    }
}
