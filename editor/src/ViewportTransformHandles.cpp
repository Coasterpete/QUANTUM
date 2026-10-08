#include <quantum/editor/ViewportTransformHandles.hpp>
#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace quantum::editor
{
    std::optional<ViewportTransformHandles> viewportTransformHandles(
        const ViewportCamera& camera, const glm::dvec3& position,
        std::uint32_t width, std::uint32_t height, double scale)
    {
        if (!width || !height || !std::isfinite(scale) || scale <= 0) return std::nullopt;
        const double aspect = static_cast<double>(width) / height;
        const auto projected = projectViewportPoint(camera, position, aspect);
        if (!projected) return std::nullopt;
        const glm::dvec2 pixels{width, height};
        const auto origin = projected->normalizedPosition * pixels;
        const double distance = camera.projection() == ViewportProjection::Perspective
            ? glm::length(position - camera.position()) : camera.distance();
        const double unitsPerPixel = 2 * distance * std::tan(camera.verticalFieldOfView() / 2) / height;
        const double length = 80 * scale * unitsPerPixel;
        ViewportTransformHandles handles{origin, {}, {36 * scale, 50 * scale, 64 * scale}, 8 * scale};
        constexpr std::array<glm::dvec2, 3> fallback{{{1, 0}, {-.7071, .7071}, {0, -1}}};
        for (unsigned i = 0; i < 3; ++i)
        {
            auto direction = fallback[i];
            double screenLength = 80 * scale, sensitivity = unitsPerPixel;
            const auto end = projectViewportPoint(camera,
                position + length * startPoseWorldAxis(static_cast<StartPoseTransformAxis>(i)), aspect);
            if (end)
            {
                const auto delta = end->normalizedPosition * pixels - origin;
                const double projectedLength = glm::length(delta);
                if (projectedLength >= 10 * scale)
                { direction = delta / projectedLength; screenLength = projectedLength; sensitivity = length / projectedLength; }
            }
            handles.move[i] = {origin + 12 * scale * direction,
                origin + screenLength * direction, direction, sensitivity};
        }
        return handles;
    }
    std::optional<StartPoseTransformAxis> pickViewportTransformHandle(
        const ViewportTransformHandles& h, StartPoseTransformMode mode, const glm::dvec2& pointer)
    {
        std::optional<StartPoseTransformAxis> picked;
        double best = std::numeric_limits<double>::max();
        for (unsigned i = 0; i < 3; ++i)
        {
            double metric;
            if (mode == StartPoseTransformMode::Move)
            {
                const auto& line = h.move[i];
                const auto delta = line.end - line.begin;
                const double t = std::clamp(glm::dot(pointer - line.begin, delta) / glm::dot(delta, delta), 0.0, 1.0);
                metric = glm::length(pointer - line.begin - t * delta);
            }
            else if (mode == StartPoseTransformMode::Rotate)
                metric = std::abs(glm::length(pointer - h.origin) - h.rotateRadii[i]);
            else return std::nullopt;
            if (metric <= h.hitRadius && metric < best)
            { best = metric; picked = static_cast<StartPoseTransformAxis>(i); }
        }
        return picked;
    }
}
