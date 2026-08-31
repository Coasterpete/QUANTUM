#include <quantum/renderer/ViewportAids.hpp>

#include <cmath>

namespace
{
    constexpr float worldAxisLength = 25.0F;
    void appendLine(
        std::vector<quantum::renderer::LineVertex>& vertices,
        const std::array<float, 3> begin,
        const std::array<float, 3> end,
        const std::array<float, 4>& color)
    {
        const auto makeVertex = [](const std::array<float, 3> position,
            const std::array<float, 4>& vertexColor)
        {
            return quantum::renderer::LineVertex{
                position[0],
                position[1],
                position[2],
                vertexColor
            };
        };

        vertices.push_back(makeVertex(begin, color));
        vertices.push_back(makeVertex(end, color));
    }

}

namespace quantum::renderer
{
    // Grid lines snap their coordinates to multiples of the spacing so the
    // modeling plane keeps stable, readable values while recentring around
    // the solved track.
    std::vector<quantum::renderer::LineVertex> createViewportAidVertices(
        const float centerX,
        const float centerY,
        const float spacing)
    {
        // Linear colors for the sRGB target: references stay below track
        // curves in contrast. Only color changes; the lattice is unchanged.
        constexpr std::array minorGridColor{0.012F, 0.012F, 0.012F, 1.0F};
        constexpr std::array majorGridColor{0.026F, 0.026F, 0.026F, 1.0F};
        constexpr std::array xAxisColor{0.22F, 0.055F, 0.045F, 1.0F};
        constexpr std::array yAxisColor{0.055F, 0.19F, 0.075F, 1.0F};
        constexpr std::array zAxisColor{0.06F, 0.12F, 0.25F, 1.0F};
        const auto gridColor = [&](const float coordinate)
            -> const std::array<float, 4>&
        {
            return std::fmod(std::round(coordinate / spacing), 5.0F) == 0.0F
                ? majorGridColor : minorGridColor;
        };

        const float halfExtent = static_cast<float>(gridHalfLineCount)
            * spacing;
        const float snappedCenterX =
            std::round(centerX / spacing) * spacing;
        const float snappedCenterY =
            std::round(centerY / spacing) * spacing;

        std::vector<quantum::renderer::LineVertex> vertices;
        vertices.reserve(viewportAidVertexCount);

        for (int line = -gridHalfLineCount;
            line <= gridHalfLineCount;
            ++line)
        {
            const float coordinate =
                snappedCenterY + static_cast<float>(line) * spacing;
            appendLine(
                vertices,
                {snappedCenterX - halfExtent, coordinate, 0.0F},
                {snappedCenterX + halfExtent, coordinate, 0.0F},
                gridColor(coordinate)
            );
            const float verticalCoordinate =
                snappedCenterX + static_cast<float>(line) * spacing;
            appendLine(
                vertices,
                {verticalCoordinate, snappedCenterY - halfExtent, 0.0F},
                {verticalCoordinate, snappedCenterY + halfExtent, 0.0F},
                gridColor(verticalCoordinate)
            );
        }

        const std::array<float, 3> origin{
            snappedCenterX, snappedCenterY, 0.0F
        };
        appendLine(
            vertices,
            origin,
            {snappedCenterX + worldAxisLength, snappedCenterY, 0.0F},
            xAxisColor
        );
        appendLine(
            vertices,
            origin,
            {snappedCenterX, snappedCenterY + worldAxisLength, 0.0F},
            yAxisColor
        );
        appendLine(
            vertices,
            origin,
            {snappedCenterX, snappedCenterY, worldAxisLength},
            zAxisColor
        );

        return vertices;
    }

}
