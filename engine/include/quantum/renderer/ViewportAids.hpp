#pragma once

#include <quantum/renderer/VulkanContext.hpp>
#include <vector>

namespace quantum::renderer
{
    inline constexpr int gridHalfLineCount = 20;
    inline constexpr std::size_t viewportAidVertexCount =
        static_cast<std::size_t>(2 * gridHalfLineCount + 1) * 4 + 6;

    // CPU geometry only; VulkanContext retains ownership of the GPU buffer.
    [[nodiscard]] std::vector<LineVertex> createViewportAidVertices(
        float centerX, float centerY, float spacing);
}
