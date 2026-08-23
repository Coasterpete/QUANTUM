#pragma once

#include <imgui.h>

#include <cmath>
#include <cstdint>

namespace quantum::editor
{
    namespace palette
    {
        [[nodiscard]] inline float linearChannel(
            const std::uint8_t srgbChannel) noexcept
        {
            const float encoded = static_cast<float>(srgbChannel) / 255.0F;
            return encoded <= 0.04045F
                ? encoded / 12.92F
                : std::pow((encoded + 0.055F) / 1.055F, 2.4F);
        }

        // QUANTUM presents the UI through an sRGB swapchain. Convert authored
        // sRGB byte values to linear inputs so the displayed colors retain
        // their exact Inkscape palette values.
        [[nodiscard]] inline ImVec4 fromSrgb(
            const std::uint8_t red,
            const std::uint8_t green,
            const std::uint8_t blue) noexcept
        {
            return {
                linearChannel(red),
                linearChannel(green),
                linearChannel(blue),
                1.0F
            };
        }

        inline const ImVec4 trackWorkspaceBlue = fromSrgb(0, 0, 255);
        inline const ImVec4 supportWorkspaceRed = fromSrgb(255, 0, 0);
        inline const ImVec4 transitionEditorBlue = fromSrgb(0, 0, 190);
        inline const ImVec4 commandHeadingGreen = fromSrgb(0, 255, 0);
        inline const ImVec4 black = fromSrgb(0, 0, 0);
        inline const ImVec4 darkestGray = fromSrgb(75, 75, 75);
        inline const ImVec4 brightestGray = fromSrgb(195, 195, 195);
        inline const ImVec4 rollChannelRed = supportWorkspaceRed;
        inline const ImVec4 pitchChannelPurple = fromSrgb(191, 0, 255);
        inline const ImVec4 yawChannelGold = fromSrgb(255, 215, 0);
    }

    void applyQuantumStyle();
}
