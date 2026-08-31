#pragma once

#include <imgui.h>

#include <cmath>
#include <cstdint>

namespace quantum::editor
{
    inline constexpr float editorFontSize = 14.0F;

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
        // the intended display values.
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

        inline const ImVec4 black = fromSrgb(0, 0, 0);
        inline const ImVec4 surface = fromSrgb(34, 34, 34);
        inline const ImVec4 surfaceRaised = fromSrgb(40, 40, 40);
        inline const ImVec4 surfaceInset = fromSrgb(26, 26, 26);
        inline const ImVec4 control = fromSrgb(46, 46, 46);
        inline const ImVec4 controlHovered = fromSrgb(58, 58, 58);
        inline const ImVec4 controlActive = fromSrgb(70, 70, 70);
        inline const ImVec4 border = fromSrgb(76, 76, 76);
        inline const ImVec4 text = fromSrgb(224, 224, 224);
        inline const ImVec4 textSecondary = fromSrgb(148, 148, 148);

        inline const ImVec4 accent = fromSrgb(80, 204, 196);
        inline const ImVec4 selection = fromSrgb(32, 83, 81);
        inline const ImVec4 selectionHovered = fromSrgb(40, 102, 98);
        inline const ImVec4 selectionActive = fromSrgb(44, 109, 104);
        inline const ImVec4 warning = fromSrgb(235, 185, 91);
        inline const ImVec4 error = fromSrgb(255, 146, 140);
        inline const ImVec4 destructive = fromSrgb(67, 38, 39);
        inline const ImVec4 destructiveHovered = fromSrgb(92, 44, 46);
        inline const ImVec4 destructiveActive = fromSrgb(110, 48, 50);

        // Data channels retain their own colors, independent of UI status.
        inline const ImVec4 rollChannelRed = fromSrgb(255, 0, 0);
        inline const ImVec4 pitchChannelPurple = fromSrgb(191, 0, 255);
        inline const ImVec4 yawChannelGold = fromSrgb(255, 215, 0);
        inline const ImVec4 normalGChannel = fromSrgb(0, 230, 130);
        inline const ImVec4 speedChannel = fromSrgb(0, 210, 255);
    }

    void applyQuantumStyle();
    void pushDestructiveStyle();
    void popDestructiveStyle();
}
