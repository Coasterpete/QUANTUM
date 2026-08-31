#pragma once

#include <imgui.h>

#include <cmath>
#include <cstdint>
#include <array>
#include <filesystem>

namespace quantum::editor
{
    inline constexpr float editorFontSize = 14.0F;
    inline constexpr float editorHeaderFontSize = 15.0F;
    inline constexpr float editorTechnicalFontSize = 14.0F;

    // Borrowed handles only: the current ImGui atlas owns all three fonts.
    struct EditorFonts
    {
        ImFont* normal = nullptr;
        ImFont* header = nullptr;
        ImFont* technical = nullptr;
    };

    [[nodiscard]] EditorFonts loadEditorFonts(const std::filesystem::path& basePath);
    void editorHeading(const char* label, const EditorFonts& fonts);

    namespace viewportStyle
    {
        inline constexpr float anchorRadius = 4.0F;
        inline constexpr float selectedAnchorRadius = 6.0F;
        inline constexpr float anchorRingRadius = 10.5F;
        inline constexpr float selectedLineWidth = 3.0F;
        inline constexpr float hoveredLineWidth = 2.0F;
        inline constexpr float selectionCapRadius = 3.5F;
        inline constexpr float anchorLabelGap = 14.0F;
        inline constexpr float orientationAxisLength = 34.0F;
        inline constexpr float moveHandleLength = 56.0F;
        inline constexpr float moveHitRadius = 8.0F;
        inline constexpr std::array<float, 3> rotateRadii{30.0F, 39.0F, 48.0F};
        inline constexpr float rotateHitRadius = 5.0F;
        inline constexpr float overlayMargin = 9.0F;
        inline constexpr float overlayPadding = 4.0F;
    }

    // Logical drawing and hit-test dimensions use the same UI scale.
    [[nodiscard]] float editorPresentationScale();
    [[nodiscard]] ImVec2 clampViewportLabel(
        ImVec2 position, ImVec2 size, ImVec2 minimum, ImVec2 maximum);

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
        inline const ImVec4 plotDot = fromSrgb(53, 53, 53);
        inline const ImVec4 plotReference = fromSrgb(72, 72, 72);

        inline const ImVec4 accent = fromSrgb(80, 204, 196);
        inline const ImVec4 selection = fromSrgb(32, 83, 81);
        inline const ImVec4 selectionHovered = fromSrgb(40, 102, 98);
        inline const ImVec4 selectionActive = fromSrgb(44, 109, 104);
        inline const ImVec4 warning = fromSrgb(235, 185, 91);
        inline const ImVec4 error = fromSrgb(255, 146, 140);
        inline const ImVec4 destructive = fromSrgb(67, 38, 39);
        inline const ImVec4 destructiveHovered = fromSrgb(92, 44, 46);
        inline const ImVec4 destructiveActive = fromSrgb(110, 48, 50);
        inline const ImVec4 viewportAnchor = fromSrgb(126, 164, 169);
        inline const ImVec4 viewportSelected = fromSrgb(153, 239, 232);
        inline const ImVec4 viewportHovered = fromSrgb(241, 209, 145);
        inline const ImVec4 viewportRing = fromSrgb(242, 247, 247);
        inline const std::array<ImVec4, 3> viewportAxes{
            fromSrgb(246, 117, 105), fromSrgb(119, 218, 151),
            fromSrgb(128, 175, 250)};

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
