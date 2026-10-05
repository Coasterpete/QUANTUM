#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>
#include <quantum/engine/Logging.hpp>

#include <algorithm>
#include <cstdarg>
#include <limits>
#include <stdexcept>
#include <string>

namespace quantum::editor
{
    EditorFonts loadEditorFonts(const std::filesystem::path& basePath)
    {
        const auto load = [&](const char* name, const float size)
        {
            const auto path = basePath / "assets/fonts" / name;
            std::error_code error;
            if (!std::filesystem::is_regular_file(path, error) || error)
            {
                throw std::runtime_error("Bundled UI font not found at " + path.string());
            }
            ImFont* const font = ImGui::GetIO().Fonts->AddFontFromFileTTF(
                path.string().c_str(), size);
            if (font == nullptr)
            {
                throw std::runtime_error("Dear ImGui could not load the bundled UI font at "
                    + path.string());
            }
            quantum::logging::logMessagef(quantum::logging::LogLevel::Info,
                "APP", "Loaded bundled UI font: %s (%.0f px)",
                path.string().c_str(), static_cast<double>(size));
            return font;
        };

        EditorFonts fonts;
        fonts.normal = load("Overpass/overpass-regular.otf", editorFontSize);
        fonts.header = load("Red_Hat_Mono/static/RedHatMono-SemiBold.ttf", editorHeaderFontSize);
        fonts.technical = load("Red_Hat_Mono/static/RedHatMono-Regular.ttf", editorTechnicalFontSize);
        ImGui::GetIO().FontDefault = fonts.normal;
        return fonts;
    }

    void editorHeading(const char* const label, const EditorFonts& fonts)
    {
        ImGui::PushFont(fonts.header, editorHeaderFontSize);
        ImGui::PushStyleColor(ImGuiCol_Text, palette::textHeading);
        ImGui::SeparatorText(label);
        ImGui::PopStyleColor();
        ImGui::PopFont();
    }

    void editorPaneHeading(const char* const title, const char* const context,
        const EditorFonts& fonts, const char* const status,
        const ImVec4& statusColor)
    {
        // A content-sized band shares the pane's scrolling and owns no state.
        // Keep all dimensions in the existing logical UI scale.
        const ImGuiStyle& style = ImGui::GetStyle();
        ImGui::PushStyleColor(ImGuiCol_ChildBg, palette::panelRaised);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
            ImVec2(style.WindowPadding.x, style.FramePadding.y));
        if (ImGui::BeginChild("##PaneHeading", {0, 0},
            ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysAutoResize
                | ImGuiChildFlags_AlwaysUseWindowPadding,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
        {
            const float width = ImGui::GetContentRegionAvail().x;
            const float statusWidth = status == nullptr
                ? 0.0F : ImGui::CalcTextSize(status).x;
            ImGui::PushFont(fonts.header, editorHeaderFontSize);
            const float titleWidth = ImGui::CalcTextSize(title).x;
            ImGui::PushStyleColor(ImGuiCol_Text, palette::textHeading);
            ImGui::TextWrapped("%s", title);
            ImGui::PopStyleColor();
            ImGui::PopFont();
            if (status != nullptr)
            {
                if (titleWidth + statusWidth + style.ItemSpacing.x * 2.0F <= width)
                    ImGui::SameLine(style.WindowPadding.x + width - statusWidth);
                ImGui::PushStyleColor(ImGuiCol_Text,
                    statusColor.w > 0.0F ? statusColor : palette::textSecondary);
                ImGui::TextWrapped("%s", status);
                ImGui::PopStyleColor();
            }
            if (context != nullptr && context[0] != '\0')
                editorSecondaryTextWrapped("%s", context);
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }

    void editorSectionHeading(const char* const label, const EditorFonts& fonts)
    {
        ImGui::Dummy({0, ImGui::GetStyle().ItemSpacing.y});
        ImGui::PushFont(fonts.header, editorFontSize);
        ImGui::TextColored(palette::textHeading, "%s", label);
        ImGui::PopFont();
    }

    void editorSecondaryText(const char* const format, ...)
    {
        va_list arguments;
        va_start(arguments, format);
        ImGui::TextColoredV(palette::textSecondary, format, arguments);
        va_end(arguments);
    }

    void editorSecondaryTextWrapped(const char* const format, ...)
    {
        va_list arguments;
        va_start(arguments, format);
        ImGui::PushStyleColor(ImGuiCol_Text, palette::textSecondary);
        ImGui::TextWrappedV(format, arguments);
        ImGui::PopStyleColor();
        va_end(arguments);
    }

    float editorPresentationScale()
    {
        const ImGuiStyle& style = ImGui::GetStyle();
        return style.FontScaleMain * style.FontScaleDpi;
    }

    float editorLogicalUiScale(const float displayScale,
        const float pixelDensity, const float overrideScale)
    {
        if (!std::isfinite(displayScale) || displayScale <= 0.0F
            || !std::isfinite(pixelDensity) || pixelDensity <= 0.0F
            || !std::isfinite(overrideScale) || overrideScale < 0.0F)
            throw std::invalid_argument("Invalid editor display scale.");
        // Windows uses pixel window coordinates; high-density platforms can
        // supply multiple framebuffer pixels per logical window coordinate.
        return (overrideScale > 0.0F ? overrideScale : displayScale) / pixelDensity;
    }

    void applyEditorUiScale(const float logicalScale)
    {
        if (!std::isfinite(logicalScale) || logicalScale <= 0.0F)
            throw std::invalid_argument("Invalid editor UI scale.");
        // Always start from the unscaled style so repeated changes cannot grow
        // spacing cumulatively. Font sizing is separate from ScaleAllSizes.
        applyQuantumStyle();
        ImGuiStyle& style = ImGui::GetStyle();
        style.ScaleAllSizes(logicalScale);
        style.FontScaleMain = 1.0F;
        style.FontScaleDpi = logicalScale;
    }

    std::uint32_t contentPixelDimension(
        const float logicalDimension,
        const float framebufferScale)
    {
        if (!std::isfinite(logicalDimension)
            || !std::isfinite(framebufferScale)
            || logicalDimension <= 0.0F
            || framebufferScale <= 0.0F)
        {
            return 0;
        }

        const double pixels = std::floor(
            static_cast<double>(logicalDimension)
                * static_cast<double>(framebufferScale)
            + 0.5
        );

        if (pixels < 1.0)
        {
            return 0;
        }

        if (pixels
            > static_cast<double>(std::numeric_limits<std::uint32_t>::max()))
        {
            throw std::length_error(
                "The Editor viewport content size exceeds a 32-bit pixel "
                "dimension."
            );
        }

        return static_cast<std::uint32_t>(pixels);
    }

    ImVec2 clampViewportLabel(const ImVec2 position, const ImVec2 size,
        const ImVec2 minimum, const ImVec2 maximum)
    {
        return {
            std::clamp(position.x, minimum.x, std::max(minimum.x, maximum.x - size.x)),
            std::clamp(position.y, minimum.y, std::max(minimum.y, maximum.y - size.y))
        };
    }

    void applyQuantumStyle()
    {
        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::StyleColorsDark(&style);
        style.FontSizeBase = editorFontSize;
        style.DisabledAlpha = 0.70F;
        style.WindowPadding = ImVec2(9.0F, 9.0F);
        style.FramePadding = ImVec2(5.0F, 4.0F);
        style.ItemSpacing = ImVec2(8.0F, 6.0F);
        style.ItemInnerSpacing = ImVec2(4.0F, 4.0F);
        style.CellPadding = ImVec2(4.0F, 3.0F);
        style.IndentSpacing = 21.0F;
        style.ScrollbarSize = 14.0F;
        style.GrabMinSize = 12.0F;
        style.FrameBorderSize = 1.0F;

        ImVec4* const colors = style.Colors;
        colors[ImGuiCol_Text] = palette::textPrimary;
        colors[ImGuiCol_TextDisabled] = palette::textDisabled;
        colors[ImGuiCol_WindowBg] = palette::panel;
        colors[ImGuiCol_ChildBg] = palette::panel;
        colors[ImGuiCol_PopupBg] = palette::panelRaised;
        colors[ImGuiCol_Border] = palette::border;
        colors[ImGuiCol_BorderShadow] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);

        colors[ImGuiCol_FrameBg] = palette::frame;
        colors[ImGuiCol_FrameBgHovered] = palette::frameHovered;
        colors[ImGuiCol_FrameBgActive] = palette::accentMuted;
        colors[ImGuiCol_TitleBg] = palette::background;
        colors[ImGuiCol_TitleBgActive] = palette::panelRaised;
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.0F, 0.0F, 0.0F, 0.8F);
        colors[ImGuiCol_MenuBarBg] = palette::menu;

        colors[ImGuiCol_ScrollbarBg] = palette::background;
        colors[ImGuiCol_ScrollbarGrab] = palette::border;
        colors[ImGuiCol_ScrollbarGrabHovered] = palette::frameActive;
        colors[ImGuiCol_ScrollbarGrabActive] = palette::accentActive;
        colors[ImGuiCol_CheckMark] = palette::accent;
        colors[ImGuiCol_CheckboxSelectedBg] = palette::accentMuted;
        colors[ImGuiCol_SliderGrab] = palette::accent;
        colors[ImGuiCol_SliderGrabActive] = palette::accentHovered;

        colors[ImGuiCol_Button] = palette::frame;
        colors[ImGuiCol_ButtonHovered] = palette::frameHovered;
        colors[ImGuiCol_ButtonActive] = palette::selection;
        colors[ImGuiCol_Header] = palette::selection;
        colors[ImGuiCol_HeaderHovered] = palette::selectionHovered;
        colors[ImGuiCol_HeaderActive] = palette::selectionActive;
        colors[ImGuiCol_Separator] = palette::separator;
        colors[ImGuiCol_SeparatorHovered] = palette::accentHovered;
        colors[ImGuiCol_SeparatorActive] = palette::accentActive;
        colors[ImGuiCol_ResizeGrip] = ImVec4(
            palette::panel.x,
            palette::panel.y,
            palette::panel.z,
            0.35F
        );
        colors[ImGuiCol_ResizeGripHovered] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.75F
        );
        colors[ImGuiCol_ResizeGripActive] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.95F
        );
        colors[ImGuiCol_InputTextCursor] = palette::accent;

        colors[ImGuiCol_Tab] = palette::background;
        colors[ImGuiCol_TabHovered] = palette::frameHovered;
        colors[ImGuiCol_TabSelected] = palette::frameActive;
        colors[ImGuiCol_TabSelectedOverline] = palette::accent;
        colors[ImGuiCol_TabDimmed] = palette::background;
        colors[ImGuiCol_TabDimmedSelected] = palette::frame;
        colors[ImGuiCol_TabDimmedSelectedOverline] = palette::textSecondary;
        colors[ImGuiCol_DockingPreview] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.35F
        );
        colors[ImGuiCol_DockingEmptyBg] = palette::black;

        colors[ImGuiCol_PlotLines] = palette::textPrimary;
        colors[ImGuiCol_PlotLinesHovered] = palette::accent;
        colors[ImGuiCol_PlotHistogram] = palette::accent;
        colors[ImGuiCol_PlotHistogramHovered] = palette::textPrimary;
        colors[ImGuiCol_TableHeaderBg] = palette::panelRaised;
        colors[ImGuiCol_TableBorderStrong] = palette::borderStrong;
        colors[ImGuiCol_TableBorderLight] = palette::frame;
        colors[ImGuiCol_TableRowBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
        colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.0F, 0.0F, 0.0F, 0.16F);

        colors[ImGuiCol_TextLink] = palette::accent;
        colors[ImGuiCol_TextSelectedBg] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.45F
        );
        colors[ImGuiCol_TreeLines] = palette::border;
        colors[ImGuiCol_DragDropTarget] = palette::accent;
        colors[ImGuiCol_DragDropTargetBg] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.18F
        );
        colors[ImGuiCol_UnsavedMarker] = palette::warning;
        colors[ImGuiCol_NavCursor] = palette::accent;
        colors[ImGuiCol_NavWindowingHighlight] = ImVec4(
            palette::textPrimary.x,
            palette::textPrimary.y,
            palette::textPrimary.z,
            0.70F
        );
        colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.65F);
        colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.65F);
    }

    void pushDestructiveStyle()
    {
        // Keep the action's explicit Remove/Delete label and readable text;
        // red is a secondary cue, not a saturated fill behind gray text.
        ImGui::PushStyleColor(ImGuiCol_Text, palette::error);
        ImGui::PushStyleColor(ImGuiCol_Border, palette::error);
        ImGui::PushStyleColor(ImGuiCol_Button, palette::destructive);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, palette::destructiveHovered);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, palette::destructiveActive);
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, palette::destructiveHovered);
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, palette::destructiveActive);
    }

    void popDestructiveStyle()
    {
        ImGui::PopStyleColor(7);
    }
}
