#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>
#include <quantum/engine/Logging.hpp>

#include <algorithm>
#include <cstdarg>
#include <stdexcept>
#include <string>

namespace quantum::editor
{
    EditorFonts loadEditorFonts(const std::filesystem::path& basePath)
    {
        const auto load = [&](const char* name, const float size)
        {
            const auto path = basePath / "assets/fonts/Overpass" / name;
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
        fonts.normal = load("overpass-regular.otf", editorFontSize);
        fonts.header = load("overpass-semibold.otf", editorHeaderFontSize);
        fonts.technical = load("overpass-mono-regular.otf", editorTechnicalFontSize);
        ImGui::GetIO().FontDefault = fonts.normal;
        return fonts;
    }

    void editorHeading(const char* const label, const EditorFonts& fonts)
    {
        ImGui::PushFont(fonts.header, editorHeaderFontSize);
        ImGui::SeparatorText(label);
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
        style.WindowPadding = ImVec2(8.0F, 8.0F);
        style.FramePadding = ImVec2(5.0F, 4.0F);
        style.ItemSpacing = ImVec2(8.0F, 5.0F);
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
        colors[ImGuiCol_MenuBarBg] = palette::panelRaised;

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
        colors[ImGuiCol_TableBorderStrong] = palette::border;
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
