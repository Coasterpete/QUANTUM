#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

namespace quantum::editor
{
    void applyQuantumStyle()
    {
        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::StyleColorsDark(&style);
        style.FontSizeBase = editorFontSize;
        style.DisabledAlpha = 0.45F;
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
        colors[ImGuiCol_Text] = palette::text;
        colors[ImGuiCol_TextDisabled] = palette::textSecondary;
        colors[ImGuiCol_WindowBg] = palette::surface;
        colors[ImGuiCol_ChildBg] = palette::surface;
        colors[ImGuiCol_PopupBg] = palette::surfaceRaised;
        colors[ImGuiCol_Border] = palette::border;
        colors[ImGuiCol_BorderShadow] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);

        colors[ImGuiCol_FrameBg] = palette::surfaceInset;
        colors[ImGuiCol_FrameBgHovered] = palette::controlHovered;
        colors[ImGuiCol_FrameBgActive] = palette::selection;
        colors[ImGuiCol_TitleBg] = palette::surfaceInset;
        colors[ImGuiCol_TitleBgActive] = palette::surfaceRaised;
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.0F, 0.0F, 0.0F, 0.8F);
        colors[ImGuiCol_MenuBarBg] = palette::surfaceRaised;

        colors[ImGuiCol_ScrollbarBg] = palette::surfaceInset;
        colors[ImGuiCol_ScrollbarGrab] = palette::border;
        colors[ImGuiCol_ScrollbarGrabHovered] = palette::textSecondary;
        colors[ImGuiCol_ScrollbarGrabActive] = palette::accent;
        colors[ImGuiCol_CheckMark] = palette::accent;
        colors[ImGuiCol_CheckboxSelectedBg] = palette::selection;
        colors[ImGuiCol_SliderGrab] = palette::accent;
        colors[ImGuiCol_SliderGrabActive] = palette::text;

        colors[ImGuiCol_Button] = palette::control;
        colors[ImGuiCol_ButtonHovered] = palette::controlHovered;
        colors[ImGuiCol_ButtonActive] = palette::selection;
        colors[ImGuiCol_Header] = palette::selection;
        colors[ImGuiCol_HeaderHovered] = palette::selectionHovered;
        colors[ImGuiCol_HeaderActive] = palette::selectionActive;
        colors[ImGuiCol_Separator] = palette::border;
        colors[ImGuiCol_SeparatorHovered] = palette::accent;
        colors[ImGuiCol_SeparatorActive] = palette::accent;
        colors[ImGuiCol_ResizeGrip] = ImVec4(
            palette::surface.x,
            palette::surface.y,
            palette::surface.z,
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

        colors[ImGuiCol_Tab] = palette::surfaceInset;
        colors[ImGuiCol_TabHovered] = palette::controlHovered;
        colors[ImGuiCol_TabSelected] = palette::controlActive;
        colors[ImGuiCol_TabSelectedOverline] = palette::accent;
        colors[ImGuiCol_TabDimmed] = palette::surfaceInset;
        colors[ImGuiCol_TabDimmedSelected] = palette::control;
        colors[ImGuiCol_TabDimmedSelectedOverline] = palette::textSecondary;
        colors[ImGuiCol_DockingPreview] = ImVec4(
            palette::accent.x,
            palette::accent.y,
            palette::accent.z,
            0.35F
        );
        colors[ImGuiCol_DockingEmptyBg] = palette::black;

        colors[ImGuiCol_PlotLines] = palette::text;
        colors[ImGuiCol_PlotLinesHovered] = palette::accent;
        colors[ImGuiCol_PlotHistogram] = palette::accent;
        colors[ImGuiCol_PlotHistogramHovered] = palette::text;
        colors[ImGuiCol_TableHeaderBg] = palette::surfaceRaised;
        colors[ImGuiCol_TableBorderStrong] = palette::border;
        colors[ImGuiCol_TableBorderLight] = palette::control;
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
            palette::text.x,
            palette::text.y,
            palette::text.z,
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
