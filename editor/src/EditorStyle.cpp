#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

namespace quantum::editor
{
    void applyQuantumStyle()
    {
        ImGuiStyle& style = ImGui::GetStyle();
        ImGui::StyleColorsDark(&style);
        style.DisabledAlpha = 0.65F;
        style.FrameBorderSize = 1.0F;

        ImVec4* const colors = style.Colors;
        colors[ImGuiCol_Text] = palette::brightestGray;
        colors[ImGuiCol_TextDisabled] = palette::brightestGray;
        colors[ImGuiCol_WindowBg] = palette::darkestGray;
        colors[ImGuiCol_ChildBg] = palette::darkestGray;
        colors[ImGuiCol_PopupBg] = palette::darkestGray;
        colors[ImGuiCol_Border] = palette::black;
        colors[ImGuiCol_BorderShadow] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);

        colors[ImGuiCol_FrameBg] = palette::black;
        colors[ImGuiCol_FrameBgHovered] = palette::darkestGray;
        colors[ImGuiCol_FrameBgActive] = palette::darkestGray;
        colors[ImGuiCol_TitleBg] = palette::black;
        colors[ImGuiCol_TitleBgActive] = palette::darkestGray;
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.0F, 0.0F, 0.0F, 0.8F);
        colors[ImGuiCol_MenuBarBg] = palette::brightestGray;

        colors[ImGuiCol_ScrollbarBg] = palette::black;
        colors[ImGuiCol_ScrollbarGrab] = palette::darkestGray;
        colors[ImGuiCol_ScrollbarGrabHovered] = palette::brightestGray;
        colors[ImGuiCol_ScrollbarGrabActive] = palette::brightestGray;
        colors[ImGuiCol_CheckMark] = palette::brightestGray;
        colors[ImGuiCol_CheckboxSelectedBg] = palette::darkestGray;
        colors[ImGuiCol_SliderGrab] = palette::brightestGray;
        colors[ImGuiCol_SliderGrabActive] = palette::brightestGray;

        colors[ImGuiCol_Button] = palette::black;
        colors[ImGuiCol_ButtonHovered] = palette::darkestGray;
        colors[ImGuiCol_ButtonActive] = palette::darkestGray;
        colors[ImGuiCol_Header] = palette::black;
        colors[ImGuiCol_HeaderHovered] = palette::darkestGray;
        colors[ImGuiCol_HeaderActive] = palette::darkestGray;
        colors[ImGuiCol_Separator] = palette::black;
        colors[ImGuiCol_SeparatorHovered] = palette::brightestGray;
        colors[ImGuiCol_SeparatorActive] = palette::brightestGray;
        colors[ImGuiCol_ResizeGrip] = ImVec4(
            palette::darkestGray.x,
            palette::darkestGray.y,
            palette::darkestGray.z,
            0.35F
        );
        colors[ImGuiCol_ResizeGripHovered] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.75F
        );
        colors[ImGuiCol_ResizeGripActive] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.95F
        );
        colors[ImGuiCol_InputTextCursor] = palette::brightestGray;

        colors[ImGuiCol_Tab] = palette::black;
        colors[ImGuiCol_TabHovered] = palette::darkestGray;
        colors[ImGuiCol_TabSelected] = palette::darkestGray;
        colors[ImGuiCol_TabSelectedOverline] = palette::brightestGray;
        colors[ImGuiCol_TabDimmed] = palette::black;
        colors[ImGuiCol_TabDimmedSelected] = palette::darkestGray;
        colors[ImGuiCol_TabDimmedSelectedOverline] = palette::brightestGray;
        colors[ImGuiCol_DockingPreview] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.35F
        );
        colors[ImGuiCol_DockingEmptyBg] = palette::black;

        colors[ImGuiCol_PlotLines] = palette::brightestGray;
        colors[ImGuiCol_PlotLinesHovered] = palette::brightestGray;
        colors[ImGuiCol_PlotHistogram] = palette::brightestGray;
        colors[ImGuiCol_PlotHistogramHovered] = palette::brightestGray;
        colors[ImGuiCol_TableHeaderBg] = palette::black;
        colors[ImGuiCol_TableBorderStrong] = palette::black;
        colors[ImGuiCol_TableBorderLight] = palette::black;
        colors[ImGuiCol_TableRowBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
        colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.0F, 0.0F, 0.0F, 0.16F);

        colors[ImGuiCol_TextLink] = palette::brightestGray;
        colors[ImGuiCol_TextSelectedBg] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.45F
        );
        colors[ImGuiCol_TreeLines] = palette::brightestGray;
        colors[ImGuiCol_DragDropTarget] = palette::brightestGray;
        colors[ImGuiCol_DragDropTargetBg] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.18F
        );
        colors[ImGuiCol_UnsavedMarker] = palette::brightestGray;
        colors[ImGuiCol_NavCursor] = palette::brightestGray;
        colors[ImGuiCol_NavWindowingHighlight] = ImVec4(
            palette::brightestGray.x,
            palette::brightestGray.y,
            palette::brightestGray.z,
            0.70F
        );
        colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.65F);
        colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.65F);
    }
}
