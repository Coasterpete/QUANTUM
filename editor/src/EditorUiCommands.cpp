#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/CoasterSetupUi.hpp>

#include <imgui_internal.h>

#include <algorithm>

namespace quantum::editor
{
    float EditorUi::drawWorkspaceCommandArea(ImGuiViewport* const mainViewport)
    {
        const ImGuiStyle& style = ImGui::GetStyle();
        const float buttonHeight = std::max(ImGui::GetFrameHeight(),
            icons_.metrics(EditorIcon::Open).buttonExtent);
        const float height = style.WindowPadding.y * 2.0F
            + editorHeaderFontSize * editorPresentationScale()
            + style.ItemSpacing.y + buttonHeight + style.ScrollbarSize;
        constexpr auto flags = ImGuiWindowFlags_NoDocking
            | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;
        ImGui::PushStyleColor(ImGuiCol_WindowBg, palette::toolbar);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, palette::toolbar);
        if (ImGui::BeginViewportSideBar("##QuantumCommandArea", mainViewport,
            ImGuiDir_Up, height, flags))
        {
            // A narrow window scrolls the same compact groups horizontally.
            // Each button below reuses an existing request or pane action.
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0F);
            ImGui::PushStyleColor(ImGuiCol_Button, palette::panelRaised);
            ImGui::BeginChild("Workspace commands", {0, 0}, ImGuiChildFlags_None,
                ImGuiWindowFlags_HorizontalScrollbar);
            const bool train = editorWorkspace_ == EditorWorkspace::Train;
            ImGui::BeginGroup();
            ImGui::PushFont(fonts_.header, editorHeaderFontSize);
            ImGui::TextColored(palette::accent, "%s", train ? "TRAIN" : "TRACK");
            ImGui::PopFont();
            ImGui::TextDisabled("%s", train ? "Consist / preview" : "Author / inspect");
            ImGui::EndGroup();
            const auto heading = [&](const char* label)
            {
                ImGui::SameLine(0.0F, style.ItemSpacing.x * 1.5F);
                const ImVec2 divider = ImGui::GetCursorScreenPos();
                const float groupHeight = ImGui::GetTextLineHeight()
                    + style.ItemSpacing.y + buttonHeight;
                ImGui::Dummy({editorPresentationScale(), groupHeight});
                ImGui::GetWindowDrawList()->AddLine(divider,
                    {divider.x, divider.y + groupHeight},
                    ImGui::GetColorU32(palette::border));
                ImGui::SameLine(0.0F, style.ItemSpacing.x * 1.5F);
                ImGui::BeginGroup();
                editorSecondaryText("%s", label);
            };
            const auto button = [&](const char* label)
            {
                return ImGui::Button(label, {0, buttonHeight});
            };
            const auto next = [&] { ImGui::SameLine(0.0F, style.ItemInnerSpacing.x); };
            const auto enterSimulator = [&]
            {
                workspaceMode_ = WorkspaceMode::Simulator;
                cameraGesture_ = CameraGesture::None;
                viewportNavigationActive_ = false;
            };
            heading("Document");
            if (button("New")) pendingFileOperation_ = FileOperationType::New;
            next();
            if (icons_.button(EditorIcon::Open, "##OpenDocument", "Open (Ctrl+O)"))
                pendingFileOperation_ = FileOperationType::Open;
            next();
            if (icons_.button(EditorIcon::Save, "##SaveDocument", "Save (Ctrl+S)"))
                pendingFileOperation_ = FileOperationType::Save;
            next();
            if (icons_.button(EditorIcon::Undo, "##UndoDocumentEdit", "Undo (Ctrl+Z)", false, canUndo_))
                pendingHistoryOperation_ = HistoryOperationType::Undo;
            next();
            if (icons_.button(EditorIcon::Redo, "##RedoDocumentEdit", "Redo (Ctrl+Y)", false, canRedo_))
                pendingHistoryOperation_ = HistoryOperationType::Redo;
            ImGui::EndGroup();

            if (train)
            {
                heading("Configuration");
                if (button("Configure")) ImGui::SetWindowFocus("Train Configuration");
                next();
                if (button("Coaster Setup"))
                {
                    coasterSetupWindowOpen_ = true;
                    ImGui::SetWindowFocus(trainCoasterSetupWindowName);
                }
                ImGui::EndGroup();
                heading("Preview");
                ImGui::BeginDisabled(!simulationAvailable_
                    || simulationPlaybackState_ == SimulationPlaybackState::Playing);
                if (icons_.button(EditorIcon::Play, "##TrainPlay", "Play preview"))
                    pendingSimulationControl_ = SimulationControlType::Play;
                ImGui::EndDisabled();
                next();
                ImGui::BeginDisabled(!simulationAvailable_
                    || simulationPlaybackState_ != SimulationPlaybackState::Playing);
                if (icons_.button(EditorIcon::Pause, "##TrainPause", "Pause preview"))
                    pendingSimulationControl_ = SimulationControlType::Pause;
                ImGui::EndDisabled();
                next();
                ImGui::BeginDisabled(!simulationAvailable_);
                if (button("Reset")) pendingSimulationControl_ = SimulationControlType::Reset;
                ImGui::EndDisabled();
                next();
                if (button("Frame All")) frameWholeTrack();
                ImGui::EndGroup();
                heading("Analyze");
                if (button("Summary")) ImGui::SetWindowFocus("Train Physical Definition");
                next();
                if (button("Simulator")) enterSimulator();
                ImGui::EndGroup();
            }
            else
            {
                heading("Route");
                if (button("Add Region..."))
                {
                    regionCreateFlow_.choicePending = true;
                    regionCreateFlow_.revealChoices = true;
                    regionCreateFlow_.anchor = RegionCreateAnchor::Append;
                    ImGui::SetWindowFocus("Track Workspace###TRACK WORKSPACE");
                }
                ImGui::EndGroup();
                heading("Geometry / Transitions");
                if (button("Edit Selected"))
                {
                    const bool geometry = authoredTrack_ != nullptr
                        && selectedSection_ < authoredTrack_->sectionCount()
                        && authoredTrack_->section(selectedSection_).kind == coaster::RegionKind::Geometry;
                    ImGui::SetWindowFocus(geometry ? "Geometry Editor" : "Transition Editor");
                }
                next();
                if (button("Input Settings")) inputSettingsWindowOpen_ = true;
                ImGui::EndGroup();
                heading("Devices / Supports");
                if (button("Devices")) ImGui::SetWindowFocus("Track Devices");
                next();
                if (button("Supports")) ImGui::SetWindowFocus("Supports###Support Workspace");
                ImGui::EndGroup();
                heading("Analyze");
                if (button("Forces"))
                {
                    riderLoadDiagnosticsWindowOpen_ = true;
                    ImGui::SetWindowFocus("Force Diagnostics");
                }
                next();
                if (button("Simulator")) enterSimulator();
                ImGui::EndGroup();
            }
            heading("View");
            if (button("Camera")) ImGui::OpenPopup("Command camera");
            if (ImGui::BeginPopup("Command camera"))
            {
                showViewportViewMenuItems();
                showViewportAxisMenuItems();
                ImGui::EndPopup();
            }
            ImGui::EndGroup();
            ImGui::EndChild();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar(2);
        }
        ImGui::End();
        ImGui::PopStyleColor(2);
        return height;
    }
}
