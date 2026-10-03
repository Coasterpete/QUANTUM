#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>

namespace quantum::editor
{
    void EditorUi::drawSimulationTelemetry()
    {
        const ImVec2 imageMinimum = ImGui::GetItemRectMin();
        const ImVec2 imageMaximum = ImGui::GetItemRectMax();
        if (imageMaximum.x <= imageMinimum.x
            || imageMaximum.y <= imageMinimum.y)
        {
            return;
        }

        char status[512]{};
        ImU32 textColor = ImGui::ColorConvertFloat4ToU32(
            palette::textPrimary);
        if (!simulationAvailable_)
        {
            std::snprintf(
                status,
                sizeof(status),
                "%s",
                simulationError_.empty()
                    ? "PREVIEW  Waiting for simulation status"
                    : simulationError_.c_str());
            textColor = ImGui::ColorConvertFloat4ToU32(palette::warning);
        }
        else
        {
            const char* state = "Stopped";
            switch (simulationPlaybackState_)
            {
            case SimulationPlaybackState::Stopped:
                break;
            case SimulationPlaybackState::Playing:
                state = "Playing";
                break;
            case SimulationPlaybackState::Paused:
                state = "Paused";
                break;
            }
            constexpr double milesPerHourPerMeterPerSecond =
                2.2369362920544;
            std::snprintf(
                status,
                sizeof(status),
                "PREVIEW  %s  |  %.2f m/s  |  %.1f mph",
                state,
                simulationSpeedMps_,
                simulationSpeedMps_
                    * milesPerHourPerMeterPerSecond);
        }

        const float scale = editorPresentationScale();
        const float margin = viewportStyle::overlayMargin * scale;
        const float padding = viewportStyle::overlayPadding * scale;
        const float wrapWidth = std::max(
            1.0F,
            std::min(420.0F * scale,
                imageMaximum.x - imageMinimum.x
                    - 2.0F * (margin + padding)));
        const ImVec2 textSize = ImGui::CalcTextSize(
            status, nullptr, false, wrapWidth);
        const ImVec2 panelMinimum{
            imageMinimum.x + margin,
            imageMinimum.y + margin
        };
        const ImVec2 panelMaximum{
            panelMinimum.x + textSize.x + 2.0F * padding,
            panelMinimum.y + textSize.y + 2.0F * padding
        };
        ImDrawList* const drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(
            panelMinimum,
            panelMaximum,
            ImGui::ColorConvertFloat4ToU32(palette::panelRaised),
            padding);
        drawList->AddText(
            ImGui::GetFont(),
            ImGui::GetFontSize(),
            {panelMinimum.x + padding, panelMinimum.y + padding},
            textColor,
            status,
            nullptr,
            wrapWidth);
    }
}
