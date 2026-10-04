#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <numbers>

namespace quantum::editor
{
    std::optional<RigidBodyProofControlType> EditorUi::takeRigidBodyProofControl() noexcept
    {
        const auto control = pendingRigidBodyProofControl_;
        pendingRigidBodyProofControl_.reset();
        return control;
    }

    void EditorUi::setRigidBodyProofStatus(const bool enabled, const std::uint64_t tick,
        const double angularSpeed, const std::string& error)
    {
        rigidBodyProofEnabled_ = enabled;
        rigidBodyProofTick_ = tick;
        rigidBodyProofAngularSpeed_ = angularSpeed;
        rigidBodyProofError_ = error;
        if (!enabled && cameraBeforeRigidBodyProof_)
        {
            viewportCamera_ = *cameraBeforeRigidBodyProof_;
            viewportSettings_.orthographic = viewportCamera_.projection()
                == ViewportProjection::Orthographic;
            cameraBeforeRigidBodyProof_.reset();
        }
    }

    void EditorUi::frameRigidBodyProof()
    {
        refreshViewportDisplayBounds();
        if (!cameraBeforeRigidBodyProof_)
            cameraBeforeRigidBodyProof_ = viewportCamera_;
        // Clip against the proof's full rotation sweep, even if the authored
        // track is far from this diagnostic mechanism.
        viewportCamera_.setBounds({-5.0, -13.0, 1.0}, {5.0, -11.0, 11.0});
        viewportSettings_.orthographic = false;
        viewportSettings_.applyCameraSettings(viewportCamera_);
        viewportCamera_.setPose({{0.0, -12.0, 6.0},
            -std::numbers::pi / 2.0, 0.15, 18.0});
        initialViewportFramePending_ = false;
    }

    void EditorUi::drawRigidBodyProofControls()
    {
        bool enabled = rigidBodyProofEnabled_;
        if (ImGui::Checkbox("Rigid-body mechanical proof (development)", &enabled))
        {
            pendingRigidBodyProofControl_ = enabled
                ? RigidBodyProofControlType::Enable : RigidBodyProofControlType::Disable;
            if (enabled)
                frameRigidBodyProof();
        }
        if (rigidBodyProofEnabled_)
        {
            ImGui::SameLine();
            if (ImGui::Button("Frame proof"))
                frameRigidBodyProof();
            ImGui::SameLine();
            if (ImGui::Button("Reset proof"))
                pendingRigidBodyProofControl_ = RigidBodyProofControlType::Reset;
            ImGui::Text("2 box bodies / 1 world hinge | tick %llu | arm %+.3f rad/s",
                static_cast<unsigned long long>(rigidBodyProofTick_),
                rigidBodyProofAngularSpeed_);
            ImGui::TextDisabled("Wire colliders: support (gray), arm (orange). Uses preview Play/Pause; Reset proof restores the arm.");
        }
        if (!rigidBodyProofError_.empty())
            ImGui::TextColored(palette::warning, "%s", rigidBodyProofError_.c_str());
    }

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
