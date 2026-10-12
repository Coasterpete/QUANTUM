#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <numbers>

namespace quantum::editor
{
    void EditorUi::drawSimulationDynamicsDiagnostics()
    {
        const auto& d = simulationDynamicsDiagnostics_;
        editorPaneHeading("TRAIN DYNAMICS", "Committed state / fixed 1/240 s steps / SI units", fonts_);
        const char* state = simulationPlaybackState_ == SimulationPlaybackState::Playing ? "Playing"
            : simulationPlaybackState_ == SimulationPlaybackState::Paused ? "Paused" : "Stopped";
        ImGui::Text("%s | committed tick %llu", state, static_cast<unsigned long long>(d.tick));
        ImGui::Text("Track %.3f m | straight bogie span %.3f m | body envelope %.3f m",
            d.trackLengthMeters, d.straightBogieSpanMeters, d.bodyEnvelopeLengthMeters);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Train dimensions come from the solved straight reference assembly. Curves change the supported bogie span; body overhang is distinct from the track-coordinate limits.");
        if (!d.placementValid)
        {
            ImGui::TextColored(palette::warning, "Playback unavailable");
            ImGui::TextWrapped("%s", simulationError_.c_str());
            return;
        }
        ImGui::Text("%zu cars / %zu bogies | loaded mass %.1f kg", d.cars.size(), d.cars.size() * 2, d.loadedMassKilograms);
        ImGui::Text("Bogie stations %.3f to %.3f m%s", d.minimumBogieStationMeters, d.maximumBogieStationMeters,
            d.circuit ? " (circuit stations can wrap at zero)" : "");
        ImGui::Text("Signed speed %+.4f m/s | net acceleration %+.4f m/s^2", d.signedSpeedMetersPerSecond, d.accelerationMetersPerSecondSquared);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Net acceleration is the actual fixed-step change in signed train reference speed divided by 1/240 s. Positive speed increases track station; kg measures mass, N measures force.");
        ImGui::Separator();
        if (!d.hasStep) ImGui::TextDisabled("No force sample since Reset or device edit. Play to integrate a fixed step.");
        else
        {
            ImGui::Text("Effective generalized mass %.3f kg", d.effectiveMassKilograms);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Mass seen by the lead reference coordinate, including distributed translation and body rotation. On a straight rail it equals loaded train mass.");
            ImGui::Text("Forces in train coordinate: device %+.1f N | gravity %+.1f N | resistance %+.1f N",
                d.externalForceNewtons, d.gravityForceNewtons, d.resistanceForceNewtons);
            ImGui::Text("Geometry/inertia %+.1f N | constraint/stop correction %+.1f N | resolved net %+.1f N",
                d.massGradientForceNewtons, d.constraintForceNewtons, d.netForceNewtons);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Resolved net = effective mass times actual net acceleration. The correction includes endpoint and zero-speed interventions; it is not an inferred wheel or coupler load.");
            ImGui::Text("Resistance: applied dry %+.1f N | linear %+.1f N | aggregate drag %+.1f N",
                d.resistance.appliedDryResistanceForceNewtons, d.resistance.linearResistanceForceNewtons, d.resistance.aggregateAerodynamicResistanceForceNewtons);
            ImGui::TextDisabled("Dry capacities: bearing %.1f N / rolling %.1f N",
                d.resistance.constantMechanicalForceMagnitudeNewtons, d.resistance.rollingResistanceForceMagnitudeNewtons);
            if (d.boundary != physics::TrackBoundary::None)
                ImGui::TextColored(palette::warning, "Complete-train endpoint restriction at %s; Reset or resume inward motion.",
                    d.boundary == physics::TrackBoundary::Start ? "track start" : "track end");
            else if (d.signedSpeedMetersPerSecond < -physics::followerRestSpeedToleranceMetersPerSecond)
                ImGui::TextColored(palette::warning, "Reverse motion / rollback: train reference speed is negative.");
            else if (std::abs(d.signedSpeedMetersPerSecond) <= physics::followerRestSpeedToleranceMetersPerSecond)
                ImGui::TextDisabled("Train at rest; compare device supply, gravity and dry resistance capacity.");
            else if (d.signedSpeedMetersPerSecond * d.accelerationMetersPerSecondSquared < 0)
                ImGui::TextDisabled("Train slowing: resolved acceleration opposes its signed motion.");
        }
        ImGui::Text("Maximum connector length residual %.3g m", d.maximumConnectorResidualMeters);
        ImGui::TextDisabled("Rigid bogies constrain track geometry. Wheel contact loads are not evaluated in this view.");
        for (const auto& device : simulationDeviceForces_.devices)
        {
            ImGui::Separator();
            ImGui::Text("Device #%llu: %zu occupied bogies | %s", static_cast<unsigned long long>(device.id),
                device.occupiedBogieCount, device.armed ? "armed" : "not armed");
            ImGui::Text("Request %+.1f N | capped tangent supply %+.1f N", device.requestedForceNewtons, device.appliedForceNewtons);
            if (device.generalizedForceNewtons) ImGui::Text("Supplied to physics coordinate %+.1f N", *device.generalizedForceNewtons);
            if (device.forceLimitReached)
            {
                ImGui::TextColored(palette::warning, "Force limit reached (request magnitude %.1f N)", device.requestedAbsoluteForceNewtons);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("The mechanism requested more force than its configured maximum. The cap applies once to the total occupied-bogie demand.");
            }
            if (device.targetSpeedLimited) ImGui::TextDisabled("Target-speed controller limited propulsion.");
            if (!device.occupiedBogieCount) ImGui::TextDisabled("Train outside this device's active interval.");
        }
        for (const auto& hardware : rideHardwareStatus_)
            if (!hardware.interlockReason.empty()) ImGui::TextWrapped("Device #%llu: %s", static_cast<unsigned long long>(hardware.id), hardware.interlockReason.c_str());
        if (ImGui::CollapsingHeader("Car / bogie stations (committed)"))
        {
            if (ImGui::BeginTable("Dynamics cars", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
            {
                for (const auto* label : {"Car ID", "Mass (kg)", "Reference (m)", "Front bogie (m)", "Rear bogie (m)"}) ImGui::TableSetupColumn(label);
                ImGui::TableHeadersRow();
                for (const auto& car : d.cars)
                {
                    ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::Text("%llu", static_cast<unsigned long long>(car.id));
                    for (const double value : {car.loadedMassKilograms, car.referenceStationMeters, car.frontBogieStationMeters, car.rearBogieStationMeters})
                    { ImGui::TableNextColumn(); ImGui::Text("%.3f", value); }
                }
                ImGui::EndTable();
            }
        }
    }

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
        viewportCamera_.setBounds({-7.0, -13.1, -1.0}, {7.0, -11.0, 13.0});
        viewportSettings_.orthographic = false;
        viewportSettings_.applyCameraSettings(viewportCamera_);
        viewportCamera_.setPose({{0.0, -12.0, 6.0},
            -std::numbers::pi / 2.0, 0.15, 24.0});
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
            ImGui::Text("3 box bodies / 2 hinges | tick %llu | arm %+.3f rad/s",
                static_cast<unsigned long long>(rigidBodyProofTick_),
                rigidBodyProofAngularSpeed_);
            ImGui::Checkbox("Show physics/debug bounds", &rigidBodyProofDebugBoundsVisible_);
            ImGui::TextDisabled("Driven GLB arm / passive GLB carrier / gray support. Preview Play/Pause; Reset proof restores both.");
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
