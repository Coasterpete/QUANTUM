#include <quantum/editor/EditorUi.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <cfloat>
#include <utility>

namespace quantum::editor
{
    std::optional<RideOperationCommand> EditorUi::takeRideOperationCommand()
    {
        auto command = std::move(rideOperationCommand_);
        rideOperationCommand_.reset();
        return command;
    }
    void EditorUi::setRideOperationStatus(const physics::RideOperations& operations)
    {
        rideHardwareStatus_ = operations.hardware();
    }

    void EditorUi::drawHardwareConfiguration(coaster::TrackDevice& device, bool& commit)
    {
        using namespace coaster;
        editorSectionHeading("Operational parameters", fonts_);
        const auto number = [&](const char* label, double& value)
        {
            ImGui::InputDouble(label, &value, 0, 0, "%.4g");
            commit |= ImGui::IsItemDeactivatedAfterEdit();
        };
        const auto direction = [&](HardwareDirection& value)
        {
            int index = value == HardwareDirection::Reverse ? 1 : 0;
            if (ImGui::Combo("Travel direction", &index, "Forward\0Reverse\0"))
            { value = index == 0 ? HardwareDirection::Forward : HardwareDirection::Reverse; commit = true; }
        };
        // Merely opening a historical document must preserve its legacy force
        // semantics. Explicitly enabling parameters upgrades that device only.
        if (std::holds_alternative<std::monostate>(device.configuration))
        {
            editorSecondaryTextWrapped("Legacy acceleration force. Enable operational parameters to configure speed targets and holding.");
            if (ImGui::Button("Enable operational parameters"))
            { device.configuration = makeTrackDevice(device.kind).configuration; commit = true; }
        }
        if (auto* c = std::get_if<LaunchConfiguration>(&device.configuration))
        {
            bool target = c->targetSpeedMetersPerSecond.has_value();
            if (ImGui::Checkbox("Limit to target speed", &target))
            { c->targetSpeedMetersPerSecond = target ? std::optional{10.0} : std::nullopt; commit = true; }
            if (c->targetSpeedMetersPerSecond) number("Target speed (m/s)", *c->targetSpeedMetersPerSecond);
            direction(c->direction);
        }
        else if (auto* c = std::get_if<BrakeConfiguration>(&device.configuration))
        {
            number("Target speed (m/s)", c->targetSpeedMetersPerSecond);
            commit |= ImGui::Checkbox("Hold at target", &c->holding);
        }
        else if (auto* c = std::get_if<TransportConfiguration>(&device.configuration))
        {
            number("Transport speed (m/s)", c->speedMetersPerSecond);
            direction(c->direction);
        }
        else if (auto* c = std::get_if<ChainLiftConfiguration>(&device.configuration))
        {
            number("Chain speed (m/s)", c->speedMetersPerSecond);
            commit |= ImGui::Checkbox("Anti-rollback", &c->antiRollback);
        }
        else if (auto* c = std::get_if<CableLiftConfiguration>(&device.configuration))
        {
            number("Cable speed (m/s)", c->speedMetersPerSecond);
            number("Return time (s)", c->returnSeconds);
            commit |= ImGui::Checkbox("Initially available", &c->initiallyAvailable);
        }
        else if (auto* c = std::get_if<StationConfiguration>(&device.configuration))
        {
            number("Stop station (m)", c->stopStationMeters);
            number("Alignment tolerance (m)", c->alignmentToleranceMeters);
            number("Stopped tolerance (m/s)", c->stoppedToleranceMetersPerSecond);
            number("Dispatch speed (m/s)", c->transportSpeedMetersPerSecond);
            const char* blockName = "None";
            for (const auto& other : authoredTrack_->trackDevices().devices)
                if (other.id == c->downstreamBlock) blockName = other.name.c_str();
            if (ImGui::BeginCombo("Downstream block", blockName))
            {
                if (ImGui::Selectable("None", c->downstreamBlock == 0))
                { c->downstreamBlock = 0; commit = true; }
                for (const auto& other : authoredTrack_->trackDevices().devices)
                    if (other.kind == TrackDeviceKind::Block)
                    {
                        ImGui::PushID(static_cast<int>(other.id));
                        if (ImGui::Selectable(other.name.c_str(), c->downstreamBlock == other.id))
                        { c->downstreamBlock = other.id; commit = true; }
                        ImGui::PopID();
                    }
                ImGui::EndCombo();
            }
            commit |= ImGui::Checkbox("Require locked access gates", &c->requireLockedGates);
            ImGui::TextDisabled("%zu saved access gate references", c->accessGates.size());
        }
        else if (std::holds_alternative<BlockConfiguration>(device.configuration))
            editorSecondaryTextWrapped("Occupancy sensor used by station dispatch interlocks.");
        else if (device.kind == TrackDeviceKind::Transfer || device.kind == TrackDeviceKind::Access)
            editorSecondaryTextWrapped("Existing configuration is preserved. Transfer routing and multi-train operation remain experimental; this panel does not author them.");
    }

    void EditorUi::drawRideOperationStatus()
    {
        if (const auto* settings = ImGui::FindWindowSettingsByID(ImHashStr("Track Devices"));
            settings && settings->DockId != 0)
            ImGui::SetNextWindowDockID(settings->DockId, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints({300 * editorPresentationScale(), 260 * editorPresentationScale()},
            {FLT_MAX, FLT_MAX});
        if (!ImGui::Begin("Ride Operations")) { ImGui::End(); return; }
        editorPaneHeading("RIDE OPERATIONS", "Live preview / runtime commands", fonts_);
        drawRideOperationControls();
        ImGui::End();
    }

    void EditorUi::drawRideOperationControls()
    {
        if (!simulationAvailable_)
            editorSecondaryTextWrapped("No accepted simulation preview is available.");
        else if (rideHardwareStatus_.empty())
            editorSecondaryTextWrapped("Add a launch, brake, lift or station in Track Devices.");
        ImGui::BeginDisabled(!simulationAvailable_);
        for (const auto& runtime : rideHardwareStatus_)
        {
            ImGui::PushID(static_cast<int>(runtime.id));
            const coaster::TrackDevice* authored = nullptr;
            if (authoredTrack_) for (const auto& d : authoredTrack_->trackDevices().devices)
                if (d.id == runtime.id) { authored = &d; break; }
            const auto label = (authored ? authored->name : "Device") + std::string("###runtime") + std::to_string(runtime.id);
            if (ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::TextColored(runtime.interlockReason.empty() ? palette::success : palette::warning,
                    "%s", runtime.state.c_str());
                if (!runtime.interlockReason.empty()) ImGui::TextWrapped("%s", runtime.interlockReason.c_str());
                ImGui::Text("Occupied: %zu trains / %zu bogies", runtime.occupancy.trains.size(), runtime.occupancy.bogies.size());
                ImGui::Text("Actual %+.3f m/s | command %+.3f m/s", runtime.actualSpeedMetersPerSecond, runtime.commandedSpeedMetersPerSecond);
                ImGui::Text("Command %+.3f m/s^2 | applied %+.1f N", runtime.commandedAccelerationMetersPerSecondSquared, runtime.appliedForceNewtons);
                if (runtime.targetSpeedLimited) ImGui::TextDisabled("Target speed limiting active");
                bool enabled = runtime.enabled;
                if (ImGui::Checkbox("Runtime enabled", &enabled))
                    rideOperationCommand_ = {runtime.id, physics::HardwareCommand::SetEnabled, enabled ? 1.0 : 0.0};
                if (const auto* station = std::get_if<physics::StationRuntime>(&runtime.mechanism))
                {
                    ImGui::Text("Aligned: %s | stopped: %s | dispatch: %s", runtime.aligned ? "yes" : "no",
                        runtime.stopped ? "yes" : "no", station->dispatchPermitted ? "permitted" : "interlocked");
                    if (ImGui::Button("Request dispatch"))
                        rideOperationCommand_ = {runtime.id, physics::HardwareCommand::RequestDispatch};
                }
                if (const auto* cable = std::get_if<physics::CableRuntime>(&runtime.mechanism))
                {
                    ImGui::Text("Return remaining: %.2f s", cable->returnRemainingSeconds);
                    bool available = cable->available;
                    if (ImGui::Checkbox("Cable available", &available))
                        rideOperationCommand_ = {runtime.id, physics::HardwareCommand::SetAvailable, available ? 1.0 : 0.0};
                }
                if (const auto* chain = std::get_if<physics::ChainRuntime>(&runtime.mechanism))
                {
                    bool available = chain->available;
                    if (ImGui::Checkbox("Chain available", &available))
                        rideOperationCommand_ = {runtime.id, physics::HardwareCommand::SetAvailable, available ? 1.0 : 0.0};
                }
                if (authored && (authored->kind == coaster::TrackDeviceKind::Transfer
                    || authored->kind == coaster::TrackDeviceKind::Access))
                    editorSecondaryTextWrapped("Experimental mechanism state; no production transfer or multi-train routing claim.");
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        ImGui::EndDisabled();
        editorSecondaryTextWrapped("Runtime commands do not edit the document. Reset restores authored device settings.");
    }
}
