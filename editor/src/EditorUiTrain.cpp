#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/CoasterSetupUi.hpp>

#include <imgui.h>

#include <utility>

namespace quantum::editor
{
    void EditorUi::setTrainPreviewInspection(
        std::optional<TrainPreviewInspection> inspection)
    {
        trainPreviewInspection_ = std::move(inspection);
    }

    void EditorUi::drawTrainWorkspace()
    {
        if (ImGui::Begin("Train Configuration"))
        {
            editorHeading("Consist", fonts_);
            if (authoredTrack_ != nullptr)
            {
                // Start from committed setup, or this frame's shared setup
                // candidate. Neither surface retains a second car-count value.
                auto draft = pendingCoasterSetupEdit_.value_or(
                    authoredTrack_->coasterSetup());
                if (drawCarsPerTrainInput(draft))
                    pendingCoasterSetupEdit_ = std::move(draft);
                editorSecondaryTextWrapped(
                    "Cars per train is saved in Coaster Setup. Accepted edits "
                    "use document Undo/Redo. The preview repeats one physical car.");
            }
            if (ImGui::Button("Open Coaster Setup"))
            {
                coasterSetupWindowOpen_ = true;
                ImGui::SetWindowFocus(trainCoasterSetupWindowName);
            }

            ImGui::Separator();
            editorHeading("Diagnostic preview", fonts_);
            if (simulationAvailable_)
            {
                const char* const state = simulationPlaybackState_
                    == SimulationPlaybackState::Playing ? "Playing"
                    : simulationPlaybackState_ == SimulationPlaybackState::Paused
                        ? "Paused" : "Stopped";
                ImGui::Text("Status: %s", state);
                ImGui::Text("Speed: %.2f m/s", simulationSpeedMps_);
                if (trainPreviewInspection_)
                    ImGui::Text("Resolved cars: %zu", trainPreviewInspection_->carCount);
            }
            else
            {
                ImGui::TextColored(palette::warning, "Status: Unavailable");
                if (!simulationError_.empty())
                    ImGui::TextWrapped("%s", simulationError_.c_str());
            }
            ImGui::BeginDisabled(!simulationAvailable_
                || simulationPlaybackState_ == SimulationPlaybackState::Playing);
            if (ImGui::Button("Play"))
                pendingSimulationControl_ = SimulationControlType::Play;
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!simulationAvailable_
                || simulationPlaybackState_ != SimulationPlaybackState::Playing);
            if (ImGui::Button("Pause"))
                pendingSimulationControl_ = SimulationControlType::Pause;
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!simulationAvailable_);
            if (ImGui::Button("Reset"))
                pendingSimulationControl_ = SimulationControlType::Reset;
            ImGui::EndDisabled();
            editorSecondaryTextWrapped(
                "Boxes, bogie markers, and connector lines share the existing "
                "Simulation Preview with Track and Simulator.");
        }
        ImGui::End();

        if (ImGui::Begin("Train Physical Definition"))
        {
            editorHeading("Resolved preview inputs", fonts_);
            editorSecondaryTextWrapped(
                "Read only. Physical car, loadout, connectors, and resistance "
                "are backend defaults, not saved in this document. Coaster "
                "Setup's style and restraint metadata do not select a different "
                "physical car yet.");
            if (!simulationAvailable_ || !trainPreviewInspection_)
            {
                ImGui::TextDisabled("No accepted preview definition is available.");
            }
            else
            {
                const auto& inspection = *trainPreviewInspection_;
                const auto& car = inspection.repeatedCar.car;
                const auto& loadout = inspection.repeatedCar.loadout;
                ImGui::Text("Repeated cars: %zu", inspection.carCount);
                ImGui::Text("Dry mass / car: %.1f kg", car.dryMassKilograms);
                ImGui::Text("Load mass / car: %.1f kg", loadout.massKilograms);
                ImGui::Text("Total mass / car: %.1f kg", inspection.loadedCarMassKilograms);
                ImGui::Text("Total train mass: %.1f kg", inspection.totalTrainMassKilograms);
                ImGui::Separator();
                ImGui::TextWrapped("Local physical axes: +X forward, +Y lateral, +Z up. Dimensions and positions are in metres.");
                const auto vectorText = [](const char* label, const glm::dvec3& value)
                {
                    ImGui::Text("%s: (%.2f, %.2f, %.2f)",
                        label, value.x, value.y, value.z);
                };
                vectorText("Body dimensions", car.bodyDimensionsMeters);
                vectorText("Dry centre of gravity", car.dryCenterOfGravityMeters);
                vectorText("Load centre of mass", loadout.centerOfMassMeters);
                for (std::size_t index = 0; index < car.bogies.size(); ++index)
                {
                    ImGui::Text("Bogie %zu reference", index + 1);
                    vectorText("  Position", car.bogies[index].referencePositionMeters);
                }
                vectorText("Front hitch", car.frontHitchPositionMeters);
                vectorText("Rear hitch", car.rearHitchPositionMeters);
                if (inspection.connectorLengthMeters)
                    ImGui::Text("Connector length: %.2f m", *inspection.connectorLengthMeters);
                else
                    ImGui::TextDisabled("Connector: not applicable to one car");
                ImGui::Separator();
                editorHeading("Resistance (whole train)", fonts_);
                const auto& resistance = inspection.resistance;
                ImGui::Text("Mechanical force: %.1f N", resistance.constantMechanicalForceNewtons);
                ImGui::Text("Linear coefficient: %.1f N s/m",
                    resistance.linearResistanceCoefficientNewtonSecondsPerMeter);
                ImGui::Text("Air density: %.3f kg/m^3", resistance.airDensityKilogramsPerCubicMeter);
                ImGui::Text("Aggregate drag area: %.2f m^2", resistance.dragAreaSquareMeters);
                ImGui::Text("Rolling coefficient: %.3f", resistance.rollingResistanceCoefficient);
                ImGui::Text("Per-car drag area: %.2f m^2", car.aerodynamicDragAreaSquareMeters);
            }
        }
        ImGui::End();
    }

    void EditorUi::drawTrainViewportToolbar()
    {
        if (ImGui::Button("Frame All"))
            frameWholeTrack();
        ImGui::SameLine();
        if (ImGui::Button("View"))
            ImGui::OpenPopup("Train View");
        if (ImGui::BeginPopup("Train View"))
        {
            showViewportViewMenuItems();
            showViewportAxisMenuItems();
            ImGui::EndPopup();
        }
        editorSecondaryTextWrapped("Diagnostic train preview | right-drag to look, Alt+right-drag to orbit, middle-drag to pan, wheel to zoom.");
    }
}
