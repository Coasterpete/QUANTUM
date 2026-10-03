#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/CoasterSetupUi.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>

namespace quantum::editor
{
    void EditorUi::setTrainPreviewInspection(
        std::optional<TrainPreviewInspection> inspection)
    {
        trainPreviewInspection_ = std::move(inspection);
    }

    namespace
    {
        void trainTitle(const char* title, const EditorFonts& fonts)
        {
            ImGui::PushFont(fonts.header, editorHeaderFontSize);
            ImGui::TextColored(palette::textHeading, "%s", title);
            ImGui::PopFont();
            ImGui::Spacing();
        }

        void trainValue(const EditorFonts& fonts, const char* label,
            const char* value, const char* unit)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            editorSecondaryText("%s", label);
            ImGui::TableNextColumn();
            ImGui::PushFont(fonts.technical, editorTechnicalFontSize);
            const float width = ImGui::CalcTextSize(value).x;
            ImGui::SetCursorPosX(ImGui::GetCursorPosX()
                + std::max(0.0F, ImGui::GetContentRegionAvail().x - width));
            ImGui::TextUnformatted(value);
            ImGui::PopFont();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", unit);
        }

        void trainNumber(const EditorFonts& fonts, const char* label,
            const double value, const char* unit, const int precision = 2)
        {
            char text[64]{};
            std::snprintf(text, sizeof(text), "%.*f", precision, value);
            trainValue(fonts, label, text, unit);
        }

        bool beginTrainValues(const char* id)
        {
            if (!ImGui::BeginTable(id, 3, ImGuiTableFlags_SizingStretchProp))
                return false;
            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Unit", ImGuiTableColumnFlags_WidthFixed);
            return true;
        }
    }

    void EditorUi::drawTrainWorkspace()
    {
        if (ImGui::Begin("Train Configuration"))
        {
            trainTitle("TRAIN CONFIGURATION", fonts_);
            ImGui::TextColored(palette::accent, "AUTHORED / editable");
            if (authoredTrack_ != nullptr)
            {
                // Borrow the committed setup or this frame's shared candidate.
                // Application still accepts it through document transactions.
                auto draft = pendingCoasterSetupEdit_.value_or(
                    authoredTrack_->coasterSetup());
                if (drawCarsPerTrainInput(draft))
                    pendingCoasterSetupEdit_ = std::move(draft);
                editorSecondaryTextWrapped("Saved in Coaster Setup. Document Undo/Redo.");
            }
            ImGui::Spacing();
            if (ImGui::Button("Open Coaster Setup"))
            {
                coasterSetupWindowOpen_ = true;
                ImGui::SetWindowFocus(trainCoasterSetupWindowName);
            }
            ImGui::Spacing();
            editorHeading("Accepted preview", fonts_);
            ImGui::TextDisabled("RESOLVED / read only");
            if (simulationAvailable_ && trainPreviewInspection_)
            {
                ImGui::TextColored(palette::success, "Preview ready");
                if (beginTrainValues("Configuration readouts"))
                {
                    trainNumber(fonts_, "Resolved cars", static_cast<double>(
                        trainPreviewInspection_->carCount), "cars", 0);
                    trainNumber(fonts_, "Loaded train mass",
                        trainPreviewInspection_->totalTrainMassKilograms, "kg", 1);
                    ImGui::EndTable();
                }
            }
            else
            {
                ImGui::TextColored(palette::warning, "Preview unavailable");
                if (!simulationError_.empty())
                    ImGui::TextWrapped("%s", simulationError_.c_str());
            }
            ImGui::Spacing();
            editorSecondaryTextWrapped("One backend car definition is repeated across the consist.");
        }
        ImGui::End();

        // The suffix retains Recovery M1B's persisted window identity.
        if (ImGui::Begin("Train Summary###Train Physical Definition"))
        {
            trainTitle("TRAIN SUMMARY", fonts_);
            ImGui::TextDisabled("RESOLVED / accepted preview");
            if (!simulationAvailable_ || !trainPreviewInspection_)
                ImGui::TextDisabled("No accepted preview definition.");
            else
            {
                const auto& inspection = *trainPreviewInspection_;
                const auto& car = inspection.repeatedCar.car;
                const auto& loadout = inspection.repeatedCar.loadout;
                if (beginTrainValues("Resolved summary"))
                {
                    trainNumber(fonts_, "Cars", static_cast<double>(inspection.carCount), "cars", 0);
                    trainNumber(fonts_, "Loaded mass / car", inspection.loadedCarMassKilograms, "kg", 1);
                    trainNumber(fonts_, "Total train mass", inspection.totalTrainMassKilograms, "kg", 1);
                    if (inspection.connectorLengthMeters)
                        trainNumber(fonts_, "Connector length", *inspection.connectorLengthMeters, "m");
                    ImGui::EndTable();
                }
                ImGui::Spacing();
                if (ImGui::CollapsingHeader("Physical definition / backend defaults"))
                {
                    ImGui::TextDisabled("READ ONLY / not authored in this document");
                    if (beginTrainValues("Physical defaults"))
                    {
                        trainNumber(fonts_, "Dry mass / car", car.dryMassKilograms, "kg", 1);
                        trainNumber(fonts_, "Load mass / car", loadout.massKilograms, "kg", 1);
                        const auto vector = [&](const char* label, const glm::dvec3& value)
                        {
                            char text[96]{};
                            std::snprintf(text, sizeof(text), "%.2f / %.2f / %.2f", value.x, value.y, value.z);
                            trainValue(fonts_, label, text, "m");
                        };
                        vector("Body dimensions", car.bodyDimensionsMeters);
                        vector("Dry centre of gravity", car.dryCenterOfGravityMeters);
                        vector("Load centre of mass", loadout.centerOfMassMeters);
                        for (std::size_t index = 0; index < car.bogies.size(); ++index)
                        {
                            const std::string label = "Bogie " + std::to_string(index + 1);
                            vector(label.c_str(), car.bogies[index].referencePositionMeters);
                        }
                        vector("Front hitch", car.frontHitchPositionMeters);
                        vector("Rear hitch", car.rearHitchPositionMeters);
                        ImGui::EndTable();
                    }
                    editorSecondaryTextWrapped("Local axes: +X forward / +Y lateral / +Z up.");
                    editorSecondaryTextWrapped("Style and restraint metadata do not yet select a physical car.");
                }
                if (ImGui::CollapsingHeader("Resistance / backend defaults"))
                {
                    ImGui::TextDisabled("READ ONLY / whole train unless marked per car");
                    if (beginTrainValues("Resistance defaults"))
                    {
                        const auto& resistance = inspection.resistance;
                        trainNumber(fonts_, "Mechanical force", resistance.constantMechanicalForceNewtons, "N", 1);
                        trainNumber(fonts_, "Linear coefficient", resistance.linearResistanceCoefficientNewtonSecondsPerMeter, "N s/m", 1);
                        trainNumber(fonts_, "Air density", resistance.airDensityKilogramsPerCubicMeter, "kg/m^3", 3);
                        trainNumber(fonts_, "Aggregate drag area", resistance.dragAreaSquareMeters, "m^2");
                        trainNumber(fonts_, "Rolling coefficient", resistance.rollingResistanceCoefficient, "", 3);
                        trainNumber(fonts_, "Drag area / car", car.aerodynamicDragAreaSquareMeters, "m^2");
                        ImGui::EndTable();
                    }
                }
            }
        }
        ImGui::End();
    }

    void EditorUi::drawTrainViewportToolbar()
    {
        trainTitle("TRAIN PREVIEW", fonts_);
        const char* state = simulationPlaybackState_ == SimulationPlaybackState::Playing
            ? "Playing" : simulationPlaybackState_ == SimulationPlaybackState::Paused
                ? "Paused" : "Stopped";
        ImGui::TextColored(simulationAvailable_ ? palette::success : palette::warning,
            "%s", simulationAvailable_ ? state : "Unavailable");
        ImGui::SameLine();
        ImGui::PushFont(fonts_.technical, editorTechnicalFontSize);
        ImGui::Text("%.2f m/s", simulationSpeedMps_);
        ImGui::PopFont();
        ImGui::SameLine();
        if (ImGui::Button("View")) ImGui::OpenPopup("Train View");
        if (ImGui::BeginPopup("Train View"))
        {
            showViewportViewMenuItems();
            showViewportAxisMenuItems();
            ImGui::EndPopup();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Diagnostic car boxes, bogies and connectors. Right-drag: look; Alt+right-drag: orbit; middle-drag: pan; wheel: zoom.");
        ImGui::Spacing();
    }
}
