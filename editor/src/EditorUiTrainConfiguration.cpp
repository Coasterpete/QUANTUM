#include <quantum/editor/EditorUi.hpp>
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cstdio>

namespace quantum::editor
{
    namespace
    {
        bool textField(const char* label, std::string& value)
        {
            std::array<char, 1024> buffer{};
            std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
            if (!ImGui::InputText(label, buffer.data(), buffer.size())) return false;
            value = buffer.data();
            return true;
        }
        bool number(const char* label, double& value)
        {
            return ImGui::InputDouble(label, &value, 0, 0, "%.6g");
        }
        bool vector(const char* label, glm::dvec3& value)
        {
            return ImGui::InputScalarN(label, ImGuiDataType_Double,
                &value.x, 3, nullptr, nullptr, "%.6g");
        }
        constexpr std::array roles{coaster::TrainCarRole::Lead,
            coaster::TrainCarRole::Middle, coaster::TrainCarRole::Rear,
            coaster::TrainCarRole::Zero, coaster::TrainCarRole::Trailer,
            coaster::TrainCarRole::Custom};
        bool roleField(const char* label, int& index)
        {
            bool changed = false;
            if (ImGui::BeginCombo(label, coaster::trainCarRoleName(roles[index])))
            {
                for (int i = 0; i < static_cast<int>(roles.size()); ++i)
                    if (ImGui::Selectable(coaster::trainCarRoleName(roles[i]), i == index))
                    { index = i; changed = true; }
                ImGui::EndCombo();
            }
            return changed;
        }
    }

    std::optional<coaster::TrainConfiguration> EditorUi::takeTrainConfigurationEdit()
    {
        auto edit = std::move(trainConfigurationEdit_);
        trainConfigurationEdit_.reset();
        return edit;
    }
    void EditorUi::setTrainConfigurationEditError(std::string error)
    {
        trainEditError_ = std::move(error);
        if (trainEditError_.empty()) trainDraftDirty_ = false;
    }

    void EditorUi::requestNativeAuthoringActionForPreviewSmoke(const unsigned action)
    {
        if (!authoredTrack_ || authoredTrack_->sectionCount() < 3
            || authoredTrack_->section(0).kind != coaster::RegionKind::Geometry
            || coaster::isForceDrivenSection(authoredTrack_->section(0))
            || !coaster::isForceDrivenSection(authoredTrack_->section(1))
            || authoredTrack_->section(2).kind != coaster::RegionKind::RateProfiles)
            throw std::invalid_argument("Native authoring smoke requires the Arc/Force/Profile workflow fixture.");
        using namespace coaster;
        switch (action)
        {
        case 0:
            selectSection(0, true);
            break;
        case 1:
            regionCommand_ = {RegionCommandType::SetPlanarArcRadius, 0, 1200};
            break;
        case 2:
        {
            selectSection(1, true);
            const auto& profile = sectionProfileChannel(authoredTrack_->section(1), ProfileChannel::Pitch);
            profileEndpointValueEdit_ = {ScalarProfileEndpoint::End, 1.01, false,
                1, ProfileChannel::Pitch, profile.segments.back().id};
            break;
        }
        case 3:
        {
            selectSection(2, true);
            const auto& profile = sectionProfileChannel(authoredTrack_->section(2), ProfileChannel::Pitch);
            profileTransitionTypeEdit_ = {math::TransitionType::Smootherstep,
                2, ProfileChannel::Pitch, profile.segments.back().id};
            break;
        }
        case 4:
        {
            editorWorkspace_ = EditorWorkspace::Train;
            trainConfigurationInitialFocusPending_ = true;
            auto train = authoredTrack_->trainConfiguration();
            train.cars.at(0).physical.car.dryMassKilograms += 25;
            train.cars.at(0).visualAlignment.translationMeters.y = .125;
            trainConfigurationEdit_ = std::move(train);
            break;
        }
        case 5:
        {
            editorWorkspace_ = EditorWorkspace::Track;
            auto launch = makeTrackDevice(TrackDeviceKind::Launch);
            launch.startStationMeters = 0; launch.endStationMeters = 50;
            launch.maximumForceNewtons = 100000;
            trackDeviceCommand_ = {TrackDeviceCommandType::Add, std::move(launch)};
            break;
        }
        case 6:
        {
            for (const auto& device : authoredTrack_->trackDevices().devices)
                if (device.kind == TrackDeviceKind::Launch)
                {
                    auto launch = device;
                    launch.targetAccelerationMetersPerSecondSquared = 4;
                    std::get<LaunchConfiguration>(launch.configuration).targetSpeedMetersPerSecond = 30;
                    trackDeviceCommand_ = {TrackDeviceCommandType::Update, std::move(launch)};
                    selectTrackDevice(device.id);
                    ImGui::SetWindowFocus("Track Devices");
                    break;
                }
            break;
        }
        case 7: pendingHistoryOperation_ = HistoryOperationType::Undo; break;
        case 8: pendingHistoryOperation_ = HistoryOperationType::Redo; break;
        case 9:
            regionCommand_ = {RegionCommandType::AppendForceDriven, selectedSection_, 0};
            break;
        case 10:
            regionCommand_ = {RegionCommandType::AppendPlanarArc, selectedSection_, 0};
            break;
        case 11:
            enterSimulatorForPreviewSmoke();
            pendingSimulationControl_ = SimulationControlType::Play;
            break;
        case 12:
            returnToEditorForPreviewSmoke();
            pendingSimulationControl_ = SimulationControlType::Pause;
            break;
        default: throw std::invalid_argument("Unknown native authoring smoke action.");
        }
    }

    void EditorUi::drawTrainConfigurationEditor()
    {
        if (!authoredTrack_) return;
        const auto& accepted = authoredTrack_->trainConfiguration();
        const auto source = coaster::serializeTrainConfiguration(accepted);
        // External commands, undo and reload replace the authoritative input.
        // Ordinary draft typing does not repeatedly rebuild physics/geometry.
        if (source != trainDraftSource_)
        {
            trainDraft_ = accepted;
            trainDraftSource_ = source;
            trainDraftDirty_ = false;
            trainEditError_.clear();
        }
        auto& draft = trainDraft_;
        if (draft.cars.empty()) coaster::resizeTrainConsist(draft, draft.carCount);
        bool changed = false;
        editorSecondaryTextWrapped("Edit a draft, then Apply to reset the actual preview. Each Apply is one Undo action.");
        ImGui::BeginDisabled(!trainDraftDirty_);
        if (ImGui::Button("Apply train")) trainConfigurationEdit_ = draft;
        ImGui::SameLine();
        if (ImGui::Button("Revert draft"))
        {
            draft = accepted;
            trainDraftDirty_ = false;
            trainEditError_.clear();
        }
        ImGui::EndDisabled();
        if (trainDraftDirty_) ImGui::TextColored(palette::warning, "Unapplied train changes");
        if (!trainEditError_.empty())
        {
            ImGui::TextWrapped("%s", trainEditError_.c_str());
            editorSecondaryTextWrapped("The accepted train is unchanged. Correct the draft or Revert.");
        }
        changed |= textField("Train name", draft.name);
        editorSectionHeading("Ordered cars / front to rear", fonts_);
        if (ImGui::BeginChild("Consist list", {0, 125 * editorPresentationScale()}, ImGuiChildFlags_Borders))
        {
            for (std::size_t i = 0; i < draft.cars.size(); ++i)
            {
                const auto& car = draft.cars[i];
                const auto label = std::to_string(i + 1) + "  " + car.name + " ["
                    + coaster::trainCarRoleName(car.role) + "]###car" + std::to_string(car.id);
                if (ImGui::Selectable(label.c_str(), trainSelectedCar_ == car.id))
                {
                    trainSelectedCar_ = car.id;
                    trainSummaryCarIndex_ = static_cast<int>(i);
                }
            }
        }
        ImGui::EndChild();
        roleField("New car role", trainPrototypeRole_);
        if (ImGui::Button("Add car"))
        {
            trainSelectedCar_ = coaster::insertTrainCar(draft, draft.cars.size(),
                coaster::createTrainCarPrototype(roles[trainPrototypeRole_]));
            changed = true;
        }
        auto selected = std::find_if(draft.cars.begin(), draft.cars.end(),
            [&](const auto& car) { return car.id == trainSelectedCar_; });
        if (selected == draft.cars.end() && !draft.cars.empty())
        { selected = draft.cars.begin(); trainSelectedCar_ = selected->id; trainSummaryCarIndex_ = 0; }
        if (selected != draft.cars.end())
        {
            const auto index = static_cast<std::size_t>(selected - draft.cars.begin());
            ImGui::SameLine();
            if (ImGui::Button("Duplicate car"))
            {
                const auto copy = *selected;
                trainSelectedCar_ = coaster::insertTrainCar(draft, index + 1, copy);
                changed = true;
            }
            ImGui::BeginDisabled(draft.cars.size() <= 1);
            if (ImGui::Button("Remove car"))
            { coaster::removeTrainCar(draft, trainSelectedCar_); changed = true; }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(index == 0);
            if (ImGui::Button("Move forward"))
            { coaster::reorderTrainCar(draft, trainSelectedCar_, index - 1); changed = true; }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(index + 1 >= draft.cars.size());
            if (ImGui::Button("Move rearward"))
            { coaster::reorderTrainCar(draft, trainSelectedCar_, index + 1); changed = true; }
            ImGui::EndDisabled();
            // Structural actions can invalidate vector references in this frame.
            selected = std::find_if(draft.cars.begin(), draft.cars.end(),
                [&](const auto& car) { return car.id == trainSelectedCar_; });
        }
        if (selected != draft.cars.end())
        {
            auto& authored = *selected;
            auto& car = authored.physical.car;
            auto& load = authored.physical.loadout;
            editorSectionHeading("Selected car", fonts_);
            changed |= textField("Car name", authored.name);
            int role = static_cast<int>(std::find(roles.begin(), roles.end(), authored.role) - roles.begin());
            if (roleField("Role metadata", role)) { authored.role = roles[role]; changed = true; }
            editorSecondaryTextWrapped("Local coordinates: +X forward, +Y lateral, +Z up. Role metadata does not replace physics.");
            if (ImGui::CollapsingHeader("Mass and body", ImGuiTreeNodeFlags_DefaultOpen))
            {
                changed |= number("Dry mass (kg)", car.dryMassKilograms);
                changed |= number("Load mass (kg)", load.massKilograms);
                changed |= vector("Body dimensions (m)", car.bodyDimensionsMeters);
                changed |= vector("Dry COG (m)", car.dryCenterOfGravityMeters);
                changed |= vector("Load COM (m)", load.centerOfMassMeters);
                changed |= number("Car drag area CdA (m^2)", car.aerodynamicDragAreaSquareMeters);
                changed |= vector("Aerodynamic centre (m)", car.aerodynamicCenterLocalMeters);
            }
            if (ImGui::CollapsingHeader("Bogies and connections"))
            {
                for (std::size_t i = 0; i < car.bogies.size(); ++i)
                {
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::Text("Bogie %zu", i + 1);
                    changed |= vector("Pivot (m)", car.bogies[i].referencePositionMeters);
                    if (ImGui::TreeNode("Contact geometry"))
                    {
                        for (std::size_t j = 0; j < car.bogies[i].contacts.size(); ++j)
                        {
                            auto& contact = car.bogies[i].contacts[j];
                            ImGui::PushID(static_cast<int>(j));
                            ImGui::Text("Contact %zu", j + 1);
                            changed |= vector("Position (m)", contact.localPositionMeters);
                            changed |= vector("Normal", contact.contactNormalLocal);
                            changed |= number("Clearance (m)", contact.clearanceMeters);
                            ImGui::PopID();
                        }
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
                changed |= vector("Front hitch (m)", car.frontHitchPositionMeters);
                changed |= vector("Rear hitch (m)", car.rearHitchPositionMeters);
                changed |= number("Rear connector length (m)", authored.rearConnection.rigidLengthMeters);
                editorSecondaryTextWrapped("The final car retains its connector default for future additions.");
            }
            if (ImGui::CollapsingHeader("Dry inertia tensor (kg m^2)"))
            {
                auto& tensor = car.dryInertiaTensorBodyKgM2;
                changed |= number("Ixx", tensor[0][0]);
                changed |= number("Iyy", tensor[1][1]);
                changed |= number("Izz", tensor[2][2]);
                if (number("Ixy", tensor[0][1])) { tensor[1][0] = tensor[0][1]; changed = true; }
                if (number("Ixz", tensor[0][2])) { tensor[2][0] = tensor[0][2]; changed = true; }
                if (number("Iyz", tensor[1][2])) { tensor[2][1] = tensor[1][2]; changed = true; }
                editorSecondaryTextWrapped("Authoritative symmetric tensor about the dry COG; dimensions do not recalculate it.");
            }
            if (ImGui::CollapsingHeader("Visual shell", ImGuiTreeNodeFlags_DefaultOpen))
            {
                changed |= textField("GLB asset identifier", authored.visualAsset);
                changed |= vector("Shell offset (m)", authored.visualAlignment.translationMeters);
                changed |= vector("Shell rotation XYZ (deg)", authored.visualAlignment.rotationDegrees);
                editorSecondaryTextWrapped("Alignment affects the shell only. Missing assets retain physical diagnostic geometry.");
            }
        }
        if (ImGui::CollapsingHeader("Whole-train resistance"))
        {
            auto& r = draft.resistance;
            changed |= number("Mechanical force (N)", r.constantMechanicalForceNewtons);
            changed |= number("Linear coefficient (N s/m)", r.linearResistanceCoefficientNewtonSecondsPerMeter);
            changed |= number("Air density (kg/m^3)", r.airDensityKilogramsPerCubicMeter);
            changed |= number("Aggregate drag area (m^2)", r.dragAreaSquareMeters);
            changed |= number("Rolling coefficient", r.rollingResistanceCoefficient);
            editorSecondaryTextWrapped("Aggregate and per-car aerodynamic drag areas are mutually exclusive.");
        }
        trainDraftDirty_ |= changed;
    }
}
