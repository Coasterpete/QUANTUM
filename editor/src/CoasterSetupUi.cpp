#include <quantum/editor/CoasterSetupUi.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackConfiguration.hpp>
#include <quantum/editor/TrackStylePresentation.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>

namespace quantum::editor
{
    namespace
    {
        [[nodiscard]] TrackStylePresentationImpact
        computeModifiedStateImpact(
            const coaster::AuthoredTrack& track)
        {
            const std::string_view configId = track.trackConfigurationId();
            if (configId.empty())
            {
                return {};
            }

            const coaster::TrackConfigurationDefinition* definition =
                coaster::findTrackConfiguration(configId);
            if (definition == nullptr)
            {
                return {};
            }

            const coaster::TrackStylePreset defaults =
                coaster::resolveTrackConfiguration(*definition);
            return classifyResolvedTrackStylePresentationChange(
                defaults, track.trackStyle());
        }
    }

    CoasterSetupWindowEdits drawCoasterSetupWindow(
        const coaster::AuthoredTrack* const authoredTrack,
        bool* const open,
        const EditorFonts& fonts)
    {
        if (open == nullptr || !*open || authoredTrack == nullptr)
        {
            return {};
        }

        const coaster::CoasterSetup& committed =
            authoredTrack->coasterSetup();
        coaster::CoasterSetup draft = committed;
        const coaster::TrackPhysicalSettings& committedPhysicalSettings =
            authoredTrack->physicalSettings();
        coaster::TrackPhysicalSettings physicalSettings =
            committedPhysicalSettings;
        bool changed = false;
        CoasterSetupWindowEdits edits;

        ImGui::SetNextWindowPos(
            ImVec2(760.0F, 110.0F),
            ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(
                coasterSetupWindowName,
                open,
                ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::End();
            return {};
        }
        ImGui::PushID("Coaster Setup Window");

        const coaster::CoasterStyleDefinition* currentStyle =
            coaster::findCoasterStyle(draft.styleId);
        if (currentStyle == nullptr)
        {
            ImGui::TextUnformatted(
                "Unknown authored coaster style; the document cannot be "
                "edited with this catalog.");
            ImGui::PopID();
            ImGui::End();
            return {};
        }

        // Style changes use the selected definition's complete defaults while
        // preserving the setup fields that are independent of style.
        const auto rebuildForStyle = [&](const std::string_view styleId)
        {
            coaster::CoasterSetup rebuilt =
                coaster::createCoasterSetupForStyle(styleId);
            rebuilt.carsPerTrain = committed.carsPerTrain;
            rebuilt.heartline = committed.heartline;
            draft = std::move(rebuilt);
        };

        editorHeading("Coaster style", fonts);
        editorSecondaryText(
            "Placeholder catalog for validating setup behavior.");
        for (const coaster::CoasterStyleDefinition& style :
            coaster::coasterStyleCatalog())
        {
            if (ImGui::RadioButton(
                    style.displayName.data(),
                    draft.styleId == style.id))
            {
                rebuildForStyle(style.id);
                changed = true;
            }
        }

        const coaster::CoasterStyleDefinition* draftStyle =
            coaster::findCoasterStyle(draft.styleId);
        const std::string capabilitySummary =
            coaster::coasterCapabilitySummary(
                draftStyle != nullptr
                    ? draftStyle->capabilities
                    : coaster::CoasterCapability::None);
        ImGui::TextColored(
            palette::textSecondary,
            "Capabilities: %s",
            capabilitySummary.c_str());

        ImGui::Separator();
        editorHeading("Track configuration", fonts);

        // Track appearance is independent of the coaster/train setup above.
        {
            const std::string_view configId =
                authoredTrack->trackConfigurationId();
            const coaster::TrackConfigurationDefinition* configDef =
                configId.empty()
                    ? nullptr
                    : coaster::findTrackConfiguration(configId);

            ImGui::TextColored(palette::textSecondary,
                "Current: %s",
                configDef != nullptr
                    ? configDef->displayName.data()
                    : configId.empty() ? "Unassigned" : "Unknown");
            if (ImGui::Button("Apply Modern Steel"))
            {
                edits.trackConfigurationId =
                    std::string(coaster::modernSteelTrackConfigurationId);
            }
            editorSecondaryTextWrapped(
                "Applies the Modern Steel base appearance; region style "
                "overrides are kept.");

            if (configDef != nullptr)
            {
                const bool modified =
                    !computeModifiedStateImpact(*authoredTrack).empty();
                if (modified)
                {
                    ImGui::TextColored(palette::warning,
                        "Base appearance modified");
                }
                if (ImGui::Button("Reset track appearance and regions"))
                {
                    ImGui::OpenPopup("Reset Track Appearance?");
                }
                if (ImGui::BeginPopupModal("Reset Track Appearance?",
                        nullptr, ImGuiWindowFlags_AlwaysAutoResize))
                {
                    ImGui::TextWrapped(
                        "Restore the selected configuration defaults and "
                        "clear all authored region style overrides?");
                    if (ImGui::Button("Reset"))
                    {
                        edits.resetTrackConfiguration = true;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel"))
                    {
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndPopup();
                }
            }
        }

        if (draftStyle == nullptr)
        {
            ImGui::TextUnformatted("No options are defined for this style.");
            ImGui::PopID();
            ImGui::End();
            return {};
        }

        for (const coaster::CoasterOptionDefinition* option :
            coaster::applicableOptionsForStyle(*draftStyle))
        {
            auto value = std::find_if(
                draft.options.begin(),
                draft.options.end(),
                [option](const coaster::CoasterOptionValue& candidate)
                {
                    return candidate.optionId == option->id;
                });
            if (value == draft.options.end())
            {
                ImGui::TextColored(
                    palette::error,
                    "Missing option value: %s",
                    option->label.data());
                continue;
            }

            if (option->kind == coaster::CoasterOptionKind::Boolean)
            {
                if (ImGui::Checkbox(
                        option->label.data(),
                        &value->booleanValue))
                {
                    changed = true;
                }
                continue;
            }

            const coaster::CoasterOptionChoice* currentChoice =
                coaster::findCoasterOptionChoice(
                    *option, value->choiceValue);
            const char* currentChoiceLabel = currentChoice != nullptr
                ? currentChoice->label.data()
                : "Unknown choice";
            if (ImGui::BeginCombo(
                    option->label.data(),
                    currentChoiceLabel))
            {
                for (const coaster::CoasterOptionChoice& choice :
                    option->choices)
                {
                    const bool selected = choice.id == value->choiceValue;
                    if (ImGui::Selectable(choice.label.data(), selected))
                    {
                        value->choiceValue = choice.id;
                        changed = true;
                    }
                    if (selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }

        ImGui::Separator();
        editorHeading("Train setup summary", fonts);
        const coaster::CoasterSetupTrainSummary train =
            coaster::trainSummaryForCoasterSetup(draft);
        if (train.seatsAcross.has_value()
            && train.rowsPerCar.has_value())
        {
            ImGui::TextColored(
                palette::textSecondary,
                "Seats across: %u   rows/car: %u",
                *train.seatsAcross,
                *train.rowsPerCar);
        }
        else
        {
            ImGui::TextDisabled("Train layout: not applicable");
        }
        if (train.restraintOptionId.has_value())
        {
            ImGui::TextColored(
                palette::textSecondary,
                "Restraint option: %s",
                train.restraintOptionId->c_str());
        }
        else
        {
            ImGui::TextDisabled("Restraint option: not applicable");
        }
        int carsPerTrain = static_cast<int>(draft.carsPerTrain);
        if (ImGui::InputInt("Cars per train", &carsPerTrain, 1, 2))
        {
            carsPerTrain = std::clamp(
                carsPerTrain,
                static_cast<int>(coaster::minimumCarsPerTrain),
                static_cast<int>(coaster::maximumCarsPerTrain));
            draft.carsPerTrain =
                static_cast<std::uint32_t>(carsPerTrain);
            changed = true;
        }
        editorSecondaryTextWrapped(
            "Setup metadata only; Simulation Preview uses its own fixed "
            "four-car physics definition.");

        ImGui::Separator();
        editorHeading("Heartline reference", fonts);
        if (ImGui::Checkbox(
                "Apply reference-line offset",
                &draft.heartline.enabled))
        {
            changed = true;
        }
        float offsetMeters =
            static_cast<float>(draft.heartline.offsetMeters);
        if (ImGui::DragFloat(
                "Local +up offset (m)",
                &offsetMeters,
                0.05F,
                static_cast<float>(coaster::minimumHeartlineOffsetMeters),
                static_cast<float>(coaster::maximumHeartlineOffsetMeters),
                "%.2f",
                ImGuiSliderFlags_AlwaysClamp))
        {
            draft.heartline.offsetMeters = offsetMeters;
            changed = true;
        }
        editorSecondaryTextWrapped(
            "Affects the viewport reference curve only; authored track "
            "geometry and physics are unchanged.");

        ImGui::Separator();
        editorHeading("Physical settings", fonts);
        ImGui::InputDouble(
            "Initial speed (m/s)",
            &physicalSettings.initialSpeed,
            0.5,
            5.0,
            "%.3f");
        editorSecondaryTextWrapped(
            "Initial train speed used by Force Diagnostics and Simulation "
            "Preview.");

        ImGui::PopID();
        ImGui::End();

        if (changed && draft != committed)
        {
            edits.coasterSetup = std::move(draft);
        }
        if (physicalSettings != committedPhysicalSettings)
        {
            edits.physicalSettings = physicalSettings;
        }
        return edits;
    }
}
