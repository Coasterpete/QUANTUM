#include <quantum/editor/CoasterSetupUi.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>

#include <imgui.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>

namespace quantum::editor
{
    std::optional<coaster::CoasterSetup> drawCoasterSetupWindow(
        const coaster::AuthoredTrack* const authoredTrack,
        bool* const open,
        const EditorFonts& fonts)
    {
        if (open == nullptr || !*open || authoredTrack == nullptr)
        {
            return std::nullopt;
        }

        const coaster::CoasterSetup& committed =
            authoredTrack->coasterSetup();
        coaster::CoasterSetup draft = committed;
        bool changed = false;

        ImGui::SetNextWindowPos(
            ImVec2(760.0F, 110.0F),
            ImGuiCond_FirstUseEver);
        if (!ImGui::Begin(
                coasterSetupWindowName,
                open,
                ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::End();
            return std::nullopt;
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
            return std::nullopt;
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
        editorHeading("Configuration", fonts);
        if (draftStyle == nullptr)
        {
            ImGui::TextUnformatted("No options are defined for this style.");
            ImGui::PopID();
            ImGui::End();
            return std::nullopt;
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

        ImGui::PopID();
        ImGui::End();

        if (changed && draft != committed)
        {
            return draft;
        }
        return std::nullopt;
    }
}
