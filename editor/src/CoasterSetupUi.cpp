#include <quantum/editor/CoasterSetupUi.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackConfiguration.hpp>
#include <quantum/editor/TrackStylePresentation.hpp>

#include <imgui.h>
#include <quantum/renderer/StaticMeshAssets.hpp>
#include <array>
#include <cstdio>
#include <set>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>

namespace quantum::editor
{
    namespace
    {
        void drawTrackComponentAssignments(std::vector<coaster::TrackComponent>& components)
        {
            for (auto& component : components)
            {
                ImGui::PushID(component.id.c_str());
                if (ImGui::TreeNode(component.displayName.c_str()))
                {
                    ImGui::Checkbox("Visible", &component.enabled);
                    ImGui::TextDisabled(component.required ? "Required slot" : "Optional slot");
                    ImGui::TextDisabled("Material slot: %s", component.materialSlot.c_str());
                    std::array<char, 512> asset{};
                    std::snprintf(asset.data(), asset.size(), "%s", component.asset.path.c_str());
                    ImGui::TextUnformatted("GLB asset ID");
                    ImGui::SetNextItemWidth(-1);
                    if (ImGui::InputText("##GLB asset ID", asset.data(), asset.size())) component.asset.path = asset.data();
                    ImGui::TextWrapped("assets://track/your-component.glb; empty = procedural");
                    if (!component.asset.path.empty())
                    {
                        if (ImGui::Button("Return to blockout")) component.asset = {};
                        if (auto status = renderer::trackComponentAssetStatus(component.asset.path))
                            ImGui::TextWrapped("%s", status->detail.c_str());
                    }
                    ImGui::Checkbox("Use imported GLB materials", &component.useAssetMaterials);
                    if (!component.asset.path.empty() && component.useAssetMaterials)
                        ImGui::TextWrapped("Imported factors take priority. Disable this option to apply the color and factors below.");
                    if (!component.asset.path.empty())
                    {
                        if (component.placement == coaster::TrackComponentPlacement::Continuous)
                        {
                            ImGui::TextUnformatted("Tile length (m)");
                            ImGui::SetNextItemWidth(-1);
                            ImGui::InputDouble("##Tile length (m)", &component.assetLength, .1, 1, "%.3f");
                        }
                        ImGui::TextUnformatted("Asset scale");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputScalarN("##Asset scale", ImGuiDataType_Double, &component.assetScale.x, 3);
                    }
                    const bool continuous = component.placement == coaster::TrackComponentPlacement::Continuous;
                    ImGui::TextUnformatted(continuous ? "Lateral/up offset (m)" : "Offset X/Y/Z (m)");
                    ImGui::SetNextItemWidth(-1);
                    ImGui::InputScalarN("##Offset", ImGuiDataType_Double,
                        continuous ? &component.offset.y : &component.offset.x, continuous ? 2 : 3);
                    const bool pointMember = component.provider == coaster::TrackComponentProvider::Member
                    || component.provider == coaster::TrackComponentProvider::Beam;
                    ImGui::TextUnformatted(continuous || pointMember ? "Blockout width/height (m)" : "Blockout length/width/height (m)");
                    ImGui::SetNextItemWidth(-1);
                    ImGui::InputScalarN("##Dimensions", ImGuiDataType_Double,
                        continuous || pointMember ? &component.dimensions.y : &component.dimensions.x, continuous || pointMember ? 2 : 3);
                    if (component.provider == coaster::TrackComponentProvider::Member
                        || component.provider == coaster::TrackComponentProvider::Beam)
                    {
                        ImGui::TextUnformatted("Member endpoint (m)");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputScalarN("##Member endpoint (m)", ImGuiDataType_Double, &component.endOffset.x, 3);
                    }
                    if (component.placement != coaster::TrackComponentPlacement::Continuous)
                    {
                        ImGui::TextUnformatted("Spacing (m)");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputDouble("##Spacing (m)", &component.spacing, .1, 1, "%.3f");
                        ImGui::TextUnformatted("Phase (m)");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputDouble("##Phase (m)", &component.phase, .1, 1, "%.3f");
                    }
                    ImGui::TextUnformatted("Color");
                    ImGui::SetNextItemWidth(-1);
                    ImGui::ColorEdit3("##Color", &component.material.baseColor.x);
                    ImGui::TextUnformatted("Metallic");
                    ImGui::SetNextItemWidth(-1);
                    ImGui::SliderFloat("##Metallic", &component.material.metallic, 0, 1);
                    ImGui::TextUnformatted("Roughness");
                    ImGui::SetNextItemWidth(-1);
                    ImGui::SliderFloat("##Roughness", &component.material.roughness, 0, 1);
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
        }

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

    bool drawCarsPerTrainInput(coaster::CoasterSetup& draft)
    {
        int carsPerTrain = static_cast<int>(draft.carsPerTrain);
        if (!ImGui::InputInt("Cars per train", &carsPerTrain, 1, 2))
            return false;
        const auto count = static_cast<std::uint32_t>(std::clamp(
            carsPerTrain,
            static_cast<int>(coaster::minimumCarsPerTrain),
            static_cast<int>(coaster::maximumCarsPerTrain)));
        if (count == draft.carsPerTrain)
            return false;
        draft.carsPerTrain = count;
        return true;
    }

    CoasterSetupWindowEdits drawCoasterSetupWindow(
        const coaster::AuthoredTrack* const authoredTrack,
        bool* const open,
        const EditorFonts& fonts,
        const char* const windowName)
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

        if (!ImGui::Begin(
                windowName,
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
                    ? configDef->displayName.c_str()
                    : configId.empty() ? "Unassigned" : authoredTrack->trackStyle().name.c_str());
            static std::string selectedCategory;
            static std::string categorySource;
            if (categorySource != configId || selectedCategory.empty())
            {
                selectedCategory = configDef ? configDef->category : "Steel Truss";
                categorySource = configId;
            }
            ImGui::TextUnformatted("Construction category");
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##Construction category", selectedCategory.c_str()))
            {
                std::set<std::string> categories;
                for (const auto& [id, def] : coaster::trackFamilyRegistry().types())
                    if (!def.reserved) categories.insert(def.category);
                for (const auto& category : categories)
                    if (ImGui::Selectable(category.c_str(), category == selectedCategory)) selectedCategory = category;
                ImGui::EndCombo();
            }
            ImGui::TextUnformatted("Numbered type");
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##Numbered type", configDef ? configDef->displayName.c_str() : "Select type"))
            {
                for (const auto& [id, def] : coaster::trackFamilyRegistry().types())
                    if (!def.reserved && def.category == selectedCategory
                        && ImGui::Selectable(def.displayName.c_str(), id == configId)) edits.trackConfigurationId = id;
                ImGui::EndCombo();
            }
            if (configDef) editorSecondaryTextWrapped(configDef->reference.c_str());
            if (!configId.empty() && !configDef)
                editorSecondaryTextWrapped("Catalog definition unavailable. Saved component geometry and asset references are retained.");
            editorSecondaryTextWrapped("Temporary structural blockouts. Track appearance is independent of the current train.");
            if (authoredTrack->trackStyle().geometryFamily == coaster::TrackGeometryFamily::Components)
                for (std::size_t region=0; region<authoredTrack->sectionCount(); ++region)
                    if (coaster::hasTrackStylePropertyOverrides(authoredTrack->section(region).trackStyleOverrides))
                    {
                        editorSecondaryTextWrapped("Legacy Region dimension/material overrides are retained but inactive for component geometry. Region visibility remains active.");
                        break;
                    }
            if (authoredTrack->trackStyle().geometryFamily == coaster::TrackGeometryFamily::Components)
            {
                static coaster::TrackStylePreset styleDraft;
                static std::string draftSource;
                const auto source = coaster::serializeTrackComponents(authoredTrack->trackStyle());
                if (draftSource != source)
                {
                    styleDraft = authoredTrack->trackStyle();
                    draftSource = source;
                }
                if (!styleDraft.variants.empty())
                {
                    ImGui::TextUnformatted("Construction variant");
                    ImGui::SetNextItemWidth(-1);
                    if (ImGui::BeginCombo("##Construction variant", styleDraft.variantId.c_str()))
                    {
                        for (const auto& variant : styleDraft.variants)
                            if (ImGui::Selectable(variant.displayName.c_str(), variant.id == styleDraft.variantId))
                            {
                                styleDraft.variantId = variant.id;
                                styleDraft.components = variant.components;
                                styleDraft.supportAttachment = variant.supportAttachment;
                            }
                        ImGui::EndCombo();
                    }
                }
                if (ImGui::TreeNode("Components, dimensions and materials"))
                {
                    drawTrackComponentAssignments(styleDraft.components);
                    ImGui::TreePop();
                }
                ImGui::TextUnformatted("Support attachment X/Y/Z (m)");
                ImGui::SetNextItemWidth(-1);
                ImGui::InputScalarN("##Support attachment X/Y/Z (m)", ImGuiDataType_Double, &styleDraft.supportAttachment.x, 3);
                if (!styleDraft.variants.empty() && ImGui::TreeNode("Construction intervals"))
                {
                    editorSecondaryTextWrapped("Absolute solved stations in metres. Out-of-range portions are clipped after shaping; authored ranges remain saved. Intervals are independent of mathematical Regions.");
                    for (std::size_t index = 0; index < styleDraft.constructionIntervals.size();)
                    {
                        auto& interval = styleDraft.constructionIntervals[index];
                        ImGui::PushID(std::to_string(interval.id).c_str());
                        ImGui::Text("Interval %llu", static_cast<unsigned long long>(interval.id));
                        ImGui::TextUnformatted("Start (m)");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputDouble("##Start (m)", &interval.start, 1, 10, "%.3f");
                        ImGui::TextUnformatted("End (m)");
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputDouble("##End (m)", &interval.end, 1, 10, "%.3f");
                        if (ImGui::BeginCombo("Variant", interval.variantId.c_str()))
                        {
                            for (const auto& variant : styleDraft.variants)
                                if (ImGui::Selectable(variant.displayName.c_str(), interval.variantId == variant.id))
                                { interval.variantId = variant.id; interval.components.clear(); }
                            ImGui::EndCombo();
                        }
                        if (ImGui::TreeNode("Component and material overrides"))
                        {
                            bool active=!interval.components.empty();
                            if (ImGui::Checkbox("Override interval components", &active))
                            {
                                if (active)
                                    for (const auto& variant : styleDraft.variants)
                                        if (variant.id==interval.variantId) interval.components=variant.components;
                                if (!active) interval.components.clear();
                            }
                            if (active) drawTrackComponentAssignments(interval.components);
                            ImGui::TreePop();
                        }
                        const bool remove = ImGui::Button("Remove interval");
                        ImGui::PopID();
                        if (remove) styleDraft.constructionIntervals.erase(styleDraft.constructionIntervals.begin()+index);
                        else ++index;
                    }
                    if (ImGui::Button("Add construction interval"))
                    {
                        const double start = styleDraft.constructionIntervals.empty() ? 0 : styleDraft.constructionIntervals.back().end;
                        styleDraft.constructionIntervals.push_back({styleDraft.nextConstructionIntervalId++, start, start+50,
                            styleDraft.typeId, styleDraft.variants.back().id, {}});
                    }
                    ImGui::TreePop();
                }
                if (ImGui::Button("Apply appearance changes"))
                {
                    std::ranges::sort(styleDraft.constructionIntervals, {}, &coaster::TrackConstructionInterval::start);
                    try { coaster::validateTrackStyle(styleDraft); edits.trackStyle = styleDraft; }
                    catch (const std::exception&) { /* Diagnostic drawn below; document remains unchanged. */ }
                }
                if (ImGui::Button("Discard appearance draft")) styleDraft = authoredTrack->trackStyle();
                if (ImGui::Button("Reload assigned GLBs")) edits.reloadTrackComponents = true;
                try { coaster::validateTrackStyle(styleDraft); }
                catch (const std::exception& error) { ImGui::TextColored(palette::error, "%s", error.what()); }
            }

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
        changed |= drawCarsPerTrainInput(draft);
        editorSecondaryTextWrapped(
            "Simulation Preview uses this car count with its current repeated-car "
            "physics definition. Rider positions remain car-loadout data, not "
            "the authored heartline.");

        ImGui::Separator();
        editorHeading("Authored rider reference", fonts);
        if (ImGui::Checkbox(
                "Use heartline offset",
                &draft.heartline.enabled))
        {
            changed = true;
        }
        float offsetMeters =
            static_cast<float>(draft.heartline.offsetMeters);
        ImGui::SetNextItemWidth(std::max(75.0F, ImGui::GetContentRegionAvail().x
            - ImGui::CalcTextSize("Local +up offset (m)").x
            - ImGui::GetStyle().ItemInnerSpacing.x));
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
            "Measured from the track construction reference along local +up. "
            "Force-Based targets and rider-load diagnostics use this point; "
            "rails, supports, bogies, and train poses stay on the construction "
            "reference.");

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
