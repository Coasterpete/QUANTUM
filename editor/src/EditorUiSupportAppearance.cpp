#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <utility>

namespace quantum::editor
{
    void EditorUi::drawSupportAppearancePanel()
    {
        ImGui::Text("Support Appearance");
        editorSecondaryText("Appearance is independent of the structural "
            "family. The wood texture supplies neutral grain detail; the tint "
            "sets the timber's color.");

        // Solid timber and the technical overlay are independent so a user can
        // inspect presentation and authored topology at the same time. These
        // are viewport settings, so they are not part of the document history.
        bool displayChanged = ImGui::Checkbox("Solid Supports",
            &supportDisplaySettings_.solidVisible);
        displayChanged = ImGui::Checkbox("Debug Lines",
            &supportDisplaySettings_.debugLinesVisible) || displayChanged;
        if (displayChanged)
        {
            pendingSupportDisplaySettings_ = supportDisplaySettings_;
        }
        ImGui::Separator();

        const coaster::SupportCollection& supports = authoredTrack_->supports();
        const auto active = selectedSupport_.has_value()
            ? std::ranges::find_if(supports.structures,
                [this](const coaster::SupportStructure& structure)
                {
                    return structure.id == selectedSupport_->structureId;
                })
            : (supports.structures.empty()
                ? supports.structures.end() : supports.structures.begin());
        if (active == supports.structures.end())
        {
            editorSecondaryText("No structure selected.");
            return;
        }

        // Reload the draft only when the shown structure changes. Reloading it
        // every frame would fight the drag that is currently writing to it.
        if (supportAppearanceDraftStructureId_ != active->id)
        {
            synchronizeSupportAppearance(*active);
            supportAppearanceDraftStructureId_ = active->id;
        }

        bool timberChanged = false;
        bool timberDragActive = false;
        {
            ImGui::PushID("SupportTimberAppearance");
            // The base-color map is neutral grayscale detail, so this tint is
            // the timber's dominant color rather than a filter over a baked
            // wood hue. ColorEdit4 edits a vec4, so the widget runs against a
            // scratch buffer whose alpha stays 1.
            supportAppearanceTintBuffer_ = glm::vec4(
                supportAppearanceDraft_.baseColorTint, 1.0F);
            timberChanged = ImGui::ColorEdit4("Timber Color",
                &supportAppearanceTintBuffer_.x,
                ImGuiColorEditFlags_NoInputs)
                || timberChanged;
            timberDragActive = ImGui::IsItemActive();
            supportAppearanceDraft_.baseColorTint =
                glm::vec3(supportAppearanceTintBuffer_);
            // A full-width widget keeps the values readable in the narrow
            // docked Supports window.
            ImGui::SetNextItemWidth(-1.0F);
            timberChanged = ImGui::SliderFloat("Roughness Multiplier",
                &supportAppearanceDraft_.roughnessMultiplier, 0.1F, 3.0F,
                "%.2f") || timberChanged;
            timberDragActive = ImGui::IsItemActive() || timberDragActive;
            ImGui::SetNextItemWidth(-1.0F);
            timberChanged = ImGui::SliderFloat("Normal Strength",
                &supportAppearanceDraft_.normalStrength, 0.01F, 2.0F,
                "%.2f") || timberChanged;
            timberDragActive = ImGui::IsItemActive() || timberDragActive;
            ImGui::SetNextItemWidth(-1.0F);
            timberChanged = ImGui::SliderFloat("Texture Scale",
                &supportAppearanceDraft_.textureScale, 0.1F, 8.0F,
                "%.2f units") || timberChanged;
            timberDragActive = ImGui::IsItemActive() || timberDragActive;
            ImGui::PopID();
        }
        if (timberChanged)
        {
            supportAppearanceCommand_ = SupportAppearanceCommand{
                SupportAppearanceEdit::Timber, active->id,
                supportAppearanceDraft_,
                supportFoundationAppearanceDraft_, timberDragActive};
        }

        bool foundationChanged = false;
        bool foundationDragActive = false;
        {
            ImGui::PushID("SupportFoundationAppearance");
            if (ImGui::TreeNode("Foundations"))
            {
                foundationChanged = ImGui::ColorEdit3("Concrete Color",
                    &supportFoundationAppearanceDraft_.baseColorTint.x,
                    ImGuiColorEditFlags_NoInputs) || foundationChanged;
                foundationDragActive = ImGui::IsItemActive();
                foundationChanged = ImGui::SliderFloat("Concrete Roughness",
                    &supportFoundationAppearanceDraft_.roughness, 0.1F, 1.0F,
                    "%.2f") || foundationChanged;
                foundationDragActive = ImGui::IsItemActive()
                    || foundationDragActive;
                // Footing dimensions are authored in Core coordinate units,
                // which the rest of the supports panel edits as doubles. A
                // zero dimension or depth means "derive from the members this
                // foundation carries", so the range starts at zero.
                static const double footingMinimum = 0.0;
                static const double footingMaximum = 100.0;
                static constexpr std::array<const char*, 3> footingLabels{
                    "Pad Width", "Pad Depth", "Pad Thickness"};
                std::array<double*, 3> footingValues{
                    &supportFoundationAppearanceDraft_.padDimensions.x,
                    &supportFoundationAppearanceDraft_.padDimensions.y,
                    &supportFoundationAppearanceDraft_.padDepth};
                for (std::size_t index = 0;
                    index < footingValues.size(); ++index)
                {
                    ImGui::SetNextItemWidth(-1.0F);
                    const bool changed = ImGui::DragScalar(
                        footingLabels[index], ImGuiDataType_Double,
                        footingValues[index], 0.05F, &footingMinimum,
                        &footingMaximum, "%.2f units",
                        ImGuiSliderFlags_AlwaysClamp);
                    foundationChanged = foundationChanged || changed;
                    foundationDragActive = foundationDragActive
                        || ImGui::IsItemActive();
                }
                editorSecondaryText("Zero pad dimensions or depth derive the "
                    "footing from the members it carries.");
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        if (foundationChanged)
        {
            supportAppearanceCommand_ = SupportAppearanceCommand{
                SupportAppearanceEdit::Foundation, active->id,
                supportAppearanceDraft_,
                supportFoundationAppearanceDraft_, foundationDragActive};
        }

        if (ImGui::Button("Reset Appearance"))
        {
            supportAppearanceDraft_ = {};
            supportFoundationAppearanceDraft_ = {};
            supportAppearanceCommand_ = SupportAppearanceCommand{
                SupportAppearanceEdit::Both, active->id,
                supportAppearanceDraft_, supportFoundationAppearanceDraft_,
                false};
        }
    }

    std::optional<SupportAppearanceCommand>
    EditorUi::takeSupportAppearanceCommand() noexcept
    {
        return std::exchange(supportAppearanceCommand_, std::nullopt);
    }

    void EditorUi::synchronizeSupportAppearance(
        const coaster::SupportStructure& structure) noexcept
    {
        // A structure that never authored an appearance resolves to the same
        // conservative default the renderer uses, so an old document and a
        // newly authored one present identically in the panel.
        supportAppearanceDraft_ =
            structure.appearance.value_or(coaster::SupportAppearance{});
        supportFoundationAppearanceDraft_ =
            structure.foundationAppearance.value_or(
                coaster::SupportFoundationAppearance{});
        supportAppearanceCommand_.reset();
    }

    void EditorUi::syncSupportAppearanceDraft() noexcept
    {
        if (authoredTrack_ == nullptr || !selectedSupport_)
        {
            return;
        }
        const auto& structures = authoredTrack_->supports().structures;
        const auto selected = std::ranges::find_if(structures,
            [this](const coaster::SupportStructure& structure)
            {
                return structure.id == selectedSupport_->structureId;
            });
        if (selected == structures.end())
        {
            return;
        }
        // An Undo or document switch can change the committed appearance
        // without changing the selection, so the draft is refreshed whenever
        // the document value actually differs from what the panel holds.
        if (selected->appearance.value_or(coaster::SupportAppearance{})
            != supportAppearanceDraft_
            || selected->foundationAppearance.value_or(
                   coaster::SupportFoundationAppearance{})
                != supportFoundationAppearanceDraft_)
        {
            synchronizeSupportAppearance(*selected);
        }
        supportAppearanceDraftStructureId_ = selected->id;
    }

    std::optional<SupportDisplaySettings>
    EditorUi::takeSupportDisplaySettings() noexcept
    {
        return std::exchange(pendingSupportDisplaySettings_, std::nullopt);
    }

}
