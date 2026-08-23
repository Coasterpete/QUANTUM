#include <quantum/editor/EditorUi.hpp>

#include <quantum/editor/EditorStyle.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_log.h>
#include <SDL3/SDL_stdinc.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <imgui_internal.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>

namespace
{
    constexpr char bundledFontRelativePath[] =
        "assets/fonts/RedHatMono-SemiBold.ttf";
    // The v2 file migrates the authored shell from the generic Properties
    // layout while preserving normal Dear ImGui persistence thereafter.
    constexpr char editorLayoutIniFileName[] = "imgui-layout-v2.ini";
    constexpr char editorDockspaceName[] = "QuantumEditorDockSpace";
    // Preserve the existing ImGui window ID so the capitalization correction
    // does not discard the user's persisted docking placement.
    constexpr char supportWorkspaceWindowName[] =
        "SUPPORT WORKSPACE###Support Workspace";
    constexpr double orbitRadiansPerPixel = 0.005;

    struct WorkspaceAccent
    {
        ImVec4 quiet;
        ImVec4 normal;
        ImVec4 hovered;
        ImVec4 active;
    };

    const WorkspaceAccent trackWorkspaceAccent{
        .quiet = quantum::editor::palette::fromSrgb(0, 0, 128),
        .normal = quantum::editor::palette::trackWorkspaceBlue,
        .hovered = quantum::editor::palette::fromSrgb(51, 51, 255),
        .active = quantum::editor::palette::fromSrgb(0, 0, 204)
    };
    const WorkspaceAccent supportWorkspaceAccent{
        .quiet = quantum::editor::palette::fromSrgb(128, 0, 0),
        .normal = quantum::editor::palette::supportWorkspaceRed,
        .hovered = quantum::editor::palette::fromSrgb(255, 51, 51),
        .active = quantum::editor::palette::fromSrgb(204, 0, 0)
    };
    const WorkspaceAccent transitionEditorAccent{
        .quiet = quantum::editor::palette::fromSrgb(0, 0, 95),
        .normal = quantum::editor::palette::transitionEditorBlue,
        .hovered = quantum::editor::palette::fromSrgb(51, 51, 203),
        .active = quantum::editor::palette::fromSrgb(0, 0, 152)
    };
    const ImVec4 topRegionHovered =
        quantum::editor::palette::fromSrgb(176, 176, 176);
    const ImVec4 topRegionActive =
        quantum::editor::palette::fromSrgb(156, 156, 156);
    constexpr int workspaceAccentColorCount = 16;
    constexpr int topRegionColorCount = 12;
    const ImU32 transitionCanvasColor = ImGui::ColorConvertFloat4ToU32(
        quantum::editor::palette::black
    );
    const std::array<ImU32, 3> profileCurveColors{
        ImGui::ColorConvertFloat4ToU32(
            quantum::editor::palette::rollChannelRed
        ),
        ImGui::ColorConvertFloat4ToU32(
            quantum::editor::palette::pitchChannelPurple
        ),
        ImGui::ColorConvertFloat4ToU32(
            quantum::editor::palette::yawChannelGold
        )
    };

    void pushWorkspaceAccent(const WorkspaceAccent& accent)
    {
        ImGui::PushStyleColor(ImGuiCol_Button, accent.normal);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, accent.hovered);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, accent.active);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, accent.normal);
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, accent.hovered);
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive, accent.active);
        ImGui::PushStyleColor(ImGuiCol_Header, accent.normal);
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, accent.hovered);
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, accent.active);
        ImGui::PushStyleColor(ImGuiCol_Tab, accent.quiet);
        ImGui::PushStyleColor(ImGuiCol_TabHovered, accent.hovered);
        ImGui::PushStyleColor(ImGuiCol_TabSelected, accent.normal);
        ImGui::PushStyleColor(ImGuiCol_TabSelectedOverline, accent.active);
        ImGui::PushStyleColor(ImGuiCol_TabDimmed, accent.quiet);
        ImGui::PushStyleColor(ImGuiCol_TabDimmedSelected, accent.normal);
        ImGui::PushStyleColor(
            ImGuiCol_TabDimmedSelectedOverline,
            accent.active
        );
    }

    void pushTopRegionStyle()
    {
        using namespace quantum::editor;

        ImGui::PushStyleColor(ImGuiCol_Text, palette::black);
        ImGui::PushStyleColor(ImGuiCol_TextDisabled, palette::black);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, palette::brightestGray);
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, palette::brightestGray);
        ImGui::PushStyleColor(ImGuiCol_PopupBg, palette::brightestGray);
        ImGui::PushStyleColor(ImGuiCol_Border, palette::darkestGray);
        ImGui::PushStyleColor(ImGuiCol_Button, palette::brightestGray);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, topRegionHovered);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, topRegionActive);
        ImGui::PushStyleColor(ImGuiCol_Header, palette::brightestGray);
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, topRegionHovered);
        ImGui::PushStyleColor(ImGuiCol_HeaderActive, topRegionActive);
    }

    struct AuthoredDomainView
    {
        double domainBegin;
        double domainEnd;
        float pixelBegin;
        float pixelEnd;

        [[nodiscard]] float toPixel(const double domainValue) const
        {
            const double progress = (domainValue - domainBegin)
                / (domainEnd - domainBegin);
            return pixelBegin + static_cast<float>(progress)
                * (pixelEnd - pixelBegin);
        }
    };

    constexpr int domainDivisionCount = 5;
    constexpr int valueDivisionCount = 4;
    constexpr std::size_t profileSampleCount = 129;
    constexpr float endpointHandleRadius = 4.0F;
    constexpr float endpointHoverRadius = 8.0F;

    struct ScalarProfileMapping
    {
        const AuthoredDomainView& domainView;
        double valueMagnitude;
        float pixelBeginY;
        float pixelEndY;

        [[nodiscard]] ImVec2 toPixel(
            const double domainValue,
            const double value) const
        {
            const double valueRange = 2.0 * valueMagnitude;
            const float normalizedValue = valueRange > 0.0
                ? static_cast<float>(
                    (value + valueMagnitude) / valueRange
                )
                : 0.5F;

            return {
                domainView.toPixel(domainValue),
                pixelEndY
                    - normalizedValue * (pixelEndY - pixelBeginY)
            };
        }

        [[nodiscard]] double toValue(const float pixelY) const
        {
            const float pixelRange = pixelEndY - pixelBeginY;

            if (valueMagnitude <= 0.0 || pixelRange <= 0.0F)
            {
                return 0.0;
            }

            const float clampedPixelY = std::clamp(
                pixelY,
                pixelBeginY,
                pixelEndY
            );
            const double normalizedValue = static_cast<double>(
                (pixelEndY - clampedPixelY) / pixelRange
            );
            return -valueMagnitude
                + normalizedValue * (2.0 * valueMagnitude);
        }

        // Natural drag slope of this row in authored value units per
        // cursor pixel; the basis for configurable drag sensitivity.
        [[nodiscard]] double valueUnitsPerPixel() const
        {
            const float pixelRange = pixelEndY - pixelBeginY;
            if (valueMagnitude <= 0.0 || pixelRange <= 0.0F)
            {
                return 0.0;
            }
            return 2.0 * valueMagnitude / static_cast<double>(pixelRange);
        }
    };

    struct ScalarProfileEndpointInteraction
    {
        quantum::editor::ScalarProfileEndpoint clickedEndpoint =
            quantum::editor::ScalarProfileEndpoint::None;
        bool plotClicked = false;
    };

    // Value-axis geometry of one drawn profile row; drives centralized
    // drag sensitivity math in showTransitionEditor.
    struct ScalarRowEditGeometry
    {
        double valueMagnitude = 0.0;
        double unitsPerPixel = 0.0;
    };

    struct ScalarProfileRowEdit
    {
        bool valueEndEdited = false;
        std::optional<quantum::math::TransitionType> transitionType;
        ScalarProfileEndpointInteraction endpointInteraction;
        ScalarRowEditGeometry rowGeometry;
    };

    // Snaps an authored value to a positive increment grid. The caller
    // checks that snapping is enabled.
    [[nodiscard]] double snapToIncrement(
        const double value,
        const double increment) noexcept
    {
        return std::round(value / increment) * increment;
    }

    // While a drag is held, the platform may report the cursor position
    // in screen coordinates rather than window coordinates (observed as
    // a constant window-origin offset appearing between one frame and
    // the next). Per-frame delta integration is invariant to a constant
    // offset, so an implausible single-frame delta is absorbed by
    // re-basing the anchor's pixel reference without changing the value.
    constexpr double maxPlausiblePixelDeltaPerFrame = 64.0;

    struct TransitionEditorEdit
    {
        // Endpoint None together with clickedChannel means a plot click on
        // that channel's row (clears its selection).
        struct Click
        {
            quantum::editor::RateChannel channel;
            quantum::editor::ScalarProfileEndpoint endpoint;
        };

        struct TypeChange
        {
            quantum::editor::RateChannel channel;
            quantum::math::TransitionType type;
        };

        std::optional<quantum::editor::ScalarProfileEndpointValueEdit>
            endpointValueEdit;
        std::optional<TypeChange> transitionType;
        std::optional<Click> click;
    };

    // Read-only view into one authored channel row of the Transition Editor.
    struct ProfileRowView
    {
        const char* label;
        const quantum::math::ScalarTransition* transition;
        quantum::editor::RateChannel channel;
    };

    struct TrackWorkspaceEdit
    {
        std::optional<std::size_t> selectRequest;
        bool appendRequested = false;
        bool prependRequested = false;
        bool removeRequested = false;
        bool moveUpRequested = false;
        bool moveDownRequested = false;
        bool lengthEdited = false;
    };

    [[nodiscard]] float squaredDistance(
        const ImVec2 first,
        const ImVec2 second)
    {
        const float deltaX = first.x - second.x;
        const float deltaY = first.y - second.y;
        return deltaX * deltaX + deltaY * deltaY;
    }

    // Diagnostics/gesture logic only need to know whether ANY channel
    // endpoint interaction is active; returns the first active one.
    [[nodiscard]] quantum::editor::ScalarProfileEndpoint firstActiveEndpoint(
        const std::array<quantum::editor::ScalarProfileEndpoint,
            quantum::editor::rateChannelCount>& perChannelEndpoints) noexcept
    {
        for (const quantum::editor::ScalarProfileEndpoint endpoint
            : perChannelEndpoints)
        {
            if (endpoint != quantum::editor::ScalarProfileEndpoint::None)
            {
                return endpoint;
            }
        }

        return quantum::editor::ScalarProfileEndpoint::None;
    }

    [[nodiscard]] ScalarProfileEndpointInteraction
    drawScalarProfileEndpoints(
        ImDrawList* const drawList,
        const ScalarProfileMapping& mapping,
        const quantum::math::ScalarTransition& transition,
        const float rowBeginY,
        const float rowEndY,
        const ImU32 curveColor,
        const char* const valueLabel,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint)
    {
        using quantum::editor::ScalarProfileEndpoint;

        ScalarProfileEndpointInteraction interaction;
        const ImVec2 beginPosition = mapping.toPixel(
            transition.domainBegin,
            transition.valueBegin
        );
        const ImVec2 endPosition = mapping.toPixel(
            transition.domainEnd,
            transition.valueEnd
        );
        const ImVec2 mousePosition = ImGui::GetIO().MousePos;
        const float hoverRadiusSquared = endpointHoverRadius
            * endpointHoverRadius;
        const bool windowHovered = ImGui::IsWindowHovered();
        const bool beginHovered = windowHovered
            && squaredDistance(mousePosition, beginPosition)
                <= hoverRadiusSquared;
        const bool endHovered = windowHovered
            && squaredDistance(mousePosition, endPosition)
                <= hoverRadiusSquared;

        const auto drawEndpoint = [drawList, curveColor](
            const ImVec2 position,
            const bool hovered,
            const bool selected)
        {
            const float radius = hovered
                ? endpointHandleRadius + 1.5F
                : endpointHandleRadius;
            drawList->AddCircleFilled(position, radius, curveColor);

            if (selected)
            {
                drawList->AddCircle(
                    position,
                    endpointHandleRadius + 3.0F,
                    ImGui::GetColorU32(ImGuiCol_Text),
                    0,
                    1.5F
                );
            }
            else if (hovered)
            {
                drawList->AddCircle(
                    position,
                    endpointHandleRadius + 2.5F,
                    curveColor,
                    0,
                    1.5F
                );
            }
        };

        drawEndpoint(
            beginPosition,
            beginHovered,
            selectedEndpoint == ScalarProfileEndpoint::Begin
        );
        drawEndpoint(
            endPosition,
            endHovered,
            selectedEndpoint == ScalarProfileEndpoint::End
        );

        if (beginHovered || endHovered)
        {
            const bool showBegin = beginHovered && !endHovered;
            const double domainValue = showBegin
                ? transition.domainBegin
                : transition.domainEnd;
            const double value = showBegin
                ? transition.valueBegin
                : transition.valueEnd;
            ImGui::BeginTooltip();
            ImGui::Text("Distance: %.6g", domainValue);
            ImGui::Text("%s: %.6f", valueLabel, value);
            ImGui::EndTooltip();
        }

        if (windowHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            if (beginHovered)
            {
                interaction.clickedEndpoint = ScalarProfileEndpoint::Begin;
            }
            else if (endHovered)
            {
                interaction.clickedEndpoint = ScalarProfileEndpoint::End;
            }
            else
            {
                interaction.plotClicked =
                    mousePosition.x >= mapping.domainView.pixelBegin
                    && mousePosition.x <= mapping.domainView.pixelEnd
                    && mousePosition.y >= rowBeginY
                    && mousePosition.y <= rowEndY;
            }
        }

        return interaction;
    }

    [[nodiscard]] std::optional<quantum::math::TransitionType>
    drawTransitionTypeCombo(
        const char* const label,
        const quantum::math::TransitionType currentType)
    {
        const quantum::editor::TransitionTypePreset* const currentPreset =
            quantum::editor::findTransitionTypePreset(currentType);
        const char* const preview = currentPreset != nullptr
            ? currentPreset->displayName.data()
            : "Unsupported";
        std::optional<quantum::math::TransitionType> selectedType;

        const bool comboOpen = ImGui::BeginCombo(label, preview);
        const bool comboHovered = ImGui::IsItemHovered();

        if (comboOpen)
        {
            for (const quantum::editor::TransitionTypePreset& preset
                : quantum::editor::transitionTypePresets)
            {
                const bool selected = preset.type == currentType;

                if (ImGui::Selectable(preset.displayName.data(), selected)
                    && !selected)
                {
                    selectedType = preset.type;
                }

                if (selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }

            ImGui::EndCombo();
        }

        if (comboHovered)
        {
            ImGui::SetTooltip("Transition: %s", preview);
        }

        return selectedType;
    }

    void drawAuthoredDomainRuler(
        ImDrawList* const drawList,
        const AuthoredDomainView& domainView,
        const ImVec2 canvasBegin,
        const float rulerLineY,
        const float profileBeginY,
        const float profileEndY)
    {
        const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
        const ImU32 gridColor = ImGui::GetColorU32(ImGuiCol_Separator);
        const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);
        const ImU32 disabledTextColor = ImGui::GetColorU32(
            ImGuiCol_TextDisabled
        );

        drawList->AddText(canvasBegin, textColor, "AUTHORED DOMAIN");

        char domainLabel[64]{};
        std::snprintf(
            domainLabel,
            sizeof(domainLabel),
            "%.0f -> %.0f",
            domainView.domainBegin,
            domainView.domainEnd
        );
        drawList->AddText(
            ImVec2(canvasBegin.x, canvasBegin.y + ImGui::GetTextLineHeight()),
            disabledTextColor,
            domainLabel
        );

        drawList->AddLine(
            ImVec2(domainView.pixelBegin, rulerLineY),
            ImVec2(domainView.pixelEnd, rulerLineY),
            borderColor
        );

        for (int division = 0;
            division <= domainDivisionCount;
            ++division)
        {
            const double progress = static_cast<double>(division)
                / static_cast<double>(domainDivisionCount);
            const double domainValue = domainView.domainBegin
                + progress
                    * (domainView.domainEnd - domainView.domainBegin);
            const float x = domainView.toPixel(domainValue);

            drawList->AddLine(
                ImVec2(x, rulerLineY - 4.0F),
                ImVec2(x, rulerLineY + 4.0F),
                borderColor
            );
            drawList->AddLine(
                ImVec2(x, profileBeginY),
                ImVec2(x, profileEndY),
                gridColor
            );

            char tickLabel[32]{};
            std::snprintf(
                tickLabel,
                sizeof(tickLabel),
                "%.0f",
                domainValue
            );
            const float tickLabelWidth = ImGui::CalcTextSize(tickLabel).x;
            const float tickLabelX = std::clamp(
                x - tickLabelWidth * 0.5F,
                domainView.pixelBegin,
                domainView.pixelEnd - tickLabelWidth
            );
            drawList->AddText(
                ImVec2(tickLabelX, rulerLineY + 5.0F),
                disabledTextColor,
                tickLabel
            );
        }
    }

    [[nodiscard]] ScalarProfileRowEdit drawScalarProfileRow(
        ImDrawList* const drawList,
        const AuthoredDomainView& domainView,
        const ProfileRowView& row,
        const float labelX,
        const float rowBeginY,
        const float rowEndY,
        const ImU32 curveColor,
        double* const valueEndBuffer,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint)
    {
        const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
        const ImU32 gridColor = ImGui::GetColorU32(ImGuiCol_Separator);
        const float rowHeight = rowEndY - rowBeginY;
        const float textHeight = ImGui::GetTextLineHeight();
        const float contentHeight = textHeight
            + ImGui::GetFrameHeight()
            + ImGui::GetStyle().ItemSpacing.y;
        const float labelY = rowBeginY
            + std::max(0.0F, (rowHeight - contentHeight) * 0.5F);
        const float plotPaddingY = std::clamp(
            rowHeight * 0.14F,
            4.0F,
            8.0F
        );
        const float plotBeginY = rowBeginY + plotPaddingY;
        const float plotEndY = rowEndY - plotPaddingY;

        drawList->AddText(ImVec2(labelX, labelY), curveColor, row.label);

        ScalarProfileRowEdit edit;

        {
            const ImVec2 savedCursorPosition = ImGui::GetCursorScreenPos();
            const float controlWidth = std::max(
                64.0F,
                domainView.pixelBegin - labelX
                    - ImGui::GetStyle().ItemSpacing.x
            );
            const float valueEndWidth = std::clamp(
                controlWidth * 0.42F,
                52.0F,
                76.0F
            );
            const float transitionTypeWidth = std::max(
                64.0F,
                controlWidth - valueEndWidth
                    - ImGui::GetStyle().ItemSpacing.x
            );
            const float controlsY = labelY + textHeight
                + ImGui::GetStyle().ItemSpacing.y;
            ImGui::PushID(row.label);
            ImGui::SetCursorScreenPos(ImVec2(labelX, controlsY));
            ImGui::SetNextItemWidth(valueEndWidth);
            edit.valueEndEdited = ImGui::InputDouble(
                "##ValueEnd",
                valueEndBuffer,
                0.001,
                0.005,
                "%.6f"
            );

            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Authored end value");
            }

            ImGui::SetCursorScreenPos(ImVec2(
                labelX + valueEndWidth
                    + ImGui::GetStyle().ItemSpacing.x,
                controlsY
            ));
            ImGui::SetNextItemWidth(transitionTypeWidth);
            edit.transitionType = drawTransitionTypeCombo(
                "##TransitionType",
                row.transition->transitionType
            );

            ImGui::PopID();
            ImGui::SetCursorScreenPos(savedCursorPosition);
        }
        drawList->AddRect(
            ImVec2(domainView.pixelBegin, rowBeginY),
            ImVec2(domainView.pixelEnd, rowEndY),
            borderColor
        );

        for (int division = 0;
            division <= valueDivisionCount;
            ++division)
        {
            const float progress = static_cast<float>(division)
                / static_cast<float>(valueDivisionCount);
            const float y = plotEndY
                - progress * (plotEndY - plotBeginY);
            drawList->AddLine(
                ImVec2(domainView.pixelBegin, y),
                ImVec2(domainView.pixelEnd, y),
                gridColor
            );
        }

        std::array<ImVec2, profileSampleCount> points{};
        // Straight sections would collapse the value axis to a zero span,
        // which makes endpoint drags emit no value change at all; clamp
        // the half-span so every row stays draggable. The fallback matches
        // the magnitude of the default authored roll/pitch/yaw rates.
        constexpr double minimumValueMagnitude = 0.02;
        const double valueMagnitude = std::max(
            std::max(
                std::abs(row.transition->valueBegin),
                std::abs(row.transition->valueEnd)
            ),
            minimumValueMagnitude
        );
        const ScalarProfileMapping mapping{
            .domainView = domainView,
            .valueMagnitude = valueMagnitude,
            .pixelBeginY = plotBeginY,
            .pixelEndY = plotEndY
        };

        for (std::size_t sample = 0; sample < points.size(); ++sample)
        {
            const double progress = static_cast<double>(sample)
                / static_cast<double>(points.size() - 1);
            const double domainValue = domainView.domainBegin
                + progress
                    * (domainView.domainEnd - domainView.domainBegin);
            const double value = quantum::math::evaluateScalarTransition(
                *row.transition,
                domainValue
            );
            points[sample] = mapping.toPixel(domainValue, value);
        }

        drawList->AddPolyline(
            points.data(),
            static_cast<int>(points.size()),
            curveColor,
            ImDrawFlags_None,
            2.0F
        );

        edit.endpointInteraction = drawScalarProfileEndpoints(
            drawList,
            mapping,
            *row.transition,
            rowBeginY,
            rowEndY,
            curveColor,
            row.label,
            selectedEndpoint
        );
        edit.rowGeometry = {
            mapping.valueMagnitude,
            mapping.valueUnitsPerPixel()
        };

        return edit;
    }

    // One-line audit of the shared Transition Editor input settings; used
    // at startup and whenever the user commits a change in the settings
    // window. Tooling parses this format.
    void logTransitionEditorInputSettings(
        const quantum::editor::TransitionEditorInputSettings& settings)
    {
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[CFG] transition editor input: normalGain=%.2f fineGain=%.2f "
            "snapping=%s increment=%.4f",
            settings.normalDragGain,
            settings.fineDragGain,
            settings.snapEnabled ? "enabled" : "disabled",
            settings.snapIncrement
        );
    }

    void configureIniPath(std::string& pathStorage)    {
        char* const preferencePath = SDL_GetPrefPath("QUANTUM", "Editor");

        if (preferencePath == nullptr)
        {
            SDL_LogWarn(
                SDL_LOG_CATEGORY_APPLICATION,
                "SDL_GetPrefPath failed while locating editor settings: %s. "
                "Dear ImGui layout persistence will be disabled.",
                SDL_GetError()
            );
            ImGui::GetIO().IniFilename = nullptr;
            return;
        }

        pathStorage = (
            std::filesystem::path(preferencePath) / editorLayoutIniFileName
        ).string();
        SDL_free(preferencePath);
        ImGui::GetIO().IniFilename = pathStorage.c_str();
    }

    void loadBundledFont()
    {
        ImGuiIO& io = ImGui::GetIO();
        const char* const basePath = SDL_GetBasePath();

        if (basePath == nullptr)
        {
            throw std::runtime_error(
                std::string(
                    "SDL_GetBasePath failed while locating the bundled "
                    "Red Hat Mono SemiBold UI font: "
                ) + SDL_GetError()
            );
        }

        const std::filesystem::path fontPath =
            std::filesystem::path(basePath) / bundledFontRelativePath;
        std::error_code error;
        const bool fontExists = std::filesystem::is_regular_file(
            fontPath,
            error
        );

        if (!fontExists || error)
        {
            throw std::runtime_error(
                std::string(
                    "Bundled Red Hat Mono SemiBold UI font not found at "
                ) + fontPath.string()
            );
        }

        ImFont* const font = io.Fonts->AddFontFromFileTTF(
            fontPath.string().c_str()
        );

        if (font == nullptr)
        {
            throw std::runtime_error(
                std::string(
                    "Dear ImGui could not load the bundled Red Hat Mono "
                    "SemiBold UI font at "
                ) + fontPath.string()
            );
        }

        io.FontDefault = font;
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "Loaded bundled Red Hat Mono SemiBold UI font: %s",
            fontPath.string().c_str()
        );
    }

    void checkVulkanResult(const VkResult result)
    {
        if (result != VK_SUCCESS)
        {
            SDL_LogError(
                SDL_LOG_CATEGORY_RENDER,
                "Dear ImGui Vulkan backend reported VkResult %d.",
                static_cast<int>(result)
            );
        }
    }

    std::uint32_t contentPixelDimension(
        const float logicalDimension,
        const float framebufferScale)
    {
        if (!std::isfinite(logicalDimension)
            || !std::isfinite(framebufferScale)
            || logicalDimension <= 0.0F
            || framebufferScale <= 0.0F)
        {
            return 0;
        }

        const double pixels = std::floor(
            static_cast<double>(logicalDimension)
                * static_cast<double>(framebufferScale)
            + 0.5
        );

        if (pixels < 1.0)
        {
            return 0;
        }

        if (pixels
            > static_cast<double>(std::numeric_limits<std::uint32_t>::max()))
        {
            throw std::length_error(
                "The Editor viewport content size exceeds a 32-bit pixel "
                "dimension."
            );
        }

        return static_cast<std::uint32_t>(pixels);
    }

    float showMainMenuBar(bool* const inputSettingsOpen)
    {
        const float height = ImGui::GetFrameHeight();
        pushTopRegionStyle();

        if (!ImGui::BeginMainMenuBar())
        {
            ImGui::PopStyleColor(topRegionColorCount);
            return 0.0F;
        }

        if (ImGui::BeginMenu("File"))
        {
            ImGui::MenuItem("New", nullptr, false, false);
            ImGui::MenuItem("Open", nullptr, false, false);
            ImGui::MenuItem("Save", nullptr, false, false);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit"))
        {
            ImGui::MenuItem("Undo", nullptr, false, false);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Visualization", nullptr, false, false);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Preferences"))
        {
            if (ImGui::MenuItem(
                "Transition Editor Input",
                nullptr,
                inputSettingsOpen
            ))
            {
                // One-line audit per user action so tooling can track
                // the settings window without screen capture.
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[CFG] transition editor input window %s",
                    *inputSettingsOpen ? "open" : "closed"
                );
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
        ImGui::PopStyleColor(topRegionColorCount);
        return height;
    }

    float showCommandArea(ImGuiViewport* const mainViewport)
    {
        const ImGuiStyle& style = ImGui::GetStyle();
        const float height = style.WindowPadding.y * 2.0F
            + ImGui::GetTextLineHeight()
            + style.ItemSpacing.y
            + ImGui::GetFrameHeight();
        constexpr ImGuiWindowFlags windowFlags =
            ImGuiWindowFlags_NoDocking
            | ImGuiWindowFlags_NoTitleBar
            | ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoScrollbar
            | ImGuiWindowFlags_NoSavedSettings;

        pushTopRegionStyle();
        ImGui::PushStyleVar(ImGuiStyleVar_DisabledAlpha, 1.0F);
        const bool visible = ImGui::BeginViewportSideBar(
            "##QuantumCommandArea",
            mainViewport,
            ImGuiDir_Up,
            height,
            windowFlags
        );

        if (visible)
        {
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                quantum::editor::palette::commandHeadingGreen
            );
            ImGui::TextUnformatted("COMMAND");
            ImGui::PopStyleColor();

            ImGui::BeginDisabled();
            ImGui::Button("New");
            ImGui::SameLine();
            ImGui::Button("Open");
            ImGui::SameLine();
            ImGui::Button("Save");
            ImGui::SameLine();
            ImGui::Button("Undo");
            ImGui::SameLine();
            ImGui::Button("Toggle");
            ImGui::SameLine();
            ImGui::Button("Visualization");
            ImGui::SameLine();
            ImGui::Button("Start Test");
            ImGui::SameLine();
            ImGui::Button("Stop Test");
            ImGui::SameLine();
            ImGui::Button("Clearance Envelope");
            ImGui::EndDisabled();
        }

        ImGui::End();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(topRegionColorCount);
        return height;
    }

    [[nodiscard]] TrackWorkspaceEdit showTrackWorkspace(
        const quantum::coaster::AuthoredTrack& track,
        const std::size_t selectedIndex,
        double* const sectionLengthEdit)
    {
        TrackWorkspaceEdit edit;
        pushWorkspaceAccent(trackWorkspaceAccent);
        ImGui::Begin("TRACK WORKSPACE");

        ImGui::BeginDisabled();
        ImGui::Button("Track Style Properties...");
        ImGui::Button("Mechanism Segment Properties...");
        ImGui::Button("Track Segment Properties...");
        ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::BeginChild(
            "Section List",
            ImVec2(0.0F, 0.0F),
            ImGuiChildFlags_Borders))
        {
            ImGui::TextUnformatted("Section List");

            const std::size_t sectionCount = track.sectionCount();
            for (std::size_t index = 0; index < sectionCount; ++index)
            {
                char label[64]{};
                std::snprintf(
                    label,
                    sizeof(label),
                    "Section %llu  (%.3g)",
                    static_cast<unsigned long long>(index + 1),
                    quantum::coaster::sectionLength(track.section(index))
                );

                if (ImGui::Selectable(label, index == selectedIndex))
                {
                    edit.selectRequest = index;
                }
            }

            if (selectedIndex < sectionCount && sectionLengthEdit != nullptr)
            {
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::TextUnformatted("Selected Section");

                if (ImGui::InputDouble(
                    "Length",
                    sectionLengthEdit,
                    1.0,
                    10.0,
                    "%.3f"
                ))
                {
                    edit.lengthEdited = true;
                }

                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip(
                        "Authored distance-domain length of the selected "
                        "section"
                    );
                }
            }

            const ImGuiStyle& style = ImGui::GetStyle();
            const float buttonAreaHeight = 3.0F * ImGui::GetFrameHeight()
                + 2.0F * style.ItemSpacing.y;
            const float buttonAreaY = ImGui::GetWindowHeight()
                - style.WindowPadding.y
                - buttonAreaHeight;

            if (buttonAreaY > ImGui::GetCursorPosY())
            {
                ImGui::SetCursorPosY(buttonAreaY);
            }

            const bool hasSelection = selectedIndex < sectionCount;
            const float halfWidth = (ImGui::GetContentRegionAvail().x
                - style.ItemSpacing.x) * 0.5F;

            if (ImGui::Button(
                "Append Section",
                ImVec2(halfWidth, 0.0F)))
            {
                edit.appendRequested = true;
            }

            ImGui::SameLine();
            if (ImGui::Button(
                "Prepend Section",
                ImVec2(-1.0F, 0.0F)))
            {
                edit.prependRequested = true;
            }

            ImGui::BeginDisabled(!hasSelection || selectedIndex == 0);
            if (ImGui::Button("Move Up", ImVec2(halfWidth, 0.0F)))
            {
                edit.moveUpRequested = true;
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::BeginDisabled(
                !hasSelection || selectedIndex + 1 >= sectionCount);
            if (ImGui::Button("Move Down", ImVec2(-1.0F, 0.0F)))
            {
                edit.moveDownRequested = true;
            }
            ImGui::EndDisabled();

            ImGui::BeginDisabled(sectionCount <= 1);
            if (ImGui::Button("Remove Selected", ImVec2(-1.0F, 0.0F)))
            {
                edit.removeRequested = true;
            }
            ImGui::EndDisabled();
        }
        ImGui::EndChild();

        ImGui::End();
        ImGui::PopStyleColor(workspaceAccentColorCount);
        return edit;
    }

    void showSupportWorkspace()
    {
        pushWorkspaceAccent(supportWorkspaceAccent);
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            quantum::editor::palette::black
        );
        ImGui::Begin(supportWorkspaceWindowName);
        ImGui::PopStyleColor();

        ImGui::BeginDisabled();
        ImGui::Button("Prefab Panel...");
        ImGui::Button("Foundation Generator...");
        ImGui::Button("Rail Connector Generator...");
        ImGui::Button("Support Settings...");
        ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::PushStyleColor(
            ImGuiCol_Text,
            ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled)
        );
        ImGui::TextWrapped("Copy/Define (Autosaves as .qcwPrefab)...");
        ImGui::Spacing();
        ImGui::TextWrapped("Paste (Pastes as the autosaved Prefab)...");
        ImGui::PopStyleColor();

        ImGui::End();
        ImGui::PopStyleColor(workspaceAccentColorCount);
    }

    // Floating preferences window for the shared Transition Editor drag
    // and snapping settings. Widgets mutate the live settings struct, so
    // changes apply on the next drag frame without restart.
    void showTransitionEditorInputSettings(
        quantum::editor::TransitionEditorInputSettings& settings,
        bool* const open)
    {
        // Appearing keeps a sensible first-session position while letting
        // a user-moved (ini-persisted) position win afterwards.
        ImGui::SetNextWindowPos(ImVec2(480.0F, 110.0F), ImGuiCond_Appearing);
        if (!ImGui::Begin("Transition Editor Input", open))
        {
            ImGui::End();
            return;
        }

        bool committed = false;

        ImGui::DragFloat(
            "Drag Sensitivity",
            &settings.normalDragGain,
            0.01F,
            0.05F,
            5.0F,
            "%.2f",
            ImGuiSliderFlags_AlwaysClamp
        );
        committed = committed || ImGui::IsItemDeactivatedAfterEdit();

        ImGui::DragFloat(
            "Fine Drag Sensitivity",
            &settings.fineDragGain,
            0.005F,
            0.01F,
            5.0F,
            "%.2f",
            ImGuiSliderFlags_AlwaysClamp
        );
        committed = committed || ImGui::IsItemDeactivatedAfterEdit();

        if (ImGui::Checkbox("Enable Value Snapping", &settings.snapEnabled))
        {
            committed = true;
        }

        if (ImGui::InputDouble(
            "Snap Increment",
            &settings.snapIncrement,
            0.001,
            0.01,
            "%.4f"))
        {
            // The snapping math divides by this value, so it must stay
            // strictly positive and finite; typed garbage falls back to
            // the default increment.
            const double requested = settings.snapIncrement;
            settings.snapIncrement =
                std::isfinite(requested) && requested > 0.0
                    ? std::min(requested, 0.5)
                    : 0.005;
            committed = true;
        }

        if (committed)
        {
            logTransitionEditorInputSettings(settings);
        }

        ImGui::End();
    }

    [[nodiscard]] TransitionEditorEdit showTransitionEditor(
        const std::span<const ProfileRowView> profileRows,
        std::array<double, quantum::editor::rateChannelCount>&
            valueEndBuffers,
        const std::array<quantum::editor::ScalarProfileEndpoint,
            quantum::editor::rateChannelCount>& endpointSelections,
        const std::array<quantum::editor::ScalarProfileEndpoint,
            quantum::editor::rateChannelCount>& endpointDrags,
        std::array<std::optional<quantum::editor::ScalarDragAnchor>,
            quantum::editor::rateChannelCount>& dragAnchors,
        const quantum::editor::TransitionEditorInputSettings&
            inputSettings)
    {
        TransitionEditorEdit edit;
        pushWorkspaceAccent(transitionEditorAccent);
        ImGui::PushStyleColor(
            ImGuiCol_Border,
            quantum::editor::palette::transitionEditorBlue
        );
        ImGui::Begin("Transition Editor");

        constexpr ImGuiWindowFlags timelineWindowFlags =
            ImGuiWindowFlags_NoScrollbar
            | ImGuiWindowFlags_NoScrollWithMouse;

        if (ImGui::BeginChild(
            "##TransitionTimeline",
            ImVec2(0.0F, 0.0F),
            ImGuiChildFlags_Borders,
            timelineWindowFlags))
        {
            const ImGuiStyle& style = ImGui::GetStyle();
            const ImVec2 canvasBegin = ImGui::GetCursorScreenPos();
            const ImVec2 canvasSize = ImGui::GetContentRegionAvail();
            const ImVec2 canvasEnd{
                canvasBegin.x + canvasSize.x,
                canvasBegin.y + canvasSize.y
            };
            const float labelWidth = std::clamp(
                canvasSize.x * 0.22F,
                132.0F,
                196.0F
            );
            const float rulerHeight = std::clamp(
                canvasSize.y * 0.27F,
                42.0F,
                58.0F
            );
            const float plotBeginX = canvasBegin.x + labelWidth;
            const float plotEndX = canvasEnd.x
                - (endpointHandleRadius + 3.0F);
            const float profileBeginY = canvasBegin.y
                + rulerHeight
                + style.ItemSpacing.y;
            const float profileEndY = canvasEnd.y;
            const float rowSpacing = style.ItemSpacing.y;
            const float rowCount = static_cast<float>(profileRows.size());
            const float availableRowsHeight = profileEndY - profileBeginY
                - rowSpacing * (rowCount - 1.0F);
            const float rowHeight = rowCount > 0.0F
                ? availableRowsHeight / rowCount
                : 0.0F;

            if (plotEndX - plotBeginX < 48.0F
                || rowHeight < 28.0F)
            {
                ImGui::TextDisabled(
                    "Resize the Transition Editor to view the authored "
                    "domain."
                );
            }
            else
            {
                if (profileRows.front().transition == nullptr)
                {
                    throw std::logic_error(
                        "Transition Editor profile rows require valid "
                        "transitions."
                    );
                }

                const quantum::math::ScalarTransition&
                    authoritativeTransition = *profileRows.front().transition;

                for (const ProfileRowView& row
                    : profileRows)
                {
                    if (row.transition == nullptr
                        || row.transition->domainBegin
                            != authoritativeTransition.domainBegin
                        || row.transition->domainEnd
                            != authoritativeTransition.domainEnd)
                    {
                        throw std::logic_error(
                            "Transition Editor profiles require one exactly "
                            "shared authored domain."
                        );
                    }
                }

                const AuthoredDomainView domainView{
                    .domainBegin = authoritativeTransition.domainBegin,
                    .domainEnd = authoritativeTransition.domainEnd,
                    .pixelBegin = plotBeginX,
                    .pixelEnd = plotEndX
                };
                ImDrawList* const drawList = ImGui::GetWindowDrawList();
                const float textHeight = ImGui::GetTextLineHeight();
                const float rulerLineY = canvasBegin.y + textHeight + 8.0F;

                drawList->PushClipRect(canvasBegin, canvasEnd, true);
                drawList->AddRectFilled(
                    ImVec2(plotBeginX, canvasBegin.y),
                    ImVec2(canvasEnd.x, canvasBegin.y + rulerHeight),
                    transitionCanvasColor
                );
                drawList->AddRectFilled(
                    ImVec2(plotBeginX, profileBeginY),
                    ImVec2(canvasEnd.x, profileEndY),
                    transitionCanvasColor
                );
                drawAuthoredDomainRuler(
                    drawList,
                    domainView,
                    canvasBegin,
                    rulerLineY,
                    profileBeginY,
                    profileEndY
                );

                std::array<std::optional<ScalarRowEditGeometry>,
                    quantum::editor::rateChannelCount> rowGeometries{};
                for (std::size_t rowIndex = 0;
                    rowIndex < profileRows.size();
                    ++rowIndex)
                {
                    const float rowBeginY = profileBeginY
                        + static_cast<float>(rowIndex)
                            * (rowHeight + rowSpacing);
                    const ProfileRowView& row = profileRows[rowIndex];
                    const std::size_t channelIndex =
                        static_cast<std::size_t>(row.channel);
                    const ScalarProfileRowEdit rowEdit = drawScalarProfileRow(
                        drawList,
                        domainView,
                        row,
                        canvasBegin.x,
                        rowBeginY,
                        rowBeginY + rowHeight,
                        profileCurveColors[rowIndex],
                        &valueEndBuffers[channelIndex],
                        endpointSelections[channelIndex]
                    );
                    rowGeometries[channelIndex] = rowEdit.rowGeometry;

                    if (rowEdit.valueEndEdited)
                    {
                        edit.endpointValueEdit = {
                            .endpoint = quantum::editor::
                                ScalarProfileEndpoint::End,
                            .value = valueEndBuffers[channelIndex],
                            .continuous = false,
                            .channel = row.channel
                        };
                    }

                    if (rowEdit.endpointInteraction.clickedEndpoint
                        != quantum::editor::ScalarProfileEndpoint::None)
                    {
                        edit.click = TransitionEditorEdit::Click{
                            row.channel,
                            rowEdit.endpointInteraction.clickedEndpoint
                        };
                    }
                    else if (rowEdit.endpointInteraction.plotClicked)
                    {
                        edit.click = TransitionEditorEdit::Click{
                            row.channel,
                            quantum::editor::ScalarProfileEndpoint::None
                        };
                    }

                    if (rowEdit.transitionType.has_value())
                    {
                        edit.transitionType = TransitionEditorEdit::
                            TypeChange{row.channel, *rowEdit.transitionType};
                    }
                }

                // Active handle drags emit one edit per frame by
                // integrating cursor deltas from a rolling anchor. This is
                // the single drag path for every section and channel; the
                // gain multiplier makes Shift a live precision mode.
                const ImGuiIO& dragIo = ImGui::GetIO();
                for (std::size_t channelIndex = 0;
                    channelIndex < quantum::editor::rateChannelCount;
                    ++channelIndex)
                {
                    if (endpointDrags[channelIndex]
                        == quantum::editor::ScalarProfileEndpoint::None
                        || !dragAnchors[channelIndex].has_value()
                        || !rowGeometries[channelIndex].has_value())
                    {
                        continue;
                    }

                    const ScalarRowEditGeometry& geometry =
                        *rowGeometries[channelIndex];
                    const float gainMultiplier = dragIo.KeyShift
                        ? inputSettings.fineDragGain
                        : inputSettings.normalDragGain;
                    const double currentPixelY = static_cast<double>(
                        dragIo.MousePos.y);

                    quantum::editor::ScalarDragAnchor& anchor =
                        *dragAnchors[channelIndex];
                    const double deltaPixels =
                        currentPixelY - anchor.pixelY;
                    if (std::abs(deltaPixels)
                        > maxPlausiblePixelDeltaPerFrame)
                    {
                        // Coordinate-space switch, not hand motion:
                        // re-base the pixel reference without changing
                        // the value.
                        anchor.pixelY = currentPixelY;
                        continue;
                    }
                    // Screen Y grows downward while authored values grow
                    // upward, hence the subtraction.
                    double value = anchor.value
                        - deltaPixels
                            * geometry.unitsPerPixel
                            * static_cast<double>(gainMultiplier);
                    value = std::clamp(
                        value,
                        -geometry.valueMagnitude,
                        geometry.valueMagnitude
                    );
                    if (inputSettings.snapEnabled
                        && inputSettings.snapIncrement > 0.0)
                    {
                        value = snapToIncrement(
                            value,
                            inputSettings.snapIncrement
                        );
                    }

                    anchor.pixelY = currentPixelY;
                    anchor.value = value;

                    edit.endpointValueEdit = {
                        .endpoint = endpointDrags[channelIndex],
                        .value = value,
                        .continuous = true,
                        .channel = static_cast<quantum::editor::RateChannel>(
                            channelIndex)
                    };
                }

                drawList->PopClipRect();
            }

            ImGui::Dummy(canvasSize);
        }
        ImGui::EndChild();

        ImGui::End();
        ImGui::PopStyleColor(workspaceAccentColorCount + 1);
        return edit;
    }

    void buildDefaultDockLayout(
        const ImGuiID dockspaceId,
        const ImVec2 dockspaceSize)
    {
        ImGui::DockBuilderRemoveNode(dockspaceId);
        ImGui::DockBuilderAddNode(
            dockspaceId,
            ImGuiDockNodeFlags_DockSpace
                | ImGuiDockNodeFlags_PassthruCentralNode
        );
        ImGui::DockBuilderSetNodeSize(dockspaceId, dockspaceSize);

        ImGuiID upperId = dockspaceId;
        ImGuiID bottomId = 0;
        ImGui::DockBuilderSplitNode(
            upperId,
            ImGuiDir_Down,
            0.27F,
            &bottomId,
            &upperId
        );

        ImGuiID leftId = 0;
        ImGui::DockBuilderSplitNode(
            upperId,
            ImGuiDir_Left,
            0.19F,
            &leftId,
            &upperId
        );

        ImGuiID rightId = 0;
        ImGuiID centerId = 0;
        ImGui::DockBuilderSplitNode(
            upperId,
            ImGuiDir_Right,
            0.19F / 0.81F,
            &rightId,
            &centerId
        );

        ImGui::DockBuilderDockWindow("TRACK WORKSPACE", leftId);
        ImGui::DockBuilderDockWindow("3D Viewport", centerId);
        ImGui::DockBuilderDockWindow(supportWorkspaceWindowName, rightId);
        ImGui::DockBuilderDockWindow("Transition Editor", bottomId);
        ImGui::DockBuilderFinish(dockspaceId);
    }
}

namespace quantum::editor
{
    EditorUi::~EditorUi()
    {
        shutdown();
    }

    void EditorUi::initialize(
        SDL_Window* const window,
        const renderer::VulkanContext& vulkan,
        const coaster::AuthoredTrack& authoredTrack,
        const glm::dvec3& centerlineMinimum,
        const glm::dvec3& centerlineMaximum)
    {
        if (window == nullptr)
        {
            throw std::invalid_argument(
                "EditorUi requires a valid SDL window."
            );
        }

        if (authoredTrack.sectionCount() == 0)
        {
            throw std::invalid_argument(
                "EditorUi requires an authored track with at least one "
                "section."
            );
        }

        if (contextCreated_)
        {
            throw std::logic_error("EditorUi is already initialized.");
        }

        viewportCamera_.setBounds(
            centerlineMinimum,
            centerlineMaximum
        );
        window_ = window;
        authoredTrack_ = &authoredTrack;
        selectedSection_ = 0;
        for (std::size_t channelIndex = 0;
            channelIndex < rateChannelCount;
            ++channelIndex)
        {
            const auto channel = static_cast<RateChannel>(channelIndex);
            valueEndEditBuffers_[channelIndex] =
                sectionRateProfile(authoredTrack.section(0), channel)
                    .valueEnd;
            endpointSelections_[channelIndex] =
                ScalarProfileEndpoint::None;
            endpointDrags_[channelIndex] = ScalarProfileEndpoint::None;
            dragLastValues_[channelIndex].reset();
            scalarDragAnchors_[channelIndex].reset();
        }
        sectionLengthEditBuffer_ =
            coaster::sectionLength(authoredTrack.section(0));
        profileEndpointValueEdit_.reset();
        profileTransitionTypeEdit_.reset();
        trackCommand_.reset();
        sectionLengthEdit_.reset();
        initialViewportFramePending_ = true;
        cameraGesture_ = CameraGesture::None;

        // Headless-test escape hatch for value snapping, now redundant
        // with the Preferences > Transition Editor Input window but kept
        // for automation: an increment via QUANTUM_TRANSITION_SNAP enables
        // snapping at startup.
        if (const char* snapEnvironment =
            std::getenv("QUANTUM_TRANSITION_SNAP"))
        {
            const double parsedIncrement =
                std::strtod(snapEnvironment, nullptr);
            if (parsedIncrement > 0.0 && std::isfinite(parsedIncrement))
            {
                transitionEditorInputSettings_.snapEnabled = true;
                transitionEditorInputSettings_.snapIncrement =
                    parsedIncrement;
            }
        }
        logTransitionEditorInputSettings(transitionEditorInputSettings_);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        contextCreated_ = true;

        try
        {
            ImGuiIO& io = ImGui::GetIO();
            io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
            configureIniPath(iniPath_);

            applyQuantumStyle();
            loadBundledFont();

            if (!ImGui_ImplSDL3_InitForVulkan(window))
            {
                throw std::runtime_error(
                    "ImGui_ImplSDL3_InitForVulkan failed."
                );
            }
            sdlBackendInitialized_ = true;

            initializeVulkanBackend(vulkan);
        }
        catch (...)
        {
            shutdown();
            throw;
        }

        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "Dear ImGui initialized with SDL3, Vulkan, and docking support."
        );
    }

    void EditorUi::initializeVulkanBackend(
        const renderer::VulkanContext& vulkan)
    {
        const std::uint32_t imageCount = vulkan.swapchainImageCount();
        const int minimumImageCount =
            ImGui_ImplVulkanH_GetMinImageCountFromPresentMode(
                VK_PRESENT_MODE_FIFO_KHR
            );

        if (minimumImageCount < 2
            || imageCount < static_cast<std::uint32_t>(minimumImageCount))
        {
            throw std::runtime_error(
                "The Vulkan swapchain does not provide enough images for "
                "Dear ImGui."
            );
        }

        const VkFormat colorFormat = vulkan.swapchainFormat();

        if (colorFormat == VK_FORMAT_UNDEFINED)
        {
            throw std::runtime_error(
                "Dear ImGui cannot initialize without a swapchain format."
            );
        }

        ImGui_ImplVulkan_InitInfo initInfo{};
        initInfo.ApiVersion = VK_API_VERSION_1_4;
        initInfo.Instance = vulkan.instance();
        initInfo.PhysicalDevice = vulkan.physicalDevice();
        initInfo.Device = vulkan.device();
        initInfo.QueueFamily = vulkan.graphicsQueueFamily();
        initInfo.Queue = vulkan.graphicsQueue();
        initInfo.DescriptorPoolSize =
            IMGUI_IMPL_VULKAN_MINIMUM_SAMPLED_IMAGE_POOL_SIZE;
        initInfo.MinImageCount = static_cast<std::uint32_t>(
            minimumImageCount
        );
        initInfo.ImageCount = imageCount;
        initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        initInfo.PipelineInfoMain.PipelineRenderingCreateInfo.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        initInfo.PipelineInfoMain.PipelineRenderingCreateInfo
            .colorAttachmentCount = 1;
        initInfo.PipelineInfoMain.PipelineRenderingCreateInfo
            .pColorAttachmentFormats = &colorFormat;
        initInfo.UseDynamicRendering = true;
        initInfo.CheckVkResultFn = checkVulkanResult;

        if (!ImGui_ImplVulkan_Init(&initInfo))
        {
            throw std::runtime_error("ImGui_ImplVulkan_Init failed.");
        }

        // ImGui_ImplVulkan_Shutdown() clears every viewport's PlatformHandle,
        // including the main viewport's SDL_WindowID association that
        // ImGui_ImplSDL3 requires to route window/input events into Dear
        // ImGui. That association is established only by the SDL backend's
        // Init, so restore it after each Vulkan backend reinitialization.
        if (sdlBackendInitialized_)
        {
            const auto windowId =
                static_cast<std::uintptr_t>(SDL_GetWindowID(window_));
            ImGui::GetMainViewport()->PlatformHandle = (void*)(windowId);
        }

        device_ = vulkan.device();
        vulkanBackendInitialized_ = true;
        swapchainGeneration_ = vulkan.swapchainGeneration();
    }

    void EditorUi::shutdownVulkanBackend() noexcept
    {
        if (!vulkanBackendInitialized_)
        {
            return;
        }

        // The backend owns device resources referenced by submitted frames.
        // They must not be destroyed until those frames have completed.
        const VkResult result = vkDeviceWaitIdle(device_);

        if (result != VK_SUCCESS)
        {
            SDL_LogError(
                SDL_LOG_CATEGORY_RENDER,
                "vkDeviceWaitIdle failed before Dear ImGui shutdown with "
                "VkResult %d.",
                static_cast<int>(result)
            );
        }

        removeViewportTexture();
        ImGui_ImplVulkan_Shutdown();
        vulkanBackendInitialized_ = false;
        device_ = VK_NULL_HANDLE;
    }

    void EditorUi::processEvent(const SDL_Event& event)
    {
        // Low-frequency input/window audit trail. High-frequency events
        // (motion, enter/leave, move) are intentionally not logged.
        switch (event.type)
        {
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP:
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] %s btn=%d pos=(%.0f,%.0f)",
                    event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
                        ? "MOUSE_DOWN"
                        : "MOUSE_UP",
                    event.button.button,
                    event.button.x,
                    event.button.y
                );
                break;
            case SDL_EVENT_WINDOW_RESIZED:
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] RESIZED %dx%d",
                    event.window.data1,
                    event.window.data2
                );
                break;
            case SDL_EVENT_WINDOW_MAXIMIZED:
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] MAXIMIZED"
                );
                break;
            case SDL_EVENT_WINDOW_MINIMIZED:
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] MINIMIZED"
                );
                break;
            case SDL_EVENT_WINDOW_RESTORED:
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] RESTORED"
                );
                break;
            default:
                break;
        }

        if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
        {
            cameraGesture_ = CameraGesture::None;
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            for (std::optional<double>& lastValue : dragLastValues_)
            {
                lastValue.reset();
            }
            for (std::optional<ScalarDragAnchor>& anchor :
                scalarDragAnchors_)
            {
                anchor.reset();
            }
        }

        if (sdlBackendInitialized_)
        {
            ImGui_ImplSDL3_ProcessEvent(&event);
        }
    }

    void EditorUi::updateViewportTexture(
        renderer::VulkanContext& vulkan,
        const std::uint32_t width,
        const std::uint32_t height)
    {
        const VkExtent2D currentExtent = vulkan.viewportExtent();

        if (currentExtent.width != width || currentExtent.height != height)
        {
            vulkan.resizeViewportTarget(
                width,
                height,
                &EditorUi::retireViewportTexture,
                this
            );
        }

        if (viewportTexture_ == VK_NULL_HANDLE
            && vulkan.viewportImageView() != VK_NULL_HANDLE)
        {
            viewportTexture_ = ImGui_ImplVulkan_AddTexture(
                vulkan.viewportImageView(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            );

            if (viewportTexture_ == VK_NULL_HANDLE)
            {
                throw std::runtime_error(
                    "Dear ImGui could not register the Editor viewport image."
                );
            }
        }
    }

    void EditorUi::retireViewportTexture(void* const userData) noexcept
    {
        static_cast<EditorUi*>(userData)->removeViewportTexture();
    }

    void EditorUi::removeViewportTexture() noexcept
    {
        if (viewportTexture_ != VK_NULL_HANDLE)
        {
            ImGui_ImplVulkan_RemoveTexture(viewportTexture_);
            viewportTexture_ = VK_NULL_HANDLE;
        }
    }

    void EditorUi::updateViewportCamera(
        renderer::VulkanContext& vulkan,
        const bool viewportHovered,
        const std::uint32_t pixelWidth,
        const std::uint32_t pixelHeight,
        const float logicalWidth,
        const float logicalHeight)
    {
        const double aspectRatio = static_cast<double>(pixelWidth)
            / static_cast<double>(pixelHeight);

        if (initialViewportFramePending_)
        {
            viewportCamera_.frame(aspectRatio);
            initialViewportFramePending_ = false;
        }

        ImGuiIO& io = ImGui::GetIO();

        if (io.AppFocusLost)
        {
            cameraGesture_ = CameraGesture::None;
        }

        if (firstActiveEndpoint(endpointDrags_)
            != ScalarProfileEndpoint::None)
        {
            cameraGesture_ = CameraGesture::None;
        }

        if (cameraGesture_ == CameraGesture::None
            && firstActiveEndpoint(endpointDrags_)
                == ScalarProfileEndpoint::None
            && viewportHovered)
        {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] CAMERA_GESTURE_START Orbit MousePos=(%.0f,%.0f)", io.MousePos.x, io.MousePos.y);
                cameraGesture_ = CameraGesture::Orbit;
            }
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] CAMERA_GESTURE_START Pan MousePos=(%.0f,%.0f)", io.MousePos.x, io.MousePos.y);
                cameraGesture_ = CameraGesture::Pan;
            }
        }

        if (cameraGesture_ == CameraGesture::Orbit)
        {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] CAMERA_GESTURE_END Orbit");
                cameraGesture_ = CameraGesture::None;
            }
            else
            {
                viewportCamera_.orbit(
                    -static_cast<double>(io.MouseDelta.x)
                        * orbitRadiansPerPixel,
                    -static_cast<double>(io.MouseDelta.y)
                        * orbitRadiansPerPixel
                );
            }
        }
        else if (cameraGesture_ == CameraGesture::Pan)
        {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle))
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] CAMERA_GESTURE_END Pan");
                cameraGesture_ = CameraGesture::None;
            }
            else
            {
                viewportCamera_.pan(
                    static_cast<double>(io.MouseDelta.x),
                    static_cast<double>(io.MouseDelta.y),
                    static_cast<double>(logicalWidth),
                    static_cast<double>(logicalHeight)
                );
            }
        }

        if (viewportHovered)
        {
            if (io.MouseWheel != 0.0F)
            {
                viewportCamera_.zoom(static_cast<double>(io.MouseWheel));
            }

            if (ImGui::IsKeyPressed(ImGuiKey_F, false))
            {
                viewportCamera_.frame(aspectRatio);
            }
        }

        vulkan.setViewportViewProjection(
            viewportCamera_.viewProjection(aspectRatio)
        );
    }

    void EditorUi::beginFrame(renderer::VulkanContext& vulkan)
    {
        if (!vulkanBackendInitialized_ || !sdlBackendInitialized_)
        {
            throw std::logic_error("EditorUi is not initialized.");
        }

        if (swapchainGeneration_ != vulkan.swapchainGeneration())
        {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "[BEG] swapchain gen changed %llu->%llu reinit",
                swapchainGeneration_, vulkan.swapchainGeneration());
            shutdownVulkanBackend();
            initializeVulkanBackend(vulkan);
        }

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        const bool anyEndpointDragReleased =
            firstActiveEndpoint(endpointDrags_)
                != ScalarProfileEndpoint::None
            && !ImGui::IsMouseDown(ImGuiMouseButton_Left);

        const float menuBarHeight = showMainMenuBar(
            &inputSettingsWindowOpen_);

        ImGuiViewport* const mainViewport = ImGui::GetMainViewport();
        const float commandAreaHeight = showCommandArea(mainViewport);

        showTransitionEditorInputSettings(
            transitionEditorInputSettings_,
            &inputSettingsWindowOpen_
        );

        const ImGuiID dockspaceId = ImGui::GetID(editorDockspaceName);
        const bool defaultLayoutRequired =
            ImGui::DockBuilderGetNode(dockspaceId) == nullptr;

        ImGui::DockSpaceOverViewport(
            dockspaceId,
            mainViewport,
            ImGuiDockNodeFlags_PassthruCentralNode
        );

        if (defaultLayoutRequired)
        {
            ImVec2 defaultDockspaceSize = mainViewport->Size;
            defaultDockspaceSize.y -= menuBarHeight + commandAreaHeight;

            if (defaultDockspaceSize.y <= 0.0F)
            {
                throw std::runtime_error(
                    "The Editor command area leaves no room for its dockspace."
                );
            }

            buildDefaultDockLayout(dockspaceId, defaultDockspaceSize);
        }

        constexpr ImGuiWindowFlags viewportWindowFlags =
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
        const bool viewportContentVisible = ImGui::Begin(
            "3D Viewport",
            nullptr,
            viewportWindowFlags
        );

        if (viewportContentVisible)
        {
            const ImVec2 availableSize = ImGui::GetContentRegionAvail();
            const ImVec2 framebufferScale =
                ImGui::GetIO().DisplayFramebufferScale;
            const std::uint32_t viewportWidth = contentPixelDimension(
                availableSize.x,
                framebufferScale.x
            );
            const std::uint32_t viewportHeight = contentPixelDimension(
                availableSize.y,
                framebufferScale.y
            );

            updateViewportTexture(
                vulkan,
                viewportWidth,
                viewportHeight
            );

            if (viewportTexture_ != VK_NULL_HANDLE
                && viewportWidth != 0
                && viewportHeight != 0)
            {
                const ImTextureID textureId = static_cast<ImTextureID>(
                    reinterpret_cast<std::uintptr_t>(viewportTexture_)
                );
                ImGui::Image(ImTextureRef{textureId}, availableSize);
                updateViewportCamera(
                    vulkan,
                    ImGui::IsItemHovered(),
                    viewportWidth,
                    viewportHeight,
                    availableSize.x,
                    availableSize.y
                );
            }
            else if (availableSize.x > 0.0F && availableSize.y > 0.0F)
            {
                cameraGesture_ = CameraGesture::None;
                ImGui::Dummy(availableSize);
            }
        }
        else
        {
            cameraGesture_ = CameraGesture::None;
            updateViewportTexture(vulkan, 0, 0);
        }

        ImGui::End();

        showSupportWorkspace();

        // Clamp the selection against the live document before any panel
        // reads it; structural commits can shrink or reorder sections.
        const std::size_t sectionCount = authoredTrack_->sectionCount();
        if (selectedSection_ >= sectionCount)
        {
            selectedSection_ = sectionCount - 1;
            endpointSelections_.fill(ScalarProfileEndpoint::None);
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            for (std::optional<double>& lastValue : dragLastValues_)
            {
                lastValue.reset();
            }
            for (std::optional<ScalarDragAnchor>& anchor :
                scalarDragAnchors_)
            {
                anchor.reset();
            }
        }

        const TrackWorkspaceEdit workspaceEdit = showTrackWorkspace(
            *authoredTrack_,
            selectedSection_,
            &sectionLengthEditBuffer_
        );

        if (workspaceEdit.selectRequest.has_value()
            && *workspaceEdit.selectRequest != selectedSection_)
        {
            selectedSection_ = *workspaceEdit.selectRequest;
            endpointSelections_.fill(ScalarProfileEndpoint::None);
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            for (std::optional<double>& lastValue : dragLastValues_)
            {
                lastValue.reset();
            }
            for (std::optional<ScalarDragAnchor>& anchor :
                scalarDragAnchors_)
            {
                anchor.reset();
            }

            const coaster::AuthoredTrackSection& selected =
                authoredTrack_->section(selectedSection_);
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[SEL] selected=%zu rollEnd=%.6f pitchEnd=%.6f "
                "yawEnd=%.6f",
                selectedSection_,
                selected.rateProfiles.roll.valueEnd,
                selected.rateProfiles.pitch.valueEnd,
                selected.rateProfiles.yaw.valueEnd
            );
            for (std::size_t channelIndex = 0;
                channelIndex < rateChannelCount;
                ++channelIndex)
            {
                valueEndEditBuffers_[channelIndex] =
                    sectionRateProfile(
                        selected,
                        static_cast<RateChannel>(channelIndex)
                    ).valueEnd;
            }
            sectionLengthEditBuffer_ =
                coaster::sectionLength(selected);
        }

        if (workspaceEdit.appendRequested)
        {
            trackCommand_ = {TrackCommandType::AppendSection,
                             selectedSection_, 0.0};
        }
        else if (workspaceEdit.prependRequested)
        {
            trackCommand_ = {TrackCommandType::PrependSection,
                             selectedSection_, 0.0};
        }
        else if (workspaceEdit.removeRequested)
        {
            trackCommand_ = {TrackCommandType::RemoveSection,
                             selectedSection_, 0.0};
        }
        else if (workspaceEdit.moveUpRequested)
        {
            trackCommand_ = {TrackCommandType::MoveSectionUp,
                             selectedSection_, 0.0};
        }
        else if (workspaceEdit.moveDownRequested)
        {
            trackCommand_ = {TrackCommandType::MoveSectionDown,
                             selectedSection_, 0.0};
        }

        if (workspaceEdit.lengthEdited)
        {
            sectionLengthEdit_ = {TrackCommandType::SetSectionLength,
                                  selectedSection_,
                                  sectionLengthEditBuffer_};
        }
        // The Transition Editor always shows the currently selected
        // section's authored rate profiles over that section's local
        // distance domain. Every channel row is editable the same way.
        const coaster::AuthoredTrackSection& editedSection =
            authoredTrack_->section(selectedSection_);
        const std::array<ProfileRowView, rateChannelCount> profileRows{{
            {"Roll Rate",
             &editedSection.rateProfiles.roll,
             RateChannel::Roll},
            {"Pitch Rate",
             &editedSection.rateProfiles.pitch,
             RateChannel::Pitch},
            {"Yaw Rate",
             &editedSection.rateProfiles.yaw,
             RateChannel::Yaw},
        }};
        const TransitionEditorEdit transitionEdit = showTransitionEditor(
            profileRows,
            valueEndEditBuffers_,
            endpointSelections_,
            endpointDrags_,
            scalarDragAnchors_,
            transitionEditorInputSettings_
        );

        if (transitionEdit.endpointValueEdit.has_value())
        {
            const ScalarProfileEndpointValueEdit& valueEdit =
                *transitionEdit.endpointValueEdit;
            const math::ScalarTransition& channelProfile =
                sectionRateProfile(editedSection, valueEdit.channel);
            const double acceptedValue =
                valueEdit.endpoint == ScalarProfileEndpoint::Begin
                ? channelProfile.valueBegin
                : channelProfile.valueEnd;
            const std::size_t channelIndex =
                static_cast<std::size_t>(valueEdit.channel);

            // Snapping applies to every user-proposed edit at this single
            // choke point; stored track data is never touched here unless
            // the user actually edits it.
            double candidateValue = valueEdit.value;
            const TransitionEditorInputSettings& inputSettings =
                transitionEditorInputSettings_;
            if (inputSettings.snapEnabled
                && inputSettings.snapIncrement > 0.0)
            {
                candidateValue = snapToIncrement(
                    candidateValue,
                    inputSettings.snapIncrement
                );
            }

            const bool repeatsLastDragValue = valueEdit.continuous
                && dragLastValues_[channelIndex].has_value()
                && candidateValue == *dragLastValues_[channelIndex];

            if (std::isfinite(candidateValue)
                && valueEdit.endpoint != ScalarProfileEndpoint::None
                && candidateValue != acceptedValue
                && !repeatsLastDragValue)
            {
                ScalarProfileEndpointValueEdit queuedEdit = valueEdit;
                queuedEdit.value = candidateValue;
                queuedEdit.sectionIndex = selectedSection_;
                profileEndpointValueEdit_ = queuedEdit;

                if (valueEdit.continuous)
                {
                    pendingDragSummary_ = true;
                    dragSummaryEdit_ = queuedEdit;
                }
            }

            if (valueEdit.continuous)
            {
                dragLastValues_[channelIndex] = candidateValue;
            }
        }

        if (transitionEdit.transitionType.has_value())
        {
            profileTransitionTypeEdit_ = ProfileTransitionTypeEdit{
                transitionEdit.transitionType->type,
                selectedSection_,
                transitionEdit.transitionType->channel
            };
        }

        if (transitionEdit.click.has_value())
        {
            const std::size_t clickChannel = static_cast<std::size_t>(
                transitionEdit.click->channel);
            if (transitionEdit.click->endpoint
                != ScalarProfileEndpoint::None)
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] TRANSITION_CLICK channel=%zu endpoint=%d",
                    clickChannel,
                    static_cast<int>(transitionEdit.click->endpoint));
                endpointSelections_[clickChannel] =
                    transitionEdit.click->endpoint;
                endpointDrags_[clickChannel] =
                    transitionEdit.click->endpoint;
                dragLastValues_[clickChannel].reset();
                // Seed the rolling drag anchor at the committed value and
                // current cursor so arming a drag never jumps the value.
                const math::ScalarTransition& clickedProfile =
                    sectionRateProfile(
                        editedSection,
                        transitionEdit.click->channel
                    );
                const double committedValue =
                    transitionEdit.click->endpoint
                        == ScalarProfileEndpoint::Begin
                    ? clickedProfile.valueBegin
                    : clickedProfile.valueEnd;
                scalarDragAnchors_[clickChannel] = ScalarDragAnchor{
                    static_cast<double>(ImGui::GetIO().MousePos.y),
                    committedValue
                };
            }
            else
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] TRANSITION_PLOT_CLICK channel=%zu",
                    clickChannel);
                endpointSelections_[clickChannel] =
                    ScalarProfileEndpoint::None;
                endpointDrags_[clickChannel] =
                    ScalarProfileEndpoint::None;
                dragLastValues_[clickChannel].reset();
                scalarDragAnchors_[clickChannel].reset();
            }
        }

        if (anyEndpointDragReleased)
        {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "[INP] DRAG_RELEASED drag was=%d",
                static_cast<int>(firstActiveEndpoint(endpointDrags_)));
            // One summary line per completed drag gesture: continuous
            // edits regenerate the track silently every frame.
            if (pendingDragSummary_)
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[EDIT] section=%zu channel=%d endpoint=%d value=%.6f",
                    dragSummaryEdit_.sectionIndex,
                    static_cast<int>(dragSummaryEdit_.channel),
                    static_cast<int>(dragSummaryEdit_.endpoint),
                    dragSummaryEdit_.value);
                pendingDragSummary_ = false;
            }
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            for (std::optional<double>& lastValue : dragLastValues_)
            {
                lastValue.reset();
            }
            for (std::optional<ScalarDragAnchor>& anchor :
                scalarDragAnchors_)
            {
                anchor.reset();
            }
        }

        ImGui::Render();
    }

    std::optional<ScalarProfileEndpointValueEdit>
    EditorUi::takeProfileEndpointValueEdit() noexcept
    {
        const std::optional<ScalarProfileEndpointValueEdit> edit =
            profileEndpointValueEdit_;
        profileEndpointValueEdit_.reset();
        return edit;
    }

    std::optional<ProfileTransitionTypeEdit>
    EditorUi::takeProfileTransitionTypeEdit() noexcept
    {
        const std::optional<ProfileTransitionTypeEdit> edit =
            profileTransitionTypeEdit_;
        profileTransitionTypeEdit_.reset();
        return edit;
    }

    std::optional<TrackCommand> EditorUi::takeTrackCommand() noexcept
    {
        const std::optional<TrackCommand> command = trackCommand_;
        trackCommand_.reset();
        return command;
    }

    std::optional<TrackCommand> EditorUi::takeSectionLengthEdit() noexcept
    {
        const std::optional<TrackCommand> edit = sectionLengthEdit_;
        sectionLengthEdit_.reset();
        return edit;
    }

    void EditorUi::synchronizeProfileValueEnd(
        const RateChannel channel,
        const double acceptedValueEnd)
    {
        if (!std::isfinite(acceptedValueEnd))
        {
            throw std::invalid_argument(
                "EditorUi requires a finite accepted profile end value."
            );
        }

        valueEndEditBuffers_[static_cast<std::size_t>(channel)] =
            acceptedValueEnd;
    }

    void EditorUi::synchronizeSectionLength(const double acceptedLength)
    {
        if (!std::isfinite(acceptedLength) || acceptedLength <= 0.0)
        {
            throw std::invalid_argument(
                "EditorUi requires a positive finite accepted section "
                "length."
            );
        }

        sectionLengthEditBuffer_ = acceptedLength;
    }

    void EditorUi::setCenterlineBounds(
        const glm::dvec3& centerlineMinimum,
        const glm::dvec3& centerlineMaximum)
    {
        viewportCamera_.setBounds(
            centerlineMinimum,
            centerlineMaximum
        );
    }

    void EditorUi::render(const VkCommandBuffer commandBuffer)
    {
        if (!vulkanBackendInitialized_)
        {
            throw std::logic_error("EditorUi is not initialized.");
        }

        ImGui_ImplVulkan_RenderDrawData(
            ImGui::GetDrawData(),
            commandBuffer
        );
    }

    void EditorUi::shutdown() noexcept
    {
        if (vulkanBackendInitialized_)
        {
            shutdownVulkanBackend();
        }

        if (sdlBackendInitialized_)
        {
            ImGui_ImplSDL3_Shutdown();
            sdlBackendInitialized_ = false;
        }

        if (contextCreated_)
        {
            ImGui::DestroyContext();
            contextCreated_ = false;
        }

        swapchainGeneration_ = 0;
        cameraGesture_ = CameraGesture::None;
        initialViewportFramePending_ = true;
        authoredTrack_ = nullptr;
        selectedSection_ = 0;
        valueEndEditBuffers_.fill(0.0);
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        for (std::optional<double>& lastValue : dragLastValues_)
        {
            lastValue.reset();
        }
        for (std::optional<ScalarDragAnchor>& anchor : scalarDragAnchors_)
        {
            anchor.reset();
        }
        sectionLengthEditBuffer_ = 0.0;
        profileEndpointValueEdit_.reset();
        profileTransitionTypeEdit_.reset();
        trackCommand_.reset();
        sectionLengthEdit_.reset();
        iniPath_.clear();
    }
}
