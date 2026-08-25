#include <quantum/editor/EditorUi.hpp>

#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/EditorStyle.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_log.h>
#include <SDL3/SDL_messagebox.h>
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
    constexpr double piRadians = 3.14159265358979323846;
    // The Geometry Editor presents angles in degrees; Core keeps radians.
    constexpr double degreesPerRadian = 180.0 / piRadians;
    constexpr double radiansPerDegree = piRadians / 180.0;
    // Dedicated detail window for geometry-authored regions. It replaces
    // the Transition Editor for those selections instead of idling beside
    // it, so each region kind owns exactly one primary editor surface.
    constexpr char geometryEditorWindowName[] = "Geometry Editor";

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
    // Interior joints idle as small dots so a multi-segment curve stays
    // readable; they grow into full handles on hover.
    constexpr float jointHandleRadius = 2.5F;
    constexpr float jointSelectedRadius = 3.5F;
    // Cumulative cursor travel required before a drag commits to one
    // axis; below this threshold neither value nor distance integrates.
    constexpr double dragAxisLockPixelThreshold = 4.0;

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
        std::uint32_t clickedSegmentId = 0;
        bool plotClicked = false;
        std::uint32_t plotClickedSegmentId = 0;
        double hoverDomainDistance = 0.0;
        bool hoverInsidePlot = false;
    };

    // Value-axis geometry of one drawn profile row; drives centralized
    // drag sensitivity math in showTransitionEditor.
    struct ScalarRowEditGeometry
    {
        double valueMagnitude = 0.0;
        double unitsPerPixel = 0.0;
        double distanceUnitsPerPixel = 0.0;
    };

    struct ScalarProfileRowEdit
    {
        bool valueEndEdited = false;
        std::optional<quantum::math::TransitionType> transitionType;
        ScalarProfileEndpointInteraction endpointInteraction;
        ScalarRowEditGeometry rowGeometry;
        // Structural requests raised by the row's right-click menu; both
        // address the row's currently selected segment.
        bool splitRequested = false;
        double splitDistance = 0.0;
        bool removeRequested = false;
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
        // that channel's row; clickedSegmentId carries the segment under
        // the cursor, which becomes the row's selection.
        struct Click
        {
            quantum::editor::RateChannel channel;
            quantum::editor::ScalarProfileEndpoint endpoint;
            std::uint32_t segmentId;
        };

        struct TypeChange
        {
            quantum::editor::RateChannel channel;
            quantum::math::TransitionType type;
            std::uint32_t segmentId;
        };

        struct DistanceMove
        {
            quantum::editor::RateChannel channel;
            std::uint32_t segmentId;
            quantum::editor::ScalarProfileEndpoint endpoint;
            double distance;
        };

        struct SplitRequest
        {
            quantum::editor::RateChannel channel;
            std::uint32_t segmentId;
            double distance;
        };

        struct RemoveRequest
        {
            quantum::editor::RateChannel channel;
            std::uint32_t segmentId;
        };

        std::optional<quantum::editor::ScalarProfileEndpointValueEdit>
            endpointValueEdit;
        std::optional<TypeChange> transitionType;
        std::optional<Click> click;
        std::optional<DistanceMove> distanceMove;
        std::optional<SplitRequest> splitRequest;
        std::optional<RemoveRequest> removeRequest;
    };

    // Read-only view into one authored channel row of the Transition Editor.
    struct ProfileRowView
    {
        const char* label;
        const quantum::coaster::ChannelProfile* profile;
        quantum::editor::RateChannel channel;
    };

    struct TrackWorkspaceEdit
    {
        std::optional<std::size_t> selectRequest;
        bool removeRequested = false;
        bool moveUpRequested = false;
        bool moveDownRequested = false;
        bool duplicateRequested = false;
        bool lengthEdited = false;
        bool convertToGeometryRequested = false;
        bool convertToRateProfilesRequested = false;
        bool completeCircuitRequested = false;
        std::optional<quantum::coaster::LayoutMode> layoutModeChanged;
        // Present when the user picked an authoring type in the create
        // flow; the append/prepend direction lives in RegionCreateFlow.
        std::optional<quantum::coaster::RegionKind> createdRegionKind;
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

    // One interactive boundary marker of a drawn profile row. Interior
    // joints are owned by their left segment's End endpoint so every
    // handle maps to exactly one segment/endpoint pair.
    struct RowHandlePoint
    {
        ImVec2 position;
        double domainValue;
        double value;
        std::uint32_t segmentId;
        quantum::editor::ScalarProfileEndpoint endpoint;
        bool outer;
    };

    // Interactive clamp window for horizontally dragging one interior
    // boundary; mirrors the authoritative Core constraints so a live drag
    // never proposes a candidate the move operation would reject. Returns
    // false when the addressed boundary is pinned or unknown.
    [[nodiscard]] bool boundaryMoveBounds(
        const quantum::coaster::ChannelProfile& profile,
        const std::uint32_t segmentId,
        const quantum::editor::ScalarProfileEndpoint endpoint,
        double& lowerBound,
        double& upperBound) noexcept
    {
        using quantum::editor::ScalarProfileEndpoint;

        if (endpoint == ScalarProfileEndpoint::None)
        {
            return false;
        }

        for (std::size_t index = 0;
            index < profile.segments.size();
            ++index)
        {
            if (profile.segments[index].id != segmentId)
            {
                continue;
            }

            if (endpoint == ScalarProfileEndpoint::Begin)
            {
                if (index == 0)
                {
                    return false;
                }
                lowerBound = index >= 2
                    ? profile.segments[index - 2].transition.domainEnd
                    : profile.segments.front().transition.domainBegin;
                upperBound =
                    profile.segments[index].transition.domainEnd;
            }
            else
            {
                if (index + 1 >= profile.segments.size())
                {
                    return false;
                }
                lowerBound =
                    profile.segments[index].transition.domainBegin;
                upperBound = index + 2 < profile.segments.size()
                    ? profile.segments[index + 2].transition.domainBegin
                    : profile.segments.back().transition.domainEnd;
            }

            return true;
        }

        return false;
    }

    [[nodiscard]] ScalarProfileEndpointInteraction
    drawScalarProfileEndpoints(
        ImDrawList* const drawList,
        const ScalarProfileMapping& mapping,
        const quantum::coaster::ChannelProfile& profile,
        const float rowBeginY,
        const float rowEndY,
        const ImU32 curveColor,
        const char* const valueLabel,
        const std::uint32_t selectedSegmentId,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint)
    {
        using quantum::editor::ScalarProfileEndpoint;

        ScalarProfileEndpointInteraction interaction;
        const AuthoredDomainView& domainView = mapping.domainView;
        const ImVec2 mousePosition = ImGui::GetIO().MousePos;
        const float hoverRadiusSquared = endpointHoverRadius
            * endpointHoverRadius;
        const bool windowHovered = ImGui::IsWindowHovered();

        std::vector<RowHandlePoint> handles;
        handles.reserve(profile.segments.size() + 1);

        for (std::size_t index = 0;
            index < profile.segments.size();
            ++index)
        {
            const quantum::coaster::ProfileSegment& segment =
                profile.segments[index];

            if (index > 0)
            {
                // A shared joint was already added as the previous
                // segment's End handle; never duplicate it.
                const bool jointAlreadyAdded =
                    profile.segments[index - 1].transition.domainEnd
                        == segment.transition.domainBegin;
                if (jointAlreadyAdded)
                {
                    continue;
                }
            }

            handles.push_back(RowHandlePoint{
                mapping.toPixel(
                    segment.transition.domainBegin,
                    segment.transition.valueBegin
                ),
                segment.transition.domainBegin,
                segment.transition.valueBegin,
                segment.id,
                ScalarProfileEndpoint::Begin,
                index == 0
            });

            handles.push_back(RowHandlePoint{
                mapping.toPixel(
                    segment.transition.domainEnd,
                    segment.transition.valueEnd
                ),
                segment.transition.domainEnd,
                segment.transition.valueEnd,
                segment.id,
                ScalarProfileEndpoint::End,
                index + 1 == profile.segments.size()
            });
        }

        std::size_t hoveredIndex = handles.size();
        if (windowHovered)
        {
            float bestDistanceSquared = hoverRadiusSquared;
            for (std::size_t handleIndex = 0;
                handleIndex < handles.size();
                ++handleIndex)
            {
                const float distanceSquared = squaredDistance(
                    mousePosition,
                    handles[handleIndex].position
                );
                if (distanceSquared <= bestDistanceSquared)
                {
                    bestDistanceSquared = distanceSquared;
                    hoveredIndex = handleIndex;
                }
            }
        }

        for (std::size_t handleIndex = 0;
            handleIndex < handles.size();
            ++handleIndex)
        {
            const RowHandlePoint& handle = handles[handleIndex];
            const bool hovered = handleIndex == hoveredIndex;
            const bool ownedBySelected =
                handle.segmentId == selectedSegmentId;

            if (!handle.outer && !hovered)
            {
                // Idle interior joints stay visually quiet.
                const ImU32 dotColor = ownedBySelected
                    ? curveColor
                    : ImGui::GetColorU32(ImGuiCol_TextDisabled);
                drawList->AddCircleFilled(
                    handle.position,
                    ownedBySelected ? jointSelectedRadius
                        : jointHandleRadius,
                    dotColor
                );

                if (ownedBySelected
                    && selectedEndpoint == handle.endpoint)
                {
                    drawList->AddCircle(
                        handle.position,
                        jointSelectedRadius + 3.0F,
                        ImGui::GetColorU32(ImGuiCol_Text),
                        0,
                        1.5F
                    );
                }
                continue;
            }

            const float radius = hovered
                ? endpointHandleRadius + 1.5F
                : endpointHandleRadius;
            drawList->AddCircleFilled(handle.position, radius, curveColor);

            if (ownedBySelected && selectedEndpoint == handle.endpoint)
            {
                drawList->AddCircle(
                    handle.position,
                    endpointHandleRadius + 3.0F,
                    ImGui::GetColorU32(ImGuiCol_Text),
                    0,
                    1.5F
                );
            }
            else if (hovered)
            {
                drawList->AddCircle(
                    handle.position,
                    endpointHandleRadius + 2.5F,
                    curveColor,
                    0,
                    1.5F
                );
            }
        }

        if (hoveredIndex < handles.size())
        {
            const RowHandlePoint& handle = handles[hoveredIndex];
            ImGui::BeginTooltip();
            ImGui::Text("Distance: %.6g", handle.domainValue);
            ImGui::Text("%s: %.6f", valueLabel, handle.value);
            ImGui::Text("Segment: %u", handle.segmentId);
            ImGui::EndTooltip();
        }

        if (windowHovered
            && mousePosition.x >= domainView.pixelBegin
            && mousePosition.x <= domainView.pixelEnd
            && mousePosition.y >= rowBeginY
            && mousePosition.y <= rowEndY)
        {
            const double progress =
                static_cast<double>(
                    (mousePosition.x - domainView.pixelBegin)
                        / (domainView.pixelEnd - domainView.pixelBegin));
            interaction.hoverDomainDistance = domainView.domainBegin
                + progress * (domainView.domainEnd - domainView.domainBegin);
            interaction.hoverInsidePlot = true;
        }

        if (windowHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            if (hoveredIndex < handles.size())
            {
                interaction.clickedEndpoint =
                    handles[hoveredIndex].endpoint;
                interaction.clickedSegmentId =
                    handles[hoveredIndex].segmentId;
            }
            else
            {
                interaction.plotClicked =
                    mousePosition.x >= mapping.domainView.pixelBegin
                    && mousePosition.x <= mapping.domainView.pixelEnd
                    && mousePosition.y >= rowBeginY
                    && mousePosition.y <= rowEndY;
                interaction.plotClickedSegmentId =
                    interaction.plotClicked
                    ? quantum::coaster::findChannelSegmentAtDistance(
                        profile,
                        interaction.hoverDomainDistance)
                    : quantum::coaster::invalidSegmentId;
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

        drawList->AddText(canvasBegin, textColor, "REGION DISTANCE");

        char domainLabel[64]{};
        std::snprintf(
            domainLabel,
            sizeof(domainLabel),
            "%.0f -> %.0f (region-local)",
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
        const std::uint32_t selectedSegmentId,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint,
        double& contextMenuSplitDistance,
        const quantum::editor::TransitionEditorInputSettings& inputSettings)
    {
        using quantum::editor::ScalarProfileEndpoint;

        ScalarProfileRowEdit edit;
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

        const auto findSegmentById = [](
            const quantum::coaster::ChannelProfile& profile,
            const std::uint32_t segmentId)
                -> const quantum::coaster::ProfileSegment*
        {
            for (const quantum::coaster::ProfileSegment& segment :
                profile.segments)
            {
                if (segment.id == segmentId)
                {
                    return &segment;
                }
            }

            return nullptr;
        };

        // Pruning keeps selections valid, but the defensive tail fallback
        // guarantees the controls address a live segment even between a
        // structural commit and the next resolution pass.
        const quantum::coaster::ProfileSegment* focused =
            findSegmentById(*row.profile, selectedSegmentId);
        if (focused == nullptr && !row.profile->segments.empty())
        {
            focused = &row.profile->segments.back();
        }

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
                ImGui::SetTooltip(
                    "Focused segment end rate (radians per unit distance)"
                );
            }

            ImGui::SetCursorScreenPos(ImVec2(
                labelX + valueEndWidth
                    + ImGui::GetStyle().ItemSpacing.x,
                controlsY
            ));
            ImGui::SetNextItemWidth(transitionTypeWidth);
            edit.transitionType = drawTransitionTypeCombo(
                "##TransitionType",
                focused != nullptr
                    ? focused->transition.transitionType
                    : quantum::math::TransitionType::Linear
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

        // Straight sections would collapse the value axis to a zero span,
        // which makes endpoint drags emit no value change at all; clamp
        // the half-span so every row stays draggable. The fallback matches
        // the magnitude of the default authored roll/pitch/yaw rates.
        constexpr double minimumValueMagnitude = 0.02;
        double endpointMagnitude = minimumValueMagnitude;
        for (const quantum::coaster::ProfileSegment& segment :
            row.profile->segments)
        {
            endpointMagnitude = std::max(endpointMagnitude,
                std::abs(segment.transition.valueBegin));
            endpointMagnitude = std::max(endpointMagnitude,
                std::abs(segment.transition.valueEnd));
        }
        const ScalarProfileMapping mapping{
            .domainView = domainView,
            .valueMagnitude = endpointMagnitude,
            .pixelBeginY = plotBeginY,
            .pixelEndY = plotEndY
        };
        const float domainPixelWidth =
            domainView.pixelEnd - domainView.pixelBegin;
        const double distanceUnitsPerPixel = domainPixelWidth > 0.0F
            ? (domainView.domainEnd - domainView.domainBegin)
                / static_cast<double>(domainPixelWidth)
            : 0.0;

        // Sample every segment proportionally to its pixel width so the
        // concatenated polyline reads as one continuous authored curve.
        const std::size_t segmentTotal = row.profile->segments.size();
        const double domainWidth =
            domainView.domainEnd - domainView.domainBegin;
        std::vector<ImVec2> points;
        std::vector<std::size_t> spanFirsts;
        std::vector<std::size_t> spanCounts;
        points.reserve(profileSampleCount + segmentTotal);
        spanFirsts.reserve(segmentTotal);
        spanCounts.reserve(segmentTotal);
        std::size_t allocatedSamples = 0;

        for (std::size_t index = 0; index < segmentTotal; ++index)
        {
            const quantum::math::ScalarTransition& transition =
                row.profile->segments[index].transition;

            std::size_t sampleCount;
            if (index + 1 == segmentTotal)
            {
                sampleCount = std::max<std::size_t>(
                    2,
                    profileSampleCount - allocatedSamples
                );
            }
            else
            {
                const double fraction =
                    (transition.domainEnd - transition.domainBegin)
                        / domainWidth;
                sampleCount = std::max<std::size_t>(
                    2,
                    static_cast<std::size_t>(
                        std::llround(fraction * profileSampleCount))
                );
            }

            spanFirsts.push_back(points.size());
            for (std::size_t sample = 0;
                sample < sampleCount;
                ++sample)
            {
                const double progress = static_cast<double>(sample)
                    / static_cast<double>(sampleCount - 1);
                const double domainValue = transition.domainBegin
                    + progress
                        * (transition.domainEnd - transition.domainBegin);
                const double value = quantum::math::evaluateScalarTransition(
                    transition,
                    domainValue
                );
                points.push_back(mapping.toPixel(domainValue, value));
            }
            spanCounts.push_back(sampleCount);
            allocatedSamples += sampleCount;
        }

        drawList->AddPolyline(
            points.data(),
            static_cast<int>(points.size()),
            curveColor,
            ImDrawFlags_None,
            2.0F
        );

        // Emphasize the selected segment by restriking its own slice of
        // the shared polyline slightly thicker.
        for (std::size_t index = 0; index < segmentTotal; ++index)
        {
            if (row.profile->segments[index].id != selectedSegmentId)
            {
                continue;
            }

            drawList->AddPolyline(
                points.data() + spanFirsts[index],
                static_cast<int>(spanCounts[index]),
                curveColor,
                ImDrawFlags_None,
                3.5F
            );
            break;
        }

        // Interior boundaries stay visible but quiet: thin dim dividers
        // marking where two authored pieces meet.
        for (std::size_t index = 1; index < segmentTotal; ++index)
        {
            const quantum::math::ScalarTransition& previous =
                row.profile->segments[index - 1].transition;
            const quantum::math::ScalarTransition& current =
                row.profile->segments[index].transition;

            if (previous.domainEnd != current.domainBegin)
            {
                continue;
            }

            const float jointX = domainView.toPixel(current.domainBegin);
            drawList->AddLine(
                ImVec2(jointX, plotBeginY),
                ImVec2(jointX, plotEndY),
                ImGui::GetColorU32(ImGuiCol_TextDisabled),
                1.0F
            );
        }

        edit.endpointInteraction = drawScalarProfileEndpoints(
            drawList,
            mapping,
            *row.profile,
            rowBeginY,
            rowEndY,
            curveColor,
            row.label,
            selectedSegmentId,
            selectedEndpoint
        );
        edit.rowGeometry = {
            mapping.valueMagnitude,
            mapping.valueUnitsPerPixel(),
            distanceUnitsPerPixel
        };

        // Per-row right-click menu for the structural segment operations.
        ImGui::PushID(row.label);
        if (edit.endpointInteraction.hoverInsidePlot
            && ImGui::IsMouseReleased(ImGuiMouseButton_Right))
        {
            contextMenuSplitDistance =
                edit.endpointInteraction.hoverDomainDistance;
            ImGui::OpenPopup("##SegmentMenu");
        }

        if (ImGui::BeginPopup("##SegmentMenu"))
        {
            double targetDistance = contextMenuSplitDistance;
            bool splitAllowed = false;
            if (inputSettings.distanceSnapEnabled
                && inputSettings.distanceSnapIncrement > 0.0)
            {
                targetDistance = snapToIncrement(
                    targetDistance,
                    inputSettings.distanceSnapIncrement
                );
            }
            if (focused != nullptr)
            {
                splitAllowed =
                    focused->transition.domainBegin < targetDistance
                    && targetDistance < focused->transition.domainEnd;
            }

            if (focused != nullptr)
            {
                const quantum::editor::TransitionTypePreset* const preset =
                    quantum::editor::findTransitionTypePreset(
                        focused->transition.transitionType);
                ImGui::Text(
                    "Segment %u  [%.6g, %.6g]",
                    focused->id,
                    focused->transition.domainBegin,
                    focused->transition.domainEnd
                );
                ImGui::TextDisabled(
                    "%s",
                    preset != nullptr
                        ? preset->displayName.data()
                        : "Unsupported"
                );
            }
            ImGui::Separator();

            char splitLabel[48]{};
            std::snprintf(
                splitLabel,
                sizeof(splitLabel),
                "Split here (%.6g)",
                targetDistance
            );
            if (ImGui::Selectable(splitLabel, false, splitAllowed
                    ? ImGuiSelectableFlags_None
                    : ImGuiSelectableFlags_Disabled))
            {
                edit.splitRequested = true;
                edit.splitDistance = targetDistance;
            }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            {
                ImGui::SetTooltip(
                    "Inserts a boundary in the highlighted segment."
                );
            }

            if (ImGui::Selectable("Remove Segment", false,
                row.profile->segments.size() > 1 && focused != nullptr
                    ? ImGuiSelectableFlags_None
                    : ImGuiSelectableFlags_Disabled))
            {
                edit.removeRequested = true;
            }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
            {
                ImGui::SetTooltip(
                    "Merges the highlighted segment into its neighbour."
                );
            }

            ImGui::EndPopup();
        }
        ImGui::PopID();

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
            "snapping=%s increment=%.4f distSnapping=%s distIncrement=%.2f",
            settings.normalDragGain,
            settings.fineDragGain,
            settings.snapEnabled ? "enabled" : "disabled",
            settings.snapIncrement,
            settings.distanceSnapEnabled ? "enabled" : "disabled",
            settings.distanceSnapIncrement
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

    void logViewportSettings(
        const quantum::editor::ViewportSettings& settings)
    {
        // One-line audit per committed change so tooling can track the
        // live viewport configuration without screen capture.
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[CFG] viewport settings orthographic=%d fov=%.1f "
            "orbit=%.2f zoom=%.2f grid=%d centerline=%d leftRail=%d "
            "rightRail=%d heartline=%d",
            settings.orthographic ? 1 : 0,
            static_cast<double>(settings.fieldOfViewDegrees),
            static_cast<double>(settings.orbitSensitivity),
            static_cast<double>(settings.zoomSensitivity),
            settings.gridVisible ? 1 : 0,
            settings.centerlineVisible ? 1 : 0,
            settings.leftRailVisible ? 1 : 0,
            settings.rightRailVisible ? 1 : 0,
            settings.heartlineVisible ? 1 : 0
        );
    }

    void showViewportSettingsWindow(
        quantum::editor::ViewportSettings& settings,
        bool* const open)
    {
        // ImGui's p_open parameter is only written by the title bar close
        // button; it does not hide the window. The flag must gate whether
        // Begin is called at all.
        if (!*open)
        {
            return;
        }

        // Appearing keeps a sensible first-session position while letting
        // a user-moved (ini-persisted) position win afterwards.
        ImGui::SetNextWindowPos(ImVec2(640.0F, 110.0F), ImGuiCond_Appearing);
        if (!ImGui::Begin("Viewport Settings", open))
        {
            ImGui::End();
            return;
        }

        bool committed = false;

        const char* const projectionNames[] = {
            "Perspective", "Orthographic"
        };
        int projectionIndex = settings.orthographic ? 1 : 0;
        if (ImGui::Combo(
            "Projection",
            &projectionIndex,
            projectionNames,
            2))
        {
            settings.orthographic = projectionIndex == 1;
            committed = true;
        }

        if (ImGui::SliderFloat(
            "Field of View",
            &settings.fieldOfViewDegrees,
            10.0F,
            120.0F,
            "%.0f deg",
            ImGuiSliderFlags_AlwaysClamp))
        {
            committed = true;
        }

        if (ImGui::SliderFloat(
            "Orbit Sensitivity",
            &settings.orbitSensitivity,
            0.05F,
            4.0F,
            "%.2fx",
            ImGuiSliderFlags_AlwaysClamp))
        {
            committed = true;
        }

        if (ImGui::SliderFloat(
            "Zoom Sensitivity",
            &settings.zoomSensitivity,
            0.05F,
            4.0F,
            "%.2fx",
            ImGuiSliderFlags_AlwaysClamp))
        {
            committed = true;
        }

        ImGui::SeparatorText("Reference Elements");

        committed = ImGui::Checkbox(
            "Ground Grid", &settings.gridVisible
        ) || committed;
        committed = ImGui::Checkbox(
            "Centerline", &settings.centerlineVisible
        ) || committed;
        committed = ImGui::Checkbox(
            "Left Rail", &settings.leftRailVisible
        ) || committed;
        committed = ImGui::Checkbox(
            "Right Rail", &settings.rightRailVisible
        ) || committed;
        committed = ImGui::Checkbox(
            "Heartline", &settings.heartlineVisible
        ) || committed;

        if (committed)
        {
            logViewportSettings(settings);
        }

        ImGui::End();
    }

    float showCommandArea(
        ImGuiViewport* const mainViewport,
        std::optional<quantum::editor::FileOperationType>&
            pendingFileOperation)
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

            if (ImGui::Button("New"))
            {
                pendingFileOperation =
                    quantum::editor::FileOperationType::New;
            }
            ImGui::SameLine();
            if (ImGui::Button("Open"))
            {
                pendingFileOperation =
                    quantum::editor::FileOperationType::Open;
            }
            ImGui::SameLine();
            if (ImGui::Button("Save"))
            {
                pendingFileOperation =
                    quantum::editor::FileOperationType::Save;
            }
            ImGui::SameLine();
            ImGui::BeginDisabled();
            ImGui::Button("Undo");
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled();
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
        double* const sectionLengthEdit,
        quantum::editor::RegionCreateFlow& regionCreateFlow,
        const std::optional<double>& selectedHeightDelta,
        const quantum::coaster::TrackTopology& topology)
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

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Layout Mode selector
            ImGui::TextUnformatted("LAYOUT MODE");
            const quantum::coaster::LayoutMode currentMode =
                track.layoutMode();
            const bool isCircuit =
                currentMode == quantum::coaster::LayoutMode::Circuit;

            const float layoutModeHalfWidth =
                (ImGui::GetContentRegionAvail().x
                - ImGui::GetStyle().ItemSpacing.x) * 0.5F;

            if (ImGui::Button(
                isCircuit ? "[ Circuit ]" : "Circuit",
                ImVec2(layoutModeHalfWidth, 0.0F)))
            {
                edit.layoutModeChanged =
                    quantum::coaster::LayoutMode::Circuit;
            }
            ImGui::SameLine();
            if (ImGui::Button(
                !isCircuit ? "[ Shuttle ]" : "Shuttle",
                ImVec2(-1.0F, 0.0F)))
            {
                edit.layoutModeChanged =
                    quantum::coaster::LayoutMode::Shuttle;
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Track Topology summary
            const quantum::coaster::LayoutStatus status =
                quantum::coaster::computeLayoutStatus(
                    currentMode, topology.kind);

            ImGui::TextUnformatted("TRACK TOPOLOGY");
            ImGui::Text(
                "%s",
                quantum::coaster::layoutStatusLabel(status));

            if (topology.kind
                == quantum::coaster::TopologyKind::OpenLinear)
            {
                ImGui::Text(
                    "Closure gap: %.1f m",
                    topology.diagnostics.positionalGap);
                ImGui::Text(
                    "Tangent mismatch: %.1f deg",
                    topology.diagnostics.tangentMismatchDegrees);
                ImGui::Text(
                    "Frame mismatch: %.1f deg",
                    topology.diagnostics.frameMismatchDegrees);
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            const std::size_t sectionCount = track.sectionCount();
            for (std::size_t index = 0; index < sectionCount; ++index)
            {
                // Region terminology is the user-facing prototype wording;
                // the internal Section naming is unchanged.
                const char* kindName =
                    track.section(index).kind
                            == quantum::coaster::RegionKind::Geometry
                        ? "Geometry / Planar Arc"
                        : "Rate/Profile";
                char label[80]{};
                std::snprintf(
                    label,
                    sizeof(label),
                    "Region %llu - %s (%.3g)",
                    static_cast<unsigned long long>(index + 1),
                    kindName,
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
                ImGui::TextUnformatted("Selected Region");

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
                        "region"
                    );
                }

                const quantum::coaster::AuthoredTrackSection& selected =
                    track.section(selectedIndex);

                const quantum::editor::RegionStations stations =
                    quantum::editor::computeRegionStations(
                        track,
                        selectedIndex
                    );
                ImGui::Spacing();
                ImGui::TextUnformatted("Track Position");
                ImGui::Text(
                    "Stations %.3f -> %.3f",
                    stations.startStation,
                    stations.endStation
                );
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip(
                        "Cumulative distance along the whole track at this "
                        "region's start and end"
                    );
                }

                ImGui::Text(
                    "Track total length %.3f",
                    stations.totalLength
                );

                if (selectedHeightDelta.has_value())
                {
                    ImGui::Text(
                        "Entry-to-exit height change %+.3f",
                        *selectedHeightDelta
                    );
                }

                if (selected.kind == quantum::coaster::RegionKind::RateProfiles)
                {
                    const quantum::editor::RegionNetRotation netRotation =
                        quantum::editor::computeNetRotationDegrees(
                            selected.rateProfileRegion().rateProfiles
                        );
                    ImGui::Spacing();
                    ImGui::TextUnformatted("Net Rotation (integrated)");
                    ImGui::Text("Roll %+.2f deg", netRotation.rollDegrees);
                    ImGui::Text("Pitch %+.2f deg", netRotation.pitchDegrees);
                    ImGui::Text("Yaw %+.2f deg", netRotation.yawDegrees);
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip(
                            "Total angle accumulated by each rate channel "
                            "across this region"
                        );
                    }
                }

                // Kind conversion remains a secondary operation; geometry
                // authoring is discovered through the typed create flow and
                // edited in the dedicated Geometry Editor.
                const bool isGeometry =
                    selected.kind == quantum::coaster::RegionKind::Geometry;
                if (ImGui::Button(
                    isGeometry ? "Convert to Rate Profiles"
                               : "Convert to Planar Arc",
                    ImVec2(-1.0F, 0.0F)))
                {
                    if (isGeometry)
                    {
                        edit.convertToRateProfilesRequested = true;
                    }
                    else
                    {
                        edit.convertToGeometryRequested = true;
                    }
                }
            }

            const ImGuiStyle& style = ImGui::GetStyle();
            const float buttonAreaHeight = 5.0F * ImGui::GetFrameHeight()
                + 4.0F * style.ItemSpacing.y;
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

            // Typed region creation: the first click swaps this row's
            // content to the authoring-type choice. The row keeps its
            // height in both states so nothing below it shifts while a
            // choice is pending. The strip is shared by all three
            // creation anchors; the clicked trigger button carries the
            // direction context.
            if (!regionCreateFlow.choicePending)
            {
                if (ImGui::Button(
                    "Append Region...",
                    ImVec2(halfWidth, 0.0F)))
                {
                    regionCreateFlow.choicePending = true;
                    regionCreateFlow.anchor =
                        quantum::editor::RegionCreateAnchor::Append;
                }

                ImGui::SameLine();
                if (ImGui::Button(
                    "Prepend Region...",
                    ImVec2(-1.0F, 0.0F)))
                {
                    regionCreateFlow.choicePending = true;
                    regionCreateFlow.anchor =
                        quantum::editor::RegionCreateAnchor::Prepend;
                }
            }
            else
            {
                const float thirdWidth = (ImGui::GetContentRegionAvail().x
                    - 2.0F * style.ItemSpacing.x) / 3.0F;
                if (ImGui::Button(
                    "Rate/Profile",
                    ImVec2(thirdWidth, 0.0F)))
                {
                    edit.createdRegionKind =
                        quantum::coaster::RegionKind::RateProfiles;
                }

                ImGui::SameLine();
                if (ImGui::Button(
                    "Geometry / Planar Arc",
                    ImVec2(thirdWidth, 0.0F)))
                {
                    edit.createdRegionKind =
                        quantum::coaster::RegionKind::Geometry;
                }

                ImGui::SameLine();
                if (ImGui::Button(
                    "Cancel",
                    ImVec2(-1.0F, 0.0F)))
                {
                    regionCreateFlow.choicePending = false;
                }
            }

            ImGui::BeginDisabled(!hasSelection);
            if (ImGui::Button(
                "Insert After Selected...",
                ImVec2(-1.0F, 0.0F)))
            {
                regionCreateFlow.choicePending = true;
                regionCreateFlow.anchor =
                    quantum::editor::RegionCreateAnchor::AfterSelected;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip(
                    "Creates a new region immediately after the selected "
                    "region"
                );
            }

            ImGui::BeginDisabled(!hasSelection);
            if (ImGui::Button(
                "Duplicate Selected",
                ImVec2(halfWidth, 0.0F)))
            {
                edit.duplicateRequested = true;
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::BeginDisabled(sectionCount <= 1);
            if (ImGui::Button("Remove Selected", ImVec2(-1.0F, 0.0F)))
            {
                edit.removeRequested = true;
            }
            ImGui::EndDisabled();

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

            if (topology.kind
                    == quantum::coaster::TopologyKind::OpenLinear
                && track.layoutMode()
                    == quantum::coaster::LayoutMode::Circuit)
            {
                if (ImGui::Button(
                    "Complete Circuit...",
                    ImVec2(-1.0F, 0.0F)))
                {
                    edit.completeCircuitRequested = true;
                }
            }
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
        // ImGui's p_open parameter is only written by the title bar close
        // button; it does not hide the window. The flag must gate whether
        // Begin is called at all.
        if (!*open)
        {
            return;
        }

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

        if (ImGui::Checkbox(
            "Enable Distance Snapping",
            &settings.distanceSnapEnabled))
        {
            committed = true;
        }

        if (ImGui::InputDouble(
            "Distance Snap Increment",
            &settings.distanceSnapIncrement,
            1.0,
            5.0,
            "%.2f"))
        {
            const double requested = settings.distanceSnapIncrement;
            settings.distanceSnapIncrement =
                std::isfinite(requested) && requested > 0.0
                    ? std::min(requested, 50.0)
                    : 5.0;
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
        const double sectionLength,
        std::array<double, quantum::editor::rateChannelCount>&
            valueEndBuffers,
        const std::array<quantum::editor::ScalarProfileEndpoint,
            quantum::editor::rateChannelCount>& endpointSelections,
        const std::array<quantum::editor::ScalarProfileEndpoint,
            quantum::editor::rateChannelCount>& endpointDrags,
        const std::array<std::uint32_t,
            quantum::editor::rateChannelCount>& selectedSegmentIds,
        std::array<std::uint32_t,
            quantum::editor::rateChannelCount>& dragSegmentIds,
        std::array<quantum::editor::DragAxisLock,
            quantum::editor::rateChannelCount>& dragAxisLocks,
        std::array<double, quantum::editor::rateChannelCount>&
            dragAxisTravelX,
        std::array<double, quantum::editor::rateChannelCount>&
            dragAxisTravelY,
        std::array<double, quantum::editor::rateChannelCount>&
            contextMenuSplitDistances,
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
                // The committed document guarantees every channel covers
                // [0, sectionLength]; the guard keeps a corrupted row from
                // drawing outside its authored domain.
                for (const ProfileRowView& row : profileRows)
                {
                    if (row.profile == nullptr
                        || row.profile->segments.empty()
                        || row.profile->segments.back().transition.domainEnd
                            != sectionLength)
                    {
                        throw std::logic_error(
                            "Transition Editor profiles must cover exactly "
                            "the shared authored domain."
                        );
                    }
                }

                const AuthoredDomainView domainView{
                    .domainBegin = 0.0,
                    .domainEnd = sectionLength,
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

                // Designer-readable summary of what the three authored rate
                // channels accumulate to over this region, drawn in the
                // free right side of the ruler band.
                if (std::all_of(
                    profileRows.begin(),
                    profileRows.end(),
                    [](const ProfileRowView& row)
                    {
                        return row.profile != nullptr;
                    }))
                {
                    const double netRollDegrees =
                        quantum::editor::computeChannelNetRotationDegrees(
                            *profileRows[0].profile
                        );
                    const double netPitchDegrees =
                        quantum::editor::computeChannelNetRotationDegrees(
                            *profileRows[1].profile
                        );
                    const double netYawDegrees =
                        quantum::editor::computeChannelNetRotationDegrees(
                            *profileRows[2].profile
                        );

                    char netRotationLabel[128]{};
                    std::snprintf(
                        netRotationLabel,
                        sizeof(netRotationLabel),
                        "NET ROTATION  Roll %+.1f  Pitch %+.1f  "
                        "Yaw %+.1f deg",
                        netRollDegrees,
                        netPitchDegrees,
                        netYawDegrees
                    );
                    const ImVec2 netLabelSize = ImGui::CalcTextSize(
                        netRotationLabel
                    );
                    drawList->AddText(
                        ImVec2(
                            std::max(
                                plotBeginX + 8.0F,
                                canvasEnd.x - netLabelSize.x - 10.0F
                            ),
                            canvasBegin.y + 4.0F
                        ),
                        ImGui::GetColorU32(ImGuiCol_Text),
                        netRotationLabel
                    );
                }


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
                        selectedSegmentIds[channelIndex],
                        endpointSelections[channelIndex],
                        contextMenuSplitDistances[channelIndex],
                        inputSettings
                    );
                    rowGeometries[channelIndex] = rowEdit.rowGeometry;

                    if (rowEdit.valueEndEdited)
                    {
                        edit.endpointValueEdit = {
                            .endpoint = quantum::editor::
                                ScalarProfileEndpoint::End,
                            .value = valueEndBuffers[channelIndex],
                            .continuous = false,
                            .channel = row.channel,
                            .segmentId = selectedSegmentIds[channelIndex]
                        };
                    }

                    if (rowEdit.endpointInteraction.clickedEndpoint
                        != quantum::editor::ScalarProfileEndpoint::None)
                    {
                        edit.click = TransitionEditorEdit::Click{
                            row.channel,
                            rowEdit.endpointInteraction.clickedEndpoint,
                            rowEdit.endpointInteraction.clickedSegmentId
                        };
                    }
                    else if (rowEdit.endpointInteraction.plotClicked)
                    {
                        edit.click = TransitionEditorEdit::Click{
                            row.channel,
                            quantum::editor::ScalarProfileEndpoint::None,
                            rowEdit.endpointInteraction.plotClickedSegmentId
                        };
                    }

                    if (rowEdit.transitionType.has_value())
                    {
                        edit.transitionType = TransitionEditorEdit::
                            TypeChange{
                                row.channel,
                                *rowEdit.transitionType,
                                selectedSegmentIds[channelIndex]
                            };
                    }

                    if (rowEdit.splitRequested)
                    {
                        edit.splitRequest = TransitionEditorEdit::
                            SplitRequest{
                                row.channel,
                                selectedSegmentIds[channelIndex],
                                rowEdit.splitDistance
                            };
                    }

                    if (rowEdit.removeRequested)
                    {
                        edit.removeRequest = TransitionEditorEdit::
                            RemoveRequest{
                                row.channel,
                                selectedSegmentIds[channelIndex]
                            };
                    }
                }

                // Active handle drags emit one edit per frame by
                // integrating cursor deltas from a rolling anchor. The
                // axis lock engages after unambiguous motion: horizontal
                // drags move interior boundaries in distance, vertical
                // drags edit values exactly as before. Shift stays the
                // live precision mode on both axes.
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

                    quantum::editor::ScalarDragAnchor& anchor =
                        *dragAnchors[channelIndex];
                    const double currentPixelX = static_cast<double>(
                        dragIo.MousePos.x);
                    const double currentPixelY = static_cast<double>(
                        dragIo.MousePos.y);

                    // Coordinate-space switches are absorbed per axis by
                    // re-basing the pixel reference without integrating.
                    double deltaX = currentPixelX - anchor.pixelX;
                    if (std::abs(deltaX) > maxPlausiblePixelDeltaPerFrame)
                    {
                        deltaX = 0.0;
                    }
                    anchor.pixelX = currentPixelX;

                    double deltaY = currentPixelY - anchor.pixelY;
                    if (std::abs(deltaY) > maxPlausiblePixelDeltaPerFrame)
                    {
                        deltaY = 0.0;
                    }
                    anchor.pixelY = currentPixelY;

                    quantum::editor::DragAxisLock& axisLock =
                        dragAxisLocks[channelIndex];
                    if (axisLock
                        == quantum::editor::DragAxisLock::None)
                    {
                        dragAxisTravelX[channelIndex] += std::abs(deltaX);
                        dragAxisTravelY[channelIndex] += std::abs(deltaY);
                        if (std::max(
                                dragAxisTravelX[channelIndex],
                                dragAxisTravelY[channelIndex])
                            > dragAxisLockPixelThreshold)
                        {
                            const bool horizontalIntent =
                                dragAxisTravelX[channelIndex]
                                    > dragAxisTravelY[channelIndex];
                            const ProfileRowView& draggedRow =
                                profileRows[channelIndex];
                            double lowerBound = 0.0;
                            double upperBound = 0.0;
                            const bool boundaryMovable = horizontalIntent
                                && draggedRow.profile != nullptr
                                && boundaryMoveBounds(
                                    *draggedRow.profile,
                                    dragSegmentIds[channelIndex],
                                    endpointDrags[channelIndex],
                                    lowerBound,
                                    upperBound
                                );
                            axisLock = boundaryMovable
                                ? quantum::editor::DragAxisLock::Horizontal
                                : quantum::editor::DragAxisLock::Vertical;
                        }
                    }

                    if (axisLock
                        == quantum::editor::DragAxisLock::Horizontal)
                    {
                        // Screen X and authored distance grow together.
                        double distance = anchor.distance
                            + deltaX
                                * geometry.distanceUnitsPerPixel
                                * static_cast<double>(gainMultiplier);

                        const ProfileRowView& draggedRow =
                            profileRows[channelIndex];
                        double lowerBound = 0.0;
                        double upperBound = 0.0;
                        if (draggedRow.profile != nullptr
                            && boundaryMoveBounds(
                                *draggedRow.profile,
                                dragSegmentIds[channelIndex],
                                endpointDrags[channelIndex],
                                lowerBound,
                                upperBound
                            ))
                        {
                            // Keep the proposal strictly interior so the
                            // Core move never rejects mid-drag.
                            const double margin = std::max(
                                (upperBound - lowerBound) * 1e-9,
                                1e-9
                            );
                            distance = std::clamp(
                                distance,
                                lowerBound + margin,
                                upperBound - margin
                            );

                            if (inputSettings.distanceSnapEnabled
                                && inputSettings.distanceSnapIncrement
                                    > 0.0)
                            {
                                distance = snapToIncrement(
                                    distance,
                                    inputSettings.distanceSnapIncrement
                                );
                                distance = std::clamp(
                                    distance,
                                    lowerBound + margin,
                                    upperBound - margin
                                );
                            }

                            if (distance != anchor.distance)
                            {
                                edit.distanceMove =
                                    TransitionEditorEdit::DistanceMove{
                                        static_cast<quantum::editor::
                                            RateChannel>(channelIndex),
                                        dragSegmentIds[channelIndex],
                                        endpointDrags[channelIndex],
                                        distance
                                    };
                                anchor.distance = distance;
                            }
                        }
                    }
                    else if (axisLock
                        == quantum::editor::DragAxisLock::Vertical)
                    {
                        // Screen Y grows downward while authored values grow
                        // upward, hence the subtraction.
                        double value = anchor.value
                            - deltaY
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

                        anchor.value = value;

                        edit.endpointValueEdit = {
                            .endpoint = endpointDrags[channelIndex],
                            .value = value,
                            .continuous = true,
                            .channel = static_cast<quantum::editor::
                                RateChannel>(channelIndex),
                            .segmentId = dragSegmentIds[channelIndex]
                        };
                    }
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
            const auto& profile =
                sectionRateChannel(authoredTrack.section(0), channel);
            valueEndEditBuffers_[channelIndex] =
                profile.segments.empty()
                    ? 0.0
                    : profile.segments.back().transition.valueEnd;
            endpointSelections_[channelIndex] =
                ScalarProfileEndpoint::None;
            endpointDrags_[channelIndex] = ScalarProfileEndpoint::None;
            selectedSegmentIds_[channelIndex] =
                coaster::invalidSegmentId;
            dragSegmentIds_[channelIndex] = coaster::invalidSegmentId;
            dragAxisLocks_[channelIndex] =
                DragAxisLock::None;
            dragAxisTravelX_[channelIndex] = 0.0;
            dragAxisTravelY_[channelIndex] = 0.0;
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
        viewportAspectRatio_ = aspectRatio;

        if (initialViewportFramePending_)
        {
            viewportCamera_.frame(aspectRatio);
            initialViewportFramePending_ = false;
        }

        applyViewportSettings(vulkan);

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
                        * orbitRadiansPerPixel
                        * viewportSettings_.orbitSensitivity,
                    -static_cast<double>(io.MouseDelta.y)
                        * orbitRadiansPerPixel
                        * viewportSettings_.orbitSensitivity
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
                viewportCamera_.zoom(
                    static_cast<double>(io.MouseWheel),
                    defaultViewportZoomExponentPerWheelUnit
                        * viewportSettings_.zoomSensitivity
                );
            }

            if (ImGui::IsKeyPressed(ImGuiKey_F, false))
            {
                frameWholeTrack();
            }
        }

        vulkan.setViewportViewProjection(
            viewportCamera_.viewProjection(aspectRatio)
        );
    }

    float EditorUi::showMainMenuBar()
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
            if (ImGui::MenuItem("New"))
            {
                pendingFileOperation_ = FileOperationType::New;
            }
            if (ImGui::MenuItem("Open"))
            {
                pendingFileOperation_ = FileOperationType::Open;
            }
            if (ImGui::MenuItem("Save"))
            {
                pendingFileOperation_ = FileOperationType::Save;
            }
            if (ImGui::MenuItem("Save As"))
            {
                pendingFileOperation_ = FileOperationType::SaveAs;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit"))
        {
            ImGui::MenuItem("Undo", nullptr, false, false);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            const auto presetItem = [this](
                const char* const label,
                const ViewportCameraPreset preset)
            {
                if (ImGui::MenuItem(label))
                {
                    applyViewportPreset(preset);
                }
            };

            presetItem("Perspective", ViewportCameraPreset::Perspective);
            presetItem("Isometric", ViewportCameraPreset::Isometric);
            ImGui::Separator();
            presetItem("Top", ViewportCameraPreset::Top);
            presetItem("Bottom", ViewportCameraPreset::Bottom);
            presetItem("Left", ViewportCameraPreset::Left);
            presetItem("Right", ViewportCameraPreset::Right);
            ImGui::Separator();
            if (ImGui::MenuItem(
                "Track View",
                nullptr,
                false,
                selectedSectionSlice() != nullptr
            ))
            {
                applyTrackViewPreset(false);
            }
            if (ImGui::MenuItem(
                "Walking View",
                nullptr,
                false,
                selectedSectionSlice() != nullptr
            ))
            {
                applyTrackViewPreset(true);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(
                "Focus Selected",
                nullptr,
                false,
                selectedSectionSlice() != nullptr
            ))
            {
                focusSelectedSection();
            }
            if (ImGui::MenuItem("Frame All"))
            {
                frameWholeTrack();
            }
            ImGui::Separator();
            if (ImGui::MenuItem(
                "Viewport Settings",
                nullptr,
                &viewportSettingsWindowOpen_
            ))
            {
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[CFG] viewport settings window %s",
                    viewportSettingsWindowOpen_ ? "open" : "closed"
                );
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Preferences"))
        {
            if (ImGui::MenuItem(
                "Transition Editor Input",
                nullptr,
                &inputSettingsWindowOpen_
            ))
            {
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[CFG] transition editor input window %s",
                    inputSettingsWindowOpen_ ? "open" : "closed"
                );
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
        ImGui::PopStyleColor(topRegionColorCount);
        return height;
    }

    void EditorUi::applyViewportPreset(const ViewportCameraPreset preset)
    {
        static constexpr std::array<const char*, 6> presetNames{
            "perspective", "isometric", "top", "bottom", "left", "right"
        };

        viewportCamera_.applyPreset(preset);
        initialViewportFramePending_ = false;
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[INP] VIEW_PRESET %s",
            presetNames[static_cast<std::size_t>(preset)]
        );
    }

    void EditorUi::applyTrackViewPreset(const bool walkingView)
    {
        const CenterlineSectionSlice* const slice = selectedSectionSlice();

        if (slice == nullptr)
        {
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[INP] VIEW_TRACK rejected reason=no-valid-section"
            );
            return;
        }

        // Ride-style views look along the section's authored start tangent.
        // The tangent is horizontal-only here so the camera never pitches
        // with vertical track elements.
        glm::dvec3 forward{slice->startTangent.x, slice->startTangent.y, 0.0};
        double forwardLength = glm::length(forward);

        if (forwardLength < 1.0e-9)
        {
            forward = glm::dvec3{1.0, 0.0, 0.0};
            forwardLength = 1.0;
        }

        forward /= forwardLength;

        if (!walkingView)
        {
            // Roller-coaster track view: a low chase position behind the
            // section start looking down the tangent.
            const ViewportCameraPose pose{
                .focus = slice->startPosition + forward * 20.0,
                .yaw = std::atan2(-forward.y, -forward.x),
                .pitch = 8.0 * piRadians / 180.0,
                .distance = 22.0
            };
            viewportCamera_.setPose(pose);
        }
        else
        {
            // Walking view: eye height over the section start on the ground
            // plane, looking ahead along the tangent.
            const glm::dvec3 eye{
                slice->startPosition.x, slice->startPosition.y, 1.7
            };
            const ViewportCameraPose pose{
                .focus = eye + forward * 6.0,
                .yaw = std::atan2(-forward.y, -forward.x),
                .pitch = 0.0,
                .distance = 6.0
            };
            viewportCamera_.setPose(pose);
        }

        initialViewportFramePending_ = false;
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[INP] VIEW_%s applied section=%zu",
            walkingView ? "WALKING" : "TRACK",
            selectedSection_
        );
    }

    void EditorUi::focusSelectedSection()
    {
        const CenterlineSectionSlice* const slice = selectedSectionSlice();

        if (slice == nullptr)
        {
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[INP] VIEW_FOCUS rejected reason=no-valid-section"
            );
            return;
        }

        const glm::dvec3 center =
            (slice->minimumPosition + slice->maximumPosition) / 2.0;
        double radius = 0.5 * glm::length(
            slice->maximumPosition - slice->minimumPosition
        );

        // Degenerate (straight short) slices still need a sensible framing
        // sphere so the camera never lands inside the geometry.
        radius = std::max(radius, 1.0e-3);

        if (!viewportCamera_.frameSphere(
            center,
            radius,
            viewportAspectRatio_))
        {
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[INP] VIEW_FOCUS rejected reason=invalid-geometry"
            );
            return;
        }

        initialViewportFramePending_ = false;
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[INP] VIEW_FOCUS applied section=%zu",
            selectedSection_
        );
    }

    void EditorUi::frameWholeTrack()
    {
        viewportCamera_.frame(viewportAspectRatio_);
        initialViewportFramePending_ = false;
        SDL_LogInfo(
            SDL_LOG_CATEGORY_APPLICATION,
            "[INP] VIEW_FRAME_ALL"
        );
    }

    const CenterlineSectionSlice*
    EditorUi::selectedSectionSlice() const noexcept
    {
        if (centerlineSlices_.empty()
            || selectedSection_ >= centerlineSlices_.size())
        {
            return nullptr;
        }

        const CenterlineSectionSlice& slice =
            centerlineSlices_[selectedSection_];

        return slice.vertexCount >= 2 ? &slice : nullptr;
    }

    void EditorUi::applyViewportSettings(renderer::VulkanContext& vulkan)
    {
        viewportCamera_.setProjection(
            viewportSettings_.orthographic
                ? ViewportProjection::Orthographic
                : ViewportProjection::Perspective
        );
        viewportCamera_.setVerticalFieldOfView(
            static_cast<double>(viewportSettings_.fieldOfViewDegrees)
                * piRadians / 180.0
        );

        std::uint32_t curveMask = 0;

        if (viewportSettings_.leftRailVisible)
        {
            curveMask |= 1u << renderer::viewportLeftRailCurve;
        }
        if (viewportSettings_.rightRailVisible)
        {
            curveMask |= 1u << renderer::viewportRightRailCurve;
        }
        if (viewportSettings_.centerlineVisible)
        {
            curveMask |= 1u << renderer::viewportCenterlineCurve;
        }
        if (viewportSettings_.heartlineVisible)
        {
            curveMask |= 1u << renderer::viewportHeartlineCurve;
        }

        vulkan.setViewportElementVisibility(
            viewportSettings_.gridVisible,
            curveMask
        );

        // Recentre the ground grid around the solved track's bounds so it
        // stays a stable modeling reference at any pan position.
        if (viewportCamera_.hasBounds())
        {
            vulkan.updateViewportAidReference(
                static_cast<float>(viewportCamera_.boundsCenter().x),
                static_cast<float>(viewportCamera_.boundsCenter().y),
                static_cast<float>(viewportCamera_.boundsRadius())
            );
        }
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

        const float menuBarHeight = showMainMenuBar();

        ImGuiViewport* const mainViewport = ImGui::GetMainViewport();
        const float commandAreaHeight = showCommandArea(
            mainViewport, pendingFileOperation_);

        showTransitionEditorInputSettings(
            transitionEditorInputSettings_,
            &inputSettingsWindowOpen_
        );

        showViewportSettingsWindow(
            viewportSettings_,
            &viewportSettingsWindowOpen_
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
            const auto presetButton = [this](
                const char* const label,
                const ViewportCameraPreset preset)
            {
                if (ImGui::SmallButton(label))
                {
                    applyViewportPreset(preset);
                }
            };

            presetButton("Persp", ViewportCameraPreset::Perspective);
            ImGui::SameLine();
            presetButton("Iso", ViewportCameraPreset::Isometric);
            ImGui::SameLine();
            presetButton("Top", ViewportCameraPreset::Top);
            ImGui::SameLine();
            presetButton("Bot", ViewportCameraPreset::Bottom);
            ImGui::SameLine();
            presetButton("Lft", ViewportCameraPreset::Left);
            ImGui::SameLine();
            presetButton("Rgt", ViewportCameraPreset::Right);
            ImGui::SameLine();
            if (ImGui::SmallButton("Trk"))
            {
                applyTrackViewPreset(false);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Wlk"))
            {
                applyTrackViewPreset(true);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Focus"))
            {
                focusSelectedSection();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("All"))
            {
                frameWholeTrack();
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Settings"))
            {
                viewportSettingsWindowOpen_
                    = !viewportSettingsWindowOpen_;
                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "[CFG] viewport settings window %s",
                    viewportSettingsWindowOpen_ ? "open" : "closed"
                );
            }

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
            selectedSegmentIds_.fill(coaster::invalidSegmentId);
            dragSegmentIds_.fill(coaster::invalidSegmentId);
            dragAxisLocks_.fill(DragAxisLock::None);
            dragAxisTravelX_.fill(0.0);
            dragAxisTravelY_.fill(0.0);
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

        // Entry-to-exit height change of the selected region read from the
        // already-solved centerline slices; unavailable before the first
        // solve arrives.
        std::optional<double> selectedRegionHeightDelta;
        if (selectedSection_ < centerlineSlices_.size())
        {
            const CenterlineSectionSlice& slice =
                centerlineSlices_[selectedSection_];
            selectedRegionHeightDelta =
                slice.endPosition.y - slice.startPosition.y;
        }

        const TrackWorkspaceEdit workspaceEdit = showTrackWorkspace(
            *authoredTrack_,
            selectedSection_,
            &sectionLengthEditBuffer_,
            regionCreateFlow_,
            selectedRegionHeightDelta,
            quantum::coaster::computeTrackTopology(*authoredTrack_)
        );

        if (workspaceEdit.selectRequest.has_value())
        {
            selectSection(*workspaceEdit.selectRequest);
        }

        if (workspaceEdit.createdRegionKind.has_value())
        {
            const quantum::editor::RegionCreateAnchor anchor =
                regionCreateFlow_.anchor;
            regionCreateFlow_.choicePending = false;

            RegionCommandType createType =
                RegionCommandType::AppendRateProfiles;
            switch (anchor)
            {
            case quantum::editor::RegionCreateAnchor::Prepend:
                createType =
                    *workspaceEdit.createdRegionKind
                            == coaster::RegionKind::Geometry
                        ? RegionCommandType::PrependPlanarArc
                        : RegionCommandType::PrependRateProfiles;
                break;
            case quantum::editor::RegionCreateAnchor::AfterSelected:
                createType =
                    *workspaceEdit.createdRegionKind
                            == coaster::RegionKind::Geometry
                        ? RegionCommandType::InsertAfterPlanarArc
                        : RegionCommandType::InsertAfterRateProfiles;
                break;
            case quantum::editor::RegionCreateAnchor::Append:
                createType =
                    *workspaceEdit.createdRegionKind
                            == coaster::RegionKind::Geometry
                        ? RegionCommandType::AppendPlanarArc
                        : RegionCommandType::AppendRateProfiles;
                break;
            }

            regionCommand_ = {createType, selectedSection_, 0.0};
        }
        else if (workspaceEdit.duplicateRequested)
        {
            trackCommand_ = {TrackCommandType::DuplicateSection,
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

        if (workspaceEdit.layoutModeChanged.has_value())
        {
            pendingLayoutModeChange_ = workspaceEdit.layoutModeChanged;
        }

        if (workspaceEdit.completeCircuitRequested)
        {
            const quantum::coaster::TrackTopology analysis =
                quantum::coaster::computeTrackTopology(*authoredTrack_);

            char message[256]{};
            if (analysis.kind
                == quantum::coaster::TopologyKind::ClosedCircuit)
            {
                std::snprintf(
                    message,
                    sizeof(message),
                    "Track is already a closed circuit.\n\n"
                    "Gap: %.4f m\nTangent: %.4f deg\nFrame: %.4f deg",
                    analysis.diagnostics.positionalGap,
                    analysis.diagnostics.tangentMismatchDegrees,
                    analysis.diagnostics.frameMismatchDegrees);
            }
            else
            {
                std::snprintf(
                    message,
                    sizeof(message),
                    "Geometry must be added or adjusted before exact "
                    "closure is possible.\n\n"
                    "End -> Start gap: %.1f m\n"
                    "Tangent mismatch: %.1f deg\n"
                    "Frame mismatch: %.1f deg",
                    analysis.diagnostics.positionalGap,
                    analysis.diagnostics.tangentMismatchDegrees,
                    analysis.diagnostics.frameMismatchDegrees);
            }

            SDL_ShowSimpleMessageBox(
                SDL_MESSAGEBOX_INFORMATION,
                "Complete Circuit Analysis",
                message,
                window_);
        }

        // Conversions stay available as secondary operations from the
        // workspace; planar-arc parameter edits are queued directly by the
        // Geometry Editor window below.
        if (workspaceEdit.convertToGeometryRequested)
        {
            regionCommand_ = {RegionCommandType::ConvertToPlanarArc,
                              selectedSection_, 0.0};
        }
        else if (workspaceEdit.convertToRateProfilesRequested)
        {
            regionCommand_ = {RegionCommandType::ConvertToRateProfiles,
                              selectedSection_, 0.0};
        }
        // The Transition Editor is the primary detailed editor for
        // rate-profile regions and shows the selected section's authored
        // rate profiles over that section's local distance domain, with
        // every channel row editable the same way.
        //
        // Geometry regions get the dedicated Geometry Editor instead; the
        // Transition Editor window is not submitted at all for those
        // selections so each authoring model owns exactly one primary
        // editor surface and no irrelevant editor idles in a disabled
        // state. The dock slot stays registered in the layout ini, so
        // switching back to a rate-profile region restores the previous
        // placement.
        const coaster::AuthoredTrackSection& editedSection =
            authoredTrack_->section(selectedSection_);

        if (editedSection.kind != coaster::RegionKind::RateProfiles)
        {
            pushWorkspaceAccent(transitionEditorAccent);
            ImGui::PushStyleColor(
                ImGuiCol_Border,
                quantum::editor::palette::transitionEditorBlue
            );
            ImGui::SetNextWindowPos(
                ImVec2(80.0F, 260.0F),
                ImGuiCond_Appearing
            );
            ImGui::Begin(geometryEditorWindowName);

            const auto& committedArc =
                std::get<coaster::PlanarArcRegion>(
                    std::get<coaster::GeometryRegion>(
                        editedSection.region).construction);

            ImGui::TextDisabled("Geometry / Planar Arc authoring");
            ImGui::Spacing();

            // Angle fields present degrees; commands carry Core radians.
            // Every edit flows through the same candidate/commit pipeline
            // as rate-profile edits.
            if (ImGui::InputDouble(
                "Radius",
                &planarArcEditBuffers_[0],
                1.0,
                10.0,
                "%.3f"
            ))
            {
                regionCommand_ = {RegionCommandType::SetPlanarArcRadius,
                                  selectedSection_,
                                  planarArcEditBuffers_[0]};
            }

            if (ImGui::InputDouble(
                "Swept Angle (deg)",
                &planarArcEditBuffers_[1],
                5.0,
                15.0,
                "%.3f"
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcSweptAngle,
                    selectedSection_,
                    planarArcEditBuffers_[1] * radiansPerDegree};
            }

            if (ImGui::InputDouble(
                "Plane Tilt (deg)",
                &planarArcEditBuffers_[2],
                5.0,
                15.0,
                "%.3f"
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcPlaneTilt,
                    selectedSection_,
                    planarArcEditBuffers_[2] * radiansPerDegree};
            }

            if (ImGui::InputDouble(
                "Bank Change (deg)",
                &planarArcEditBuffers_[3],
                5.0,
                15.0,
                "%.3f"
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcBankChange,
                    selectedSection_,
                    planarArcEditBuffers_[3] * radiansPerDegree};
            }

            ImGui::Spacing();
            ImGui::Text(
                "Resulting length %.6g",
                coaster::planarArcLength(committedArc)
            );

            ImGui::Spacing();
            if (ImGui::Button("Convert to Rate Profiles",
                ImVec2(-1.0F, 0.0F)))
            {
                regionCommand_ = {RegionCommandType::ConvertToRateProfiles,
                                  selectedSection_, 0.0};
            }

            ImGui::End();
            ImGui::PopStyleColor(workspaceAccentColorCount + 1);
        }
        else
        {

        // Resolve per-channel segment selections against the committed
        // document; stale ids fall back to the tail segment so the numeric
        // field and type combo always address a live segment.
        const auto containsSegment = [](
            const coaster::ChannelProfile& profile,
            const std::uint32_t segmentId)
        {
            for (const coaster::ProfileSegment& segment :
                profile.segments)
            {
                if (segment.id == segmentId)
                {
                    return true;
                }
            }

            return false;
        };

        for (std::size_t channelIndex = 0;
            channelIndex < rateChannelCount;
            ++channelIndex)
        {
            const auto channel = static_cast<RateChannel>(channelIndex);
            const auto& profile = sectionRateChannel(editedSection, channel);

            if (profile.segments.empty())
            {
                continue;
            }

            const bool selectionLive = containsSegment(
                profile,
                selectedSegmentIds_[channelIndex]
            );
            if (!selectionLive)
            {
                selectedSegmentIds_[channelIndex] =
                    profile.segments.back().id;
                endpointSelections_[channelIndex] =
                    ScalarProfileEndpoint::None;
                valueEndEditBuffers_[channelIndex] =
                    profile.segments.back().transition.valueEnd;
            }

            const bool dragLive = endpointDrags_[channelIndex]
                == ScalarProfileEndpoint::None
                || containsSegment(profile, dragSegmentIds_[channelIndex]);
            if (!dragLive)
            {
                endpointDrags_[channelIndex] = ScalarProfileEndpoint::None;
                dragLastValues_[channelIndex].reset();
                scalarDragAnchors_[channelIndex].reset();
                dragAxisLocks_[channelIndex] = DragAxisLock::None;
                dragAxisTravelX_[channelIndex] = 0.0;
                dragAxisTravelY_[channelIndex] = 0.0;
            }
        }

        const std::array<ProfileRowView, rateChannelCount> profileRows{{
            {"Roll Rate",
             &sectionRateChannel(editedSection, RateChannel::Roll),
             RateChannel::Roll},
            {"Pitch Rate",
             &sectionRateChannel(editedSection, RateChannel::Pitch),
             RateChannel::Pitch},
            {"Yaw Rate",
             &sectionRateChannel(editedSection, RateChannel::Yaw),
             RateChannel::Yaw},
        }};
        const TransitionEditorEdit transitionEdit = showTransitionEditor(
            profileRows,
            coaster::sectionLength(editedSection),
            valueEndEditBuffers_,
            endpointSelections_,
            endpointDrags_,
            selectedSegmentIds_,
            dragSegmentIds_,
            dragAxisLocks_,
            dragAxisTravelX_,
            dragAxisTravelY_,
            contextMenuSplitDistances_,
            scalarDragAnchors_,
            transitionEditorInputSettings_
        );

        if (transitionEdit.endpointValueEdit.has_value())
        {
            const ScalarProfileEndpointValueEdit& valueEdit =
                *transitionEdit.endpointValueEdit;
            const auto& channelProfile = sectionRateChannel(
                editedSection,
                valueEdit.channel
            );
            // The numeric buffer semantics track the addressed segment's
            // End value, so that is what accepted edits compare against.
            double acceptedValue = 0.0;
            for (const coaster::ProfileSegment& segment :
                channelProfile.segments)
            {
                if (segment.id == valueEdit.segmentId)
                {
                    acceptedValue = segment.transition.valueEnd;
                    break;
                }
            }
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
                transitionEdit.transitionType->channel,
                transitionEdit.transitionType->segmentId
            };
        }

        if (transitionEdit.splitRequest.has_value())
        {
            const TransitionEditorEdit::SplitRequest& request =
                *transitionEdit.splitRequest;
            double splitDistance = request.distance;
            const TransitionEditorInputSettings& inputSettings =
                transitionEditorInputSettings_;
            if (inputSettings.distanceSnapEnabled
                && inputSettings.distanceSnapIncrement > 0.0)
            {
                // Snapping applies at this single choke point like value
                // snapping does; Core still validates the final distance.
                splitDistance = snapToIncrement(
                    splitDistance,
                    inputSettings.distanceSnapIncrement
                );
            }

            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "[INP] TRANSITION_SPLIT channel=%d segment=%u "
                "distance=%.6f",
                static_cast<int>(request.channel),
                request.segmentId,
                splitDistance
            );
            profileSegmentCommand_ = ProfileSegmentCommand{
                ProfileSegmentOperation::Split,
                selectedSection_,
                request.channel,
                request.segmentId,
                splitDistance
            };
        }

        if (transitionEdit.removeRequest.has_value())
        {
            const TransitionEditorEdit::RemoveRequest& request =
                *transitionEdit.removeRequest;
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "[INP] TRANSITION_REMOVE channel=%d segment=%u",
                static_cast<int>(request.channel),
                request.segmentId
            );
            profileSegmentCommand_ = ProfileSegmentCommand{
                ProfileSegmentOperation::Remove,
                selectedSection_,
                request.channel,
                request.segmentId,
                0.0
            };
        }

        if (transitionEdit.distanceMove.has_value())
        {
            const TransitionEditorEdit::DistanceMove& move =
                *transitionEdit.distanceMove;
            ProfileSegmentDistanceEdit queuedEdit{};
            queuedEdit.sectionIndex = selectedSection_;
            queuedEdit.channel = move.channel;
            queuedEdit.segmentId = move.segmentId;
            queuedEdit.endpoint = move.endpoint;
            queuedEdit.distance = move.distance;
            profileSegmentDistanceEdit_ = queuedEdit;

            pendingDistanceSummary_ = true;
            distanceSummaryEdit_ = queuedEdit;
        }

        if (transitionEdit.click.has_value())
        {
            const std::size_t clickChannel = static_cast<std::size_t>(
                transitionEdit.click->channel);
            if (transitionEdit.click->endpoint
                != ScalarProfileEndpoint::None)
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] TRANSITION_CLICK channel=%zu endpoint=%d "
                    "segment=%u",
                    clickChannel,
                    static_cast<int>(transitionEdit.click->endpoint),
                    transitionEdit.click->segmentId);
                selectedSegmentIds_[clickChannel] =
                    transitionEdit.click->segmentId;
                dragSegmentIds_[clickChannel] =
                    transitionEdit.click->segmentId;
                endpointSelections_[clickChannel] =
                    transitionEdit.click->endpoint;
                endpointDrags_[clickChannel] =
                    transitionEdit.click->endpoint;
                dragAxisLocks_[clickChannel] = DragAxisLock::None;
                dragAxisTravelX_[clickChannel] = 0.0;
                dragAxisTravelY_[clickChannel] = 0.0;
                dragLastValues_[clickChannel].reset();

                // Seed the rolling drag anchor at the committed value and
                // current cursor so arming a drag never jumps the value.
                const auto& clickedProfile = sectionRateChannel(
                    editedSection,
                    transitionEdit.click->channel
                );
                double committedValue = 0.0;
                double committedDistance = 0.0;
                for (const coaster::ProfileSegment& segment :
                    clickedProfile.segments)
                {
                    if (segment.id == transitionEdit.click->segmentId)
                    {
                        committedValue =
                            transitionEdit.click->endpoint
                                == ScalarProfileEndpoint::Begin
                            ? segment.transition.valueBegin
                            : segment.transition.valueEnd;
                        committedDistance =
                            transitionEdit.click->endpoint
                                == ScalarProfileEndpoint::Begin
                            ? segment.transition.domainBegin
                            : segment.transition.domainEnd;
                        break;
                    }
                }
                scalarDragAnchors_[clickChannel] = ScalarDragAnchor{
                    static_cast<double>(ImGui::GetIO().MousePos.x),
                    static_cast<double>(ImGui::GetIO().MousePos.y),
                    committedValue,
                    committedDistance
                };
            }
            else
            {
                const auto& plotProfile = sectionRateChannel(
                    editedSection,
                    transitionEdit.click->channel
                );
                std::uint32_t clickedSegmentId =
                    transitionEdit.click->segmentId;
                if (!containsSegment(plotProfile, clickedSegmentId))
                {
                    clickedSegmentId = plotProfile.segments.back().id;
                }

                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[INP] TRANSITION_PLOT_CLICK channel=%zu segment=%u",
                    clickChannel,
                    clickedSegmentId);
                selectedSegmentIds_[clickChannel] = clickedSegmentId;
                endpointSelections_[clickChannel] =
                    ScalarProfileEndpoint::None;
                endpointDrags_[clickChannel] =
                    ScalarProfileEndpoint::None;
                dragLastValues_[clickChannel].reset();
                scalarDragAnchors_[clickChannel].reset();
                for (const coaster::ProfileSegment& segment :
                    plotProfile.segments)
                {
                    if (segment.id == clickedSegmentId)
                    {
                        valueEndEditBuffers_[clickChannel] =
                            segment.transition.valueEnd;
                        break;
                    }
                }
            }
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
                    "[EDIT] section=%zu channel=%d endpoint=%d value=%.6f "
                    "segment=%u",
                    dragSummaryEdit_.sectionIndex,
                    static_cast<int>(dragSummaryEdit_.channel),
                    static_cast<int>(dragSummaryEdit_.endpoint),
                    dragSummaryEdit_.value,
                    dragSummaryEdit_.segmentId);
                pendingDragSummary_ = false;
            }
            if (pendingDistanceSummary_)
            {
                SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "[EDIT] section=%zu channel=%d endpoint=%d "
                    "segment=%u distance=%.6f",
                    distanceSummaryEdit_.sectionIndex,
                    static_cast<int>(distanceSummaryEdit_.channel),
                    static_cast<int>(distanceSummaryEdit_.endpoint),
                    distanceSummaryEdit_.segmentId,
                    distanceSummaryEdit_.distance);
                pendingDistanceSummary_ = false;
            }
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            dragSegmentIds_.fill(coaster::invalidSegmentId);
            dragAxisLocks_.fill(DragAxisLock::None);
            dragAxisTravelX_.fill(0.0);
            dragAxisTravelY_.fill(0.0);
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

    std::optional<ProfileSegmentCommand>
    EditorUi::takeProfileSegmentCommand() noexcept
    {
        const std::optional<ProfileSegmentCommand> command =
            profileSegmentCommand_;
        profileSegmentCommand_.reset();
        return command;
    }

    std::optional<ProfileSegmentDistanceEdit>
    EditorUi::takeProfileSegmentDistanceEdit() noexcept
    {
        const std::optional<ProfileSegmentDistanceEdit> edit =
            profileSegmentDistanceEdit_;
        profileSegmentDistanceEdit_.reset();
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

    std::optional<RegionCommand> EditorUi::takeRegionCommand() noexcept
    {
        const std::optional<RegionCommand> command = regionCommand_;
        regionCommand_.reset();
        return command;
    }

    void EditorUi::selectSection(const std::size_t index)
    {
        const std::size_t sectionCount = authoredTrack_ != nullptr
            ? authoredTrack_->sectionCount()
            : 0;
        if (sectionCount == 0 || index >= sectionCount
            || index == selectedSection_)
        {
            return;
        }

        selectedSection_ = index;
        regionCreateFlow_.choicePending = false;
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        dragAxisLocks_.fill(DragAxisLock::None);
        dragAxisTravelX_.fill(0.0);
        dragAxisTravelY_.fill(0.0);
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

        // Both selection log formats are consumed by tooling; keep
        // the rate-profile format byte-identical to its historical
        // shape and give geometry sections their own line.
        if (selected.kind == coaster::RegionKind::RateProfiles)
        {
            const auto& rollProfile =
                sectionRateChannel(selected, RateChannel::Roll);
            const auto& pitchProfile =
                sectionRateChannel(selected, RateChannel::Pitch);
            const auto& yawProfile =
                sectionRateChannel(selected, RateChannel::Yaw);
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[SEL] selected=%zu rollEnd=%.6f pitchEnd=%.6f "
                "yawEnd=%.6f",
                selectedSection_,
                rollProfile.segments.back().transition.valueEnd,
                pitchProfile.segments.back().transition.valueEnd,
                yawProfile.segments.back().transition.valueEnd
            );
            for (std::size_t channelIndex = 0;
                channelIndex < rateChannelCount;
                ++channelIndex)
            {
                valueEndEditBuffers_[channelIndex] =
                    sectionRateChannel(
                        selected,
                        static_cast<RateChannel>(channelIndex)
                    ).segments.back().transition.valueEnd;
            }
        }
        else
        {
            const auto& arc = std::get<coaster::PlanarArcRegion>(
                std::get<coaster::GeometryRegion>(
                    selected.region).construction);
            SDL_LogInfo(
                SDL_LOG_CATEGORY_APPLICATION,
                "[SEL] selected=%zu kind=planarArc length=%.6f "
                "radius=%.6f sweptAngle=%.6f planeTilt=%.6f "
                "bankChange=%.6f",
                selectedSection_,
                selected.length,
                arc.radius,
                arc.sweptAngle,
                arc.planeTilt,
                arc.bankChange
            );
        }

        if (selected.kind == coaster::RegionKind::Geometry)
        {
            const auto& arc = std::get<coaster::PlanarArcRegion>(
                std::get<coaster::GeometryRegion>(
                    selected.region).construction);
            planarArcEditBuffers_ = {
                arc.radius,
                arc.sweptAngle * degreesPerRadian,
                arc.planeTilt * degreesPerRadian,
                arc.bankChange * degreesPerRadian};
        }

        sectionLengthEditBuffer_ =
            coaster::sectionLength(selected);
    }

    void EditorUi::synchronizeSegmentValueEnd(
        const RateChannel channel,
        const std::uint32_t segmentId,
        const double acceptedValueEnd)
    {
        if (!std::isfinite(acceptedValueEnd))
        {
            throw std::invalid_argument(
                "EditorUi requires a finite accepted profile end value."
            );
        }

        // The numeric buffer addresses the row's focused segment; edits
        // committed to any other segment must not overwrite the display.
        if (selectedSegmentIds_[static_cast<std::size_t>(channel)]
            == segmentId)
        {
            valueEndEditBuffers_[static_cast<std::size_t>(channel)] =
                acceptedValueEnd;
        }
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

    void EditorUi::synchronizePlanarArcParams(
        const coaster::PlanarArcRegion& committedParams)
    {
        if (!std::isfinite(committedParams.radius)
            || !std::isfinite(committedParams.sweptAngle)
            || !std::isfinite(committedParams.planeTilt)
            || !std::isfinite(committedParams.bankChange))
        {
            throw std::invalid_argument(
                "EditorUi requires finite committed planar-arc "
                "parameters."
            );
        }

        planarArcEditBuffers_ = {
            committedParams.radius,
            committedParams.sweptAngle * degreesPerRadian,
            committedParams.planeTilt * degreesPerRadian,
            committedParams.bankChange * degreesPerRadian};
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

    void EditorUi::setCenterlineSections(
        std::vector<CenterlineSectionSlice> sectionSlices)
    {
        centerlineSlices_ = std::move(sectionSlices);
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
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        dragAxisLocks_.fill(DragAxisLock::None);
        dragAxisTravelX_.fill(0.0);
        dragAxisTravelY_.fill(0.0);
        pendingDistanceSummary_ = false;
        for (double& splitDistance : contextMenuSplitDistances_)
        {
            splitDistance = 0.0;
        }
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
        profileSegmentCommand_.reset();
        profileSegmentDistanceEdit_.reset();
        trackCommand_.reset();
        sectionLengthEdit_.reset();
        iniPath_.clear();
        pendingFileOperation_.reset();
    }

    std::optional<FileOperationType>
    EditorUi::takePendingFileOperation() noexcept
    {
        const auto op = pendingFileOperation_;
        pendingFileOperation_.reset();
        return op;
    }

    std::optional<coaster::LayoutMode>
    EditorUi::takePendingLayoutModeChange() noexcept
    {
        const auto mode = pendingLayoutModeChange_;
        pendingLayoutModeChange_.reset();
        return mode;
    }

    void EditorUi::resetTransientState()
    {
        selectedSection_ = 0;
        valueEndEditBuffers_.fill(0.0);
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        dragAxisLocks_.fill(DragAxisLock::None);
        dragAxisTravelX_.fill(0.0);
        dragAxisTravelY_.fill(0.0);
        pendingDragSummary_ = false;
        pendingDistanceSummary_ = false;

        for (std::optional<double>& lastValue : dragLastValues_)
        {
            lastValue.reset();
        }

        for (std::optional<ScalarDragAnchor>& anchor : scalarDragAnchors_)
        {
            anchor.reset();
        }

        for (double& splitDistance : contextMenuSplitDistances_)
        {
            splitDistance = 0.0;
        }

        regionCreateFlow_.choicePending = false;
        regionCreateFlow_.anchor = RegionCreateAnchor::Append;
        sectionLengthEditBuffer_ = 0.0;
        planarArcEditBuffers_.fill(0.0);
        profileEndpointValueEdit_.reset();
        profileTransitionTypeEdit_.reset();
        profileSegmentCommand_.reset();
        profileSegmentDistanceEdit_.reset();
        trackCommand_.reset();
        sectionLengthEdit_.reset();
        regionCommand_.reset();
    }

    void EditorUi::updateWindowTitle(const std::string& title)
    {
        if (window_ != nullptr)
        {
            SDL_SetWindowTitle(window_, title.c_str());
        }
    }
}
