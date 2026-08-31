#include <quantum/editor/EditorUi.hpp>

#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/editor/EditorStyle.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/editor/ViewportPicking.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_messagebox.h>
#include <SDL3/SDL_stdinc.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <imgui_internal.h>

#include <glm/geometric.hpp>

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
#include <utility>

namespace
{
    constexpr char bundledFontRelativePath[] =
        "assets/fonts/RedHatMono-SemiBold.ttf";
    // The v2 file migrates the authored shell from the generic Properties
    // layout while preserving normal Dear ImGui persistence thereafter.
    constexpr char editorLayoutIniFileName[] = "imgui-layout-v2.ini";
    constexpr char editorDockspaceName[] = "QuantumEditorDockSpace";
    // Display names can change; these suffixes retain the existing window
    // IDs and their persisted docking placements.
    constexpr char trackWorkspaceWindowName[] =
        "Track Workspace###TRACK WORKSPACE";
    constexpr char supportWorkspaceWindowName[] =
        "Supports###Support Workspace";
    constexpr double orbitRadiansPerPixel = 0.005;
    constexpr double piRadians = 3.14159265358979323846;
    // The Geometry Editor presents angles in degrees; Core keeps radians.
    constexpr double degreesPerRadian = 180.0 / piRadians;
    constexpr double radiansPerDegree = piRadians / 180.0;
    // Dedicated detail window for geometry-authored regions. It replaces
    // the Transition Editor for those selections instead of idling beside
    // it, so each region kind owns exactly one primary editor surface.
    constexpr char geometryEditorWindowName[] = "Geometry Editor";
    constexpr char riderLoadDiagnosticsWindowName[] = "Force Diagnostics";
    constexpr float startPoseMoveHandlePixels = 56.0F;
    constexpr float startPoseMoveHitRadiusPixels = 8.0F;
    constexpr std::array<float, 3> startPoseRotateRadiiPixels{
        30.0F, 39.0F, 48.0F};
    constexpr float startPoseRotateHitRadiusPixels = 5.0F;

    [[nodiscard]] const char* startPoseAxisName(
        const quantum::editor::StartPoseTransformAxis axis) noexcept
    {
        using quantum::editor::StartPoseTransformAxis;
        switch (axis)
        {
        case StartPoseTransformAxis::X: return "X";
        case StartPoseTransformAxis::Y: return "Y";
        case StartPoseTransformAxis::Z: return "Z";
        }
        return "X";
    }

    [[nodiscard]] const char* startPoseModeName(
        const quantum::editor::StartPoseTransformMode mode) noexcept
    {
        return mode == quantum::editor::StartPoseTransformMode::Move
            ? "move"
            : "rotate";
    }

    [[nodiscard]] double pointSegmentDistanceSquared(
        const ImVec2 point,
        const ImVec2 begin,
        const ImVec2 end) noexcept
    {
        const double dx = static_cast<double>(end.x - begin.x);
        const double dy = static_cast<double>(end.y - begin.y);
        const double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared == 0.0)
        {
            const double offsetX = static_cast<double>(point.x - begin.x);
            const double offsetY = static_cast<double>(point.y - begin.y);
            return offsetX * offsetX + offsetY * offsetY;
        }

        const double offsetX = static_cast<double>(point.x - begin.x);
        const double offsetY = static_cast<double>(point.y - begin.y);
        const double parameter = std::clamp(
            (offsetX * dx + offsetY * dy) / lengthSquared,
            0.0,
            1.0
        );
        const double closestX = static_cast<double>(begin.x) + parameter * dx;
        const double closestY = static_cast<double>(begin.y) + parameter * dy;
        const double distanceX = static_cast<double>(point.x) - closestX;
        const double distanceY = static_cast<double>(point.y) - closestY;
        return distanceX * distanceX + distanceY * distanceY;
    }

    [[nodiscard]] std::uint32_t visibleTrackCurveMask(
        const quantum::editor::ViewportSettings& settings) noexcept
    {
        std::uint32_t curveMask = 0;
        if (settings.leftRailVisible)
        {
            curveMask |= 1u
                << quantum::renderer::viewportLeftRailCurve;
        }
        if (settings.rightRailVisible)
        {
            curveMask |= 1u
                << quantum::renderer::viewportRightRailCurve;
        }
        if (settings.centerlineVisible)
        {
            curveMask |= 1u
                << quantum::renderer::viewportCenterlineCurve;
        }
        if (settings.heartlineVisible)
        {
            curveMask |= 1u
                << quantum::renderer::viewportHeartlineCurve;
        }
        return curveMask;
    }

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

    // Continue a small group only when it fits; otherwise ImGui's normal
    // next-row cursor placement keeps narrow docked panels usable.
    void sameLineIfFits(const float width)
    {
        const float nextX = ImGui::GetItemRectMax().x
            + ImGui::GetStyle().ItemSpacing.x;
        const float right = ImGui::GetWindowPos().x
            + ImGui::GetWindowContentRegionMax().x;
        if (nextX + width <= right)
        {
            ImGui::SameLine();
        }
    }

    [[nodiscard]] float buttonWidth(const char* const label)
    {
        return ImGui::CalcTextSize(label, nullptr, true).x
            + 2.0F * ImGui::GetStyle().FramePadding.x;
    }

    void itemTooltip(const char* const text)
    {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
        {
            ImGui::SetTooltip("%s", text);
        }
    }

    struct AuthoredDomainView
    {
        double domainBegin;
        double domainEnd;
        float pixelBegin;
        float pixelEnd;

        [[nodiscard]] float toPixel(const double domainValue) const
        {
            const double progress = quantum::editor::
                graphDistanceToNormalized(
                    domainValue,
                    domainBegin,
                    domainEnd
                );
            return pixelBegin + static_cast<float>(progress)
                * (pixelEnd - pixelBegin);
        }
    };

    constexpr int domainDivisionCount = 5;
    constexpr int valueDivisionCount = 4;
    constexpr std::size_t profileSampleCount = 129;
    constexpr float endpointHandleRadius = 5.0F;
    constexpr float endpointHoverRadius = 10.0F;
    constexpr float curveHoverRadius = 7.0F;
    // Interior joints idle as small dots so a multi-segment curve stays
    // readable; they grow into full handles on hover.
    constexpr float jointHandleRadius = 2.5F;
    constexpr float jointSelectedRadius = 3.5F;
    // Cumulative cursor travel required before a drag commits to one
    // axis; below this threshold neither value nor distance integrates.
    constexpr double dragAxisLockPixelThreshold = 4.0;

    struct ScalarProfileMapping
    {
        AuthoredDomainView domainView;
        quantum::editor::GraphValueRange valueRange;
        float pixelBeginY;
        float pixelEndY;

        [[nodiscard]] ImVec2 toPixel(
            const double domainValue,
            const double value) const
        {
            const float normalizedValue = static_cast<float>(
                quantum::editor::graphValueToNormalized(value, valueRange)
            );

            return {
                domainView.toPixel(domainValue),
                pixelEndY
                    - normalizedValue * (pixelEndY - pixelBeginY)
            };
        }

        // Natural drag slope of this row in authored value units per
        // cursor pixel; the basis for configurable drag sensitivity.
        [[nodiscard]] double valueUnitsPerPixel() const
        {
            const float pixelRange = pixelEndY - pixelBeginY;
            if (!valueRange.valid() || pixelRange <= 0.0F)
            {
                return 0.0;
            }
            return quantum::editor::graphValueUnitsPerPixel(
                valueRange,
                static_cast<double>(pixelRange)
            );
        }
    };

    // Value-axis geometry of one drawn profile row; drives centralized
    // drag sensitivity math in showTransitionEditor.
    struct ScalarRowEditGeometry
    {
        quantum::editor::GraphValueRange valueRange;
        double unitsPerPixel = 0.0;
        double distanceUnitsPerPixel = 0.0;
    };

    struct ScalarProfileRowEdit
    {
        bool activateRequested = false;
        bool valueEndEdited = false;
        std::optional<quantum::math::TransitionType> transitionType;
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

    struct ChannelPlotGeometry
    {
        ScalarProfileMapping mapping;
        std::vector<ImVec2> points;
        std::vector<std::size_t> spanFirsts;
        std::vector<std::size_t> spanCounts;
        std::vector<RowHandlePoint> handles;
        ScalarRowEditGeometry editGeometry;
    };

    [[nodiscard]] std::vector<RowHandlePoint> buildProfileHandles(
        const ScalarProfileMapping& mapping,
        const quantum::coaster::ChannelProfile& profile)
    {
        using quantum::editor::ScalarProfileEndpoint;

        const std::vector<quantum::editor::SemanticProfileMarker> markers =
            quantum::editor::extractSemanticProfileMarkers(profile);
        std::vector<RowHandlePoint> handles;
        handles.reserve(markers.size());

        for (const quantum::editor::SemanticProfileMarker& marker : markers)
        {
            handles.push_back(RowHandlePoint{
                mapping.toPixel(marker.distance, marker.value),
                marker.distance,
                marker.value,
                marker.segmentId,
                marker.endpoint,
                marker.regionBoundary
            });
        }

        return handles;
    }

    void drawProfileHandles(
        ImDrawList* const drawList,
        const std::span<const RowHandlePoint> handles,
        const ImU32 curveColor,
        const quantum::editor::RateChannel channel,
        const std::uint32_t selectedSegmentId,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint,
        const std::optional<quantum::editor::GraphMarkerId> hoveredMarker,
        const bool activeChannel)
    {
        using quantum::editor::ScalarProfileEndpoint;

        const auto drawFilledMarker = [drawList, channel](
            const ImVec2 position,
            const float radius,
            const ImU32 color)
        {
            switch (channel)
            {
            case quantum::editor::RateChannel::Roll:
                drawList->AddRectFilled(
                    ImVec2(position.x - radius, position.y - radius),
                    ImVec2(position.x + radius, position.y + radius),
                    color
                );
                break;
            case quantum::editor::RateChannel::Yaw:
                drawList->AddQuadFilled(
                    ImVec2(position.x, position.y - radius),
                    ImVec2(position.x + radius, position.y),
                    ImVec2(position.x, position.y + radius),
                    ImVec2(position.x - radius, position.y),
                    color
                );
                break;
            case quantum::editor::RateChannel::Pitch:
            default:
                drawList->AddCircleFilled(position, radius, color);
                break;
            }
        };
        const auto drawOpenMarker = [drawList, channel](
            const ImVec2 position,
            const float radius,
            const ImU32 color)
        {
            switch (channel)
            {
            case quantum::editor::RateChannel::Roll:
                drawList->AddRect(
                    ImVec2(position.x - radius, position.y - radius),
                    ImVec2(position.x + radius, position.y + radius),
                    color,
                    0.0F,
                    ImDrawFlags_None,
                    1.5F
                );
                break;
            case quantum::editor::RateChannel::Yaw:
                drawList->AddQuad(
                    ImVec2(position.x, position.y - radius),
                    ImVec2(position.x + radius, position.y),
                    ImVec2(position.x, position.y + radius),
                    ImVec2(position.x - radius, position.y),
                    color,
                    1.5F
                );
                break;
            case quantum::editor::RateChannel::Pitch:
            default:
                drawList->AddCircle(
                    position,
                    radius,
                    color,
                    0,
                    1.5F
                );
                break;
            }
        };

        for (std::size_t handleIndex = 0;
            handleIndex < handles.size();
            ++handleIndex)
        {
            const RowHandlePoint& handle = handles[handleIndex];
            const bool hovered = hoveredMarker.has_value()
                && hoveredMarker->channel == channel
                && hoveredMarker->segmentId == handle.segmentId
                && hoveredMarker->endpoint == handle.endpoint;
            const bool ownedBySelected =
                handle.segmentId == selectedSegmentId;
            const bool selectedMarker = ownedBySelected
                && selectedEndpoint == handle.endpoint;
            const float baseRadius = handle.outer
                ? endpointHandleRadius
                : (ownedBySelected
                    ? jointSelectedRadius
                    : jointHandleRadius);
            const float radius = hovered ? baseRadius + 1.5F : baseRadius;
            const ImU32 inactiveColor =
                (curveColor & ~IM_COL32_A_MASK)
                | (static_cast<ImU32>(190) << IM_COL32_A_SHIFT);

            // Active authored points are filled; inactive points remain open
            // at their exact coordinates. This preserves coincident values
            // while making the foreground channel visually deterministic.
            if (activeChannel || hovered)
            {
                drawFilledMarker(
                    handle.position,
                    radius,
                    curveColor
                );
            }
            else
            {
                drawOpenMarker(
                    handle.position,
                    radius + 1.0F,
                    inactiveColor
                );
            }

            if (selectedMarker)
            {
                drawList->AddCircle(
                    handle.position,
                    radius + 3.0F,
                    ImGui::GetColorU32(ImGuiCol_Text),
                    0,
                    2.0F
                );
            }
            else if (ownedBySelected && activeChannel)
            {
                drawList->AddCircle(
                    handle.position,
                    radius + 1.75F,
                    curveColor,
                    0,
                    1.25F
                );
            }
            else if (hovered)
            {
                drawList->AddCircle(
                    handle.position,
                    radius + 2.0F,
                    curveColor,
                    0,
                    1.5F
                );
            }
        }

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
        const float rulerLineY)
    {
        const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
        const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);
        const ImU32 disabledTextColor = ImGui::GetColorU32(
            ImGuiCol_TextDisabled
        );

        drawList->AddText(canvasBegin, textColor, "Distance in region");

        char domainLabel[64]{};
        std::snprintf(
            domainLabel,
            sizeof(domainLabel),
            "%.4g -> %.4g (region-local)",
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

        const int divisions = std::clamp(static_cast<int>(
            (domainView.pixelEnd - domainView.pixelBegin)
                / (ImGui::CalcTextSize("-0.000e+00").x + 16.0F)),
            1, domainDivisionCount);
        for (int division = 0;
            division <= divisions;
            ++division)
        {
            const double progress = static_cast<double>(division)
                / static_cast<double>(divisions);
            const double domainValue = domainView.domainBegin
                + progress
                    * (domainView.domainEnd - domainView.domainBegin);
            const float x = domainView.toPixel(domainValue);

            drawList->AddLine(
                ImVec2(x, rulerLineY - 4.0F),
                ImVec2(x, rulerLineY + 4.0F),
                borderColor
            );
            char tickLabel[32]{};
            std::snprintf(
                tickLabel,
                sizeof(tickLabel),
                "%.4g",
                domainValue
            );
            const float tickLabelWidth = ImGui::CalcTextSize(tickLabel).x;
            const float tickLabelX = std::clamp(
                x - tickLabelWidth * 0.5F,
                domainView.pixelBegin,
                std::max(domainView.pixelBegin,
                    domainView.pixelEnd - tickLabelWidth)
            );
            drawList->AddText(
                ImVec2(tickLabelX, rulerLineY + 5.0F),
                disabledTextColor,
                tickLabel
            );
        }
    }

    struct RiderLoadPlotRange
    {
        double minimum = -1.0;
        double maximum = 1.0;
    };

    using RiderLoadSampleMember = double quantum::editor::
        RiderLoadDiagnosticSample::*;

    struct RiderLoadChannelView
    {
        const char* label;
        const char* unit;
        RiderLoadSampleMember value;
        double minimumSpan;
        bool nonNegative;
        ImU32 color;
    };

    [[nodiscard]] RiderLoadPlotRange fitRiderLoadPlotRange(
        const std::span<const quantum::editor::RiderLoadDiagnosticSample>
            samples,
        const RiderLoadChannelView& channel)
    {
        if (samples.empty())
        {
            return channel.nonNegative
                ? RiderLoadPlotRange{0.0, channel.minimumSpan}
                : RiderLoadPlotRange{
                    -0.5 * channel.minimumSpan,
                    0.5 * channel.minimumSpan};
        }

        double minimum = samples.front().*channel.value;
        double maximum = minimum;
        for (const quantum::editor::RiderLoadDiagnosticSample& sample
            : samples)
        {
            const double value = sample.*channel.value;
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
        }

        // A zero reference keeps signed G channels legible and gives speed
        // an honest physical baseline without altering the sampled data.
        minimum = std::min(minimum, 0.0);
        maximum = std::max(maximum, 0.0);

        const double span = maximum - minimum;
        if (span < channel.minimumSpan)
        {
            const double padding = 0.5 * (channel.minimumSpan - span);
            minimum -= padding;
            maximum += padding;
        }
        else
        {
            const double padding = span * 0.08;
            minimum -= padding;
            maximum += padding;
        }

        if (channel.nonNegative)
        {
            minimum = 0.0;
        }
        return {minimum, maximum};
    }

    void showRiderLoadDiagnostics(
        const quantum::editor::SectionRiderLoadDiagnostics& diagnostics,
        const quantum::coaster::TrackPhysicalSettings& settings,
        bool* const open)
    {
        using quantum::editor::RiderLoadUnreachableLocation;

        if (!*open)
        {
            return;
        }
        ImGui::SetNextWindowSize(ImVec2(520.0F, 460.0F),
            ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(280.0F, 240.0F),
            ImVec2(FLT_MAX, FLT_MAX));
        if (!ImGui::Begin(riderLoadDiagnosticsWindowName, open))
        {
            ImGui::End();
            return;
        }

        ImGui::PushTextWrapPos();
        ImGui::Text(
            "Region %zu - Actual rider loads (read-only)",
            diagnostics.sectionIndex + 1
        );
        ImGui::PopTextWrapPos();
        if (ImGui::TreeNode("Evaluation settings"))
        {
            ImGui::PushTextWrapPos();
            ImGui::TextDisabled(
                "Document settings: Initial Speed %.2f m/s | "
                "Scale %.3f m/coordinate unit | gravity-only point mass",
                settings.initialSpeed,
                settings.metersPerCoordinateUnit
            );
            ImGui::PopTextWrapPos();
            ImGui::TreePop();
        }

        ImGui::PushTextWrapPos();
        if (diagnostics.unreachable.has_value())
        {
            ImGui::PushTextWrapPos();
            const double stopDistance = diagnostics.unreachable->distance;
            switch (diagnostics.unreachableLocation)
            {
            case RiderLoadUnreachableLocation::BeforeSelectedSection:
                ImGui::TextColored(
                    quantum::editor::palette::error,
                    "Evaluation stopped: energetically unreachable at "
                    "track distance %.4g before this region. No later "
                    "values are fabricated.",
                    stopDistance
                );
                break;
            case RiderLoadUnreachableLocation::WithinSelectedSection:
                ImGui::TextColored(
                    quantum::editor::palette::error,
                    "Evaluation stopped: energetically unreachable at "
                    "region-local distance %.4g (track %.4g). Valid "
                    "earlier samples remain visible.",
                    diagnostics.unreachableLocalDistance.value_or(0.0),
                    stopDistance
                );
                break;
            case RiderLoadUnreachableLocation::AfterSelectedSection:
                ImGui::TextColored(
                    quantum::editor::palette::error,
                    "Whole-track evaluation stopped at track distance "
                    "%.4g, at or after this region's exit; this region's "
                    "valid samples remain visible.",
                    stopDistance
                );
                break;
            case RiderLoadUnreachableLocation::None:
            default:
                break;
            }
            ImGui::PopTextWrapPos();
        }
        else if (diagnostics.samples.empty())
        {
            ImGui::TextDisabled(
                "No valid rider-load history is available for this region."
            );
        }
        else
        {
            ImGui::TextDisabled(
                "Whole-track evaluation completed; displaying this "
                "region's section-local samples."
            );
        }
        ImGui::PopTextWrapPos();

        if (ImGui::BeginChild(
            "##RiderLoadDiagnosticPlots",
            ImVec2(0.0F, 0.0F),
            ImGuiChildFlags_Borders))
        {
            const ImVec2 canvasBegin = ImGui::GetCursorScreenPos();
            constexpr float minimumPlotHeight = 64.0F;
            const float labelHeight = 2.0F * ImGui::GetTextLineHeight() + 8.0F;
            const float rulerHeight = 3.0F * ImGui::GetTextLineHeight() + 14.0F;
            constexpr float rowGap = 10.0F;
            constexpr std::size_t diagnosticChannelCount = 4;
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const ImVec2 canvasSize{available.x, std::max(available.y,
                rulerHeight + static_cast<float>(diagnosticChannelCount)
                    * (labelHeight + minimumPlotHeight)
                    + rowGap * static_cast<float>(diagnosticChannelCount - 1))};
            const ImVec2 canvasEnd{
                canvasBegin.x + canvasSize.x,
                canvasBegin.y + canvasSize.y
            };
            const float plotBeginX = canvasBegin.x + 4.0F;
            const float plotEndX = canvasEnd.x - 8.0F;
            const float plotsBeginY = canvasBegin.y + rulerHeight;
            const float availablePlotHeight = canvasEnd.y - plotsBeginY;
            const float rowHeight = std::max(labelHeight + minimumPlotHeight, (
                availablePlotHeight
                    - rowGap * static_cast<float>(
                        diagnosticChannelCount - 1))
                / static_cast<float>(diagnosticChannelCount));

            if (diagnostics.sectionLength <= 0.0
                || plotEndX - plotBeginX < 80.0F)
            {
                ImGui::TextDisabled(
                    "Resize Force Diagnostics to view the shared "
                    "region-distance plots."
                );
            }
            else
            {
                static const std::array<ImU32, diagnosticChannelCount>
                    channelColors{
                        ImGui::ColorConvertFloat4ToU32(
                            quantum::editor::palette::normalGChannel),
                        ImGui::ColorConvertFloat4ToU32(
                            quantum::editor::palette::yawChannelGold),
                        ImGui::ColorConvertFloat4ToU32(
                            quantum::editor::palette::rollChannelRed),
                        ImGui::ColorConvertFloat4ToU32(
                            quantum::editor::palette::speedChannel)
                    };
                const std::array<RiderLoadChannelView,
                    diagnosticChannelCount> channels{{
                    {"Normal G", "G",
                        &quantum::editor::RiderLoadDiagnosticSample::normalG,
                        2.0, false, channelColors[0]},
                    {"Lateral G", "G",
                        &quantum::editor::RiderLoadDiagnosticSample::lateralG,
                        2.0, false, channelColors[1]},
                    {"Longitudinal G", "G",
                        &quantum::editor::RiderLoadDiagnosticSample::
                            longitudinalG,
                        2.0, false, channelColors[2]},
                    {"Vehicle Speed", "m/s",
                        &quantum::editor::RiderLoadDiagnosticSample::
                            vehicleSpeed,
                        10.0, true, channelColors[3]}
                }};

                const AuthoredDomainView domainView{
                    .domainBegin = 0.0,
                    .domainEnd = diagnostics.sectionLength,
                    .pixelBegin = plotBeginX,
                    .pixelEnd = plotEndX
                };
                ImDrawList* const drawList = ImGui::GetWindowDrawList();
                drawList->PushClipRect(canvasBegin, canvasEnd, true);
                drawAuthoredDomainRuler(
                    drawList,
                    domainView,
                    canvasBegin,
                    canvasBegin.y + 2.0F * ImGui::GetTextLineHeight() + 4.0F
                );

                std::vector<ImVec2> points;
                points.reserve(diagnostics.samples.size());
                for (std::size_t channelIndex = 0;
                    channelIndex < channels.size();
                    ++channelIndex)
                {
                    const RiderLoadChannelView& channel =
                        channels[channelIndex];
                    const float labelBeginY = plotsBeginY
                        + static_cast<float>(channelIndex)
                            * (rowHeight + rowGap);
                    const float rowBeginY = labelBeginY + labelHeight;
                    const float rowEndY = labelBeginY + rowHeight;
                    const float plotHeight = rowEndY - rowBeginY;
                    const RiderLoadPlotRange valueRange =
                        fitRiderLoadPlotRange(
                            diagnostics.samples,
                            channel);
                    const double valueSpan =
                        valueRange.maximum - valueRange.minimum;

                    drawList->AddRectFilled(
                        ImVec2(plotBeginX, rowBeginY),
                        ImVec2(plotEndX, rowEndY),
                        transitionCanvasColor
                    );
                    drawList->AddRect(
                        ImVec2(plotBeginX, rowBeginY),
                        ImVec2(plotEndX, rowEndY),
                        ImGui::GetColorU32(ImGuiCol_Border)
                    );

                    char rangeLabel[64]{};
                    std::snprintf(
                        rangeLabel,
                        sizeof(rangeLabel),
                        "%+.3g to %+.3g %s",
                        valueRange.minimum,
                        valueRange.maximum,
                        channel.unit
                    );
                    drawList->AddText(
                        ImVec2(plotBeginX, labelBeginY),
                        channel.color,
                        channel.label
                    );
                    drawList->AddText(
                        ImVec2(
                            plotBeginX,
                            labelBeginY + ImGui::GetTextLineHeight() + 2.0F),
                        ImGui::GetColorU32(ImGuiCol_TextDisabled),
                        rangeLabel
                    );

                    if (valueRange.minimum < 0.0
                        && valueRange.maximum > 0.0)
                    {
                        const float zeroY = rowEndY
                            - static_cast<float>(
                                (0.0 - valueRange.minimum) / valueSpan)
                                * plotHeight;
                        drawList->AddLine(
                            ImVec2(plotBeginX, zeroY),
                            ImVec2(plotEndX, zeroY),
                            ImGui::GetColorU32(ImGuiCol_Separator)
                        );
                    }

                    points.clear();
                    for (const quantum::editor::RiderLoadDiagnosticSample&
                        sample : diagnostics.samples)
                    {
                        const double value = sample.*channel.value;
                        const float x = domainView.toPixel(std::clamp(
                            sample.localDistance,
                            0.0,
                            diagnostics.sectionLength));
                        const float y = rowEndY
                            - static_cast<float>(
                                (value - valueRange.minimum) / valueSpan)
                                * plotHeight;
                        points.push_back({x, y});
                    }

                    drawList->PushClipRect(
                        ImVec2(plotBeginX, rowBeginY),
                        ImVec2(plotEndX, rowEndY),
                        true
                    );
                    if (points.size() >= 2)
                    {
                        drawList->AddPolyline(
                            points.data(),
                            static_cast<int>(points.size()),
                            channel.color,
                            ImDrawFlags_None,
                            2.0F
                        );
                    }
                    else if (points.size() == 1)
                    {
                        drawList->AddCircleFilled(
                            points.front(),
                            2.5F,
                            channel.color
                        );
                    }
                    drawList->PopClipRect();
                }

                if (diagnostics.unreachableLocation
                        == RiderLoadUnreachableLocation::
                            WithinSelectedSection
                    && diagnostics.unreachableLocalDistance.has_value())
                {
                    const float stopX = domainView.toPixel(
                        *diagnostics.unreachableLocalDistance);
                    drawList->AddLine(
                        ImVec2(stopX, plotsBeginY),
                        ImVec2(stopX, canvasEnd.y),
                        ImGui::ColorConvertFloat4ToU32(
                            quantum::editor::palette::error),
                        2.0F
                    );
                }
                drawList->PopClipRect();
            }
            // Register the full plot extent so smaller windows scroll
            // vertically instead of shrinking the channels into slivers.
            ImGui::Dummy(canvasSize);
        }
        ImGui::EndChild();
        ImGui::End();
    }

    [[nodiscard]] const quantum::coaster::ProfileSegment*
    findProfileSegment(
        const quantum::coaster::ChannelProfile& profile,
        const std::uint32_t segmentId) noexcept
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
    }

    [[nodiscard]] ScalarProfileRowEdit drawSelectedProfileControls(
        const ProfileRowView& row,
        double* const selectedValueBuffer,
        const std::uint32_t selectedSegmentId,
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint,
        const float groupWidth)
    {
        ScalarProfileRowEdit edit;
        const quantum::coaster::ProfileSegment* focused =
            findProfileSegment(*row.profile, selectedSegmentId);
        if (focused == nullptr && !row.profile->segments.empty())
        {
            focused = &row.profile->segments.back();
        }

        ImGui::PushID(row.label);
        const bool sideBySide = groupWidth >= 460.0F;
        const float valueWidth = sideBySide ? 220.0F : groupWidth;
        const float typeWidth = sideBySide
            ? std::min(300.0F, groupWidth - valueWidth
                - ImGui::GetStyle().ItemSpacing.x) : groupWidth;

        const quantum::editor::ScalarProfileEndpoint numericEndpoint =
            selectedEndpoint
                == quantum::editor::ScalarProfileEndpoint::Begin
            ? quantum::editor::ScalarProfileEndpoint::Begin
            : quantum::editor::ScalarProfileEndpoint::End;
        ImGui::BeginGroup();
        ImGui::TextDisabled(
            numericEndpoint == quantum::editor::ScalarProfileEndpoint::Begin
                ? "Start rate"
                : "End rate"
        );
        ImGui::SetNextItemWidth(valueWidth);
        edit.valueEndEdited = ImGui::InputDouble(
            "##SelectedValue",
            selectedValueBuffer,
            0.25,
            1.0,
            "%.5f deg/m"
        );
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                numericEndpoint
                    == quantum::editor::ScalarProfileEndpoint::Begin
                ? "Selected marker angular rate (degrees per meter)"
                : "Selected segment end angular rate (degrees per meter)"
            );
        }

        ImGui::EndGroup();
        sameLineIfFits(typeWidth);
        ImGui::BeginGroup();
        ImGui::TextDisabled("Shape");
        ImGui::SetNextItemWidth(typeWidth);
        edit.transitionType = drawTransitionTypeCombo(
            "##TransitionType",
            focused != nullptr
                ? focused->transition.transitionType
                : quantum::math::TransitionType::Linear
        );
        ImGui::EndGroup();
        ImGui::PopID();
        return edit;
    }

    [[nodiscard]] ScalarProfileRowEdit drawScalarProfileCurve(
        ImDrawList* const drawList,
        const AuthoredDomainView& domainView,
        const ProfileRowView& row,
        const float plotBeginY,
        const float plotEndY,
        const ImU32 curveColor,
        const std::uint32_t selectedSegmentId,
        const quantum::editor::GraphValueRange valueRange,
        const bool activeChannel,
        ChannelPlotGeometry& plotGeometry)
    {
        ScalarProfileRowEdit edit;
        const ImU32 displayColor = activeChannel
            ? curveColor
            : (curveColor & ~IM_COL32_A_MASK)
                | (static_cast<ImU32>(150) << IM_COL32_A_SHIFT);
        const ScalarProfileMapping mapping{
            .domainView = domainView,
            .valueRange = valueRange,
            .pixelBeginY = plotBeginY,
            .pixelEndY = plotEndY
        };
        plotGeometry.mapping = mapping;
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
        std::vector<ImVec2>& points = plotGeometry.points;
        std::vector<std::size_t>& spanFirsts = plotGeometry.spanFirsts;
        std::vector<std::size_t>& spanCounts = plotGeometry.spanCounts;
        points.clear();
        spanFirsts.clear();
        spanCounts.clear();
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

        drawList->PushClipRect(
            ImVec2(domainView.pixelBegin, plotBeginY),
            ImVec2(domainView.pixelEnd, plotEndY),
            true
        );
        drawList->AddPolyline(
            points.data(),
            static_cast<int>(points.size()),
            displayColor,
            ImDrawFlags_None,
            activeChannel ? 2.75F : 1.75F
        );

        // Emphasize the selected segment by restriking its own slice of
        // the shared polyline slightly thicker.
        for (std::size_t index = 0; index < segmentTotal; ++index)
        {
            if (row.profile->segments[index].id != selectedSegmentId)
            {
                continue;
            }

            if (activeChannel)
            {
                drawList->AddPolyline(
                    points.data() + spanFirsts[index],
                    static_cast<int>(spanCounts[index]),
                    curveColor,
                    ImDrawFlags_None,
                    4.0F
                );

                const quantum::math::ScalarTransition& selected =
                    row.profile->segments[index].transition;
                const ImU32 boundaryColor =
                    (curveColor & ~IM_COL32_A_MASK)
                    | (static_cast<ImU32>(150) << IM_COL32_A_SHIFT);
                const std::array<double, 2> selectedBoundaries{
                    selected.domainBegin,
                    selected.domainEnd
                };
                for (const double boundary : selectedBoundaries)
                {
                    if (boundary <= domainView.domainBegin
                        || boundary >= domainView.domainEnd)
                    {
                        continue;
                    }
                    const float x = domainView.toPixel(boundary);
                    drawList->AddLine(
                        ImVec2(x, plotBeginY),
                        ImVec2(x, plotEndY),
                        boundaryColor,
                        1.5F
                    );
                    drawList->AddTriangleFilled(
                        ImVec2(x - 4.0F, plotBeginY),
                        ImVec2(x + 4.0F, plotBeginY),
                        ImVec2(x, plotBeginY + 6.0F),
                        curveColor
                    );
                }
            }
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
            if (activeChannel)
            {
                drawList->AddLine(
                    ImVec2(jointX, plotBeginY),
                    ImVec2(jointX, plotEndY),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled),
                    1.0F
                );
            }
        }
        drawList->PopClipRect();

        plotGeometry.handles = buildProfileHandles(mapping, *row.profile);
        plotGeometry.editGeometry = {
            mapping.valueRange,
            mapping.valueUnitsPerPixel(),
            distanceUnitsPerPixel
        };
        edit.rowGeometry = plotGeometry.editGeometry;

        return edit;
    }

    [[nodiscard]] quantum::editor::GraphValueRange fitProfileGraphRange(
        const quantum::coaster::ChannelProfile& profile,
        const quantum::editor::RateChannel channel)
    {
        std::vector<double> endpointValues;
        endpointValues.reserve(profile.segments.size() * 2);
        for (const quantum::coaster::ProfileSegment& segment :
            profile.segments)
        {
            endpointValues.push_back(segment.transition.valueBegin);
            endpointValues.push_back(segment.transition.valueEnd);
        }

        // Channel-specific flat-profile spans keep first drags practical
        // while each curve retains its own engineering transform. These are
        // view defaults only, never clamps on authored values.
        return quantum::editor::fitSymmetricGraphRange(
            endpointValues,
            quantum::editor::defaultGraphMagnitude(channel)
        );
    }

    [[nodiscard]] double plotLatticeStep(
        const double span, const float pixels)
    {
        // 1/2/5 decades give meaningful display-unit coordinates while
        // maintaining a quiet density as the plot or value range changes.
        const double intervals = std::clamp(
            std::floor(static_cast<double>(pixels) / 24.0), 1.0, 512.0);
        const double rawStep = span / intervals;
        const double decade = std::pow(10.0, std::floor(std::log10(rawStep)));
        const double fraction = rawStep / decade;
        return (fraction <= 1.0 ? 1.0 : fraction <= 2.0 ? 2.0
            : fraction <= 5.0 ? 5.0 : 10.0) * decade;
    }

    void drawScalarDotGrid(
        ImDrawList* const drawList,
        const ScalarProfileMapping& mapping)
    {
        const double distanceSpan = mapping.domainView.domainEnd
            - mapping.domainView.domainBegin;
        const double valueSpan = mapping.valueRange.maximum
            - mapping.valueRange.minimum;
        const float width = mapping.domainView.pixelEnd
            - mapping.domainView.pixelBegin;
        const float height = mapping.pixelEndY - mapping.pixelBeginY;
        if (!std::isfinite(distanceSpan) || distanceSpan <= 0.0
            || !std::isfinite(valueSpan) || valueSpan <= 0.0
            || width <= 0.0F || height <= 0.0F)
        {
            return;
        }

        const double distanceStep = plotLatticeStep(distanceSpan, width);
        const double valueStep = plotLatticeStep(valueSpan, height);
        if (!std::isfinite(distanceStep) || distanceStep <= 0.0
            || !std::isfinite(valueStep) || valueStep <= 0.0)
        {
            return;
        }
        const double firstDistance = std::ceil(
            mapping.domainView.domainBegin / distanceStep) * distanceStep;
        const double firstValue = std::ceil(
            mapping.valueRange.minimum / valueStep) * valueStep;
        const int columns = static_cast<int>(distanceSpan / distanceStep) + 1;
        const int rows = static_cast<int>(valueSpan / valueStep) + 1;
        const ImVec2 begin{mapping.domainView.pixelBegin, mapping.pixelBeginY};
        const ImVec2 end{mapping.domainView.pixelEnd, mapping.pixelEndY};
        const ImU32 dotColor = ImGui::ColorConvertFloat4ToU32(
            quantum::editor::palette::plotDot);

        // The mapping is in display units, with no knowledge of angular
        // channels or snapping. It can also draw future scalar G targets.
        drawList->PushClipRect(begin, end, true);
        for (int column = 0; column < columns; ++column)
        {
            const double distance = firstDistance + column * distanceStep;
            for (int row = 0; row < rows; ++row)
            {
                const double value = firstValue + row * valueStep;
                if (distance <= mapping.domainView.domainEnd
                    && value <= mapping.valueRange.maximum)
                {
                    drawList->AddCircleFilled(
                        mapping.toPixel(distance, value), 1.0F, dotColor, 4);
                }
            }
        }
        if (mapping.valueRange.minimum <= 0.0
            && mapping.valueRange.maximum >= 0.0)
        {
            const float zeroY = mapping.toPixel(
                mapping.domainView.domainBegin, 0.0).y;
            drawList->AddLine({begin.x, zeroY}, {end.x, zeroY},
                ImGui::ColorConvertFloat4ToU32(
                    quantum::editor::palette::plotReference));
        }
        drawList->PopClipRect();
    }

    void drawSharedValueGrid(
        ImDrawList* const drawList,
        const AuthoredDomainView& domainView,
        const float plotBeginY,
        const float plotEndY,
        const quantum::editor::GraphValueRange activeRange,
        const ImU32 activeColor)
    {
        const ImU32 disabledTextColor = ImGui::GetColorU32(
            ImGuiCol_TextDisabled
        );
        drawScalarDotGrid(drawList, {
            domainView,
            {activeRange.minimum * quantum::editor::degreesPerRadian,
                activeRange.maximum * quantum::editor::degreesPerRadian},
            plotBeginY, plotEndY});

        drawList->AddRect(
            ImVec2(domainView.pixelBegin, plotBeginY),
            ImVec2(domainView.pixelEnd, plotEndY),
            ImGui::GetColorU32(ImGuiCol_Border)
        );

        const int divisions = plotEndY - plotBeginY
                >= valueDivisionCount * (2.0F * ImGui::GetTextLineHeight() + 4.0F)
            ? valueDivisionCount : 2;
        for (int division = 0;
            division <= divisions;
            ++division)
        {
            const double normalized = static_cast<double>(division)
                / static_cast<double>(divisions);
            const float y = plotEndY
                - static_cast<float>(normalized)
                    * (plotEndY - plotBeginY);
            const double radians =
                quantum::editor::normalizedToGraphValue(
                    normalized,
                    activeRange
                );
            char label[32]{};
            std::snprintf(
                label,
                sizeof(label),
                "%+.3g",
                radians * quantum::editor::degreesPerRadian
            );
            drawList->AddText(
                ImVec2(domainView.pixelBegin + 4.0F,
                    std::min(y + 2.0F,
                        plotEndY - ImGui::GetTextLineHeight() - 2.0F)),
                division == divisions / 2
                    ? activeColor
                    : disabledTextColor,
                label
            );
        }
    }

    [[nodiscard]] double nearestPolylineDistanceSquared(
        const ImVec2 point,
        const std::span<const ImVec2> polyline) noexcept
    {
        if (polyline.empty())
        {
            return std::numeric_limits<double>::infinity();
        }
        if (polyline.size() == 1)
        {
            return static_cast<double>(squaredDistance(
                point,
                polyline.front()
            ));
        }

        double nearest = std::numeric_limits<double>::infinity();
        for (std::size_t index = 1; index < polyline.size(); ++index)
        {
            nearest = std::min(
                nearest,
                quantum::editor::squaredDistanceToLineSegment(
                    point.x,
                    point.y,
                    polyline[index - 1].x,
                    polyline[index - 1].y,
                    polyline[index].x,
                    polyline[index].y
                )
            );
        }
        return nearest;
    }

    void showRadiusLine(
        const char* const label,
        const quantum::editor::CurvatureDiagnostic& diagnostic)
    {
        if (diagnostic.radiusMeters.has_value())
        {
            ImGui::Text(
                "%s %.4f m",
                label,
                *diagnostic.radiusMeters
            );
        }
        else
        {
            ImGui::Text("%s Straight", label);
        }
    }

    [[nodiscard]] ScalarProfileRowEdit drawProfileSegmentMenu(
        const ProfileRowView& row,
        const std::uint32_t selectedSegmentId,
        double& contextMenuSplitDistance,
        const bool openRequested,
        const double hoverDistance,
        const quantum::editor::TransitionEditorInputSettings& inputSettings)
    {
        ScalarProfileRowEdit edit;
        const quantum::coaster::ProfileSegment* focused = nullptr;
        for (const quantum::coaster::ProfileSegment& segment :
            row.profile->segments)
        {
            if (segment.id == selectedSegmentId)
            {
                focused = &segment;
                break;
            }
        }

        ImGui::PushID(row.label);
        if (openRequested)
        {
            contextMenuSplitDistance = hoverDistance;
            ImGui::OpenPopup("##SegmentMenu");
        }

        if (ImGui::BeginPopup("##SegmentMenu"))
        {
            double targetDistance = contextMenuSplitDistance;
            if (inputSettings.distanceSnapEnabled
                && inputSettings.distanceSnapIncrement > 0.0)
            {
                targetDistance = snapToIncrement(
                    targetDistance,
                    inputSettings.distanceSnapIncrement
                );
            }
            const bool splitAllowed = focused != nullptr
                && focused->transition.domainBegin < targetDistance
                && targetDistance < focused->transition.domainEnd;

            if (focused != nullptr)
            {
                const quantum::editor::TransitionTypePreset* const preset =
                    quantum::editor::findTransitionTypePreset(
                        focused->transition.transitionType
                    );
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
            if (ImGui::Selectable(
                splitLabel,
                false,
                splitAllowed
                    ? ImGuiSelectableFlags_None
                    : ImGuiSelectableFlags_Disabled))
            {
                edit.splitRequested = true;
                edit.splitDistance = targetDistance;
            }

            quantum::editor::pushDestructiveStyle();
            if (ImGui::Selectable(
                "Remove Segment",
                false,
                row.profile->segments.size() > 1 && focused != nullptr
                    ? ImGuiSelectableFlags_None
                    : ImGuiSelectableFlags_Disabled))
            {
                edit.removeRequested = true;
            }
            quantum::editor::popDestructiveStyle();
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
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Info,
            "CFG",
            "transition editor input: normalGain=%.2f fineGain=%.2f "
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
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "CFG",
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
            fontPath.string().c_str(),
            quantum::editor::editorFontSize
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
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Info,
            "APP",
            "Loaded bundled Red Hat Mono SemiBold UI font: %s",
            fontPath.string().c_str()
        );
    }

    void checkVulkanResult(const VkResult result)
    {
        if (result != VK_SUCCESS)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Error,
                "VK:ImGui",
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
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Info,
            "CFG",
            "viewport settings orthographic=%d fov=%.1f "
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

        const bool visible = ImGui::BeginViewportSideBar(
            "##QuantumCommandArea",
            mainViewport,
            ImGuiDir_Up,
            height,
            windowFlags
        );

        if (visible)
        {
            ImGui::TextUnformatted("Commands");

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
        ImGui::Begin(trackWorkspaceWindowName);

        if (ImGui::BeginChild(
            "Section List",
            ImVec2(0.0F, 0.0F),
            ImGuiChildFlags_Borders))
        {
            ImGui::SeparatorText("Regions");
            const std::size_t sectionCount = track.sectionCount();
            for (std::size_t index = 0; index < sectionCount; ++index)
            {
                // Region terminology is the user-facing prototype wording;
                // the internal Section naming is unchanged.
                const bool isGeometry = track.section(index).kind
                    == quantum::coaster::RegionKind::Geometry;
                const bool isForceDriven =
                    quantum::coaster::isForceDrivenSection(track.section(index));
                const char* kindName = isGeometry
                    ? (isForceDriven ? "Force-Based" : "Circular Arc")
                    : "Profile";
                // Keep the legacy selectable ID, including its region/length
                // components, while changing only the visible kind name.
                const char* stableKindName = isGeometry
                    ? (isForceDriven
                        ? "Geometry / Force Driven" : "Geometry / Planar Arc")
                    : "Rate/Profile";
                char label[160]{};
                std::snprintf(
                    label,
                    sizeof(label),
                    "%llu. %s\nLength %.3g###Region %llu - %s (%.3g)",
                    static_cast<unsigned long long>(index + 1),
                    kindName,
                    quantum::coaster::sectionLength(track.section(index)),
                    static_cast<unsigned long long>(index + 1),
                    stableKindName,
                    quantum::coaster::sectionLength(track.section(index))
                );

                if (ImGui::Selectable(label, index == selectedIndex))
                {
                    edit.selectRequest = index;
                }
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
                {
                    ImGui::SetTooltip("Region %zu - %s\nLength %.6g coordinate units",
                        index + 1, kindName,
                        quantum::coaster::sectionLength(track.section(index)));
                }
            }

            if (selectedIndex < sectionCount && sectionLengthEdit != nullptr)
            {
                ImGui::Spacing();
                ImGui::SeparatorText("Selected region");

                ImGui::TextDisabled("Length");
                ImGui::SetNextItemWidth(-1.0F);
                if (ImGui::InputDouble(
                    "###Length",
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

                if (ImGui::TreeNode("Position & rotation"))
                {
                    const quantum::editor::RegionStations stations =
                        quantum::editor::computeRegionStations(
                            track,
                            selectedIndex
                        );
                    ImGui::PushStyleColor(ImGuiCol_Text,
                        quantum::editor::palette::textSecondary);
                    ImGui::PushTextWrapPos();
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
                        ImGui::TextUnformatted("Total rotation");
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
                    ImGui::PopTextWrapPos();
                    ImGui::PopStyleColor();
                    ImGui::TreePop();
                }
            }

            ImGui::Spacing();
            ImGui::SeparatorText("Region actions");
            const bool hasSelection = selectedIndex < sectionCount;
            if (ImGui::Button("Append Region..."))
            {
                regionCreateFlow.choicePending = true;
                regionCreateFlow.anchor = quantum::editor::RegionCreateAnchor::Append;
            }
            itemTooltip("Add a Profile or Circular Arc at the end of the track");
            sameLineIfFits(buttonWidth("More..."));
            if (ImGui::Button("More..."))
            {
                ImGui::OpenPopup("RegionActions");
            }
            itemTooltip("Prepend, insert, duplicate, reorder, or convert a region");
            if (ImGui::BeginPopup("RegionActions"))
            {
                if (ImGui::MenuItem("Prepend Region..."))
                {
                    regionCreateFlow.choicePending = true;
                    regionCreateFlow.anchor = quantum::editor::RegionCreateAnchor::Prepend;
                }
                if (ImGui::MenuItem("Insert After Selected...", nullptr, false, hasSelection))
                {
                    regionCreateFlow.choicePending = true;
                    regionCreateFlow.anchor = quantum::editor::RegionCreateAnchor::AfterSelected;
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Duplicate Selected", nullptr, false, hasSelection))
                {
                    edit.duplicateRequested = true;
                }
                if (ImGui::MenuItem("Move Up", nullptr, false,
                    hasSelection && selectedIndex > 0))
                {
                    edit.moveUpRequested = true;
                }
                if (ImGui::MenuItem("Move Down", nullptr, false,
                    hasSelection && selectedIndex + 1 < sectionCount))
                {
                    edit.moveDownRequested = true;
                }
                if (hasSelection)
                {
                    ImGui::Separator();
                    const auto& selected = track.section(selectedIndex);
                    const bool isGeometry = selected.kind
                        == quantum::coaster::RegionKind::Geometry
                        && !quantum::coaster::isForceDrivenSection(selected);
                    if (ImGui::MenuItem(isGeometry ? "Convert to Profile"
                        : "Convert to Circular Arc"))
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
                ImGui::EndPopup();
            }

            // Preserve the typed creation request/acceptance flow. Choices
            // wrap instead of squeezing full names into fixed thirds.
            if (regionCreateFlow.choicePending)
            {
                const char* const placement = regionCreateFlow.anchor
                        == quantum::editor::RegionCreateAnchor::Append
                    ? "Append" : regionCreateFlow.anchor
                        == quantum::editor::RegionCreateAnchor::Prepend
                    ? "Prepend" : "Insert after selected";
                ImGui::TextWrapped("%s: choose region type", placement);
                if (ImGui::Button("Profile###Rate/Profile"))
                {
                    edit.createdRegionKind = quantum::coaster::RegionKind::RateProfiles;
                }
                sameLineIfFits(buttonWidth("Circular Arc"));
                if (ImGui::Button("Circular Arc###Geometry / Planar Arc"))
                {
                    edit.createdRegionKind = quantum::coaster::RegionKind::Geometry;
                }
                sameLineIfFits(buttonWidth("Cancel"));
                if (ImGui::Button("Cancel"))
                {
                    regionCreateFlow.choicePending = false;
                }
            }

            ImGui::Spacing();
            ImGui::BeginDisabled(!hasSelection || sectionCount <= 1);
            quantum::editor::pushDestructiveStyle();
            if (ImGui::SmallButton("Remove Selected"))
            {
                edit.removeRequested = true;
            }
            quantum::editor::popDestructiveStyle();
            ImGui::EndDisabled();
            itemTooltip("Remove the selected region; the final region cannot be removed");

            ImGui::Spacing();
            ImGui::SeparatorText("Layout & connectivity");
            const auto currentMode = track.layoutMode();
            const auto status = quantum::coaster::computeLayoutStatus(
                currentMode, topology.kind);
            ImGui::PushTextWrapPos();
            ImGui::TextColored(
                status == quantum::coaster::LayoutStatus::CircuitIncomplete
                    ? quantum::editor::palette::warning
                    : quantum::editor::palette::textSecondary,
                "%s", quantum::coaster::layoutStatusLabel(status));
            ImGui::PopTextWrapPos();
            ImGui::SetNextItemWidth(-1.0F);
            if (ImGui::BeginCombo("##LayoutMode",
                currentMode == quantum::coaster::LayoutMode::Circuit
                    ? "Circuit" : "Shuttle"))
            {
                if (ImGui::Selectable("Circuit",
                    currentMode == quantum::coaster::LayoutMode::Circuit))
                {
                    edit.layoutModeChanged = quantum::coaster::LayoutMode::Circuit;
                }
                if (ImGui::Selectable("Shuttle",
                    currentMode == quantum::coaster::LayoutMode::Shuttle))
                {
                    edit.layoutModeChanged = quantum::coaster::LayoutMode::Shuttle;
                }
                ImGui::EndCombo();
            }
            itemTooltip("Authored layout mode: Circuit or Shuttle");
            if (topology.kind == quantum::coaster::TopologyKind::OpenLinear)
            {
                if (ImGui::TreeNode("Closure details"))
                {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                        quantum::editor::palette::textSecondary);
                    ImGui::PushTextWrapPos();
                    ImGui::Text("Closure gap: %.1f m", topology.diagnostics.positionalGap);
                    ImGui::Text("Tangent mismatch: %.1f deg",
                        topology.diagnostics.tangentMismatchDegrees);
                    ImGui::Text("Frame mismatch: %.1f deg",
                        topology.diagnostics.frameMismatchDegrees);
                    ImGui::PopTextWrapPos();
                    ImGui::PopStyleColor();
                    ImGui::TreePop();
                }
                if (currentMode == quantum::coaster::LayoutMode::Circuit
                    && ImGui::SmallButton("Complete Circuit..."))
                {
                    edit.completeCircuitRequested = true;
                }
            }
            if (ImGui::TreeNode("Planned properties"))
            {
                ImGui::BeginDisabled();
                ImGui::TextWrapped("Track style, mechanism segments, and track segment properties are not available yet.");
                ImGui::EndDisabled();
                ImGui::TreePop();
            }
        }
        ImGui::EndChild();

        ImGui::End();
        return edit;
    }

    void showSupportWorkspace()
    {
        ImGui::Begin(supportWorkspaceWindowName);

        ImGui::PushStyleColor(ImGuiCol_Text,
            quantum::editor::palette::textSecondary);
        ImGui::TextWrapped("Support tools are not available yet.");
        ImGui::PopStyleColor();
        if (ImGui::TreeNode("Planned tools"))
        {
            ImGui::BeginDisabled();
            ImGui::TextWrapped("Support prefabs\nFoundations\nRail connectors\nSupport settings");
            ImGui::Spacing();
            ImGui::TextWrapped("Prefab copy / define / paste (.qcwPrefab)");
            ImGui::EndDisabled();
            ImGui::TreePop();
        }

        ImGui::End();
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

        double snapDegrees = settings.snapIncrement
            * quantum::editor::degreesPerRadian;
        if (ImGui::InputDouble(
            "Value Snap (deg/m)",
            &snapDegrees,
            0.01,
            0.1,
            "%.4f"))
        {
            // The snapping math divides by this value, so it must stay
            // strictly positive and finite; typed garbage falls back to
            // the default increment.
            const double requested = snapDegrees
                * quantum::editor::radiansPerDegree;
            settings.snapIncrement =
                std::isfinite(requested) && requested > 0.0
                    ? requested
                    : 0.05 * quantum::editor::radiansPerDegree;
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
        std::array<quantum::editor::GraphValueRange,
            quantum::editor::rateChannelCount>& graphRanges,
        quantum::editor::RateChannel& activeChannel,
        std::optional<quantum::editor::RateChannel>& hoveredChannel,
        std::optional<quantum::editor::GraphMarkerId>& hoveredMarker,
        const quantum::editor::TransitionEditorInputSettings&
            inputSettings)
    {
        TransitionEditorEdit edit;
        ImGui::Begin("Transition Editor");

        for (const ProfileRowView& row : profileRows)
        {
            const std::size_t channelIndex = static_cast<std::size_t>(
                row.channel
            );
            if (!graphRanges[channelIndex].valid())
            {
                graphRanges[channelIndex] = fitProfileGraphRange(
                    *row.profile,
                    row.channel
                );
            }
        }

        const auto focusedEndpoint = [&](const std::size_t channelIndex)
        {
            const quantum::coaster::ChannelProfile& profile =
                *profileRows[channelIndex].profile;
            for (const quantum::coaster::ProfileSegment& segment :
                profile.segments)
            {
                if (segment.id != selectedSegmentIds[channelIndex])
                {
                    continue;
                }
                const bool useBegin = endpointSelections[channelIndex]
                    == quantum::editor::ScalarProfileEndpoint::Begin;
                return std::pair{
                    useBegin
                        ? segment.transition.domainBegin
                        : segment.transition.domainEnd,
                    useBegin
                        ? segment.transition.valueBegin
                        : segment.transition.valueEnd
                };
            }
            const quantum::math::ScalarTransition& tail =
                profile.segments.back().transition;
            return std::pair{tail.domainEnd, tail.valueEnd};
        };

        // Channel choice is the only persistent three-channel control row.
        // Precise value/shape controls below address the active channel's
        // selected semantic object, leaving the graph as the dominant area.
        const float controlsAvailableWidth = ImGui::GetContentRegionAvail().x;
        for (std::size_t rowIndex = 0;
            rowIndex < profileRows.size();
            ++rowIndex)
        {
            if (rowIndex > 0)
            {
                sameLineIfFits(buttonWidth("[Pitch]"));
            }
            const ProfileRowView& row = profileRows[rowIndex];
            const std::size_t channelIndex = static_cast<std::size_t>(
                row.channel
            );
            ImGui::PushID(row.label);
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                ImGui::ColorConvertU32ToFloat4(
                    profileCurveColors[channelIndex]
                )
            );
            char channelButtonLabel[48]{};
            std::snprintf(
                channelButtonLabel,
                sizeof(channelButtonLabel),
                row.channel == activeChannel
                    ? "[%s]###Channel"
                    : "%s###Channel",
                row.label
            );
            if (ImGui::SmallButton(channelButtonLabel))
            {
                activeChannel = row.channel;
            }
            ImGui::PopStyleColor();
            ImGui::PopID();
        }

        const std::size_t activeIndex = static_cast<std::size_t>(
            activeChannel
        );
        const char* const activeLabel = profileRows[activeIndex].label;

        const quantum::coaster::ProfileSegment* const focusedSegment =
            findProfileSegment(
                *profileRows[activeIndex].profile,
                selectedSegmentIds[activeIndex]
            );
        const quantum::editor::ScalarProfileEndpoint selectedEndpoint =
            endpointSelections[activeIndex];
        const char* const selectionKind = selectedEndpoint
                == quantum::editor::ScalarProfileEndpoint::Begin
            ? "begin marker"
            : selectedEndpoint
                == quantum::editor::ScalarProfileEndpoint::End
            ? "end marker"
            : "curve segment";
        if (focusedSegment != nullptr)
        {
            ImGui::PushTextWrapPos();
            ImGui::TextDisabled(
                "Selected: segment %u - %s [%.6g to %.6g m]",
                selectedSegmentIds[activeIndex],
                selectionKind,
                focusedSegment->transition.domainBegin,
                focusedSegment->transition.domainEnd
            );
            ImGui::PopTextWrapPos();
        }

        const ScalarProfileRowEdit controlEdit =
            drawSelectedProfileControls(
                profileRows[activeIndex],
                &valueEndBuffers[activeIndex],
                selectedSegmentIds[activeIndex],
                selectedEndpoint,
                controlsAvailableWidth
            );
        const quantum::editor::ScalarProfileEndpoint numericEndpoint =
            selectedEndpoint
                == quantum::editor::ScalarProfileEndpoint::Begin
            ? quantum::editor::ScalarProfileEndpoint::Begin
            : quantum::editor::ScalarProfileEndpoint::End;
        if (controlEdit.valueEndEdited)
        {
            if (std::isfinite(valueEndBuffers[activeIndex]))
            {
                const double valueRadians = quantum::editor::
                    angularRateDegreesToRadians(
                        valueEndBuffers[activeIndex]
                    );
                graphRanges[activeIndex] = quantum::editor::
                    expandGraphRangeToInclude(
                        graphRanges[activeIndex],
                        valueRadians
                    );
                edit.endpointValueEdit = {
                    .endpoint = numericEndpoint,
                    .value = valueRadians,
                    .continuous = false,
                    .channel = activeChannel,
                    .segmentId = selectedSegmentIds[activeIndex]
                };
            }
            else
            {
                // Core accepts every finite rate but never non-finite data.
                valueEndBuffers[activeIndex] =
                    focusedEndpoint(activeIndex).second
                        * quantum::editor::degreesPerRadian;
            }
        }
        if (controlEdit.transitionType.has_value())
        {
            edit.transitionType = TransitionEditorEdit::TypeChange{
                activeChannel,
                *controlEdit.transitionType,
                selectedSegmentIds[activeIndex]
            };
        }

        if (ImGui::SmallButton("Fit Y"))
        {
            graphRanges[activeIndex] = fitProfileGraphRange(
                *profileRows[activeIndex].profile,
                activeChannel
            );
        }
        itemTooltip("Fit the active channel's vertical range to its authored values");
        sameLineIfFits(buttonWidth("Y In"));
        if (ImGui::SmallButton("Y In"))
        {
            graphRanges[activeIndex] = quantum::editor::scaleGraphRange(
                graphRanges[activeIndex],
                0.8
            );
        }
        itemTooltip("Zoom in on the active channel's vertical range");
        sameLineIfFits(buttonWidth("Y Out"));
        if (ImGui::SmallButton("Y Out"))
        {
            graphRanges[activeIndex] = quantum::editor::scaleGraphRange(
                graphRanges[activeIndex],
                1.25
            );
        }
        itemTooltip("Zoom out on the active channel's vertical range");
        sameLineIfFits(ImGui::CalcTextSize("Y +/- 0.000e+00 deg/m").x);
        ImGui::TextDisabled(
            "Y +/- %.4g deg/m",
            graphRanges[activeIndex].magnitude()
                * quantum::editor::degreesPerRadian
        );

        sameLineIfFits(buttonWidth("Details..."));
        if (ImGui::SmallButton("Details..."))
        {
            ImGui::OpenPopup("ProfileDiagnostics");
        }
        itemTooltip("Read curvature, radius, and integrated rotation at the selected endpoint");
        ImGui::SetNextWindowSize(ImVec2(360.0F, 0.0F), ImGuiCond_Appearing);
        if (ImGui::BeginPopup("ProfileDiagnostics"))
        {
            ImGui::PushTextWrapPos();
            ImGui::SeparatorText("Selected endpoint");
            const auto [diagnosticDistance, diagnosticRate] =
                focusedEndpoint(activeIndex);
            if (activeChannel == quantum::editor::RateChannel::Roll)
            {
                const double integratedRollDegrees = quantum::editor::
                    computeChannelNetRotationDegrees(
                        *profileRows[activeIndex].profile
                    );
                ImGui::TextDisabled(
                    "Roll Rate @ %.4g m: %+.5f deg/m  "
                    "Integrated region roll: %+.3f deg",
                    diagnosticDistance,
                    quantum::editor::angularRateRadiansToDegrees(
                        diagnosticRate
                    ),
                    integratedRollDegrees
                );
            }
            else
            {
                const quantum::editor::CurvatureDiagnostic activeDiagnostic =
                    quantum::editor::curvatureDiagnosticFromRateRadians(
                        diagnosticRate
                    );
                ImGui::TextDisabled(
                    "%s Rate @ %.4g m: %+.5f deg/m  "
                    "Curvature %+.6f 1/m",
                    activeLabel,
                    diagnosticDistance,
                    activeDiagnostic.rateDegreesPerMeter,
                    activeDiagnostic.curvaturePerMeter
                );
                showRadiusLine("Radius", activeDiagnostic);
            }
            const double pitchRate = quantum::coaster::evaluateChannelProfile(
                *profileRows[static_cast<std::size_t>(
                    quantum::editor::RateChannel::Pitch)].profile,
                diagnosticDistance
            );
            const double yawRate = quantum::coaster::evaluateChannelProfile(
                *profileRows[static_cast<std::size_t>(
                    quantum::editor::RateChannel::Yaw)].profile,
                diagnosticDistance
            );
            const auto resultant = quantum::editor::
                resultantCurvatureDiagnostic(pitchRate, yawRate);
            ImGui::TextDisabled(
                "Local centerline curvature @ %.4g m: %.6f 1/m",
                diagnosticDistance,
                resultant.curvaturePerMeter
            );
            showRadiusLine("Resultant radius", resultant);
            ImGui::SeparatorText("Total rotation");
            for (const ProfileRowView& row : profileRows)
            {
                ImGui::TextDisabled("%s %+.3f deg", row.label,
                    quantum::editor::computeChannelNetRotationDegrees(*row.profile));
            }
            ImGui::PopTextWrapPos();
            ImGui::EndPopup();
        }

        constexpr ImGuiWindowFlags timelineWindowFlags =
            ImGuiWindowFlags_NoScrollbar
            | ImGuiWindowFlags_NoScrollWithMouse;

        if (ImGui::BeginChild(
            "##TransitionTimeline",
            ImVec2(0.0F, std::max(200.0F, ImGui::GetContentRegionAvail().y)),
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
            const float rulerHeight = 3.0F * ImGui::GetTextLineHeight() + 14.0F;
            const float plotBeginX = canvasBegin.x
                + endpointHandleRadius + 3.0F;
            const float plotEndX = canvasEnd.x
                - (endpointHandleRadius + 3.0F);
            const float profileBeginY = canvasBegin.y
                + rulerHeight
                + style.ItemSpacing.y;
            const float profileEndY = canvasEnd.y;
            const float plotBeginY = profileBeginY + 8.0F;
            const float plotEndY = profileEndY - 8.0F;

            if (plotEndX - plotBeginX < 48.0F
                || plotEndY - plotBeginY < 48.0F)
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
                const float rulerLineY = canvasBegin.y + 2.0F * textHeight + 4.0F;

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
                    rulerLineY
                );
                drawSharedValueGrid(
                    drawList,
                    domainView,
                    plotBeginY,
                    plotEndY,
                    graphRanges[activeIndex],
                    profileCurveColors[activeIndex]
                );

                std::array<std::optional<ScalarRowEditGeometry>,
                    quantum::editor::rateChannelCount> rowGeometries{};
                std::array<ChannelPlotGeometry,
                    quantum::editor::rateChannelCount> plotGeometries{};
                for (std::size_t rowIndex = 0;
                    rowIndex < profileRows.size();
                    ++rowIndex)
                {
                    const ProfileRowView& row = profileRows[rowIndex];
                    const std::size_t channelIndex =
                        static_cast<std::size_t>(row.channel);
                    const ScalarProfileRowEdit rowEdit =
                        drawScalarProfileCurve(
                        drawList,
                        domainView,
                        row,
                        plotBeginY,
                        plotEndY,
                        profileCurveColors[channelIndex],
                        selectedSegmentIds[channelIndex],
                        graphRanges[channelIndex],
                        row.channel == activeChannel,
                        plotGeometries[channelIndex]
                    );
                    rowGeometries[channelIndex] = rowEdit.rowGeometry;
                }

                const ImVec2 mousePosition = ImGui::GetIO().MousePos;
                const bool mouseInPlot = ImGui::IsWindowHovered()
                    && mousePosition.x >= plotBeginX
                    && mousePosition.x <= plotEndX
                    && mousePosition.y >= plotBeginY
                    && mousePosition.y <= plotEndY;
                const std::optional<quantum::editor::RateChannel>
                    previousHovered = hoveredChannel;
                const std::optional<quantum::editor::GraphMarkerId>
                    previousHoveredMarker = hoveredMarker;
                hoveredChannel.reset();
                hoveredMarker.reset();
                double hoverDistance = 0.0;
                std::optional<std::size_t> hoveredHandleIndex;

                if (mouseInPlot)
                {
                    std::array<quantum::editor::CurveHitCandidate,
                        quantum::editor::rateChannelCount> curveCandidates{};
                    std::vector<quantum::editor::MarkerHitCandidate>
                        markerCandidates;
                    markerCandidates.reserve(
                        profileRows[0].profile->segments.size()
                            + profileRows[1].profile->segments.size()
                            + profileRows[2].profile->segments.size()
                            + quantum::editor::rateChannelCount
                    );

                    for (std::size_t channelIndex = 0;
                        channelIndex < quantum::editor::rateChannelCount;
                        ++channelIndex)
                    {
                        curveCandidates[channelIndex] = {
                            static_cast<quantum::editor::RateChannel>(
                                channelIndex),
                            nearestPolylineDistanceSquared(
                                mousePosition,
                                plotGeometries[channelIndex].points
                            )
                        };

                        for (const RowHandlePoint& handle :
                            plotGeometries[channelIndex].handles)
                        {
                            markerCandidates.push_back({
                                {
                                    static_cast<quantum::editor::
                                        RateChannel>(channelIndex),
                                    handle.segmentId,
                                    handle.endpoint
                                },
                                static_cast<double>(squaredDistance(
                                    mousePosition,
                                    handle.position
                                ))
                            });
                        }
                    }

                    const auto markerHit = quantum::editor::chooseMarkerHit(
                        markerCandidates,
                        endpointHoverRadius,
                        activeChannel,
                        previousHoveredMarker
                    );
                    hoveredMarker = markerHit;
                    hoveredChannel = markerHit.has_value()
                        ? std::optional<quantum::editor::RateChannel>{
                            markerHit->channel}
                        : quantum::editor::chooseCurveHit(
                            curveCandidates,
                            curveHoverRadius,
                            activeChannel,
                            previousHovered
                        );

                    const double normalizedDistance = static_cast<double>(
                        (mousePosition.x - plotBeginX)
                            / (plotEndX - plotBeginX)
                    );
                    hoverDistance = quantum::editor::
                        normalizedToGraphDistance(
                            normalizedDistance,
                            domainView.domainBegin,
                            domainView.domainEnd
                        );

                    if (markerHit.has_value())
                    {
                        const std::size_t channelIndex =
                            static_cast<std::size_t>(markerHit->channel);
                        const auto& handles =
                            plotGeometries[channelIndex].handles;
                        for (std::size_t handleIndex = 0;
                            handleIndex < handles.size();
                            ++handleIndex)
                        {
                            const RowHandlePoint& handle =
                                handles[handleIndex];
                            if (handle.segmentId == markerHit->segmentId
                                && handle.endpoint == markerHit->endpoint)
                            {
                                hoveredHandleIndex = handleIndex;
                                hoverDistance = handle.domainValue;
                                break;
                            }
                        }
                    }
                }

                drawList->PushClipRect(
                    ImVec2(plotBeginX, plotBeginY),
                    ImVec2(plotEndX, plotEndY),
                    true
                );
                for (std::size_t channelIndex = 0;
                    channelIndex < quantum::editor::rateChannelCount;
                    ++channelIndex)
                {
                    const auto channel = static_cast<
                        quantum::editor::RateChannel>(channelIndex);
                    const bool channelHovered = hoveredChannel.has_value()
                        && *hoveredChannel == channel;
                    if (channel == activeChannel || channelHovered)
                    {
                        const auto& points =
                            plotGeometries[channelIndex].points;
                        drawList->AddPolyline(
                            points.data(),
                            static_cast<int>(points.size()),
                            profileCurveColors[channelIndex],
                            ImDrawFlags_None,
                            channel == activeChannel ? 3.25F : 2.75F
                        );
                    }
                }

                if (hoveredMarker.has_value()
                    && hoveredHandleIndex.has_value())
                {
                    const std::size_t channelIndex =
                        static_cast<std::size_t>(hoveredMarker->channel);
                    const RowHandlePoint& handle =
                        plotGeometries[channelIndex].handles[
                            *hoveredHandleIndex];
                    if (!handle.outer)
                    {
                        drawList->AddLine(
                            ImVec2(handle.position.x, plotBeginY),
                            ImVec2(handle.position.x, plotEndY),
                            profileCurveColors[channelIndex],
                            2.0F
                        );
                        drawList->AddTriangleFilled(
                            ImVec2(handle.position.x - 5.0F, plotBeginY),
                            ImVec2(handle.position.x + 5.0F, plotBeginY),
                            ImVec2(handle.position.x, plotBeginY + 7.0F),
                            profileCurveColors[channelIndex]
                        );
                    }
                }

                // Draw inactive open markers first and the active channel
                // last. Hit-testing remains semantic and independent of this
                // visual order, while coincident active points stay legible.
                for (int foregroundPass = 0; foregroundPass < 2;
                    ++foregroundPass)
                {
                    for (std::size_t channelIndex = 0;
                        channelIndex < quantum::editor::rateChannelCount;
                        ++channelIndex)
                    {
                        const auto channel = static_cast<quantum::editor::
                            RateChannel>(channelIndex);
                        const bool foreground = channel == activeChannel
                            || (hoveredChannel.has_value()
                                && *hoveredChannel == channel);
                        if (foreground != (foregroundPass != 0))
                        {
                            continue;
                        }
                        drawProfileHandles(
                            drawList,
                            plotGeometries[channelIndex].handles,
                            profileCurveColors[channelIndex],
                            channel,
                            selectedSegmentIds[channelIndex],
                            endpointSelections[channelIndex],
                            hoveredMarker,
                            channel == activeChannel
                        );
                    }
                }
                drawList->PopClipRect();

                if (hoveredChannel.has_value())
                {
                    const std::size_t channelIndex =
                        static_cast<std::size_t>(*hoveredChannel);
                    const ProfileRowView& hoveredRow =
                        profileRows[channelIndex];
                    const RowHandlePoint* hoveredHandle =
                        hoveredHandleIndex.has_value()
                        ? &plotGeometries[channelIndex].handles[
                            *hoveredHandleIndex]
                        : nullptr;
                    const double value = hoveredHandle != nullptr
                        ? hoveredHandle->value
                        : quantum::coaster::evaluateChannelProfile(
                            *hoveredRow.profile,
                            hoverDistance
                        );
                    const std::uint32_t hoveredSegmentId =
                        hoveredHandle != nullptr
                        ? hoveredHandle->segmentId
                        : quantum::coaster::findChannelSegmentAtDistance(
                            *hoveredRow.profile,
                            hoverDistance
                        );
                    const quantum::coaster::ProfileSegment* const
                        hoveredSegment = findProfileSegment(
                            *hoveredRow.profile,
                            hoveredSegmentId
                        );
                    ImGui::BeginTooltip();
                    ImGui::Text("%s Rate", hoveredRow.label);
                    ImGui::Text("Distance: %.6g m", hoverDistance);
                    ImGui::Text(
                        "Rate: %+.6f deg/m",
                        value * quantum::editor::degreesPerRadian
                    );
                    if (*hoveredChannel
                        != quantum::editor::RateChannel::Roll)
                    {
                        const auto diagnostic = quantum::editor::
                            curvatureDiagnosticFromRateRadians(value);
                        ImGui::Text(
                            "Curvature: %+.6f 1/m",
                            diagnostic.curvaturePerMeter
                        );
                        showRadiusLine("Radius:", diagnostic);
                    }
                    else
                    {
                        ImGui::Text(
                            "Integrated region roll: %+.3f deg",
                            quantum::editor::
                                computeChannelNetRotationDegrees(
                                    *hoveredRow.profile
                                )
                        );
                    }
                    ImGui::TextDisabled(
                        *hoveredChannel == activeChannel
                            ? "Active channel"
                            : "Click to make this the active channel"
                    );
                    if (hoveredSegment != nullptr)
                    {
                        const auto* const preset = quantum::editor::
                            findTransitionTypePreset(
                                hoveredSegment->transition.transitionType
                            );
                        ImGui::Text(
                            "Segment: %u (%s)",
                            hoveredSegment->id,
                            preset != nullptr ? preset->displayName.data()
                                : "Unsupported"
                        );
                    }
                    if (hoveredHandle != nullptr)
                    {
                        ImGui::TextDisabled(
                            hoveredHandle->outer
                                ? "Drag vertically: value (distance pinned)"
                                : "Drag vertically: value | horizontally: boundary"
                        );
                    }
                    ImGui::EndTooltip();

                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                    {
                        activeChannel = *hoveredChannel;
                        if (hoveredHandle != nullptr)
                        {
                            edit.click = TransitionEditorEdit::Click{
                                *hoveredChannel,
                                hoveredHandle->endpoint,
                                hoveredHandle->segmentId
                            };
                        }
                        else
                        {
                            edit.click = TransitionEditorEdit::Click{
                                *hoveredChannel,
                                quantum::editor::ScalarProfileEndpoint::None,
                                quantum::coaster::
                                    findChannelSegmentAtDistance(
                                        *profileRows[channelIndex].profile,
                                        hoverDistance
                                    )
                            };
                        }
                    }
                }

                for (std::size_t channelIndex = 0;
                    channelIndex < quantum::editor::rateChannelCount;
                    ++channelIndex)
                {
                    const auto channel = static_cast<
                        quantum::editor::RateChannel>(channelIndex);
                    const bool openMenu = hoveredChannel.has_value()
                        && *hoveredChannel == channel
                        && ImGui::IsMouseReleased(
                            ImGuiMouseButton_Right
                        );
                    const ScalarProfileRowEdit menuEdit =
                        drawProfileSegmentMenu(
                            profileRows[channelIndex],
                            selectedSegmentIds[channelIndex],
                            contextMenuSplitDistances[channelIndex],
                            openMenu,
                            hoverDistance,
                            inputSettings
                        );
                    if (menuEdit.splitRequested)
                    {
                        edit.splitRequest = TransitionEditorEdit::
                            SplitRequest{
                                channel,
                                selectedSegmentIds[channelIndex],
                                menuEdit.splitDistance
                            };
                    }
                    if (menuEdit.removeRequested)
                    {
                        edit.removeRequest = TransitionEditorEdit::
                            RemoveRequest{
                                channel,
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
                            const bool boundaryMovable = horizontalIntent
                                && draggedRow.profile != nullptr
                                && quantum::editor::
                                    profileBoundaryMoveBounds(
                                    *draggedRow.profile,
                                    dragSegmentIds[channelIndex],
                                    endpointDrags[channelIndex]
                                ).has_value();
                            axisLock = boundaryMovable
                                ? quantum::editor::DragAxisLock::Horizontal
                                : quantum::editor::DragAxisLock::Vertical;
                        }
                    }

                    if (axisLock
                        == quantum::editor::DragAxisLock::Horizontal)
                    {
                        const ProfileRowView& draggedRow =
                            profileRows[channelIndex];
                        const auto bounds = draggedRow.profile != nullptr
                            ? quantum::editor::profileBoundaryMoveBounds(
                                *draggedRow.profile,
                                dragSegmentIds[channelIndex],
                                endpointDrags[channelIndex]
                            )
                            : std::nullopt;
                        if (bounds.has_value())
                        {
                            // Screen X and authored distance grow together.
                            const std::optional<double> snapIncrement =
                                inputSettings.distanceSnapEnabled
                                    && inputSettings.distanceSnapIncrement
                                        > 0.0
                                ? std::optional<double>{
                                    inputSettings.distanceSnapIncrement}
                                : std::nullopt;
                            const double distance = quantum::editor::
                                proposeBoundaryDistanceDrag(
                                    anchor.distance,
                                    deltaX,
                                    geometry.distanceUnitsPerPixel,
                                    static_cast<double>(gainMultiplier),
                                    *bounds,
                                    snapIncrement
                                );

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
                        const std::optional<double> snapIncrement =
                            inputSettings.snapEnabled
                                && inputSettings.snapIncrement > 0.0
                            ? std::optional<double>{
                                inputSettings.snapIncrement}
                            : std::nullopt;
                        const double value = quantum::editor::
                            proposeMarkerValueDrag(
                                anchor.value,
                                deltaY,
                                geometry.unitsPerPixel,
                                static_cast<double>(gainMultiplier),
                                snapIncrement
                            );

                        anchor.value = value;
                        graphRanges[channelIndex] = quantum::editor::
                            expandGraphRangeToInclude(
                                graphRanges[channelIndex],
                                value
                            );

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

        ImGui::DockBuilderDockWindow(trackWorkspaceWindowName, leftId);
        ImGui::DockBuilderDockWindow("3D Viewport", centerId);
        ImGui::DockBuilderDockWindow(supportWorkspaceWindowName, rightId);
        ImGui::DockBuilderDockWindow("Transition Editor", bottomId);
        ImGui::DockBuilderDockWindow(
            riderLoadDiagnosticsWindowName,
            bottomId
        );
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
        selectedTrackAnchor_ = 0;
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
                    : profile.segments.back().transition.valueEnd
                        * degreesPerRadian;
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
            graphValueRanges_[channelIndex] = {};
        }
        activeRateChannel_ = RateChannel::Pitch;
        hoveredRateChannel_.reset();
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

        quantum::logging::logMessage(
            quantum::logging::LogLevel::Info,
            "APP",
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
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Error,
                "VK:ImGui",
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
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "%s btn=%d pos=(%.0f,%.0f)",
                    event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
                        ? "MOUSE_DOWN"
                        : "MOUSE_UP",
                    event.button.button,
                    event.button.x,
                    event.button.y
                );
                break;
            case SDL_EVENT_WINDOW_RESIZED:
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Debug,
                    "INP",
                    "RESIZED %dx%d",
                    event.window.data1,
                    event.window.data2
                );
                break;
            case SDL_EVENT_WINDOW_MAXIMIZED:
                quantum::logging::logMessage(
                    quantum::logging::LogLevel::Debug,
                    "INP",
                    "MAXIMIZED"
                );
                break;
            case SDL_EVENT_WINDOW_MINIMIZED:
                quantum::logging::logMessage(
                    quantum::logging::LogLevel::Debug,
                    "INP",
                    "MINIMIZED"
                );
                break;
            case SDL_EVENT_WINDOW_RESTORED:
                quantum::logging::logMessage(
                    quantum::logging::LogLevel::Debug,
                    "INP",
                    "RESTORED"
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

    bool EditorUi::updateStartPoseManipulation(
        const bool viewportHovered,
        const float imageWidth,
        const float imageHeight)
    {
        ImGuiIO& io = ImGui::GetIO();

        if (startPoseManipulation_.has_value())
        {
            StartPoseManipulation& manipulation =
                *startPoseManipulation_;
            if (io.AppFocusLost)
            {
                startPoseManipulation_.reset();
                return true;
            }

            // Finish on the frame after release so the final mouse position
            // has gone through candidate/commit. Rejection cancels this state
            // before a success summary can be emitted.
            if (manipulation.released)
            {
                if (manipulation.changed)
                {
                    const coaster::AuthoredStartPose& pose =
                        authoredTrack_->startPose();
                    quantum::logging::logMessagef(
                        quantum::logging::LogLevel::Info,
                        "EDIT",
                        "completed start-pose manipulation mode=%s axis=%s "
                        "position=(%.6f,%.6f,%.6f) "
                        "orientation=(%.9f,%.9f,%.9f,%.9f)",
                        startPoseModeName(manipulation.mode),
                        startPoseAxisName(manipulation.axis),
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    );
                }
                startPoseManipulation_.reset();
                return true;
            }

            // Motion and button-up may arrive in the same SDL/ImGui frame.
            // Consume that final position even though the button is now up.
            manipulation.released =
                !ImGui::IsMouseDown(ImGuiMouseButton_Left);
            coaster::AuthoredStartPose candidate;
            if (manipulation.mode == StartPoseTransformMode::Move)
            {
                const double mouseDeltaX = static_cast<double>(
                    io.MousePos.x - manipulation.mouseStart.x);
                const double mouseDeltaY = static_cast<double>(
                    io.MousePos.y - manipulation.mouseStart.y);
                const double projectedPixels =
                    mouseDeltaX * manipulation.screenDirectionX
                    + mouseDeltaY * manipulation.screenDirectionY;
                candidate = translateStartPose(
                    manipulation.initialPose,
                    manipulation.axis,
                    projectedPixels * manipulation.worldUnitsPerPixel
                );
            }
            else
            {
                const auto projectedOrigin = projectViewportPoint(
                    viewportCamera_,
                    manipulation.initialPose.position,
                    viewportAspectRatio_
                );
                if (!projectedOrigin.has_value())
                {
                    return true;
                }
                const ImVec2 imageMinimum = ImGui::GetItemRectMin();
                const ImVec2 origin{
                    imageMinimum.x + static_cast<float>(
                        projectedOrigin->normalizedPosition.x) * imageWidth,
                    imageMinimum.y + static_cast<float>(
                        projectedOrigin->normalizedPosition.y) * imageHeight
                };
                const double mouseAngle = std::atan2(
                    static_cast<double>(io.MousePos.y - origin.y),
                    static_cast<double>(io.MousePos.x - origin.x)
                );
                const double angleDelta = std::remainder(
                    mouseAngle - manipulation.initialMouseAngle,
                    2.0 * piRadians
                );
                candidate = rotateStartPose(
                    manipulation.initialPose,
                    manipulation.axis,
                    angleDelta
                );
            }

            if (candidate != manipulation.candidatePose)
            {
                manipulation.candidatePose = candidate;
                manipulation.changed = true;
                startPoseEdit_ = StartPoseEdit{candidate, true};
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "EDIT",
                    "start-pose candidate mode=%s axis=%s "
                    "position=(%.6f,%.6f,%.6f)",
                    startPoseModeName(manipulation.mode),
                    startPoseAxisName(manipulation.axis),
                    candidate.position.x,
                    candidate.position.y,
                    candidate.position.z
                );
            }
            return true;
        }

        if (!viewportHovered
            || !viewportSettings_.anchorsVisible
            || io.AppFocusLost
            || cameraGesture_ != CameraGesture::None
            || firstActiveEndpoint(endpointDrags_)
                != ScalarProfileEndpoint::None
            || !ImGui::IsMouseClicked(ImGuiMouseButton_Left)
            || authoredTrack_ == nullptr
            || centerlineVisualization_ == nullptr
            || centerlineVisualization_->anchors.empty()
            || !isViewportTrackAnchorEditable(
                centerlineVisualization_->anchors.front().kind)
            || !selectedTrackAnchor_.has_value()
            || *selectedTrackAnchor_ != 0
            || imageWidth <= 0.0F
            || imageHeight <= 0.0F)
        {
            return false;
        }

        const coaster::AuthoredStartPose& pose = authoredTrack_->startPose();
        const auto projectedOrigin = projectViewportPoint(
            viewportCamera_,
            pose.position,
            viewportAspectRatio_
        );
        if (!projectedOrigin.has_value())
        {
            return false;
        }

        const ImVec2 imageMinimum = ImGui::GetItemRectMin();
        const ImVec2 origin{
            imageMinimum.x + static_cast<float>(
                projectedOrigin->normalizedPosition.x) * imageWidth,
            imageMinimum.y + static_cast<float>(
                projectedOrigin->normalizedPosition.y) * imageHeight
        };
        const ImVec2 mouse = io.MousePos;
        std::optional<StartPoseTransformAxis> pickedAxis;
        double pickedMetric = std::numeric_limits<double>::max();
        double pickedDirectionX = 1.0;
        double pickedDirectionY = 0.0;
        double pickedWorldUnitsPerPixel = 0.0;

        if (startPoseTransformMode_ == StartPoseTransformMode::Move)
        {
            const double scaleDistance = viewportCamera_.projection()
                    == ViewportProjection::Perspective
                ? glm::length(pose.position - viewportCamera_.position())
                : viewportCamera_.distance();
            const double baseWorldUnitsPerPixel = 2.0 * scaleDistance
                * std::tan(0.5 * viewportCamera_.verticalFieldOfView())
                / static_cast<double>(imageHeight);
            const double axisLength = startPoseMoveHandlePixels
                * baseWorldUnitsPerPixel;
            constexpr std::array<ImVec2, 3> fallbackDirections{
                ImVec2{1.0F, 0.0F},
                ImVec2{-0.7071F, 0.7071F},
                ImVec2{0.0F, -1.0F}
            };

            for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                const auto axis = static_cast<StartPoseTransformAxis>(
                    axisIndex);
                const auto projectedEnd = projectViewportPoint(
                    viewportCamera_,
                    pose.position + axisLength * startPoseWorldAxis(axis),
                    viewportAspectRatio_
                );
                double directionX = fallbackDirections[axisIndex].x;
                double directionY = fallbackDirections[axisIndex].y;
                double screenLength = startPoseMoveHandlePixels;
                double worldUnitsPerPixel = baseWorldUnitsPerPixel;
                if (projectedEnd.has_value())
                {
                    const ImVec2 projectedScreenEnd{
                        imageMinimum.x + static_cast<float>(
                            projectedEnd->normalizedPosition.x) * imageWidth,
                        imageMinimum.y + static_cast<float>(
                            projectedEnd->normalizedPosition.y) * imageHeight
                    };
                    const double dx = projectedScreenEnd.x - origin.x;
                    const double dy = projectedScreenEnd.y - origin.y;
                    const double projectedLength = std::hypot(dx, dy);
                    if (projectedLength >= 10.0)
                    {
                        directionX = dx / projectedLength;
                        directionY = dy / projectedLength;
                        screenLength = projectedLength;
                        worldUnitsPerPixel = axisLength / projectedLength;
                    }
                }

                const ImVec2 begin{
                    origin.x + static_cast<float>(8.0 * directionX),
                    origin.y + static_cast<float>(8.0 * directionY)
                };
                const ImVec2 end{
                    origin.x + static_cast<float>(screenLength * directionX),
                    origin.y + static_cast<float>(screenLength * directionY)
                };
                const double metric = pointSegmentDistanceSquared(
                    mouse,
                    begin,
                    end
                );
                if (metric <= startPoseMoveHitRadiusPixels
                        * startPoseMoveHitRadiusPixels
                    && metric < pickedMetric)
                {
                    pickedAxis = axis;
                    pickedMetric = metric;
                    pickedDirectionX = directionX;
                    pickedDirectionY = directionY;
                    pickedWorldUnitsPerPixel = worldUnitsPerPixel;
                }
            }
        }
        else
        {
            const double radius = std::hypot(
                static_cast<double>(mouse.x - origin.x),
                static_cast<double>(mouse.y - origin.y)
            );
            for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                const double metric = std::abs(
                    radius - startPoseRotateRadiiPixels[axisIndex]
                );
                if (metric <= startPoseRotateHitRadiusPixels
                    && metric < pickedMetric)
                {
                    pickedAxis = static_cast<StartPoseTransformAxis>(
                        axisIndex);
                    pickedMetric = metric;
                }
            }
        }

        if (!pickedAxis.has_value())
        {
            return false;
        }

        StartPoseManipulation manipulation;
        manipulation.mode = startPoseTransformMode_;
        manipulation.axis = *pickedAxis;
        manipulation.initialPose = pose;
        manipulation.candidatePose = pose;
        manipulation.mouseStart = {mouse.x, mouse.y};
        manipulation.screenDirectionX = pickedDirectionX;
        manipulation.screenDirectionY = pickedDirectionY;
        manipulation.worldUnitsPerPixel = pickedWorldUnitsPerPixel;
        manipulation.initialMouseAngle = std::atan2(
            static_cast<double>(mouse.y - origin.y),
            static_cast<double>(mouse.x - origin.x)
        );
        startPoseManipulation_ = manipulation;

        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Trace,
            "EDIT",
            "start-pose manipulation begin mode=%s axis=%s",
            startPoseModeName(manipulation.mode),
            startPoseAxisName(manipulation.axis)
        );
        return true;
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

        const bool startPoseManipulationCaptured =
            updateStartPoseManipulation(
                viewportHovered,
                logicalWidth,
                logicalHeight
            );

        if (io.AppFocusLost)
        {
            cameraGesture_ = CameraGesture::None;
        }

        if (firstActiveEndpoint(endpointDrags_)
            != ScalarProfileEndpoint::None)
        {
            cameraGesture_ = CameraGesture::None;
        }

        // ImGui owns the raw SDL event stream. Treat a left click as a
        // viewport action only when the submitted viewport Image itself is
        // hovered; popup/modal blocking and clicks on the toolbar therefore
        // remain UI input even while WantCaptureMouse is true for the editor.
        if (viewportHovered
            && !io.AppFocusLost
            && !startPoseManipulationCaptured
            && cameraGesture_ == CameraGesture::None
            && firstActiveEndpoint(endpointDrags_)
                == ScalarProfileEndpoint::None
            && ImGui::IsMouseClicked(ImGuiMouseButton_Left)
            && centerlineVisualization_ != nullptr)
        {
            const ImVec2 imageMinimum = ImGui::GetItemRectMin();
            const ImVec2 imageMaximum = ImGui::GetItemRectMax();
            const double imageWidth = static_cast<double>(
                imageMaximum.x - imageMinimum.x
            );
            const double imageHeight = static_cast<double>(
                imageMaximum.y - imageMinimum.y
            );

            if (imageWidth > 0.0 && imageHeight > 0.0)
            {
                const double normalizedX =
                    (static_cast<double>(io.MousePos.x)
                        - imageMinimum.x) / imageWidth;
                const double normalizedY =
                    (static_cast<double>(io.MousePos.y)
                        - imageMinimum.y) / imageHeight;
                const ViewportRay ray = viewportCamera_.viewportRay(
                    normalizedX,
                    normalizedY,
                    aspectRatio
                );
                const auto anchorHit = viewportSettings_.anchorsVisible
                    ? pickViewportTrackAnchor(
                        centerlineVisualization_->anchors,
                        viewportCamera_,
                        {normalizedX, normalizedY},
                        pixelWidth,
                        pixelHeight
                    )
                    : std::nullopt;

                // Semantic anchors have priority inside their marker radius.
                // Only an anchor miss falls through to reference-curve
                // picking, so a shared boundary cannot be hidden by its line.
                if (anchorHit.has_value())
                {
                    selectTrackAnchor(anchorHit->anchorIndex);
                }
                else
                {
                    const auto trackHit = pickViewportSection(
                        *centerlineVisualization_,
                        viewportCamera_,
                        ray,
                        pixelHeight,
                        visibleTrackCurveMask(viewportSettings_)
                    );

                    // Empty viewport space deliberately preserves selection,
                    // matching the Section List's always-selected behavior.
                    if (trackHit.has_value())
                    {
                        selectSection(trackHit->sectionIndex);
                    }
                }
            }
        }

        if (cameraGesture_ == CameraGesture::None
            && firstActiveEndpoint(endpointDrags_)
                == ScalarProfileEndpoint::None
            && viewportHovered
            && !startPoseManipulationCaptured)
        {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "CAMERA_GESTURE_START Orbit MousePos=(%.0f,%.0f)",
                    io.MousePos.x,
                    io.MousePos.y
                );
                cameraGesture_ = CameraGesture::Orbit;
            }
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "CAMERA_GESTURE_START Pan MousePos=(%.0f,%.0f)",
                    io.MousePos.x,
                    io.MousePos.y
                );
                cameraGesture_ = CameraGesture::Pan;
            }
        }

        if (cameraGesture_ == CameraGesture::Orbit)
        {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
            {
                quantum::logging::logMessage(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "CAMERA_GESTURE_END Orbit"
                );
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
                quantum::logging::logMessage(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "CAMERA_GESTURE_END Pan"
                );
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

        if (viewportHovered && !startPoseManipulationCaptured)
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

        if (const CenterlineSectionSlice* const selectedSlice =
            selectedSectionSlice())
        {
            vulkan.setTrackCurveHighlight(
                selectedSlice->firstVertex,
                selectedSlice->vertexCount
            );
        }
        else
        {
            vulkan.setTrackCurveHighlight(0, 0);
        }

        vulkan.setViewportViewProjection(
            viewportCamera_.viewProjection(aspectRatio)
        );
        drawViewportTrackAnchors();
    }

    void EditorUi::drawViewportTrackAnchors()
    {
        if (!viewportSettings_.anchorsVisible
            || centerlineVisualization_ == nullptr
            || centerlineVisualization_->anchors.empty())
        {
            return;
        }

        const ImVec2 imageMinimum = ImGui::GetItemRectMin();
        const ImVec2 imageMaximum = ImGui::GetItemRectMax();
        const float imageWidth = imageMaximum.x - imageMinimum.x;
        const float imageHeight = imageMaximum.y - imageMinimum.y;
        if (imageWidth <= 0.0F || imageHeight <= 0.0F)
        {
            return;
        }

        ImDrawList* const drawList = ImGui::GetWindowDrawList();
        const auto screenPosition = [imageMinimum, imageWidth, imageHeight](
            const ViewportProjectedPoint& point)
        {
            return ImVec2{
                imageMinimum.x + static_cast<float>(
                    point.normalizedPosition.x) * imageWidth,
                imageMinimum.y + static_cast<float>(
                    point.normalizedPosition.y) * imageHeight
            };
        };
        const auto visibleProjection = [](const ViewportProjectedPoint& point)
        {
            return point.normalizedPosition.x >= 0.0
                && point.normalizedPosition.x <= 1.0
                && point.normalizedPosition.y >= 0.0
                && point.normalizedPosition.y <= 1.0;
        };

        constexpr ImU32 markerOutline = IM_COL32(8, 10, 14, 245);
        constexpr ImU32 markerFill = IM_COL32(68, 205, 230, 235);
        constexpr ImU32 selectedFill = IM_COL32(255, 190, 48, 255);

        const auto drawMarker = [&](const ViewportTrackAnchor& anchor,
            const bool selected)
        {
            const auto projected = projectViewportPoint(
                viewportCamera_,
                anchor.position,
                viewportAspectRatio_
            );
            if (!projected.has_value() || !visibleProjection(*projected))
            {
                return;
            }

            const ImVec2 center = screenPosition(*projected);
            const float radius = selected ? 7.0F : 5.0F;
            drawList->AddCircleFilled(center, radius + 2.0F, markerOutline, 20);
            drawList->AddCircleFilled(
                center,
                radius,
                selected ? selectedFill : markerFill,
                20
            );
            if (selected)
            {
                drawList->AddCircle(
                    center,
                    radius + 3.5F,
                    IM_COL32(255, 255, 255, 235),
                    24,
                    1.5F
                );
            }
        };

        for (const ViewportTrackAnchor& anchor :
            centerlineVisualization_->anchors)
        {
            if (!selectedTrackAnchor_.has_value()
                || anchor.anchorIndex != *selectedTrackAnchor_)
            {
                drawMarker(anchor, false);
            }
        }

        const ViewportTrackAnchor* selectedAnchor = nullptr;
        if (selectedTrackAnchor_.has_value())
        {
            for (const ViewportTrackAnchor& anchor :
                centerlineVisualization_->anchors)
            {
                if (anchor.anchorIndex == *selectedTrackAnchor_)
                {
                    selectedAnchor = &anchor;
                    break;
                }
            }
        }

        if (selectedAnchor == nullptr)
        {
            return;
        }

        ViewportTrackAnchor displayedAnchor = *selectedAnchor;
        if (displayedAnchor.kind == ViewportTrackAnchorKind::Start
            && startPoseManipulation_.has_value())
        {
            const coaster::AuthoredStartPose& candidate =
                startPoseManipulation_->candidatePose;
            const geometry::CurveFrame frame =
                coaster::startPoseRiderFrame(candidate);
            displayedAnchor.position = candidate.position;
            displayedAnchor.forward = frame.tangent;
            displayedAnchor.lateral = frame.lateral;
            displayedAnchor.up = frame.up;
        }

        const auto projectedOrigin = projectViewportPoint(
            viewportCamera_,
            displayedAnchor.position,
            viewportAspectRatio_
        );
        if (!projectedOrigin.has_value()
            || !visibleProjection(*projectedOrigin))
        {
            return;
        }

        // A fixed apparent axis size makes the exact rider frame readable at
        // both track-wide and close inspection distances. This is an
        // orientation indicator, not an interactive gizmo.
        const double scaleDistance = viewportCamera_.projection()
                == ViewportProjection::Perspective
            ? glm::length(
                displayedAnchor.position - viewportCamera_.position())
            : viewportCamera_.distance();
        const double worldUnitsPerPixel = 2.0 * scaleDistance
            * std::tan(0.5 * viewportCamera_.verticalFieldOfView())
            / static_cast<double>(imageHeight);
        const double axisLength = 34.0 * worldUnitsPerPixel;
        const ImVec2 origin = screenPosition(*projectedOrigin);

        const auto drawAxis = [&](const glm::dvec3& axis,
            const ImU32 color,
            const char* const label)
        {
            const auto projectedEnd = projectViewportPoint(
                viewportCamera_,
                displayedAnchor.position + axisLength * axis,
                viewportAspectRatio_
            );
            if (!projectedEnd.has_value()
                || !visibleProjection(*projectedEnd))
            {
                return;
            }

            const ImVec2 end = screenPosition(*projectedEnd);
            drawList->AddLine(origin, end, markerOutline, 4.0F);
            drawList->AddLine(origin, end, color, 2.25F);
            drawList->AddCircleFilled(end, 3.0F, color, 12);
            drawList->AddText(
                ImVec2{end.x + 4.0F, end.y - 7.0F},
                color,
                label
            );
        };

        drawAxis(displayedAnchor.forward,
            IM_COL32(255, 92, 76, 255), "T");
        drawAxis(displayedAnchor.lateral,
            IM_COL32(90, 224, 138, 255), "L");
        drawAxis(displayedAnchor.up,
            IM_COL32(102, 154, 255, 255), "U");

        if (isViewportTrackAnchorEditable(displayedAnchor.kind))
        {
            constexpr std::array<ImU32, 3> axisColors{
                IM_COL32(255, 82, 72, 255),
                IM_COL32(76, 220, 116, 255),
                IM_COL32(76, 138, 255, 255)
            };
            constexpr std::array<ImVec2, 3> fallbackDirections{
                ImVec2{1.0F, 0.0F},
                ImVec2{-0.7071F, 0.7071F},
                ImVec2{0.0F, -1.0F}
            };

            if (startPoseTransformMode_ == StartPoseTransformMode::Move)
            {
                const double handleAxisLength = startPoseMoveHandlePixels
                    * worldUnitsPerPixel;
                for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex)
                {
                    const auto axis = static_cast<StartPoseTransformAxis>(
                        axisIndex);
                    double directionX = fallbackDirections[axisIndex].x;
                    double directionY = fallbackDirections[axisIndex].y;
                    double screenLength = startPoseMoveHandlePixels;
                    const auto projectedEnd = projectViewportPoint(
                        viewportCamera_,
                        displayedAnchor.position + handleAxisLength
                            * startPoseWorldAxis(axis),
                        viewportAspectRatio_
                    );
                    if (projectedEnd.has_value())
                    {
                        const ImVec2 projectedScreenEnd =
                            screenPosition(*projectedEnd);
                        const double dx = projectedScreenEnd.x - origin.x;
                        const double dy = projectedScreenEnd.y - origin.y;
                        const double projectedLength = std::hypot(dx, dy);
                        if (projectedLength >= 10.0)
                        {
                            directionX = dx / projectedLength;
                            directionY = dy / projectedLength;
                            screenLength = projectedLength;
                        }
                    }

                    const ImVec2 begin{
                        origin.x + static_cast<float>(8.0 * directionX),
                        origin.y + static_cast<float>(8.0 * directionY)
                    };
                    const ImVec2 end{
                        origin.x + static_cast<float>(
                            screenLength * directionX),
                        origin.y + static_cast<float>(
                            screenLength * directionY)
                    };
                    const bool active = startPoseManipulation_.has_value()
                        && startPoseManipulation_->axis == axis;
                    const ImU32 color = active
                        ? IM_COL32(255, 220, 96, 255)
                        : axisColors[axisIndex];
                    drawList->AddLine(origin, end, markerOutline, 6.0F);
                    drawList->AddLine(
                        begin,
                        end,
                        color,
                        active ? 4.0F : 3.0F
                    );
                    drawList->AddCircleFilled(end, 5.0F, color, 16);
                    drawList->AddText(
                        ImVec2{end.x + 5.0F, end.y - 8.0F},
                        color,
                        startPoseAxisName(axis)
                    );
                }
            }
            else
            {
                for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex)
                {
                    const auto axis = static_cast<StartPoseTransformAxis>(
                        axisIndex);
                    const bool active = startPoseManipulation_.has_value()
                        && startPoseManipulation_->axis == axis;
                    const ImU32 color = active
                        ? IM_COL32(255, 220, 96, 255)
                        : axisColors[axisIndex];
                    const float radius =
                        startPoseRotateRadiiPixels[axisIndex];
                    drawList->AddCircle(
                        origin,
                        radius,
                        markerOutline,
                        48,
                        5.0F
                    );
                    drawList->AddCircle(
                        origin,
                        radius,
                        color,
                        48,
                        active ? 4.0F : 2.5F
                    );
                    drawList->AddText(
                        ImVec2{origin.x + radius + 4.0F,
                               origin.y - 7.0F},
                        color,
                        startPoseAxisName(axis)
                    );
                }
            }
        }

        drawMarker(displayedAnchor, true);

        char anchorLabel[32]{};
        std::snprintf(
            anchorLabel,
            sizeof(anchorLabel),
            "Anchor %zu",
            displayedAnchor.anchorIndex
        );
        drawList->AddText(
            ImVec2{origin.x + 12.0F, origin.y + 8.0F},
            IM_COL32(255, 232, 162, 255),
            anchorLabel
        );

        const char* status = nullptr;
        switch (displayedAnchor.kind)
        {
        case ViewportTrackAnchorKind::Start:
            status = "Track Start \xe2\x80\x94 editable";
            break;
        case ViewportTrackAnchorKind::Interior:
            status = "Shared region boundary - constrained editing not implemented yet";
            break;
        case ViewportTrackAnchorKind::End:
            status = "Final boundary - no terminal pose constraint; read-only";
            break;
        }

        const float statusWrapWidth = std::max(1.0F,
            imageMaximum.x - imageMinimum.x - 34.0F);
        const ImVec2 textSize = ImGui::CalcTextSize(
            status, nullptr, false, statusWrapWidth);
        const ImVec2 statusMinimum{
            imageMinimum.x + 9.0F,
            imageMaximum.y - textSize.y - 17.0F
        };
        const ImVec2 statusMaximum{
            statusMinimum.x + textSize.x + 16.0F,
            statusMinimum.y + textSize.y + 8.0F
        };
        drawList->AddRectFilled(
            statusMinimum,
            statusMaximum,
            ImGui::ColorConvertFloat4ToU32(palette::surfaceRaised),
            4.0F
        );
        drawList->AddText(
            ImGui::GetFont(), ImGui::GetFontSize(),
            ImVec2{statusMinimum.x + 8.0F, statusMinimum.y + 4.0F},
            ImGui::ColorConvertFloat4ToU32(palette::text),
            status, nullptr, statusWrapWidth
        );
    }

    void EditorUi::showViewportViewMenuItems()
    {
        const auto presetItem = [this](const char* const label,
            const ViewportCameraPreset preset)
        {
            if (ImGui::MenuItem(label))
            {
                applyViewportPreset(preset);
            }
            itemTooltip(label);
        };
        presetItem("Perspective", ViewportCameraPreset::Perspective);
        presetItem("Isometric", ViewportCameraPreset::Isometric);
        ImGui::Separator();
        presetItem("Top", ViewportCameraPreset::Top);
        presetItem("Bottom", ViewportCameraPreset::Bottom);
        presetItem("Left", ViewportCameraPreset::Left);
        presetItem("Right", ViewportCameraPreset::Right);
        ImGui::Separator();
        if (ImGui::MenuItem("Track", nullptr, false,
            selectedSectionSlice() != nullptr))
        {
            applyTrackViewPreset(false);
        }
        itemTooltip("View along the selected region's track frame");
        if (ImGui::MenuItem("Walking", nullptr, false,
            selectedSectionSlice() != nullptr))
        {
            applyTrackViewPreset(true);
        }
        itemTooltip("View from walking height near the selected region");
    }

    float EditorUi::showMainMenuBar()
    {
        const float height = ImGui::GetFrameHeight();

        if (!ImGui::BeginMainMenuBar())
        {
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
            showViewportViewMenuItems();
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
            ImGui::MenuItem("Force Diagnostics", nullptr,
                &riderLoadDiagnosticsWindowOpen_);
            if (ImGui::MenuItem(
                "Viewport Settings",
                nullptr,
                &viewportSettingsWindowOpen_
            ))
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Debug,
                    "CFG",
                    "viewport settings window %s",
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
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Debug,
                    "CFG",
                    "transition editor input window %s",
                    inputSettingsWindowOpen_ ? "open" : "closed"
                );
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
        return height;
    }

    void EditorUi::applyViewportPreset(const ViewportCameraPreset preset)
    {
        static constexpr std::array<const char*, 6> presetNames{
            "perspective", "isometric", "top", "bottom", "left", "right"
        };

        viewportCamera_.applyPreset(preset);
        initialViewportFramePending_ = false;
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Debug,
            "INP",
            "VIEW_PRESET %s",
            presetNames[static_cast<std::size_t>(preset)]
        );
    }

    void EditorUi::applyTrackViewPreset(const bool walkingView)
    {
        const CenterlineSectionSlice* const slice = selectedSectionSlice();

        if (slice == nullptr)
        {
            quantum::logging::logMessage(
                quantum::logging::LogLevel::Debug,
                "INP",
                "VIEW_TRACK rejected reason=no-valid-section"
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
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Debug,
            "INP",
            "VIEW_%s applied section=%zu",
            walkingView ? "WALKING" : "TRACK",
            selectedSection_
        );
    }

    void EditorUi::focusSelectedSection()
    {
        const CenterlineSectionSlice* const slice = selectedSectionSlice();

        if (slice == nullptr)
        {
            quantum::logging::logMessage(
                quantum::logging::LogLevel::Debug,
                "INP",
                "VIEW_FOCUS rejected reason=no-valid-section"
            );
            return;
        }

        const glm::dvec3 center =
            (slice->minimumPosition + slice->maximumPosition) / 2.0;
        bool framed = viewportCamera_.frameBounds(
            slice->minimumPosition,
            slice->maximumPosition,
            viewportAspectRatio_
        );

        // A point-like slice has no box extent to fit, but still needs a
        // sensible framing sphere so the camera never lands on the geometry.
        if (!framed)
        {
            framed = viewportCamera_.frameSphere(
                center,
                1.0e-3,
                viewportAspectRatio_
            );
        }

        if (!framed)
        {
            quantum::logging::logMessage(
                quantum::logging::LogLevel::Debug,
                "INP",
                "VIEW_FOCUS rejected reason=invalid-geometry"
            );
            return;
        }

        initialViewportFramePending_ = false;
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Debug,
            "INP",
            "VIEW_FOCUS applied section=%zu",
            selectedSection_
        );
    }

    void EditorUi::frameWholeTrack()
    {
        viewportCamera_.frame(viewportAspectRatio_);
        initialViewportFramePending_ = false;
        quantum::logging::logMessage(
            quantum::logging::LogLevel::Debug,
            "INP",
            "VIEW_FRAME_ALL"
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

        vulkan.setViewportElementVisibility(
            viewportSettings_.gridVisible,
            visibleTrackCurveMask(viewportSettings_)
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
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Debug,
                "VK",
                "swapchain generation changed %llu->%llu; reinitializing "
                "Dear ImGui",
                swapchainGeneration_,
                vulkan.swapchainGeneration()
            );
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
            const float toolbarWidth = ImGui::GetContentRegionAvail().x;
            const float spacing = ImGui::GetStyle().ItemSpacing.x;
            const float viewWidth = buttonWidth("View") + ImGui::GetFrameHeight();
            ImGui::BeginGroup();
            ImGui::SetNextItemWidth(viewWidth);
            if (ImGui::BeginCombo("##ViewportView", "View",
                ImGuiComboFlags_HeightLarge))
            {
                showViewportViewMenuItems();
                ImGui::EndCombo();
            }
            itemTooltip("Choose Perspective, Isometric, Top, Bottom, Left, Right, Track, or Walking");
            sameLineIfFits(buttonWidth("Frame All"));
            if (ImGui::Button("Frame All"))
            {
                frameWholeTrack();
            }
            itemTooltip("Frame the whole track");
            sameLineIfFits(buttonWidth("Focus"));
            if (ImGui::Button("Focus"))
            {
                focusSelectedSection();
            }
            itemTooltip("Focus the selected region");
            sameLineIfFits(ImGui::GetFrameHeight()
                + ImGui::GetStyle().ItemInnerSpacing.x
                + ImGui::CalcTextSize("Anchors").x);
            if (ImGui::Checkbox("Anchors", &viewportSettings_.anchorsVisible))
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Debug,
                    "CFG",
                    "viewport semantic anchors %s",
                    viewportSettings_.anchorsVisible ? "visible" : "hidden"
                );
            }
            itemTooltip("Show or hide the track's semantic boundary anchors");
            ImGui::EndGroup();

            const float toolsWidth = buttonWidth("Move") + buttonWidth("Rotate")
                + buttonWidth("Settings") + 2.0F * spacing;
            const float navigationWidth = viewWidth + buttonWidth("Frame All")
                + buttonWidth("Focus") + ImGui::GetFrameHeight()
                + ImGui::GetStyle().ItemInnerSpacing.x
                + ImGui::CalcTextSize("Anchors").x + 3.0F * spacing;
            if (navigationWidth + toolsWidth + buttonWidth("|")
                + 2.0F * spacing <= toolbarWidth)
            {
                ImGui::SameLine();
                ImGui::TextDisabled("|");
                ImGui::SameLine();
            }
            ImGui::BeginGroup();
            const auto startPoseModeButton = [this](
                const char* const label,
                const StartPoseTransformMode mode)
            {
                const bool selected = startPoseTransformMode_ == mode;
                if (selected)
                {
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 2.0F);
                    ImGui::PushStyleColor(
                        ImGuiCol_Button,
                        quantum::editor::palette::selection
                    );
                    ImGui::PushStyleColor(ImGuiCol_Border, palette::accent);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, palette::selectionHovered);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, palette::selectionActive);
                }
                const bool clicked = ImGui::Button(label);
                if (selected)
                {
                    ImGui::PopStyleColor(4);
                    ImGui::PopStyleVar();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("%s Track Start%s", label,
                        selected ? " (active tool)" : "");
                }
                if (clicked && !selected)
                {
                    startPoseTransformMode_ = mode;
                    startPoseManipulation_.reset();
                    quantum::logging::logMessagef(
                        quantum::logging::LogLevel::Debug,
                        "CFG",
                        "start-pose transform mode=%s",
                        startPoseModeName(mode)
                    );
                }
            };
            startPoseModeButton("Move", StartPoseTransformMode::Move);
            sameLineIfFits(buttonWidth("Rotate"));
            startPoseModeButton("Rotate", StartPoseTransformMode::Rotate);
            sameLineIfFits(buttonWidth("Settings"));
            if (ImGui::Button("Settings"))
            {
                viewportSettingsWindowOpen_
                    = !viewportSettingsWindowOpen_;
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Debug,
                    "CFG",
                    "viewport settings window %s",
                    viewportSettingsWindowOpen_ ? "open" : "closed"
                );
            }

            itemTooltip("Open viewport settings");
            ImGui::EndGroup();
            ImGui::Spacing();

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
            selectedTrackAnchor_ = selectedSection_;
            if (riderLoadDiagnostics_.sectionCount() == sectionCount)
            {
                riderLoadDiagnostics_.selectSection(selectedSection_);
            }
            endpointSelections_.fill(ScalarProfileEndpoint::None);
            endpointDrags_.fill(ScalarProfileEndpoint::None);
            selectedSegmentIds_.fill(coaster::invalidSegmentId);
            dragSegmentIds_.fill(coaster::invalidSegmentId);
            graphValueRanges_.fill({});
            activeRateChannel_ = RateChannel::Pitch;
            hoveredRateChannel_.reset();
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
                             selectedSection_};
        }
        else if (workspaceEdit.removeRequested)
        {
            trackCommand_ = {TrackCommandType::RemoveSection,
                             selectedSection_};
        }
        else if (workspaceEdit.moveUpRequested)
        {
            trackCommand_ = {TrackCommandType::MoveSectionUp,
                             selectedSection_};
        }
        else if (workspaceEdit.moveDownRequested)
        {
            trackCommand_ = {TrackCommandType::MoveSectionDown,
                             selectedSection_};
        }

        if (workspaceEdit.lengthEdited)
        {
            sectionLengthEdit_ = {selectedSection_,
                                  sectionLengthEditBuffer_};
        }

        if (workspaceEdit.layoutModeChanged.has_value())
        {
            pendingLayoutModeChange_ = workspaceEdit.layoutModeChanged;
        }

        if (workspaceEdit.completeCircuitRequested)
        {
            pendingCircuitCompletion_ = true;
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
            ImGui::SetNextWindowPos(
                ImVec2(80.0F, 260.0F),
                ImGuiCond_Appearing
            );
            ImGui::Begin(geometryEditorWindowName);

            if (coaster::isForceDrivenSection(editedSection))
            {
                ImGui::TextDisabled("Force-Based \xe2\x80\x94 read-only");
                ImGui::TextWrapped("Target profile editing is not available yet. Actual loads appear in Force Diagnostics.");
                if (ImGui::SmallButton("Show Force Diagnostics"))
                {
                    riderLoadDiagnosticsWindowOpen_ = true;
                    ImGui::SetWindowFocus(riderLoadDiagnosticsWindowName);
                }
            }
            else
            {
            const auto& committedArc =
                std::get<coaster::PlanarArcRegion>(
                    std::get<coaster::GeometryRegion>(
                        editedSection.region).construction);

            ImGui::SeparatorText("Circular arc geometry");

            // Angle fields present degrees; commands carry Core radians.
            // Every edit flows through the same candidate/commit pipeline
            // as rate-profile edits.
            const auto inputProperty = [](const char* const label,
                const char* const id, double* const value,
                const double step, const double fastStep)
            {
                const float labelWidth = ImGui::CalcTextSize("Bank change (deg)").x
                    + ImGui::GetStyle().ItemSpacing.x;
                const bool inlineField = ImGui::GetContentRegionAvail().x
                    >= labelWidth + 200.0F;
                const float rowX = ImGui::GetCursorPosX();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(label);
                if (inlineField)
                {
                    ImGui::SameLine(rowX + labelWidth);
                }
                ImGui::SetNextItemWidth(std::min(320.0F,
                    ImGui::GetContentRegionAvail().x));
                return ImGui::InputDouble(id, value, step, fastStep, "%.3f");
            };
            if (inputProperty(
                "Radius", "###Radius",
                &planarArcEditBuffers_[0],
                1.0,
                10.0
            ))
            {
                regionCommand_ = {RegionCommandType::SetPlanarArcRadius,
                                  selectedSection_,
                                  planarArcEditBuffers_[0]};
            }

            if (inputProperty(
                "Arc angle (deg)", "###Swept Angle (deg)",
                &planarArcEditBuffers_[1],
                5.0,
                15.0
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcSweptAngle,
                    selectedSection_,
                    planarArcEditBuffers_[1] * radiansPerDegree};
            }

            if (inputProperty(
                "Plane tilt (deg)", "###Plane Tilt (deg)",
                &planarArcEditBuffers_[2],
                5.0,
                15.0
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcPlaneTilt,
                    selectedSection_,
                    planarArcEditBuffers_[2] * radiansPerDegree};
            }

            ImGui::Spacing();
            ImGui::SeparatorText("Banking");
            if (inputProperty(
                "Bank change (deg)", "###Bank Change (deg)",
                &planarArcEditBuffers_[3],
                5.0,
                15.0
            ))
            {
                regionCommand_ = {
                    RegionCommandType::SetPlanarArcBankChange,
                    selectedSection_,
                    planarArcEditBuffers_[3] * radiansPerDegree};
            }
            ImGui::Spacing();
            ImGui::PushTextWrapPos();
            ImGui::TextDisabled(
                "Resulting length %.6g",
                coaster::planarArcLength(committedArc)
            );
            ImGui::PopTextWrapPos();

            ImGui::Spacing();
            if (ImGui::SmallButton("Convert to Profile###Convert to Rate Profiles"))
            {
                regionCommand_ = {RegionCommandType::ConvertToRateProfiles,
                                  selectedSection_, 0.0};
            }
            }

            ImGui::End();
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
                    profile.segments.back().transition.valueEnd
                        * degreesPerRadian;
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
            {"Roll",
             &sectionRateChannel(editedSection, RateChannel::Roll),
             RateChannel::Roll},
            {"Pitch",
             &sectionRateChannel(editedSection, RateChannel::Pitch),
             RateChannel::Pitch},
            {"Yaw",
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
            graphValueRanges_,
            activeRateChannel_,
            hoveredRateChannel_,
            hoveredGraphMarker_,
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
            // Compare against the exact semantic endpoint addressed by the
            // graph marker or active numeric control.
            double acceptedValue = 0.0;
            for (const coaster::ProfileSegment& segment :
                channelProfile.segments)
            {
                if (segment.id == valueEdit.segmentId)
                {
                    acceptedValue = valueEdit.endpoint
                            == ScalarProfileEndpoint::Begin
                        ? segment.transition.valueBegin
                        : segment.transition.valueEnd;
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

            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Debug,
                "INP",
                "TRANSITION_SPLIT channel=%d segment=%u "
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
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Debug,
                "INP",
                "TRANSITION_REMOVE channel=%d segment=%u",
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
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "TRANSITION_CLICK channel=%zu endpoint=%d "
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
                valueEndEditBuffers_[clickChannel] = committedValue
                    * degreesPerRadian;
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

                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Trace,
                    "INP",
                    "TRANSITION_PLOT_CLICK channel=%zu segment=%u",
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
                            segment.transition.valueEnd
                                * degreesPerRadian;
                        break;
                    }
                }
            }
        }
        }

        if (riderLoadDiagnostics_.sectionCount() == sectionCount)
        {
            showRiderLoadDiagnostics(
                riderLoadDiagnostics_.selectedSection(),
                authoredTrack_->physicalSettings(),
                &riderLoadDiagnosticsWindowOpen_
            );
        }

        if (anyEndpointDragReleased)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Trace,
                "INP",
                "DRAG_RELEASED drag was=%d",
                static_cast<int>(firstActiveEndpoint(endpointDrags_)));
            // One summary line per completed drag gesture: continuous
            // edits regenerate the track silently every frame.
            if (pendingDragSummary_)
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Info,
                    "EDIT",
                    "section=%zu channel=%d endpoint=%d value=%.6f "
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
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Info,
                    "EDIT",
                    "section=%zu channel=%d endpoint=%d "
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

    std::optional<SectionLengthEdit> EditorUi::takeSectionLengthEdit() noexcept
    {
        const std::optional<SectionLengthEdit> edit = sectionLengthEdit_;
        sectionLengthEdit_.reset();
        return edit;
    }

    std::optional<RegionCommand> EditorUi::takeRegionCommand() noexcept
    {
        const std::optional<RegionCommand> command = regionCommand_;
        regionCommand_.reset();
        return command;
    }

    std::optional<StartPoseEdit> EditorUi::takeStartPoseEdit() noexcept
    {
        const std::optional<StartPoseEdit> edit = startPoseEdit_;
        startPoseEdit_.reset();
        return edit;
    }

    void EditorUi::rejectStartPoseManipulation() noexcept
    {
        startPoseEdit_.reset();
        startPoseManipulation_.reset();
    }

    void EditorUi::selectSection(
        const std::size_t index,
        const bool refreshIfSelected)
    {
        const std::size_t sectionCount = authoredTrack_ != nullptr
            ? authoredTrack_->sectionCount()
            : 0;
        if (sectionCount == 0 || index >= sectionCount)
        {
            return;
        }

        selectedTrackAnchor_ = selectionForViewportTrackRegion(
            index,
            sectionCount
        ).anchorIndex;
        if (index == selectedSection_ && !refreshIfSelected)
        {
            return;
        }

        selectedSection_ = index;
        if (riderLoadDiagnostics_.sectionCount() == sectionCount)
        {
            riderLoadDiagnostics_.selectSection(selectedSection_);
        }
        regionCreateFlow_.choicePending = false;
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        graphValueRanges_.fill({});
        activeRateChannel_ = RateChannel::Pitch;
        hoveredRateChannel_.reset();
        hoveredGraphMarker_.reset();
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
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Debug,
                "SEL",
                "selected=%zu rollEnd=%.6f pitchEnd=%.6f "
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
                    ).segments.back().transition.valueEnd
                        * degreesPerRadian;
            }
        }
        else if (!coaster::isForceDrivenSection(selected))
        {
            const auto& arc = std::get<coaster::PlanarArcRegion>(
                std::get<coaster::GeometryRegion>(
                    selected.region).construction);
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Debug,
                "SEL",
                "selected=%zu kind=planarArc length=%.6f "
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

        if (selected.kind == coaster::RegionKind::Geometry
            && !coaster::isForceDrivenSection(selected))
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

    void EditorUi::selectTrackAnchor(const std::size_t anchorIndex)
    {
        if (authoredTrack_ == nullptr
            || centerlineVisualization_ == nullptr)
        {
            return;
        }

        const std::size_t regionCount = authoredTrack_->sectionCount();
        if (regionCount == 0 || anchorIndex > regionCount
            || anchorIndex >= centerlineVisualization_->anchors.size())
        {
            return;
        }

        const ViewportTrackSelection selection =
            selectionForViewportTrackAnchor(anchorIndex, regionCount);
        selectSection(selection.regionIndex);
        selectedTrackAnchor_ = selection.anchorIndex;

        const ViewportTrackAnchor& anchor =
            centerlineVisualization_->anchors[anchorIndex];
        const char* kind = "interior";
        if (anchor.kind == ViewportTrackAnchorKind::Start)
        {
            kind = "start";
        }
        else if (anchor.kind == ViewportTrackAnchorKind::End)
        {
            kind = "end";
        }
        quantum::logging::logMessagef(
            quantum::logging::LogLevel::Debug,
            "SEL",
            "anchor=%zu kind=%s region=%zu distance=%.6f readOnly=%d",
            anchorIndex,
            kind,
            selection.regionIndex,
            anchor.distance,
            isViewportTrackAnchorEditable(anchor.kind) ? 0 : 1
        );
    }

    void EditorUi::synchronizeSegmentEndpointValue(
        const RateChannel channel,
        const std::uint32_t segmentId,
        const ScalarProfileEndpoint endpoint,
        const double acceptedValue)
    {
        if (!std::isfinite(acceptedValue))
        {
            throw std::invalid_argument(
                "EditorUi requires a finite accepted profile endpoint value."
            );
        }

        const std::size_t channelIndex = static_cast<std::size_t>(channel);
        const ScalarProfileEndpoint displayedEndpoint =
            endpointSelections_[channelIndex]
                == ScalarProfileEndpoint::Begin
            ? ScalarProfileEndpoint::Begin
            : ScalarProfileEndpoint::End;
        if (selectedSegmentIds_[channelIndex] == segmentId
            && displayedEndpoint == endpoint)
        {
            valueEndEditBuffers_[channelIndex] =
                acceptedValue * degreesPerRadian;
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

    void EditorUi::setCenterlineVisualization(
        const CenterlineVisualization& visualization) noexcept
    {
        centerlineVisualization_ = &visualization;
        if (visualization.anchors.empty())
        {
            selectedTrackAnchor_.reset();
        }
        else
        {
            selectedTrackAnchor_ = std::min(
                selectedSection_,
                visualization.anchors.size() - 1
            );
        }
    }

    void EditorUi::setRiderLoadHistory(coaster::RiderLoadHistory history)
    {
        if (authoredTrack_ == nullptr)
        {
            throw std::logic_error(
                "EditorUi cannot map rider loads before initialization."
            );
        }

        riderLoadDiagnostics_.update(
            *authoredTrack_,
            std::move(history)
        );
        if (riderLoadDiagnostics_.sectionCount() > 0)
        {
            riderLoadDiagnostics_.selectSection(std::min(
                selectedSection_,
                riderLoadDiagnostics_.sectionCount() - 1
            ));
        }
    }

    std::size_t EditorUi::selectedSection() const noexcept
    {
        return selectedSection_;
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
        centerlineVisualization_ = nullptr;
        riderLoadDiagnostics_.clear();
        selectedSection_ = 0;
        startPoseManipulation_.reset();
        startPoseEdit_.reset();
        valueEndEditBuffers_.fill(0.0);
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        graphValueRanges_.fill({});
        activeRateChannel_ = RateChannel::Pitch;
        hoveredRateChannel_.reset();
        hoveredGraphMarker_.reset();
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

    bool EditorUi::takeCircuitCompletionRequest() noexcept
    {
        const bool requested = pendingCircuitCompletion_;
        pendingCircuitCompletion_ = false;
        return requested;
    }

    void EditorUi::resetTransientState()
    {
        selectedSection_ = 0;
        selectedTrackAnchor_ = 0;
        startPoseTransformMode_ = StartPoseTransformMode::Move;
        startPoseManipulation_.reset();
        startPoseEdit_.reset();
        if (riderLoadDiagnostics_.sectionCount() > 0)
        {
            riderLoadDiagnostics_.selectSection(0);
        }
        valueEndEditBuffers_.fill(0.0);
        endpointSelections_.fill(ScalarProfileEndpoint::None);
        endpointDrags_.fill(ScalarProfileEndpoint::None);
        selectedSegmentIds_.fill(coaster::invalidSegmentId);
        dragSegmentIds_.fill(coaster::invalidSegmentId);
        graphValueRanges_.fill({});
        activeRateChannel_ = RateChannel::Pitch;
        hoveredRateChannel_.reset();
        hoveredGraphMarker_.reset();
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
