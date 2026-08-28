#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <SDL3/SDL_events.h>
#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

struct SDL_Window;

namespace quantum::renderer
{
    class VulkanContext;
}

namespace quantum::editor
{
    struct MousePos
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    enum class ScalarProfileEndpoint
    {
        None,
        Begin,
        End
    };

    // One authored rate-profile channel of a section. The order matches the
    // Transition Editor row layout and every channel-indexed state array.
    enum class RateChannel
    {
        Roll,
        Pitch,
        Yaw
    };

    // Number of authored rate channels; also the row count of the
    // Transition Editor.
    inline constexpr std::size_t rateChannelCount = 3;

    // Resolves one authored rate channel of a section. Channels may hold
    // any number of profile segments; segment-level edits address stable
    // SegmentIds through the ChannelProfileEditing core operations.
    [[nodiscard]] inline quantum::coaster::ChannelProfile& sectionRateChannel(
        quantum::coaster::AuthoredTrackSection& section,
        const RateChannel channel)
    {
        switch (channel)
        {
        case RateChannel::Roll:
            return section.rateProfileRegion().rateProfiles.roll;
        case RateChannel::Pitch:
            return section.rateProfileRegion().rateProfiles.pitch;
        case RateChannel::Yaw:
        default:
            return section.rateProfileRegion().rateProfiles.yaw;
        }
    }

    [[nodiscard]] inline const quantum::coaster::ChannelProfile&
    sectionRateChannel(
        const quantum::coaster::AuthoredTrackSection& section,
        const RateChannel channel)
    {
        return sectionRateChannel(
            const_cast<quantum::coaster::AuthoredTrackSection&>(section),
            channel
        );
    }

    // Authoring interaction configuration for the Transition Editor,
    // shared by every section and every rate channel. Defaults preserve
    // the original drag feel; snapping starts disabled. A future
    // Preferences/Authoring panel can expose these fields directly.
    struct TransitionEditorInputSettings
    {
        // Drag gain multipliers on each row's natural value-per-pixel
        // slope. Normal 1.0 reproduces the original absolute-cursor
        // mapping; Fine slows the value change for precision work.
        float normalDragGain = 1.0F;
        float fineDragGain = 0.25F;

        // Authored-value snapping applied while the user edits (drag or
        // numeric input). Never quantizes data on load or selection.
        bool snapEnabled = false;
        double snapIncrement = 0.005;

        // Authored-distance snapping applied while interior segment
        // boundaries are dragged horizontally. Independent of value
        // snapping so vertical precision work keeps its own grid.
        bool distanceSnapEnabled = false;
        double distanceSnapIncrement = 5.0;
    };

    // Which cursor axis an active handle drag follows. The lock engages
    // once cumulative motion makes the intended axis unambiguous, so a
    // mostly-vertical value drag cannot drift boundaries horizontally
    // through cursor jitter.
    enum class DragAxisLock
    {
        None,
        Horizontal,
        Vertical
    };

    // Rolling reference for an active handle drag: the cursor position,
    // authored value, and authored distance of the previous frame.
    // Integrating per-frame deltas lets the gain change live when Shift is
    // pressed mid-drag without a value jump.
    struct ScalarDragAnchor
    {
        double pixelX = 0.0;
        double pixelY = 0.0;
        double value = 0.0;
        double distance = 0.0;
    };

    struct ScalarProfileEndpointValueEdit
    {
        ScalarProfileEndpoint endpoint = ScalarProfileEndpoint::None;
        double value = 0.0;
        bool continuous = false;
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
        // Target segment within the channel; stable across edits.
        std::uint32_t segmentId = 0;
    };

    struct ProfileTransitionTypeEdit
    {
        math::TransitionType type = math::TransitionType::Linear;
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
        std::uint32_t segmentId = 0;
    };

    // Structural one-shot requests on a selected profile segment.
    enum class ProfileSegmentOperation
    {
        Split,
        Remove
    };

    struct ProfileSegmentCommand
    {
        ProfileSegmentOperation operation = ProfileSegmentOperation::Split;
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
        std::uint32_t segmentId = 0;
        double splitDistance = 0.0;
    };

    // Continuous per-frame edit emitted while a shared interior boundary
    // is dragged horizontally. The adjoining boundary follows in Core so
    // the chain stays contiguous.
    struct ProfileSegmentDistanceEdit
    {
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
        std::uint32_t segmentId = 0;
        ScalarProfileEndpoint endpoint = ScalarProfileEndpoint::End;
        double distance = 0.0;
    };

    enum class TrackCommandType
    {
        AppendSection,
        PrependSection,
        RemoveSection,
        MoveSectionUp,
        MoveSectionDown,
        SetSectionLength,
        DuplicateSection
    };

    struct TrackCommand
    {
        TrackCommandType type = TrackCommandType::AppendSection;
        std::size_t sectionIndex = 0;
        double length = 0.0;
    };

    // Which ordering position a typed region creation targets. The
    // choice strip is shared by all three triggers.
    enum class RegionCreateAnchor
    {
        Append,
        Prepend,
        AfterSelected
    };

    // Two-step typed region creation in the track workspace: clicking
    // "Append/Prepend Region..." or "Insert After Selected..." opens the
    // authoring-type choice strip, and choosing a kind or cancelling
    // resolves it.
    struct RegionCreateFlow
    {
        bool choicePending = false;
        RegionCreateAnchor anchor = RegionCreateAnchor::Append;
    };

    // Region-level authoring commands for one section: typed creation of a
    // new region, kind conversion between rate profiles and planar-arc
    // geometry, and planar-arc parameter updates. `value` carries the new
    // parameter for the Set* commands (radians for angles) and is ignored
    // by every other command.
    enum class RegionCommandType
    {
        AppendRateProfiles,
        PrependRateProfiles,
        AppendPlanarArc,
        PrependPlanarArc,
        InsertAfterRateProfiles,
        InsertAfterPlanarArc,
        ConvertToRateProfiles,
        ConvertToPlanarArc,
        SetPlanarArcRadius,
        SetPlanarArcSweptAngle,
        SetPlanarArcPlaneTilt,
        SetPlanarArcBankChange
    };

    struct RegionCommand
    {
        RegionCommandType type = RegionCommandType::ConvertToPlanarArc;
        std::size_t sectionIndex = 0;
        double value = 0.0;
    };

    // File workflow operations requested by the user through the menu
    // bar or command area. The Application layer processes these.
    enum class FileOperationType
    {
        New,
        Open,
        Save,
        SaveAs
    };

    // Authoritative viewport display configuration. The editor owns these
    // values and pushes them into the camera and renderer every frame.
    struct ViewportSettings
    {
        bool orthographic = false;
        float fieldOfViewDegrees = 45.0F;

        // Multipliers on the camera's natural drag response.
        float orbitSensitivity = 1.0F;
        float zoomSensitivity = 1.0F;

        bool gridVisible = true;
        bool centerlineVisible = true;
        bool leftRailVisible = true;
        bool rightRailVisible = true;
        bool heartlineVisible = true;
    };

    class EditorUi
    {
    public:
        EditorUi() = default;
        ~EditorUi();

        EditorUi(const EditorUi&) = delete;
        EditorUi& operator=(const EditorUi&) = delete;

        void initialize(
            SDL_Window* window,
            const renderer::VulkanContext& vulkan,
            const coaster::AuthoredTrack& authoredTrack,
            const glm::dvec3& centerlineMinimum,
            const glm::dvec3& centerlineMaximum
        );
        void processEvent(const SDL_Event& event);
        void beginFrame(renderer::VulkanContext& vulkan);
        [[nodiscard]] std::optional<ScalarProfileEndpointValueEdit>
        takeProfileEndpointValueEdit() noexcept;
        [[nodiscard]] std::optional<ProfileTransitionTypeEdit>
        takeProfileTransitionTypeEdit() noexcept;
        [[nodiscard]] std::optional<ProfileSegmentCommand>
        takeProfileSegmentCommand() noexcept;
        [[nodiscard]] std::optional<ProfileSegmentDistanceEdit>
        takeProfileSegmentDistanceEdit() noexcept;
        [[nodiscard]] std::optional<TrackCommand>
        takeTrackCommand() noexcept;
        [[nodiscard]] std::optional<TrackCommand>
        takeSectionLengthEdit() noexcept;
        [[nodiscard]] std::optional<RegionCommand>
        takeRegionCommand() noexcept;
        // Selects a region programmatically after an accepted structural
        // edit so newly created, duplicated, and reordered regions keep the
        // working selection on the intended region. Requests naming an
        // index outside the live document are ignored; the same interaction
        // resets and [SEL] logging apply as for list-click selection.
        void selectSection(std::size_t index);
        // Refreshes the numeric edit buffer for a channel, but only when
        // the committed segment is the one the row currently addresses.
        void synchronizeSegmentValueEnd(
            RateChannel channel,
            std::uint32_t segmentId,
            double acceptedValueEnd
        );
        void synchronizeSectionLength(double acceptedLength);
        // Refreshes the planar-arc numeric edit buffers with committed
        // construction values so accepted edits and rejections both leave
        // the fields showing authoritative data.
        void synchronizePlanarArcParams(
            const coaster::PlanarArcRegion& committedParams);
        void setCenterlineBounds(
            const glm::dvec3& centerlineMinimum,
            const glm::dvec3& centerlineMaximum
        );
        void setCenterlineSections(
            std::vector<CenterlineSectionSlice> sectionSlices
        );
        // Retains a non-owning view of the cache-owned visualization for
        // viewport picking. CenterlineVisualizationCache keeps the object
        // address stable when replacing its contents after geometry edits.
        void setCenterlineVisualization(
            const CenterlineVisualization& visualization
        ) noexcept;
        [[nodiscard]] std::size_t selectedSection() const noexcept;
        void render(VkCommandBuffer commandBuffer);
        void shutdown() noexcept;

        // File workflow: returns any pending file operation requested
        // by the user through menu items or command area buttons.
        [[nodiscard]] std::optional<FileOperationType>
        takePendingFileOperation() noexcept;

        // Layout mode: returns any pending layout mode change requested
        // through the Track Workspace selector buttons.
        [[nodiscard]] std::optional<coaster::LayoutMode>
        takePendingLayoutModeChange() noexcept;

        // Circuit completion: returns true once when the user clicks
        // Complete Circuit....  Application processes the actual
        // completion attempt.
        [[nodiscard]] bool takeCircuitCompletionRequest() noexcept;

        // Resets all transient editing state (selections, drags, edit
        // buffers) so the editor is clean for a new document.
        void resetTransientState();

        // Updates the SDL window title bar.
        void updateWindowTitle(const std::string& title);

    private:
        enum class CameraGesture
        {
            None,
            Orbit,
            Pan
        };

        void initializeVulkanBackend(
            const renderer::VulkanContext& vulkan
        );
        void updateViewportTexture(
            renderer::VulkanContext& vulkan,
            std::uint32_t width,
            std::uint32_t height
        );
        void updateViewportCamera(
            renderer::VulkanContext& vulkan,
            bool viewportHovered,
            std::uint32_t pixelWidth,
            std::uint32_t pixelHeight,
            float logicalWidth,
            float logicalHeight
        );
        static void retireViewportTexture(void* userData) noexcept;
        void removeViewportTexture() noexcept;
        void shutdownVulkanBackend() noexcept;

        [[nodiscard]] float showMainMenuBar();
        void applyViewportPreset(ViewportCameraPreset preset);
        void applyTrackViewPreset(bool walkingView);
        void focusSelectedSection();
        void frameWholeTrack();
        [[nodiscard]] const CenterlineSectionSlice*
        selectedSectionSlice() const noexcept;
        void applyViewportSettings(renderer::VulkanContext& vulkan);

        bool contextCreated_ = false;
        bool sdlBackendInitialized_ = false;
        bool vulkanBackendInitialized_ = false;
        VkDevice device_ = VK_NULL_HANDLE;
        VkDescriptorSet viewportTexture_ = VK_NULL_HANDLE;
        std::uint64_t swapchainGeneration_ = 0;
        ViewportCamera viewportCamera_;
        CameraGesture cameraGesture_ = CameraGesture::None;
        bool initialViewportFramePending_ = true;
        double viewportAspectRatio_ = 16.0 / 9.0;
        const coaster::AuthoredTrack* authoredTrack_ = nullptr;
        std::size_t selectedSection_ = 0;

        // Per-channel editing state, indexed by RateChannel. Buffers hold
        // each channel's focused segment End value; interaction slots
        // track which endpoint (per channel) is selected or being dragged.
        std::array<double, rateChannelCount> valueEndEditBuffers_{};
        std::array<ScalarProfileEndpoint, rateChannelCount>
            endpointSelections_{};
        std::array<ScalarProfileEndpoint, rateChannelCount> endpointDrags_{};
        // Per-channel selected profile segments, keyed by stable id. Stale
        // ids are re-resolved against the committed document every frame;
        // zero only ever appears before the first resolution.
        std::array<std::uint32_t, rateChannelCount> selectedSegmentIds_{};
        std::array<std::uint32_t, rateChannelCount> dragSegmentIds_{};
        std::array<DragAxisLock, rateChannelCount> dragAxisLocks_{};
        // Cumulative cursor travel per active drag; picks the drag axis
        // once motion becomes unambiguous.
        std::array<double, rateChannelCount> dragAxisTravelX_{};
        std::array<double, rateChannelCount> dragAxisTravelY_{};
        std::array<std::optional<double>, rateChannelCount>
            dragLastValues_{};
        // Rolling per-channel anchors for active handle drags; present
        // exactly while the matching slot in endpointDrags_ is active.
        std::array<std::optional<ScalarDragAnchor>, rateChannelCount>
            scalarDragAnchors_{};
        // Last continuous edit queued during the active drag, kept so the
        // release handler can emit a single end-of-drag [EDIT] summary
        // (continuous edits themselves stay console-silent). The distance
        // summary serves the same purpose for horizontal boundary drags.
        bool pendingDragSummary_ = false;
        ScalarProfileEndpointValueEdit dragSummaryEdit_{};
        bool pendingDistanceSummary_ = false;
        ProfileSegmentDistanceEdit distanceSummaryEdit_{};
        // Split-at-cursor candidate captured when a row context menu opens.
        std::array<double, rateChannelCount> contextMenuSplitDistances_{};
        TransitionEditorInputSettings transitionEditorInputSettings_;
        bool inputSettingsWindowOpen_ = false;
        ViewportSettings viewportSettings_;
        bool viewportSettingsWindowOpen_ = false;
        std::vector<CenterlineSectionSlice> centerlineSlices_;
        const CenterlineVisualization* centerlineVisualization_ = nullptr;
        double sectionLengthEditBuffer_ = 0.0;
        // Planar-arc numeric edit buffers, indexed by
        // planarArcParamIndex: radius, swept angle, plane tilt, bank.
        // Angles are held in degrees for the Geometry Editor UI and
        // converted to Core radians only when a command is queued.
        std::array<double, 4> planarArcEditBuffers_{};
        RegionCreateFlow regionCreateFlow_;
        std::optional<ScalarProfileEndpointValueEdit>
            profileEndpointValueEdit_;
        std::optional<ProfileTransitionTypeEdit> profileTransitionTypeEdit_;
        std::optional<ProfileSegmentCommand> profileSegmentCommand_;
        std::optional<ProfileSegmentDistanceEdit> profileSegmentDistanceEdit_;
        std::optional<TrackCommand> trackCommand_;
        std::optional<TrackCommand> sectionLengthEdit_;
        std::optional<RegionCommand> regionCommand_;
        std::string iniPath_;
        SDL_Window* window_ = nullptr;
        std::optional<FileOperationType> pendingFileOperation_;
        std::optional<coaster::LayoutMode> pendingLayoutModeChange_;
        bool pendingCircuitCompletion_ = false;
    };
}
