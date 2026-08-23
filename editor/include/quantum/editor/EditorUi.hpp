#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/ViewportCamera.hpp>

#include <SDL3/SDL_events.h>
#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

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

    [[nodiscard]] inline quantum::math::ScalarTransition& sectionRateProfile(
        quantum::coaster::AuthoredTrackSection& section,
        const RateChannel channel) noexcept
    {
        switch (channel)
        {
        case RateChannel::Roll:
            return section.rateProfiles.roll;
        case RateChannel::Pitch:
            return section.rateProfiles.pitch;
        case RateChannel::Yaw:
        default:
            return section.rateProfiles.yaw;
        }
    }

    [[nodiscard]] inline const quantum::math::ScalarTransition&
    sectionRateProfile(
        const quantum::coaster::AuthoredTrackSection& section,
        const RateChannel channel) noexcept
    {
        return sectionRateProfile(
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
    };

    // Rolling reference for an active handle drag: the cursor Y and
    // authored value of the previous frame. Integrating per-frame deltas
    // lets the gain change live when Shift is pressed mid-drag without a
    // value jump.
    struct ScalarDragAnchor
    {
        double pixelY = 0.0;
        double value = 0.0;
    };

    struct ScalarProfileEndpointValueEdit
    {
        ScalarProfileEndpoint endpoint = ScalarProfileEndpoint::None;
        double value = 0.0;
        bool continuous = false;
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
    };

    struct ProfileTransitionTypeEdit
    {
        math::TransitionType type = math::TransitionType::Linear;
        std::size_t sectionIndex = 0;
        RateChannel channel = RateChannel::Pitch;
    };

    enum class TrackCommandType
    {
        AppendSection,
        PrependSection,
        RemoveSection,
        MoveSectionUp,
        MoveSectionDown,
        SetSectionLength
    };

    struct TrackCommand
    {
        TrackCommandType type = TrackCommandType::AppendSection;
        std::size_t sectionIndex = 0;
        double length = 0.0;
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
        [[nodiscard]] std::optional<TrackCommand>
        takeTrackCommand() noexcept;
        [[nodiscard]] std::optional<TrackCommand>
        takeSectionLengthEdit() noexcept;
        void synchronizeProfileValueEnd(
            RateChannel channel,
            double acceptedValueEnd
        );
        void synchronizeSectionLength(double acceptedLength);
        void setCenterlineBounds(
            const glm::dvec3& centerlineMinimum,
            const glm::dvec3& centerlineMaximum
        );
        void render(VkCommandBuffer commandBuffer);
        void shutdown() noexcept;

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

        bool contextCreated_ = false;
        bool sdlBackendInitialized_ = false;
        bool vulkanBackendInitialized_ = false;
        VkDevice device_ = VK_NULL_HANDLE;
        VkDescriptorSet viewportTexture_ = VK_NULL_HANDLE;
        std::uint64_t swapchainGeneration_ = 0;
        ViewportCamera viewportCamera_;
        CameraGesture cameraGesture_ = CameraGesture::None;
        bool initialViewportFramePending_ = true;
        const coaster::AuthoredTrack* authoredTrack_ = nullptr;
        std::size_t selectedSection_ = 0;

        // Per-channel editing state, indexed by RateChannel. Buffers hold
        // each channel's End-handle value; interaction slots track which
        // endpoint (per channel) is selected or being dragged.
        std::array<double, rateChannelCount> valueEndEditBuffers_{};
        std::array<ScalarProfileEndpoint, rateChannelCount>
            endpointSelections_{};
        std::array<ScalarProfileEndpoint, rateChannelCount> endpointDrags_{};
        std::array<std::optional<double>, rateChannelCount>
            dragLastValues_{};
        // Rolling per-channel anchors for active handle drags; present
        // exactly while the matching slot in endpointDrags_ is active.
        std::array<std::optional<ScalarDragAnchor>, rateChannelCount>
            scalarDragAnchors_{};
        // Last continuous edit queued during the active drag, kept so the
        // release handler can emit a single end-of-drag [EDIT] summary
        // (continuous edits themselves stay console-silent).
        bool pendingDragSummary_ = false;
        ScalarProfileEndpointValueEdit dragSummaryEdit_{};
        TransitionEditorInputSettings transitionEditorInputSettings_;
        bool inputSettingsWindowOpen_ = false;
        double sectionLengthEditBuffer_ = 0.0;
        std::optional<ScalarProfileEndpointValueEdit>
            profileEndpointValueEdit_;
        std::optional<ProfileTransitionTypeEdit> profileTransitionTypeEdit_;
        std::optional<TrackCommand> trackCommand_;
        std::optional<TrackCommand> sectionLengthEdit_;
        std::string iniPath_;
        SDL_Window* window_ = nullptr;
    };
}
