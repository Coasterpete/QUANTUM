#pragma once

#include <quantum/renderer/Renderer.hpp>

#include <SDL3/SDL.h>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace quantum::renderer
{
    // QUANTUM intentionally permits one submitted frame at a time under FIFO
    // presentation. Every slot-owned command, synchronization, timestamp, and
    // dynamic-preview resource below is sized from this policy. Render-finished
    // semaphores remain separately owned per swapchain image.
    inline constexpr std::uint32_t maxFramesInFlight = 1;
    static_assert(maxFramesInFlight > 0);

    class VulkanContext final : public Renderer
    {
    public:
        using FrameRenderCallback = void (*)(VkCommandBuffer, void*);

        VulkanContext() = default;
        ~VulkanContext() override;

        VulkanContext(const VulkanContext&) = delete;
        VulkanContext& operator=(const VulkanContext&) = delete;

        // The track-curve vertices must be empty with zero vertices per curve,
        // or consist of exactly four runs of `trackVerticesPerCurve` vertices
        // each.
        void initialize(
            SDL_Window* window,
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve,
            const coaster::RenderableTrack& renderableTrack,
            bool enableFrameReadback = false
        ) override;
        // Optional synchronous readback after the render callback. Empty on a
        // skipped/out-of-date frame; requires opt-in during initialize().
        void drawFrame(FrameImage* readback = nullptr) override;
        // The Vulkan ImGui integration installs its command recording hook.
        // Native handles never enter the renderer-neutral Renderer contract.
        void setFrameRenderCallback(FrameRenderCallback callback,
            void* userData) noexcept;
        void resizeViewportTarget(
            std::uint32_t width,
            std::uint32_t height,
            ViewportTargetRetirementCallback retirementCallback,
            void* userData,
            bool enableMsaa
        ) override;
        [[nodiscard]] bool supportsViewportMsaa4() const noexcept;
        [[nodiscard]] RendererCapabilities capabilities() const noexcept override;
        [[nodiscard]] bool viewportMsaaEnabled() const noexcept;
        void setViewportViewProjection(
            const std::array<float, 16>& viewProjection
        );
        void setViewportCameraPosition(const glm::vec3& position);
        // Direction points from the surface toward the sun in world space.
        void setSunlight(const glm::vec3& direction, float intensity);
        void setExposure(float exposure);
        void setEnvironment(bool enabled, float rotationDegrees,
            float lightingIntensity, bool skyVisible) override;
        void updateTrackCurveVertices(
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve
        );
        void updateRenderableTrack(
            const coaster::RenderableTrack& renderableTrack
        );
        void updateRenderableTrackMesh(
            const coaster::ContinuousTrackMesh& mesh,
            std::span<const coaster::TrackMaterial> materials);
        // Material colors are copied into future command-buffer push
        // constants; already recorded/in-flight command buffers own their
        // values, so these two updates require no fence drain.
        void updateTrackMaterials(
            std::span<const coaster::TrackSubmesh> submeshes,
            std::span<const coaster::TrackMaterial> materials);
        void updateTrackHardware(
            std::span<const coaster::HardwareInstanceBatch> batches);
        void updateTrackHardwareMaterials(
            std::span<const std::optional<coaster::TrackMaterial>> materials);
        // Invalidates and reloads one package-relative hardware mesh, then
        // refreshes every draw batch in the supplied current track.
        void reloadTrackHardwareAsset(
            std::string_view identifier,
            const coaster::RenderableTrack& renderableTrack
        );
        void setTrackPresentationMode(TrackPresentationMode mode);

        // Replaces the Editor's dynamic diagnostic train line stream.
        void updateTrainPreviewVertices(
            std::span<const LineVertex> vertices);

        // Replaces the Editor's renderer-neutral support-member line stream.
        void updateSupportVertices(std::span<const LineVertex> vertices);

        // Host-side draw skipping for the viewport reference elements.
        // Idempotent; intended to be pushed every frame from the editor's
        // authoritative settings like the view-projection matrix.
        void setViewportElementVisibility(
            bool gridVisible,
            std::uint32_t curveVisibilityMask
        );

        // Draws the same authored-section range once more over every visible
        // track curve with the highlight color blend. A zero vertex count
        // disables highlighting. This changes draw state only; it never
        // rewrites or reallocates the retained geometry buffer.
        void setTrackCurveHighlight(
            std::uint32_t firstVertex,
            std::uint32_t vertexCount
        );

        // Recenters and rescales the ground grid toward a reference sphere
        // (usually the solved-track bounds), snapping to the chosen spacing
        // so the grid only rewrites its buffer when it actually moves.
        void updateViewportAidReference(
            float centerX,
            float centerY,
            float referenceRadius
        );
        void shutdown() noexcept override;

        [[nodiscard]] VkInstance instance() const noexcept;
        [[nodiscard]] VkPhysicalDevice physicalDevice() const noexcept;
        [[nodiscard]] VkDevice device() const noexcept;
        [[nodiscard]] std::uint32_t graphicsQueueFamily() const noexcept;
        [[nodiscard]] VkQueue graphicsQueue() const noexcept;
        [[nodiscard]] VkFormat swapchainFormat() const noexcept;
        [[nodiscard]] std::uint32_t swapchainImageCount() const noexcept;
        [[nodiscard]] std::uint64_t swapchainGeneration() const noexcept;
        [[nodiscard]] VkExtent2D viewportExtent() const noexcept;
        [[nodiscard]] VkImageView viewportImageView() const noexcept;
        [[nodiscard]] bool fillModeNonSolidSupported() const noexcept;
        [[nodiscard]] const DrawFrameCpuTelemetry& lastDrawFrameCpuTelemetry()
            const noexcept;
        [[nodiscard]] double lastFrameCompletionWaitMilliseconds()
            const noexcept;
        [[nodiscard]] const std::filesystem::path& runtimeAssetRoot() const noexcept;
        [[nodiscard]] std::optional<HardwareAssetLoadStatus>
        hardwareAssetLoadStatus(std::string_view identifier) const;

        [[nodiscard]] VmaAllocator allocator() const noexcept;
        [[nodiscard]] bool shaderFloat64Enabled() const noexcept;

    private:
        FrameRenderCallback frameRenderCallback_ = nullptr;
        void* frameRenderUserData_ = nullptr;
        void selectPhysicalDevice();
        void createDevice();
        [[nodiscard]] bool createSwapchain();
        void createVertexBuffers(
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve,
            const coaster::RenderableTrack& renderableTrack
        );
        void createGraphicsPipeline();
        void createTrackPipelines();
        void createSkyPipeline();
        void createEnvironmentResources();
        void destroyEnvironmentResources() noexcept;
        void createViewportTarget(std::uint32_t width, std::uint32_t height);
        void createCommandResources();
        void createSynchronizationResources();
        [[nodiscard]] std::uint32_t currentFrameSlot() const noexcept;
        void waitForFrameCompletion();
        void reserveDeferredBufferRetirements(std::size_t additionalCount);
        void deferBufferRetirement(
            VkBuffer buffer,
            VmaAllocation allocation,
            VkDeviceSize capacity
        ) noexcept;
        void reclaimDeferredBuffers(std::uint32_t frameSlot) noexcept;
        [[nodiscard]] std::size_t deferredBufferCount(
            std::uint32_t frameSlot) const noexcept;
        [[nodiscard]] VkDeviceSize deferredBufferBytes(
            std::uint32_t frameSlot) const noexcept;
        void uploadRenderableTrackMesh(
            const coaster::ContinuousTrackMesh& mesh,
            std::span<const coaster::TrackMaterial> materials);
        void uploadTrackHardware(
            std::span<const coaster::HardwareInstanceBatch> batches);
        [[nodiscard]] double waitForFrameSlot(std::uint32_t frameSlot);
        void updateTrainPreviewFrameBuffer(std::uint32_t frameSlot);
        void recreateSwapchain();
        void recordDrawCommands(
            std::uint32_t frameSlot,
            std::uint32_t imageIndex,
            FrameRenderCallback renderCallback,
            void* userData,
            bool readback
        );
        void prepareFrameReadback();
        void destroyViewportTarget() noexcept;
        void destroySwapchain() noexcept;
        [[nodiscard]] StaticMeshGpuHandle uploadStaticMeshOnce(
            const StaticMeshAsset& asset);

        // Regenerates the grid/axes vertices in place through the retained
        // persistent mapping of the static viewport-aid buffer.
        void rewriteViewportAidVertices(
            float centerX,
            float centerY,
            float spacing
        );

        SDL_Window* window_ = nullptr;
        VkInstance instance_ = VK_NULL_HANDLE;
        VkDebugUtilsMessengerEXT debugMessenger_ = VK_NULL_HANDLE;
        VkSurfaceKHR surface_ = VK_NULL_HANDLE;
        VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
        VkDevice device_ = VK_NULL_HANDLE;
        VmaAllocator allocator_ = VK_NULL_HANDLE;
        std::uint32_t graphicsQueueFamily_ = 0;
        std::uint32_t presentQueueFamily_ = 0;
        VkQueue graphicsQueue_ = VK_NULL_HANDLE;
        VkQueue presentQueue_ = VK_NULL_HANDLE;

        VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
        VkFormat swapchainFormat_ = VK_FORMAT_UNDEFINED;
        VkExtent2D swapchainExtent_{};
        std::vector<VkImage> swapchainImages_;
        std::vector<VkImageView> swapchainImageViews_;
        std::vector<std::uint8_t> swapchainImageInitialized_;
        std::uint64_t swapchainGeneration_ = 0;

        bool frameReadbackEnabled_ = false;
        VkBuffer readbackBuffer_ = VK_NULL_HANDLE;
        VmaAllocation readbackAllocation_ = VK_NULL_HANDLE;
        void* readbackMappedData_ = nullptr;
        VkDeviceSize readbackSize_ = 0;

        VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline graphicsPipeline_ = VK_NULL_HANDLE;
        VkPipelineLayout trackPipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline trackShadedPipeline_ = VK_NULL_HANDLE;
        VkPipeline trackEdgePipeline_ = VK_NULL_HANDLE;
        VkPipeline hardwareShadedPipeline_ = VK_NULL_HANDLE;
        VkPipeline hardwareEdgePipeline_ = VK_NULL_HANDLE;
        VkPipelineLayout skyPipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline skyPipeline_ = VK_NULL_HANDLE;
        VkDescriptorSetLayout environmentDescriptorLayout_ = VK_NULL_HANDLE;
        VkDescriptorPool environmentDescriptorPool_ = VK_NULL_HANDLE;
        VkDescriptorSet environmentDescriptorSet_ = VK_NULL_HANDLE;
        VkSampler environmentSampler_ = VK_NULL_HANDLE;
        struct EnvironmentImage
        {
            VkImage image = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
        };
        std::array<EnvironmentImage, 4> environmentImages_{};
        bool environmentAvailable_ = false;
        bool environmentEnabled_ = true;
        bool skyVisible_ = true;
        float environmentRotationRadians_ = 0.0F;
        float environmentIntensity_ = 0.35F;

        VkImage viewportImage_ = VK_NULL_HANDLE;
        VmaAllocation viewportAllocation_ = VK_NULL_HANDLE;
        VkImageView viewportImageView_ = VK_NULL_HANDLE;
        VkImage viewportMsaaImage_ = VK_NULL_HANDLE;
        VmaAllocation viewportMsaaAllocation_ = VK_NULL_HANDLE;
        VkImageView viewportMsaaImageView_ = VK_NULL_HANDLE;
        VkImage viewportDepthImage_ = VK_NULL_HANDLE;
        VmaAllocation viewportDepthAllocation_ = VK_NULL_HANDLE;
        VkImageView viewportDepthImageView_ = VK_NULL_HANDLE;
        VkExtent2D viewportExtent_{};
        bool viewportImageInitialized_ = false;
        bool viewportMsaa4Supported_ = false;
        VkSampleCountFlagBits viewportSamples_ = VK_SAMPLE_COUNT_1_BIT;
        std::array<float, 16> viewportViewProjection_{
            1.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F
        };
        glm::vec3 viewportCameraPosition_{0.0F, -20.0F, 10.0F};
        glm::vec3 sunlightDirection_{-0.45F, -0.35F, 0.82F};
        float sunlightIntensity_ = 3.0F;
        float exposure_ = 1.0F;

        VkBuffer staticVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation staticVertexAllocation_ = VK_NULL_HANDLE;
        void* staticVertexMappedData_ = nullptr;
        VkDeviceSize staticVertexCapacity_ = 0;
        std::uint32_t staticVertexCount_ = 0;
        // Grid placement currently written into the aid buffer, so the
        // reference update only rewrites when the snapped values change.
        float viewportAidCenterX_ = 0.0F;
        float viewportAidCenterY_ = 0.0F;
        float viewportAidSpacing_ = 0.0F;
        bool viewportGridVisible_ = true;
        std::uint32_t viewportCurveVisibilityMask_ =
            viewportAllCurvesVisibleMask;
        std::uint32_t trackHighlightFirstVertex_ = 0;
        std::uint32_t trackHighlightVertexCount_ = 0;
        TrackPresentationState trackPresentation_;
        bool fillModeNonSolidSupported_ = false;
        VkBuffer trackCurveVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackCurveVertexAllocation_ = VK_NULL_HANDLE;
        void* trackCurveVertexMappedData_ = nullptr;
        VkDeviceSize trackCurveVertexCapacity_ = 0;
        std::uint32_t trackCurveVertexCount_ = 0;
        std::uint32_t trackVerticesPerCurve_ = 0;
        VkBuffer supportVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation supportVertexAllocation_ = VK_NULL_HANDLE;
        void* supportVertexMappedData_ = nullptr;
        VkDeviceSize supportVertexCapacity_ = 0;
        std::uint32_t supportVertexCount_ = 0;

        struct DynamicLineFrameBuffer
        {
            VkBuffer vertexBuffer = VK_NULL_HANDLE;
            VmaAllocation vertexAllocation = VK_NULL_HANDLE;
            void* vertexMappedData = nullptr;
            VkDeviceSize vertexCapacity = 0;
            std::uint32_t vertexCount = 0;
            bool requiresUpdate = false;
        };

        // Each in-flight frame owns the preview allocation it records. The
        // retained CPU vertices let a slot catch up after a one-shot update.
        std::array<DynamicLineFrameBuffer, maxFramesInFlight>
            trainPreviewFrameBuffers_{};
        std::vector<LineVertex> trainPreviewVertices_;

        VkBuffer trackMeshVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackMeshVertexAllocation_ = VK_NULL_HANDLE;
        void* trackMeshVertexMappedData_ = nullptr;
        VkDeviceSize trackMeshVertexCapacity_ = 0;
        std::uint32_t trackMeshVertexCount_ = 0;
        VkBuffer trackTriangleIndexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackTriangleIndexAllocation_ = VK_NULL_HANDLE;
        void* trackTriangleIndexMappedData_ = nullptr;
        VkDeviceSize trackTriangleIndexCapacity_ = 0;
        std::uint32_t trackTriangleIndexCount_ = 0;
        VkBuffer trackEdgeIndexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackEdgeIndexAllocation_ = VK_NULL_HANDLE;
        void* trackEdgeIndexMappedData_ = nullptr;
        VkDeviceSize trackEdgeIndexCapacity_ = 0;
        std::uint32_t trackEdgeIndexCount_ = 0;
        struct TrackDrawBatch
        {
            std::uint32_t firstIndex = 0;
            std::uint32_t indexCount = 0;
            coaster::TrackMaterial material;
        };
        std::vector<TrackDrawBatch> trackDrawBatches_;

        struct GpuStaticMesh
        {
            VkBuffer vertexBuffer = VK_NULL_HANDLE;
            VmaAllocation vertexAllocation = VK_NULL_HANDLE;
            VkBuffer triangleIndexBuffer = VK_NULL_HANDLE;
            VmaAllocation triangleIndexAllocation = VK_NULL_HANDLE;
            VkBuffer edgeIndexBuffer = VK_NULL_HANDLE;
            VmaAllocation edgeIndexAllocation = VK_NULL_HANDLE;
            std::uint32_t vertexCount = 0;
            std::uint32_t triangleIndexCount = 0;
            std::uint32_t edgeIndexCount = 0;
            std::vector<StaticMeshSubmesh> submeshes;
        };

        // Instances remain one contiguous GPU stream. Each batch selects one
        // shared cached mesh and an instance range rather than owning buffers.
        struct HardwareDrawBatch
        {
            StaticMeshGpuHandle mesh;
            std::uint32_t firstInstance = 0;
            std::uint32_t instanceCount = 0;
            std::optional<coaster::TrackMaterial> materialOverride;
        };
        StaticMeshAssetCache staticMeshAssets_;
        StaticMeshGpuHandleCache staticMeshGpuHandles_;
        std::vector<GpuStaticMesh> hardwareMeshes_;
        std::vector<std::uint32_t> availableHardwareMeshHandles_;
        std::vector<HardwareAssetLoadStatus> hardwareAssetLoadStatuses_;
        VkBuffer hardwareInstanceBuffer_ = VK_NULL_HANDLE;
        VmaAllocation hardwareInstanceAllocation_ = VK_NULL_HANDLE;
        void* hardwareInstanceMappedData_ = nullptr;
        VkDeviceSize hardwareInstanceCapacity_ = 0;
        std::uint32_t hardwareInstanceCount_ = 0;
        std::vector<HardwareDrawBatch> hardwareDrawBatches_;

        VkCommandPool commandPool_ = VK_NULL_HANDLE;
        std::array<VkCommandBuffer, maxFramesInFlight> commandBuffers_{};
        VkQueryPool frameTimestampQueryPool_ = VK_NULL_HANDLE;
        float timestampPeriodNanoseconds_ = 0.0F;
        std::uint32_t timestampValidBits_ = 0;
        std::array<bool, maxFramesInFlight> frameTimestampSubmitted_{};

        std::array<VkSemaphore, maxFramesInFlight> imageAvailableSemaphores_{};
        std::vector<VkSemaphore> renderFinishedSemaphores_;
        std::array<VkFence, maxFramesInFlight> frameFences_{};
        struct DeferredBuffer
        {
            VkBuffer buffer = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkDeviceSize capacity = 0;
        };

        // A displaced retained buffer stays attached to the frame slot whose
        // most recent submission is its final possible user. This follows from
        // the intentional one-frame policy above; the slot fence is waited
        // before these allocations are reclaimed.
        static_assert(maxFramesInFlight == 1);
        std::array<std::vector<DeferredBuffer>, maxFramesInFlight>
            deferredBuffers_{};
        std::uint32_t frameIndex_ = 0;
        DrawFrameCpuTelemetry lastDrawFrameCpuTelemetry_;
        double lastFrameCompletionWaitMilliseconds_ = 0.0;
        std::array<FrameSubmissionTelemetry, maxFramesInFlight> frameSubmissions_{};
        std::uint64_t drawAttemptId_ = 0;

        bool shaderFloat64Supported_ = false;
        bool shaderFloat64Enabled_ = false;
    };
}
