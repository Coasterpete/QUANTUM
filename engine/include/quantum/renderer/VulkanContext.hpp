#pragma once

#include <SDL3/SDL.h>
#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace quantum::renderer
{
    // One endpoint of a viewport line-list segment. Every viewport geometry
    // stream (ground grid, world axes, authored-track reference curves) uses
    // this vertex layout, matching the graphics pipeline's vertex input.
    struct LineVertex
    {
        float x;
        float y;
        float z;
        std::array<float, 4> color;
    };

    // The track-curve stream concatenates four equal-length reference-curve
    // runs in this order; visibility bits address these indices.
    inline constexpr std::uint32_t viewportLeftRailCurve = 0;
    inline constexpr std::uint32_t viewportRightRailCurve = 1;
    inline constexpr std::uint32_t viewportCenterlineCurve = 2;
    inline constexpr std::uint32_t viewportHeartlineCurve = 3;
    inline constexpr std::uint32_t viewportCurveCount = 4;
    inline constexpr std::uint32_t viewportAllCurvesVisibleMask = 0xFu;

    class VulkanContext
    {
    public:
        using FrameRenderCallback = void (*)(VkCommandBuffer, void*);
        using ViewportTargetRetirementCallback = void (*)(void*) noexcept;

        VulkanContext() = default;
        ~VulkanContext();

        VulkanContext(const VulkanContext&) = delete;
        VulkanContext& operator=(const VulkanContext&) = delete;

        // The track-curve vertices must consist of exactly four runs of
        // `trackVerticesPerCurve` vertices each.
        void initialize(
            SDL_Window* window,
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve
        );
        void drawFrame(
            FrameRenderCallback renderCallback = nullptr,
            void* userData = nullptr
        );
        void resizeViewportTarget(
            std::uint32_t width,
            std::uint32_t height,
            ViewportTargetRetirementCallback retirementCallback = nullptr,
            void* userData = nullptr
        );
        void setViewportViewProjection(
            const std::array<float, 16>& viewProjection
        );
        void updateTrackCurveVertices(
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve
        );

        // Host-side draw skipping for the viewport reference elements.
        // Idempotent; intended to be pushed every frame from the editor's
        // authoritative settings like the view-projection matrix.
        void setViewportElementVisibility(
            bool gridVisible,
            std::uint32_t curveVisibilityMask
        );

        // Recenters and rescales the ground grid toward a reference sphere
        // (usually the solved-track bounds), snapping to the chosen spacing
        // so the grid only rewrites its buffer when it actually moves.
        void updateViewportAidReference(
            float centerX,
            float centerY,
            float referenceRadius
        );
        void shutdown() noexcept;

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

    private:
        void selectPhysicalDevice();
        void createDevice();
        [[nodiscard]] bool createSwapchain();
        void createVertexBuffers(
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve
        );
        void createGraphicsPipeline();
        void createViewportTarget(std::uint32_t width, std::uint32_t height);
        void createCommandResources();
        void createSynchronizationResources();
        void waitForFrameCompletion();
        void recreateSwapchain();
        void recordDrawCommands(
            std::uint32_t imageIndex,
            FrameRenderCallback renderCallback,
            void* userData
        );
        void destroyViewportTarget() noexcept;
        void destroySwapchain() noexcept;

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

        VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline graphicsPipeline_ = VK_NULL_HANDLE;

        VkImage viewportImage_ = VK_NULL_HANDLE;
        VmaAllocation viewportAllocation_ = VK_NULL_HANDLE;
        VkImageView viewportImageView_ = VK_NULL_HANDLE;
        VkImage viewportDepthImage_ = VK_NULL_HANDLE;
        VmaAllocation viewportDepthAllocation_ = VK_NULL_HANDLE;
        VkImageView viewportDepthImageView_ = VK_NULL_HANDLE;
        VkExtent2D viewportExtent_{};
        bool viewportImageInitialized_ = false;
        std::array<float, 16> viewportViewProjection_{
            1.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F
        };

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
        VkBuffer trackCurveVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackCurveVertexAllocation_ = VK_NULL_HANDLE;
        void* trackCurveVertexMappedData_ = nullptr;
        VkDeviceSize trackCurveVertexCapacity_ = 0;
        std::uint32_t trackCurveVertexCount_ = 0;
        std::uint32_t trackVerticesPerCurve_ = 0;
        VkBuffer spareTrackCurveVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation spareTrackCurveVertexAllocation_ = VK_NULL_HANDLE;
        void* spareTrackCurveVertexMappedData_ = nullptr;
        VkDeviceSize spareTrackCurveVertexCapacity_ = 0;

        VkCommandPool commandPool_ = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;

        VkSemaphore imageAvailableSemaphore_ = VK_NULL_HANDLE;
        std::vector<VkSemaphore> renderFinishedSemaphores_;
        VkFence frameFence_ = VK_NULL_HANDLE;
    };
}
