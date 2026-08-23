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

    class VulkanContext
    {
    public:
        using FrameRenderCallback = void (*)(VkCommandBuffer, void*);
        using ViewportTargetRetirementCallback = void (*)(void*) noexcept;

        VulkanContext() = default;
        ~VulkanContext();

        VulkanContext(const VulkanContext&) = delete;
        VulkanContext& operator=(const VulkanContext&) = delete;

        void initialize(
            SDL_Window* window,
            std::span<const LineVertex> trackCurveVertices
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
            std::span<const LineVertex> trackCurveVertices
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
            std::span<const LineVertex> trackCurveVertices
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
        std::uint32_t staticVertexCount_ = 0;
        VkBuffer trackCurveVertexBuffer_ = VK_NULL_HANDLE;
        VmaAllocation trackCurveVertexAllocation_ = VK_NULL_HANDLE;
        void* trackCurveVertexMappedData_ = nullptr;
        VkDeviceSize trackCurveVertexCapacity_ = 0;
        std::uint32_t trackCurveVertexCount_ = 0;
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
