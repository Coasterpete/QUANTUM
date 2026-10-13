// Vulkan-specific ImGui lifetime, texture registration and command recording.
// Workbench widgets submit ImGui data; the existing Vulkan renderer owns frames.
#include <quantum/editor/EditorUi.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>
#include <SDL3/SDL_video.h>
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include <cstdint>
#include <stdexcept>

namespace
{
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
}

namespace quantum::editor
{
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

    void EditorUi::updateViewportTexture(
        renderer::VulkanContext& vulkan,
        const std::uint32_t width,
        const std::uint32_t height)
    {
        const VkExtent2D currentExtent = vulkan.viewportExtent();

        if (currentExtent.width != width || currentExtent.height != height
            || vulkan.viewportMsaaEnabled()
                != (viewportSettings_.msaaEnabled
                    && vulkan.capabilities().viewportMsaa4))
        {
            frameBlockingEvents_.viewportResized = true;
            renderer::Renderer& renderer = vulkan;
            renderer.resizeViewportTarget(
                width,
                height,
                &EditorUi::retireViewportTexture,
                this,
                viewportSettings_.msaaEnabled
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

    void EditorUi::installFrameRenderCallback(
        renderer::VulkanContext& vulkan) noexcept
    {
        frameRenderer_ = &vulkan;
        vulkan.setFrameRenderCallback(
            [](VkCommandBuffer commandBuffer, void* userData)
            {
                static_cast<EditorUi*>(userData)->render(commandBuffer);
            },
            this);
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

    void EditorUi::beginVulkanFrame()
    {
        ImGui_ImplVulkan_NewFrame();
    }
}
