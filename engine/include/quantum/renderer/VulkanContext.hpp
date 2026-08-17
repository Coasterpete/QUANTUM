#pragma once

#include <SDL3/SDL.h>
#include <vulkan/vulkan.h>

namespace quantum::renderer
{
    class VulkanContext
    {
    public:
        VulkanContext() = default;
        ~VulkanContext();

        VulkanContext(const VulkanContext&) = delete;
        VulkanContext& operator=(const VulkanContext&) = delete;

        void initialize(SDL_Window* window);
        void shutdown();

        [[nodiscard]] VkInstance instance() const noexcept;

    private:
        VkInstance instance_ = VK_NULL_HANDLE;
        VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    };
}