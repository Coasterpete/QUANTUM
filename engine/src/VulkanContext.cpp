#include <quantum/renderer/VulkanContext.hpp>
#include <SDL3/SDL_vulkan.h>

#include <stdexcept>
#include <string>

namespace quantum::renderer
{
    VulkanContext::~VulkanContext()
    {
        shutdown();
    }

    void VulkanContext::initialize(SDL_Window* window)
    {
        VkApplicationInfo appInfo{};

        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "QUANTUM";
        appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);

        appInfo.pEngineName = "QUANTUM Coaster Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
        appInfo.apiVersion = VK_API_VERSION_1_4;

        Uint32 extensionCount = 0;

        const char* const* extensions =
            SDL_Vulkan_GetInstanceExtensions(&extensionCount);

        if (extensions == nullptr)
        {
            throw std::runtime_error(
                std::string("Failed to get Vulkan instance extensions: ")
                + SDL_GetError()
            );
        }

        VkInstanceCreateInfo createInfo{};

        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount = extensionCount;
        createInfo.ppEnabledExtensionNames = extensions;

        const VkResult result =
            vkCreateInstance(&createInfo, nullptr, &instance_);

        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(
                "Failed to create Vulkan instance."
            );
        }

        if (!SDL_Vulkan_CreateSurface(
            window,
            instance_,
            nullptr,
            &surface_))
        {
            throw std::runtime_error(
                std::string("Failed to create Vulkan surface: ")
                + SDL_GetError()
            );
        }
    }

    void VulkanContext::shutdown()
    {
        if (surface_ != VK_NULL_HANDLE)
        {
            vkDestroySurfaceKHR(instance_, surface_, nullptr);
            surface_ = VK_NULL_HANDLE;
        }

        if (instance_ != VK_NULL_HANDLE)
        {
            vkDestroyInstance(instance_, nullptr);
            instance_ = VK_NULL_HANDLE;
        }
    }

    VkInstance VulkanContext::instance() const noexcept
    {
        return instance_;
    }
}
