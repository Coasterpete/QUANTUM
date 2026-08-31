// VMA requires its implementation macro in exactly one translation unit.
#define VMA_IMPLEMENTATION
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>
#include <quantum/renderer/ViewportAids.hpp>
#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_vulkan.h>
#include <SDL3/SDL_video.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    constexpr VkFormat viewportColorFormat = VK_FORMAT_R8G8B8A8_SRGB;
    constexpr VkFormat viewportDepthFormat = VK_FORMAT_D32_SFLOAT;
    constexpr float defaultGridSpacing = 10.0F;

    // Ascending candidate spacings for the adaptive ground grid.
    constexpr std::array<float, 8> viewportAidSpacingCandidates{
        1.0F, 2.0F, 5.0F, 10.0F, 20.0F, 50.0F, 100.0F, 200.0F
    };

    struct CreatedVertexBuffer
    {
        VkBuffer buffer = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        void* mappedData = nullptr;
        VkDeviceSize capacity = 0;
        std::uint32_t vertexCount = 0;
    };

    [[noreturn]] void throwVulkanError(
        std::string_view operation,
        VkResult result
    );

    // The track-curve stream carries complete line segments: each consecutive
    // vertex pair is one independent segment, and the stream concatenates
    // exactly four equal-length curve runs.
    void requireValidTrackCurveVertices(
        const std::span<const quantum::renderer::LineVertex>
            trackCurveVertices,
        const std::uint32_t verticesPerCurve)
    {
        if (trackCurveVertices.empty() && verticesPerCurve == 0)
        {
            return;
        }

        const std::size_t runLength = verticesPerCurve;

        if (verticesPerCurve < 2 || verticesPerCurve % 2 != 0
            || trackCurveVertices.size() < 2
            || trackCurveVertices.size() % 2 != 0
            || trackCurveVertices.size() != runLength
                * quantum::renderer::viewportCurveCount)
        {
            throw std::invalid_argument(
                "VulkanContext requires track-curve vertices to form four equal-length curve runs of complete line segments."
            );
        }

        constexpr std::size_t maximumVertexCount =
            std::numeric_limits<std::uint32_t>::max();

        if (trackCurveVertices.size() > maximumVertexCount)
        {
            throw std::length_error(
                "The track-curve line vertex count exceeds Vulkan's draw limit."
            );
        }

        for (const quantum::renderer::LineVertex& vertex
            : trackCurveVertices)
        {
            if (!std::isfinite(vertex.x)
                || !std::isfinite(vertex.y)
                || !std::isfinite(vertex.z))
            {
                throw std::invalid_argument(
                    "VulkanContext cannot upload a non-finite track-curve vertex."
                );
            }
        }
    }

    CreatedVertexBuffer createHostVisibleVertexBuffer(
        const VmaAllocator allocator,
        const std::span<const quantum::renderer::LineVertex> vertices)
    {
        if (vertices.empty()
            || vertices.size() > std::numeric_limits<std::uint32_t>::max())
        {
            throw std::length_error(
                "The viewport vertex count is outside Vulkan's draw range."
            );
        }

        const VkDeviceSize vertexDataSize = sizeof(quantum::renderer::LineVertex)
            * vertices.size();

        VkBufferCreateInfo bufferCreateInfo{};
        bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferCreateInfo.size = vertexDataSize;
        bufferCreateInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocationCreateInfo{};
        allocationCreateInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;

        CreatedVertexBuffer created;
        VmaAllocationInfo allocationInfo{};
        VkResult result = vmaCreateBuffer(
            allocator,
            &bufferCreateInfo,
            &allocationCreateInfo,
            &created.buffer,
            &created.allocation,
            &allocationInfo
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vmaCreateBuffer", result);
        }

        if (allocationInfo.pMappedData == nullptr)
        {
            vmaDestroyBuffer(allocator, created.buffer, created.allocation);
            throw std::runtime_error(
                "VMA did not map the host-visible vertex-buffer allocation."
            );
        }

        std::memcpy(
            allocationInfo.pMappedData,
            vertices.data(),
            vertexDataSize
        );

        result = vmaFlushAllocation(
            allocator,
            created.allocation,
            0,
            vertexDataSize
        );

        if (result != VK_SUCCESS)
        {
            vmaDestroyBuffer(allocator, created.buffer, created.allocation);
            throwVulkanError("vmaFlushAllocation", result);
        }

        created.vertexCount = static_cast<std::uint32_t>(vertices.size());
        created.mappedData = allocationInfo.pMappedData;
        created.capacity = vertexDataSize;
        return created;
    }

    void writeHostVisibleVertexBuffer(
        const VmaAllocator allocator,
        const VmaAllocation allocation,
        void* const mappedData,
        const VkDeviceSize capacity,
        const std::span<const quantum::renderer::LineVertex> vertices)
    {
        const VkDeviceSize vertexDataSize = sizeof(quantum::renderer::LineVertex)
            * vertices.size();

        if (allocation == VK_NULL_HANDLE || mappedData == nullptr
            || vertices.empty() || vertexDataSize > capacity)
        {
            throw std::logic_error(
                "The reusable track-curve vertex buffer is invalid or too small."
            );
        }

        std::memcpy(mappedData, vertices.data(), vertexDataSize);
        const VkResult result = vmaFlushAllocation(
            allocator,
            allocation,
            0,
            vertexDataSize
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vmaFlushAllocation", result);
        }
    }

#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
    constexpr char validationLayerName[] = "VK_LAYER_KHRONOS_validation";

    bool instanceLayerAvailable(const char* const requestedLayer)
    {
        std::uint32_t layerCount = 0;
        VkResult result = vkEnumerateInstanceLayerProperties(
            &layerCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Unable to query Vulkan instance layers (VkResult %d); "
                "validation will be disabled.",
                static_cast<int>(result)
            );
            return false;
        }

        std::vector<VkLayerProperties> layers(layerCount);
        result = vkEnumerateInstanceLayerProperties(
            &layerCount,
            layers.data()
        );

        if (result != VK_SUCCESS)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Unable to enumerate Vulkan instance layers (VkResult %d); "
                "validation will be disabled.",
                static_cast<int>(result)
            );
            return false;
        }

        return std::ranges::any_of(
            layers,
            [requestedLayer](const VkLayerProperties& layer)
            {
                return std::strcmp(layer.layerName, requestedLayer) == 0;
            }
        );
    }

    bool instanceExtensionAvailable(
        const char* const requestedExtension)
    {
        std::uint32_t extensionCount = 0;
        VkResult result = vkEnumerateInstanceExtensionProperties(
            nullptr,
            &extensionCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Unable to query Vulkan instance extensions (VkResult %d); "
                "validation will be disabled.",
                static_cast<int>(result)
            );
            return false;
        }

        std::vector<VkExtensionProperties> extensions(extensionCount);
        result = vkEnumerateInstanceExtensionProperties(
            nullptr,
            &extensionCount,
            extensions.data()
        );

        if (result != VK_SUCCESS)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Unable to enumerate Vulkan instance extensions "
                "(VkResult %d); validation will be disabled.",
                static_cast<int>(result)
            );
            return false;
        }

        return std::ranges::any_of(
            extensions,
            [requestedExtension](const VkExtensionProperties& extension)
            {
                return std::strcmp(
                    extension.extensionName,
                    requestedExtension
                ) == 0;
            }
        );
    }

    VKAPI_ATTR VkBool32 VKAPI_CALL validationMessageCallback(
        const VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        const VkDebugUtilsMessageTypeFlagsEXT messageTypes,
        const VkDebugUtilsMessengerCallbackDataEXT* const callbackData,
        void*) noexcept
    {
        quantum::logging::LogLevel level =
            quantum::logging::LogLevel::Debug;

        if ((messageSeverity
            & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0)
        {
            level = quantum::logging::LogLevel::Error;
        }
        else if ((messageSeverity
            & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) != 0)
        {
            level = quantum::logging::LogLevel::Warning;
        }
        else if ((messageSeverity
            & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) != 0)
        {
            // General/loader info messages remain available when development
            // diagnostics are enabled without cluttering normal output.
            level = quantum::logging::LogLevel::Debug;
        }
        else if ((messageSeverity
            & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) != 0)
        {
            level = quantum::logging::LogLevel::Trace;
        }

        const char* category = "VK:general";

        if ((messageTypes & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT)
            != 0)
        {
            category = "VK:validation";
        }
        else if ((messageTypes
            & VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT) != 0)
        {
            category = "VK:performance";
        }

        const char* const message = callbackData != nullptr
            && callbackData->pMessage != nullptr
            ? callbackData->pMessage
            : "No message was provided.";

        const char* const source = callbackData != nullptr
            && callbackData->pMessageIdName != nullptr
            ? callbackData->pMessageIdName
            : "unspecified";

        quantum::logging::logMessagef(
            level,
            category,
            "source=%s: %s",
            source,
            message
        );

        return VK_FALSE;
    }

    VkDebugUtilsMessengerCreateInfoEXT debugMessengerCreateInfo() noexcept
    {
        VkDebugUtilsMessengerCreateInfoEXT createInfo{};
        createInfo.sType =
            VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        createInfo.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType =
            VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
            | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = validationMessageCallback;
        return createInfo;
    }

    VkResult createDebugMessenger(
        const VkInstance instance,
        const VkDebugUtilsMessengerCreateInfoEXT& createInfo,
        VkDebugUtilsMessengerEXT* const messenger) noexcept
    {
        const auto createFunction =
            reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
                vkGetInstanceProcAddr(
                    instance,
                    "vkCreateDebugUtilsMessengerEXT"
                )
            );

        if (createFunction == nullptr)
        {
            return VK_ERROR_EXTENSION_NOT_PRESENT;
        }

        return createFunction(instance, &createInfo, nullptr, messenger);
    }

    void destroyDebugMessenger(
        const VkInstance instance,
        const VkDebugUtilsMessengerEXT messenger) noexcept
    {
        const auto destroyFunction =
            reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
                vkGetInstanceProcAddr(
                    instance,
                    "vkDestroyDebugUtilsMessengerEXT"
                )
            );

        if (destroyFunction != nullptr)
        {
            destroyFunction(instance, messenger, nullptr);
        }
    }
#endif

    struct QueueFamilyIndices
    {
        std::optional<std::uint32_t> graphics;
        std::optional<std::uint32_t> present;

        [[nodiscard]] bool complete() const noexcept
        {
            return graphics.has_value() && present.has_value();
        }
    };

    struct SwapchainSupport
    {
        VkSurfaceCapabilitiesKHR capabilities{};
        std::vector<VkSurfaceFormatKHR> formats;
        std::vector<VkPresentModeKHR> presentModes;
    };

    static_assert(sizeof(std::array<float, 16>) == 16 * sizeof(float));

    [[noreturn]] void throwVulkanError(
        const std::string_view operation,
        const VkResult result)
    {
        throw std::runtime_error(
            std::string(operation)
            + " failed with VkResult "
            + std::to_string(result)
            + "."
        );
    }

    std::filesystem::path shaderPath(const std::string_view fileName)
    {
        const char* const basePath = SDL_GetBasePath();

        if (basePath == nullptr)
        {
            throw std::runtime_error(
                std::string("SDL_GetBasePath failed: ") + SDL_GetError()
            );
        }

        return std::filesystem::path(basePath) / "shaders" / fileName;
    }

    std::vector<std::uint32_t> readSpirv(
        const std::filesystem::path& path)
    {
        std::ifstream file(path, std::ios::binary | std::ios::ate);

        if (!file)
        {
            throw std::runtime_error(
                "Unable to open shader: " + path.string()
            );
        }

        const std::streampos endPosition = file.tellg();

        if (endPosition <= 0
            || (static_cast<std::uint64_t>(endPosition) % sizeof(std::uint32_t))
                != 0)
        {
            throw std::runtime_error(
                "Shader is empty or has an invalid SPIR-V byte size: "
                + path.string()
            );
        }

        const auto byteCount = static_cast<std::size_t>(endPosition);
        std::vector<std::uint32_t> code(
            byteCount / sizeof(std::uint32_t)
        );

        file.seekg(0);
        file.read(
            reinterpret_cast<char*>(code.data()),
            static_cast<std::streamsize>(byteCount)
        );

        if (!file)
        {
            throw std::runtime_error(
                "Unable to read shader: " + path.string()
            );
        }

        return code;
    }

    VkShaderModule createShaderModule(
        const VkDevice device,
        const std::vector<std::uint32_t>& code)
    {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size() * sizeof(std::uint32_t);
        createInfo.pCode = code.data();

        VkShaderModule shaderModule = VK_NULL_HANDLE;
        const VkResult result = vkCreateShaderModule(
            device,
            &createInfo,
            nullptr,
            &shaderModule
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateShaderModule", result);
        }

        return shaderModule;
    }

    QueueFamilyIndices findQueueFamilies(
        const VkPhysicalDevice device,
        const VkSurfaceKHR surface)
    {
        std::uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(
            device,
            &queueFamilyCount,
            nullptr
        );

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(
            device,
            &queueFamilyCount,
            queueFamilies.data()
        );

        QueueFamilyIndices indices;

        for (std::uint32_t index = 0; index < queueFamilyCount; ++index)
        {
            if ((queueFamilies[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0)
            {
                indices.graphics = index;
            }

            VkBool32 supportsPresentation = VK_FALSE;
            const VkResult result = vkGetPhysicalDeviceSurfaceSupportKHR(
                device,
                index,
                surface,
                &supportsPresentation
            );

            if (result != VK_SUCCESS)
            {
                throwVulkanError(
                    "vkGetPhysicalDeviceSurfaceSupportKHR",
                    result
                );
            }

            if (supportsPresentation == VK_TRUE)
            {
                indices.present = index;
            }

            if (indices.complete())
            {
                break;
            }
        }

        return indices;
    }

    bool supportsSwapchainExtension(const VkPhysicalDevice device)
    {
        std::uint32_t extensionCount = 0;
        VkResult result = vkEnumerateDeviceExtensionProperties(
            device,
            nullptr,
            &extensionCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError(
                "vkEnumerateDeviceExtensionProperties",
                result
            );
        }

        std::vector<VkExtensionProperties> extensions(extensionCount);
        result = vkEnumerateDeviceExtensionProperties(
            device,
            nullptr,
            &extensionCount,
            extensions.data()
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError(
                "vkEnumerateDeviceExtensionProperties",
                result
            );
        }

        return std::ranges::any_of(
            extensions,
            [](const VkExtensionProperties& extension)
            {
                return std::strcmp(
                    extension.extensionName,
                    VK_KHR_SWAPCHAIN_EXTENSION_NAME
                ) == 0;
            }
        );
    }

    bool supportsViewportColorTarget(const VkPhysicalDevice device) noexcept
    {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(
            device,
            viewportColorFormat,
            &properties
        );

        constexpr VkFormatFeatureFlags requiredFeatures =
            VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT
            | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT;

        return (properties.optimalTilingFeatures & requiredFeatures)
            == requiredFeatures;
    }

    bool supportsViewportDepthTarget(const VkPhysicalDevice device) noexcept
    {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(
            device,
            viewportDepthFormat,
            &properties
        );

        return (properties.optimalTilingFeatures
            & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0;
    }

    SwapchainSupport querySwapchainSupport(
        const VkPhysicalDevice device,
        const VkSurfaceKHR surface)
    {
        SwapchainSupport support;
        VkResult result = vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
            device,
            surface,
            &support.capabilities
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError(
                "vkGetPhysicalDeviceSurfaceCapabilitiesKHR",
                result
            );
        }

        std::uint32_t formatCount = 0;
        result = vkGetPhysicalDeviceSurfaceFormatsKHR(
            device,
            surface,
            &formatCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkGetPhysicalDeviceSurfaceFormatsKHR", result);
        }

        support.formats.resize(formatCount);
        result = vkGetPhysicalDeviceSurfaceFormatsKHR(
            device,
            surface,
            &formatCount,
            support.formats.data()
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkGetPhysicalDeviceSurfaceFormatsKHR", result);
        }

        std::uint32_t presentModeCount = 0;
        result = vkGetPhysicalDeviceSurfacePresentModesKHR(
            device,
            surface,
            &presentModeCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError(
                "vkGetPhysicalDeviceSurfacePresentModesKHR",
                result
            );
        }

        support.presentModes.resize(presentModeCount);
        result = vkGetPhysicalDeviceSurfacePresentModesKHR(
            device,
            surface,
            &presentModeCount,
            support.presentModes.data()
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError(
                "vkGetPhysicalDeviceSurfacePresentModesKHR",
                result
            );
        }

        return support;
    }

    VkSurfaceFormatKHR chooseSurfaceFormat(
        const std::vector<VkSurfaceFormatKHR>& formats)
    {
        if (formats.size() == 1 && formats.front().format == VK_FORMAT_UNDEFINED)
        {
            return {
                VK_FORMAT_B8G8R8A8_SRGB,
                VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
            };
        }

        const auto preferred = std::ranges::find_if(
            formats,
            [](const VkSurfaceFormatKHR& format)
            {
                return format.format == VK_FORMAT_B8G8R8A8_SRGB
                    && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
            }
        );

        return preferred != formats.end() ? *preferred : formats.front();
    }

    VkExtent2D chooseSwapchainExtent(
        SDL_Window* const window,
        const VkSurfaceCapabilitiesKHR& capabilities)
    {
        if (capabilities.currentExtent.width
            != std::numeric_limits<std::uint32_t>::max())
        {
            return capabilities.currentExtent;
        }

        int width = 0;
        int height = 0;

        if (!SDL_GetWindowSizeInPixels(window, &width, &height))
        {
            throw std::runtime_error(
                std::string("SDL_GetWindowSizeInPixels failed: ")
                + SDL_GetError()
            );
        }

        return {
            std::clamp(
                static_cast<std::uint32_t>(width),
                capabilities.minImageExtent.width,
                capabilities.maxImageExtent.width
            ),
            std::clamp(
                static_cast<std::uint32_t>(height),
                capabilities.minImageExtent.height,
                capabilities.maxImageExtent.height
            )
        };
    }

    VkCompositeAlphaFlagBitsKHR chooseCompositeAlpha(
        const VkSurfaceCapabilitiesKHR& capabilities)
    {
        constexpr std::array choices{
            VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
            VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR,
            VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR,
            VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR
        };

        for (const VkCompositeAlphaFlagBitsKHR choice : choices)
        {
            if ((capabilities.supportedCompositeAlpha & choice) != 0)
            {
                return choice;
            }
        }

        throw std::runtime_error(
            "The Vulkan surface does not support a composite alpha mode."
        );
    }
}

namespace quantum::renderer
{
    VulkanContext::~VulkanContext()
    {
        shutdown();
    }

    void VulkanContext::initialize(
        SDL_Window* window,
        const std::span<const LineVertex> trackCurveVertices,
        const std::uint32_t trackVerticesPerCurve)
    {
        if (window == nullptr)
        {
            throw std::invalid_argument(
                "VulkanContext requires a valid SDL window."
            );
        }

        requireValidTrackCurveVertices(
            trackCurveVertices,
            trackVerticesPerCurve
        );

        window_ = window;

        VkApplicationInfo appInfo{};

        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "QUANTUM";
        appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);

        appInfo.pEngineName = "QUANTUM";
        appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
        appInfo.apiVersion = VK_API_VERSION_1_4;

        Uint32 extensionCount = 0;

        const char* const* sdlExtensions =
            SDL_Vulkan_GetInstanceExtensions(&extensionCount);

        if (sdlExtensions == nullptr)
        {
            throw std::runtime_error(
                std::string("Failed to get Vulkan instance extensions: ")
                + SDL_GetError()
            );
        }

        std::vector<const char*> instanceExtensions(
            sdlExtensions,
            sdlExtensions + extensionCount
        );

#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
        const bool validationLayerAvailable = instanceLayerAvailable(
            validationLayerName
        );
        const bool debugUtilsAvailable = instanceExtensionAvailable(
            VK_EXT_DEBUG_UTILS_EXTENSION_NAME
        );
        const bool validationEnabled = validationLayerAvailable
            && debugUtilsAvailable;

        if (!validationLayerAvailable)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Vulkan validation layer %s is unavailable; continuing "
                "without validation.",
                validationLayerName
            );
        }

        if (!debugUtilsAvailable)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Warning,
                "VK",
                "Vulkan instance extension %s is unavailable; continuing "
                "without validation.",
                VK_EXT_DEBUG_UTILS_EXTENSION_NAME
            );
        }

        if (validationEnabled)
        {
            const bool debugUtilsAlreadyRequired = std::ranges::any_of(
                instanceExtensions,
                [](const char* const extension)
                {
                    return std::strcmp(
                        extension,
                        VK_EXT_DEBUG_UTILS_EXTENSION_NAME
                    ) == 0;
                }
            );

            if (!debugUtilsAlreadyRequired)
            {
                instanceExtensions.push_back(
                    VK_EXT_DEBUG_UTILS_EXTENSION_NAME
                );
            }
        }

        const std::array validationLayers{
            validationLayerName
        };
        VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};

        if (validationEnabled)
        {
            debugCreateInfo = debugMessengerCreateInfo();
        }
#endif

        VkInstanceCreateInfo createInfo{};

        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount =
            static_cast<std::uint32_t>(instanceExtensions.size());
        createInfo.ppEnabledExtensionNames = instanceExtensions.data();

#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
        if (validationEnabled)
        {
            createInfo.enabledLayerCount =
                static_cast<std::uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();
            createInfo.pNext = &debugCreateInfo;
        }
#endif

        const VkResult result =
            vkCreateInstance(&createInfo, nullptr, &instance_);

        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(
                "Failed to create Vulkan instance."
            );
        }

#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
        if (validationEnabled)
        {
            const VkResult messengerResult = createDebugMessenger(
                instance_,
                debugCreateInfo,
                &debugMessenger_
            );

            if (messengerResult != VK_SUCCESS)
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Warning,
                    "VK",
                    "Failed to create the Vulkan debug messenger "
                    "(VkResult %d); continuing without validation output.",
                    static_cast<int>(messengerResult)
                );
                debugMessenger_ = VK_NULL_HANDLE;
            }
        }
#endif

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

        selectPhysicalDevice();
        createDevice();

        VmaAllocatorCreateInfo allocatorCreateInfo{};
        allocatorCreateInfo.instance = instance_;
        allocatorCreateInfo.physicalDevice = physicalDevice_;
        allocatorCreateInfo.device = device_;
        allocatorCreateInfo.vulkanApiVersion = VK_API_VERSION_1_4;

        const VkResult allocatorResult = vmaCreateAllocator(
            &allocatorCreateInfo,
            &allocator_
        );

        if (allocatorResult != VK_SUCCESS)
        {
            throwVulkanError(
                "vmaCreateAllocator",
                allocatorResult
            );
        }

        if (!createSwapchain())
        {
            throw std::runtime_error(
                "The Vulkan swapchain cannot be created for a zero-sized "
                "window."
            );
        }
        createVertexBuffers(trackCurveVertices, trackVerticesPerCurve);
        createGraphicsPipeline();
        createCommandResources();
        createSynchronizationResources();
    }

    void VulkanContext::selectPhysicalDevice()
    {
        std::uint32_t deviceCount = 0;
        VkResult result = vkEnumeratePhysicalDevices(
            instance_,
            &deviceCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkEnumeratePhysicalDevices", result);
        }

        if (deviceCount == 0)
        {
            throw std::runtime_error(
                "No Vulkan physical devices are available."
            );
        }

        std::vector<VkPhysicalDevice> devices(deviceCount);
        result = vkEnumeratePhysicalDevices(
            instance_,
            &deviceCount,
            devices.data()
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkEnumeratePhysicalDevices", result);
        }

        for (const VkPhysicalDevice device : devices)
        {
            const QueueFamilyIndices indices = findQueueFamilies(
                device,
                surface_
            );

            if (!indices.complete()
                || !supportsSwapchainExtension(device)
                || !supportsViewportColorTarget(device)
                || !supportsViewportDepthTarget(device))
            {
                continue;
            }

            VkPhysicalDeviceProperties properties{};
            vkGetPhysicalDeviceProperties(device, &properties);

            if (properties.apiVersion < VK_API_VERSION_1_3)
            {
                continue;
            }

            VkPhysicalDeviceVulkan13Features vulkan13Features{};
            vulkan13Features.sType =
                VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;

            VkPhysicalDeviceFeatures2 features{};
            features.sType =
                VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
            features.pNext = &vulkan13Features;
            vkGetPhysicalDeviceFeatures2(device, &features);

            if (vulkan13Features.dynamicRendering != VK_TRUE)
            {
                continue;
            }

            const SwapchainSupport support = querySwapchainSupport(
                device,
                surface_
            );

            if (support.formats.empty()
                || support.presentModes.empty()
                || (support.capabilities.supportedUsageFlags
                    & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) == 0)
            {
                continue;
            }

            physicalDevice_ = device;
            graphicsQueueFamily_ = *indices.graphics;
            presentQueueFamily_ = *indices.present;
            return;
        }

        throw std::runtime_error(
            "No Vulkan physical device supports graphics, presentation, "
            "dynamic rendering, swapchain color attachments, and the "
            "Editor viewport color/depth formats for this window."
        );
    }

    void VulkanContext::createDevice()
    {
        const float queuePriority = 1.0F;
        const std::array queueFamilies{
            graphicsQueueFamily_,
            presentQueueFamily_
        };
        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;

        for (const std::uint32_t queueFamily : queueFamilies)
        {
            const bool alreadyAdded = std::ranges::any_of(
                queueCreateInfos,
                [queueFamily](const VkDeviceQueueCreateInfo& createInfo)
                {
                    return createInfo.queueFamilyIndex == queueFamily;
                }
            );

            if (alreadyAdded)
            {
                continue;
            }

            VkDeviceQueueCreateInfo queueCreateInfo{};
            queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueCreateInfo.queueFamilyIndex = queueFamily;
            queueCreateInfo.queueCount = 1;
            queueCreateInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueCreateInfo);
        }

        constexpr std::array deviceExtensions{
            VK_KHR_SWAPCHAIN_EXTENSION_NAME
        };

        VkPhysicalDeviceVulkan13Features vulkan13Features{};
        vulkan13Features.sType =
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
        vulkan13Features.dynamicRendering = VK_TRUE;

        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.pNext = &vulkan13Features;
        createInfo.queueCreateInfoCount =
            static_cast<std::uint32_t>(queueCreateInfos.size());
        createInfo.pQueueCreateInfos = queueCreateInfos.data();
        createInfo.enabledExtensionCount =
            static_cast<std::uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();

        const VkResult result = vkCreateDevice(
            physicalDevice_,
            &createInfo,
            nullptr,
            &device_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateDevice", result);
        }

        vkGetDeviceQueue(device_, graphicsQueueFamily_, 0, &graphicsQueue_);
        vkGetDeviceQueue(device_, presentQueueFamily_, 0, &presentQueue_);
    }

    bool VulkanContext::createSwapchain()
    {
        const SwapchainSupport support = querySwapchainSupport(
            physicalDevice_,
            surface_
        );

        if (support.formats.empty() || support.presentModes.empty())
        {
            throw std::runtime_error(
                "The Vulkan surface no longer supports a usable swapchain."
            );
        }

        const VkSurfaceFormatKHR surfaceFormat = chooseSurfaceFormat(
            support.formats
        );
        const VkExtent2D extent = chooseSwapchainExtent(
            window_,
            support.capabilities
        );

        if (extent.width == 0 || extent.height == 0)
        {
            return false;
        }

        std::uint32_t imageCount = support.capabilities.minImageCount + 1;

        if (support.capabilities.maxImageCount > 0)
        {
            imageCount = std::min(
                imageCount,
                support.capabilities.maxImageCount
            );
        }

        const std::array queueFamilies{
            graphicsQueueFamily_,
            presentQueueFamily_
        };

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = surface_;
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

        if (graphicsQueueFamily_ != presentQueueFamily_)
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount =
                static_cast<std::uint32_t>(queueFamilies.size());
            createInfo.pQueueFamilyIndices = queueFamilies.data();
        }
        else
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        createInfo.preTransform = support.capabilities.currentTransform;
        createInfo.compositeAlpha = chooseCompositeAlpha(
            support.capabilities
        );
        createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR;
        createInfo.clipped = VK_TRUE;
        createInfo.oldSwapchain = swapchain_;

        VkSwapchainKHR newSwapchain = VK_NULL_HANDLE;
        VkResult result = vkCreateSwapchainKHR(
            device_,
            &createInfo,
            nullptr,
            &newSwapchain
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateSwapchainKHR", result);
        }

        std::uint32_t swapchainImageCount = 0;
        result = vkGetSwapchainImagesKHR(
            device_,
            newSwapchain,
            &swapchainImageCount,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            vkDestroySwapchainKHR(device_, newSwapchain, nullptr);
            throwVulkanError("vkGetSwapchainImagesKHR", result);
        }

        std::vector<VkImage> newImages(swapchainImageCount);
        result = vkGetSwapchainImagesKHR(
            device_,
            newSwapchain,
            &swapchainImageCount,
            newImages.data()
        );

        if (result != VK_SUCCESS)
        {
            vkDestroySwapchainKHR(device_, newSwapchain, nullptr);
            throwVulkanError("vkGetSwapchainImagesKHR", result);
        }

        std::vector<VkImageView> newImageViews(
            newImages.size(),
            VK_NULL_HANDLE
        );

        for (std::size_t index = 0; index < newImages.size(); ++index)
        {
            VkImageViewCreateInfo viewCreateInfo{};
            viewCreateInfo.sType =
                VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            viewCreateInfo.image = newImages[index];
            viewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewCreateInfo.format = surfaceFormat.format;
            viewCreateInfo.subresourceRange.aspectMask =
                VK_IMAGE_ASPECT_COLOR_BIT;
            viewCreateInfo.subresourceRange.baseMipLevel = 0;
            viewCreateInfo.subresourceRange.levelCount = 1;
            viewCreateInfo.subresourceRange.baseArrayLayer = 0;
            viewCreateInfo.subresourceRange.layerCount = 1;

            result = vkCreateImageView(
                device_,
                &viewCreateInfo,
                nullptr,
                &newImageViews[index]
            );

            if (result != VK_SUCCESS)
            {
                for (const VkImageView imageView : newImageViews)
                {
                    if (imageView != VK_NULL_HANDLE)
                    {
                        vkDestroyImageView(device_, imageView, nullptr);
                    }
                }

                vkDestroySwapchainKHR(device_, newSwapchain, nullptr);
                throwVulkanError("vkCreateImageView", result);
            }
        }

        VkSemaphoreCreateInfo semaphoreCreateInfo{};
        semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        std::vector<VkSemaphore> newRenderFinishedSemaphores(
            newImages.size(),
            VK_NULL_HANDLE
        );

        for (VkSemaphore& semaphore : newRenderFinishedSemaphores)
        {
            result = vkCreateSemaphore(
                device_,
                &semaphoreCreateInfo,
                nullptr,
                &semaphore
            );

            if (result != VK_SUCCESS)
            {
                for (const VkSemaphore createdSemaphore
                    : newRenderFinishedSemaphores)
                {
                    if (createdSemaphore != VK_NULL_HANDLE)
                    {
                        vkDestroySemaphore(
                            device_,
                            createdSemaphore,
                            nullptr
                        );
                    }
                }

                for (const VkImageView imageView : newImageViews)
                {
                    vkDestroyImageView(device_, imageView, nullptr);
                }

                vkDestroySwapchainKHR(device_, newSwapchain, nullptr);
                throwVulkanError("vkCreateSemaphore", result);
            }
        }

        for (const VkSemaphore semaphore : renderFinishedSemaphores_)
        {
            vkDestroySemaphore(device_, semaphore, nullptr);
        }
        renderFinishedSemaphores_.clear();

        for (const VkImageView imageView : swapchainImageViews_)
        {
            vkDestroyImageView(device_, imageView, nullptr);
        }
        swapchainImageViews_.clear();

        if (swapchain_ != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        }

        swapchain_ = newSwapchain;
        swapchainFormat_ = surfaceFormat.format;
        swapchainExtent_ = extent;
        swapchainImages_ = std::move(newImages);
        swapchainImageViews_ = std::move(newImageViews);
        swapchainImageInitialized_.assign(swapchainImages_.size(), 0);
        renderFinishedSemaphores_ = std::move(
            newRenderFinishedSemaphores
        );
        ++swapchainGeneration_;
        return true;
    }

    void VulkanContext::createVertexBuffers(
        const std::span<const LineVertex> trackCurveVertices,
        const std::uint32_t trackVerticesPerCurve)
    {
        const CreatedVertexBuffer staticBuffer =
            createHostVisibleVertexBuffer(
                allocator_,
                createViewportAidVertices(
                    0.0F,
                    0.0F,
                    defaultGridSpacing
                )
            );
        staticVertexBuffer_ = staticBuffer.buffer;
        staticVertexAllocation_ = staticBuffer.allocation;
        staticVertexMappedData_ = staticBuffer.mappedData;
        staticVertexCapacity_ = staticBuffer.capacity;
        staticVertexCount_ = staticBuffer.vertexCount;
        viewportAidCenterX_ = 0.0F;
        viewportAidCenterY_ = 0.0F;
        viewportAidSpacing_ = defaultGridSpacing;

        const CreatedVertexBuffer trackCurveBuffer =
            trackCurveVertices.empty()
            ? CreatedVertexBuffer{}
            : createHostVisibleVertexBuffer(
                allocator_,
                trackCurveVertices
            );
        trackCurveVertexBuffer_ = trackCurveBuffer.buffer;
        trackCurveVertexAllocation_ = trackCurveBuffer.allocation;
        trackCurveVertexMappedData_ = trackCurveBuffer.mappedData;
        trackCurveVertexCapacity_ = trackCurveBuffer.capacity;
        trackCurveVertexCount_ = trackCurveBuffer.vertexCount;
        trackVerticesPerCurve_ = trackCurveVertices.empty()
            ? 0
            : trackVerticesPerCurve;
    }

    // Regenerates the grid and axes in place. The vertex count never changes,
    // so this only needs the retained persistent mapping of the static aid
    // buffer; the GPU must not be reading it during a frame, which holds
    // because updates happen between drawFrame calls.
    void VulkanContext::rewriteViewportAidVertices(
        const float centerX,
        const float centerY,
        const float spacing)
    {
        writeHostVisibleVertexBuffer(
            allocator_,
            staticVertexAllocation_,
            staticVertexMappedData_,
            staticVertexCapacity_,
            createViewportAidVertices(centerX, centerY, spacing)
        );
        viewportAidCenterX_ =
            std::round(centerX / spacing) * spacing;
        viewportAidCenterY_ =
            std::round(centerY / spacing) * spacing;
        viewportAidSpacing_ = spacing;
    }

    void VulkanContext::createGraphicsPipeline()
    {
        if (pipelineLayout_ == VK_NULL_HANDLE)
        {
            VkPushConstantRange pushConstantRange{};
            pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
            pushConstantRange.offset = 0;
            pushConstantRange.size = sizeof(viewportViewProjection_)
                + 4 * sizeof(float);

            VkPipelineLayoutCreateInfo layoutCreateInfo{};
            layoutCreateInfo.sType =
                VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            layoutCreateInfo.pushConstantRangeCount = 1;
            layoutCreateInfo.pPushConstantRanges = &pushConstantRange;

            const VkResult result = vkCreatePipelineLayout(
                device_,
                &layoutCreateInfo,
                nullptr,
                &pipelineLayout_
            );

            if (result != VK_SUCCESS)
            {
                throwVulkanError("vkCreatePipelineLayout", result);
            }
        }

        const std::vector<std::uint32_t> vertexCode = readSpirv(
            shaderPath("centerline.vert.spv")
        );
        const std::vector<std::uint32_t> fragmentCode = readSpirv(
            shaderPath("centerline.frag.spv")
        );

        const VkShaderModule vertexShader = createShaderModule(
            device_,
            vertexCode
        );

        VkShaderModule fragmentShader = VK_NULL_HANDLE;

        try
        {
            fragmentShader = createShaderModule(device_, fragmentCode);
        }
        catch (...)
        {
            vkDestroyShaderModule(device_, vertexShader, nullptr);
            throw;
        }

        const std::array shaderStages{
            VkPipelineShaderStageCreateInfo{
                VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                nullptr,
                0,
                VK_SHADER_STAGE_VERTEX_BIT,
                vertexShader,
                "main",
                nullptr
            },
            VkPipelineShaderStageCreateInfo{
                VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                nullptr,
                0,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                fragmentShader,
                "main",
                nullptr
            }
        };

        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(LineVertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        const std::array attributeDescriptions{
            VkVertexInputAttributeDescription{
                0,
                0,
                VK_FORMAT_R32G32B32_SFLOAT,
                static_cast<std::uint32_t>(
                    offsetof(LineVertex, x)
                )
            },
            VkVertexInputAttributeDescription{
                1,
                0,
                VK_FORMAT_R32G32B32A32_SFLOAT,
                static_cast<std::uint32_t>(
                    offsetof(LineVertex, color)
                )
            }
        };

        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType =
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInput.vertexBindingDescriptionCount = 1;
        vertexInput.pVertexBindingDescriptions = &bindingDescription;
        vertexInput.vertexAttributeDescriptionCount =
            static_cast<std::uint32_t>(attributeDescriptions.size());
        vertexInput.pVertexAttributeDescriptions =
            attributeDescriptions.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType =
            VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType =
            VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterization{};
        rasterization.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterization.polygonMode = VK_POLYGON_MODE_FILL;
        rasterization.cullMode = VK_CULL_MODE_NONE;
        rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterization.lineWidth = 1.0F;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType =
            VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType =
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT
            | VK_COLOR_COMPONENT_G_BIT
            | VK_COLOR_COMPONENT_B_BIT
            | VK_COLOR_COMPONENT_A_BIT;

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType =
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;

        constexpr std::array dynamicStates{
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
        };

        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType =
            VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount =
            static_cast<std::uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        VkPipelineRenderingCreateInfo renderingCreateInfo{};
        renderingCreateInfo.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        renderingCreateInfo.colorAttachmentCount = 1;
        renderingCreateInfo.pColorAttachmentFormats = &viewportColorFormat;
        renderingCreateInfo.depthAttachmentFormat = viewportDepthFormat;

        VkGraphicsPipelineCreateInfo pipelineCreateInfo{};
        pipelineCreateInfo.sType =
            VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineCreateInfo.pNext = &renderingCreateInfo;
        pipelineCreateInfo.stageCount =
            static_cast<std::uint32_t>(shaderStages.size());
        pipelineCreateInfo.pStages = shaderStages.data();
        pipelineCreateInfo.pVertexInputState = &vertexInput;
        pipelineCreateInfo.pInputAssemblyState = &inputAssembly;
        pipelineCreateInfo.pViewportState = &viewportState;
        pipelineCreateInfo.pRasterizationState = &rasterization;
        pipelineCreateInfo.pMultisampleState = &multisampling;
        pipelineCreateInfo.pDepthStencilState = &depthStencil;
        pipelineCreateInfo.pColorBlendState = &colorBlending;
        pipelineCreateInfo.pDynamicState = &dynamicState;
        pipelineCreateInfo.layout = pipelineLayout_;
        pipelineCreateInfo.renderPass = VK_NULL_HANDLE;

        VkPipeline newPipeline = VK_NULL_HANDLE;
        const VkResult result = vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineCreateInfo,
            nullptr,
            &newPipeline
        );

        vkDestroyShaderModule(device_, fragmentShader, nullptr);
        vkDestroyShaderModule(device_, vertexShader, nullptr);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateGraphicsPipelines", result);
        }

        if (graphicsPipeline_ != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(device_, graphicsPipeline_, nullptr);
        }

        graphicsPipeline_ = newPipeline;
    }

    void VulkanContext::createViewportTarget(
        const std::uint32_t width,
        const std::uint32_t height)
    {
        if (width == 0 || height == 0)
        {
            throw std::invalid_argument(
                "A Vulkan viewport target must have a nonzero extent."
            );
        }

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(physicalDevice_, &properties);

        if (width > properties.limits.maxImageDimension2D
            || height > properties.limits.maxImageDimension2D)
        {
            throw std::length_error(
                "The requested Editor viewport exceeds Vulkan's maximum 2D "
                "image dimension."
            );
        }

        VkImageCreateInfo imageCreateInfo{};
        imageCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
        imageCreateInfo.format = viewportColorFormat;
        imageCreateInfo.extent = {width, height, 1};
        imageCreateInfo.mipLevels = 1;
        imageCreateInfo.arrayLayers = 1;
        imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageCreateInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
            | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo allocationCreateInfo{};
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

        VkImage image = VK_NULL_HANDLE;
        VmaAllocation allocation = VK_NULL_HANDLE;
        VkResult result = vmaCreateImage(
            allocator_,
            &imageCreateInfo,
            &allocationCreateInfo,
            &image,
            &allocation,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vmaCreateImage", result);
        }

        VkImageViewCreateInfo viewCreateInfo{};
        viewCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewCreateInfo.image = image;
        viewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewCreateInfo.format = viewportColorFormat;
        viewCreateInfo.subresourceRange.aspectMask =
            VK_IMAGE_ASPECT_COLOR_BIT;
        viewCreateInfo.subresourceRange.baseMipLevel = 0;
        viewCreateInfo.subresourceRange.levelCount = 1;
        viewCreateInfo.subresourceRange.baseArrayLayer = 0;
        viewCreateInfo.subresourceRange.layerCount = 1;

        VkImageView imageView = VK_NULL_HANDLE;
        result = vkCreateImageView(
            device_,
            &viewCreateInfo,
            nullptr,
            &imageView
        );

        if (result != VK_SUCCESS)
        {
            vmaDestroyImage(allocator_, image, allocation);
            throwVulkanError("vkCreateImageView", result);
        }

        VkImageCreateInfo depthImageCreateInfo = imageCreateInfo;
        depthImageCreateInfo.format = viewportDepthFormat;
        depthImageCreateInfo.usage =
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

        VkImage depthImage = VK_NULL_HANDLE;
        VmaAllocation depthAllocation = VK_NULL_HANDLE;
        result = vmaCreateImage(
            allocator_,
            &depthImageCreateInfo,
            &allocationCreateInfo,
            &depthImage,
            &depthAllocation,
            nullptr
        );

        if (result != VK_SUCCESS)
        {
            vkDestroyImageView(device_, imageView, nullptr);
            vmaDestroyImage(allocator_, image, allocation);
            throwVulkanError(
                "vmaCreateImage for viewport depth target",
                result
            );
        }

        VkImageViewCreateInfo depthViewCreateInfo = viewCreateInfo;
        depthViewCreateInfo.image = depthImage;
        depthViewCreateInfo.format = viewportDepthFormat;
        depthViewCreateInfo.subresourceRange.aspectMask =
            VK_IMAGE_ASPECT_DEPTH_BIT;

        VkImageView depthImageView = VK_NULL_HANDLE;
        result = vkCreateImageView(
            device_,
            &depthViewCreateInfo,
            nullptr,
            &depthImageView
        );

        if (result != VK_SUCCESS)
        {
            vmaDestroyImage(allocator_, depthImage, depthAllocation);
            vkDestroyImageView(device_, imageView, nullptr);
            vmaDestroyImage(allocator_, image, allocation);
            throwVulkanError(
                "vkCreateImageView for viewport depth target",
                result
            );
        }

        viewportImage_ = image;
        viewportAllocation_ = allocation;
        viewportImageView_ = imageView;
        viewportDepthImage_ = depthImage;
        viewportDepthAllocation_ = depthAllocation;
        viewportDepthImageView_ = depthImageView;
        viewportExtent_ = {width, height};
        viewportImageInitialized_ = false;
    }

    void VulkanContext::createCommandResources()
    {
        VkCommandPoolCreateInfo poolCreateInfo{};
        poolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolCreateInfo.queueFamilyIndex = graphicsQueueFamily_;

        VkResult result = vkCreateCommandPool(
            device_,
            &poolCreateInfo,
            nullptr,
            &commandPool_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateCommandPool", result);
        }

        VkCommandBufferAllocateInfo allocateInfo{};
        allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocateInfo.commandPool = commandPool_;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = 1;

        result = vkAllocateCommandBuffers(
            device_,
            &allocateInfo,
            &commandBuffer_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkAllocateCommandBuffers", result);
        }
    }

    void VulkanContext::createSynchronizationResources()
    {
        VkSemaphoreCreateInfo semaphoreCreateInfo{};
        semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        VkResult result = vkCreateSemaphore(
            device_,
            &semaphoreCreateInfo,
            nullptr,
            &imageAvailableSemaphore_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateSemaphore", result);
        }

        VkFenceCreateInfo fenceCreateInfo{};
        fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceCreateInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        result = vkCreateFence(
            device_,
            &fenceCreateInfo,
            nullptr,
            &frameFence_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkCreateFence", result);
        }
    }

    void VulkanContext::waitForFrameCompletion()
    {
        if (frameFence_ == VK_NULL_HANDLE)
        {
            return;
        }

        const VkResult result = vkWaitForFences(
            device_,
            1,
            &frameFence_,
            VK_TRUE,
            std::numeric_limits<std::uint64_t>::max()
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkWaitForFences", result);
        }
    }

    void VulkanContext::resizeViewportTarget(
        std::uint32_t width,
        std::uint32_t height,
        const ViewportTargetRetirementCallback retirementCallback,
        void* const userData)
    {
        if (width == 0 || height == 0)
        {
            width = 0;
            height = 0;
        }

        if (viewportExtent_.width == width
            && viewportExtent_.height == height)
        {
            return;
        }

        if (viewportImage_ != VK_NULL_HANDLE)
        {
            // The frame fence covers the viewport color/depth writes and
            // ImGui's sampling of the color image. Descriptor retirement and
            // attachment destruction are safe once that frame completes.
            waitForFrameCompletion();

            if (retirementCallback != nullptr)
            {
                retirementCallback(userData);
            }

            destroyViewportTarget();
        }

        if (width != 0 && height != 0)
        {
            createViewportTarget(width, height);
        }
    }

    void VulkanContext::setViewportViewProjection(
        const std::array<float, 16>& viewProjection)
    {
        if (!std::ranges::all_of(
            viewProjection,
            [](const float component)
            {
                return std::isfinite(component);
            }))
        {
            throw std::invalid_argument(
                "The viewport view-projection matrix must be finite."
            );
        }

        viewportViewProjection_ = viewProjection;
    }

    void VulkanContext::updateTrackCurveVertices(
        const std::span<const LineVertex> trackCurveVertices,
        const std::uint32_t trackVerticesPerCurve)
    {
        if (allocator_ == VK_NULL_HANDLE)
        {
            throw std::logic_error(
                "VulkanContext cannot update track-curve vertices before initialization."
            );
        }

        requireValidTrackCurveVertices(
            trackCurveVertices,
            trackVerticesPerCurve
        );

        const std::span<const LineVertex> candidateVertices =
            trackCurveVertices;
        const VkDeviceSize candidateSize = sizeof(LineVertex)
            * candidateVertices.size();

        if (candidateVertices.empty())
        {
            waitForFrameCompletion();
            trackCurveVertexCount_ = 0;
            trackVerticesPerCurve_ = 0;
            return;
        }

        if (trackCurveVertexBuffer_ != VK_NULL_HANDLE
            && candidateSize <= trackCurveVertexCapacity_)
        {
            waitForFrameCompletion();
            writeHostVisibleVertexBuffer(
                allocator_,
                trackCurveVertexAllocation_,
                trackCurveVertexMappedData_,
                trackCurveVertexCapacity_,
                candidateVertices
            );
            trackCurveVertexCount_ = static_cast<std::uint32_t>(
                candidateVertices.size()
            );
            trackVerticesPerCurve_ = trackVerticesPerCurve;
            return;
        }

        if (spareTrackCurveVertexBuffer_ != VK_NULL_HANDLE
            && candidateSize <= spareTrackCurveVertexCapacity_)
        {
            // The previous update retired this spare only after the frame that
            // used it completed. Waiting here likewise retires the current
            // buffer before the two allocations exchange roles.
            waitForFrameCompletion();
            writeHostVisibleVertexBuffer(
                allocator_,
                spareTrackCurveVertexAllocation_,
                spareTrackCurveVertexMappedData_,
                spareTrackCurveVertexCapacity_,
                candidateVertices
            );
            std::swap(
                trackCurveVertexBuffer_,
                spareTrackCurveVertexBuffer_
            );
            std::swap(
                trackCurveVertexAllocation_,
                spareTrackCurveVertexAllocation_
            );
            std::swap(
                trackCurveVertexMappedData_,
                spareTrackCurveVertexMappedData_
            );
            std::swap(
                trackCurveVertexCapacity_,
                spareTrackCurveVertexCapacity_
            );
            trackCurveVertexCount_ = static_cast<std::uint32_t>(
                candidateVertices.size()
            );
            trackVerticesPerCurve_ = trackVerticesPerCurve;
            return;
        }

        const CreatedVertexBuffer candidate = createHostVisibleVertexBuffer(
            allocator_,
            candidateVertices
        );

        try
        {
            // The single frame fence covers every draw that can still read the
            // old track-curve allocation. Waiting here is needed only for an
            // authored edit, not for every buffer creation or every frame.
            waitForFrameCompletion();
        }
        catch (...)
        {
            vmaDestroyBuffer(
                allocator_,
                candidate.buffer,
                candidate.allocation
            );
            throw;
        }

        if (spareTrackCurveVertexBuffer_ != VK_NULL_HANDLE)
        {
            vmaDestroyBuffer(
                allocator_,
                spareTrackCurveVertexBuffer_,
                spareTrackCurveVertexAllocation_
            );
        }
        spareTrackCurveVertexBuffer_ = trackCurveVertexBuffer_;
        spareTrackCurveVertexAllocation_ = trackCurveVertexAllocation_;
        spareTrackCurveVertexMappedData_ = trackCurveVertexMappedData_;
        spareTrackCurveVertexCapacity_ = trackCurveVertexCapacity_;
        trackCurveVertexBuffer_ = candidate.buffer;
        trackCurveVertexAllocation_ = candidate.allocation;
        trackCurveVertexMappedData_ = candidate.mappedData;
        trackCurveVertexCapacity_ = candidate.capacity;
        trackCurveVertexCount_ = candidate.vertexCount;
        trackVerticesPerCurve_ = trackVerticesPerCurve;
    }

    void VulkanContext::setViewportElementVisibility(
        const bool gridVisible,
        const std::uint32_t curveVisibilityMask)
    {
        viewportGridVisible_ = gridVisible;
        viewportCurveVisibilityMask_ = curveVisibilityMask;
    }

    void VulkanContext::setTrackCurveHighlight(
        const std::uint32_t firstVertex,
        const std::uint32_t vertexCount)
    {
        if (vertexCount != 0
            && (firstVertex > trackVerticesPerCurve_
                || vertexCount
                    > trackVerticesPerCurve_ - firstVertex))
        {
            throw std::out_of_range(
                "The selected track-curve range is outside one curve run."
            );
        }

        trackHighlightFirstVertex_ = firstVertex;
        trackHighlightVertexCount_ = vertexCount;
    }

    void VulkanContext::updateViewportAidReference(
        const float centerX,
        const float centerY,
        const float referenceRadius)
    {
        if (staticVertexMappedData_ == nullptr || !std::isfinite(centerX)
            || !std::isfinite(centerY) || !std::isfinite(referenceRadius)
            || referenceRadius <= 0.0F)
        {
            return;
        }

        // Keep the modeled region comfortably larger than the reference
        // sphere so the track never pokes past the grid edge.
        constexpr float coverageFactor = 2.5F;
        float spacing = viewportAidSpacingCandidates.back();

        for (const float candidate : viewportAidSpacingCandidates)
        {
            if (static_cast<float>(gridHalfLineCount) * candidate
                >= coverageFactor * referenceRadius)
            {
                spacing = candidate;
                break;
            }
        }

        const float snappedCenterX =
            std::round(centerX / spacing) * spacing;
        const float snappedCenterY =
            std::round(centerY / spacing) * spacing;

        if (spacing == viewportAidSpacing_
            && snappedCenterX == viewportAidCenterX_
            && snappedCenterY == viewportAidCenterY_)
        {
            return;
        }

        rewriteViewportAidVertices(snappedCenterX, snappedCenterY, spacing);
    }

    void VulkanContext::recordDrawCommands(
        const std::uint32_t imageIndex,
        const FrameRenderCallback renderCallback,
        void* const userData)
    {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        VkResult result = vkBeginCommandBuffer(commandBuffer_, &beginInfo);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkBeginCommandBuffer", result);
        }

        if (viewportImage_ != VK_NULL_HANDLE)
        {
            VkImageMemoryBarrier toColorAttachmentBarrier{};
            toColorAttachmentBarrier.sType =
                VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            toColorAttachmentBarrier.srcAccessMask =
                viewportImageInitialized_
                ? VK_ACCESS_SHADER_READ_BIT
                : 0;
            toColorAttachmentBarrier.dstAccessMask =
                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            toColorAttachmentBarrier.oldLayout =
                viewportImageInitialized_
                ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                : VK_IMAGE_LAYOUT_UNDEFINED;
            toColorAttachmentBarrier.newLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            toColorAttachmentBarrier.srcQueueFamilyIndex =
                VK_QUEUE_FAMILY_IGNORED;
            toColorAttachmentBarrier.dstQueueFamilyIndex =
                VK_QUEUE_FAMILY_IGNORED;
            toColorAttachmentBarrier.image = viewportImage_;
            toColorAttachmentBarrier.subresourceRange.aspectMask =
                VK_IMAGE_ASPECT_COLOR_BIT;
            toColorAttachmentBarrier.subresourceRange.baseMipLevel = 0;
            toColorAttachmentBarrier.subresourceRange.levelCount = 1;
            toColorAttachmentBarrier.subresourceRange.baseArrayLayer = 0;
            toColorAttachmentBarrier.subresourceRange.layerCount = 1;

            const VkPipelineStageFlags viewportSourceStage =
                viewportImageInitialized_
                ? VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT
                : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

            vkCmdPipelineBarrier(
                commandBuffer_,
                viewportSourceStage,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                0,
                0,
                nullptr,
                0,
                nullptr,
                1,
                &toColorAttachmentBarrier
            );

            if (!viewportImageInitialized_)
            {
                VkImageMemoryBarrier toDepthAttachmentBarrier{};
                toDepthAttachmentBarrier.sType =
                    VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                toDepthAttachmentBarrier.srcAccessMask = 0;
                toDepthAttachmentBarrier.dstAccessMask =
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                    | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                toDepthAttachmentBarrier.oldLayout =
                    VK_IMAGE_LAYOUT_UNDEFINED;
                toDepthAttachmentBarrier.newLayout =
                    VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                toDepthAttachmentBarrier.srcQueueFamilyIndex =
                    VK_QUEUE_FAMILY_IGNORED;
                toDepthAttachmentBarrier.dstQueueFamilyIndex =
                    VK_QUEUE_FAMILY_IGNORED;
                toDepthAttachmentBarrier.image = viewportDepthImage_;
                toDepthAttachmentBarrier.subresourceRange.aspectMask =
                    VK_IMAGE_ASPECT_DEPTH_BIT;
                toDepthAttachmentBarrier.subresourceRange.baseMipLevel = 0;
                toDepthAttachmentBarrier.subresourceRange.levelCount = 1;
                toDepthAttachmentBarrier.subresourceRange.baseArrayLayer = 0;
                toDepthAttachmentBarrier.subresourceRange.layerCount = 1;

                vkCmdPipelineBarrier(
                    commandBuffer_,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                    VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                    0,
                    0,
                    nullptr,
                    0,
                    nullptr,
                    1,
                    &toDepthAttachmentBarrier
                );
            }

            VkRenderingAttachmentInfo viewportColorAttachment{};
            viewportColorAttachment.sType =
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
            viewportColorAttachment.imageView = viewportImageView_;
            viewportColorAttachment.imageLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            viewportColorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            viewportColorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            viewportColorAttachment.clearValue.color = {
                {0.001F, 0.001F, 0.001F, 1.0F}
            };

            VkRenderingAttachmentInfo viewportDepthAttachment{};
            viewportDepthAttachment.sType =
                VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
            viewportDepthAttachment.imageView = viewportDepthImageView_;
            viewportDepthAttachment.imageLayout =
                VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            viewportDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            viewportDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            viewportDepthAttachment.clearValue.depthStencil = {1.0F, 0};

            VkRenderingInfo viewportRenderingInfo{};
            viewportRenderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
            viewportRenderingInfo.renderArea.extent = viewportExtent_;
            viewportRenderingInfo.layerCount = 1;
            viewportRenderingInfo.colorAttachmentCount = 1;
            viewportRenderingInfo.pColorAttachments =
                &viewportColorAttachment;
            viewportRenderingInfo.pDepthAttachment =
                &viewportDepthAttachment;

            vkCmdBeginRendering(commandBuffer_, &viewportRenderingInfo);

            vkCmdBindPipeline(
                commandBuffer_,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                graphicsPipeline_
            );

            VkViewport viewport{};
            viewport.width = static_cast<float>(viewportExtent_.width);
            viewport.height = static_cast<float>(viewportExtent_.height);
            viewport.minDepth = 0.0F;
            viewport.maxDepth = 1.0F;
            vkCmdSetViewport(commandBuffer_, 0, 1, &viewport);

            VkRect2D scissor{};
            scissor.extent = viewportExtent_;
            vkCmdSetScissor(commandBuffer_, 0, 1, &scissor);

            vkCmdPushConstants(
                commandBuffer_,
                pipelineLayout_,
                VK_SHADER_STAGE_VERTEX_BIT,
                0,
                sizeof(viewportViewProjection_),
                viewportViewProjection_.data()
            );
            constexpr std::array<float, 4> noHighlight{
                1.0F, 0.82F, 0.12F, 0.0F
            };
            vkCmdPushConstants(
                commandBuffer_,
                pipelineLayout_,
                VK_SHADER_STAGE_VERTEX_BIT,
                sizeof(viewportViewProjection_),
                sizeof(noHighlight),
                noHighlight.data()
            );

            constexpr VkDeviceSize vertexOffset = 0;
            if (viewportGridVisible_)
            {
                vkCmdBindVertexBuffers(
                    commandBuffer_,
                    0,
                    1,
                    &staticVertexBuffer_,
                    &vertexOffset
                );
                vkCmdDraw(
                    commandBuffer_,
                    staticVertexCount_,
                    1,
                    0,
                    0
                );
            }

            // One bind, up to four draws: each reference-curve run occupies
            // its own contiguous vertex range inside the shared buffer.
            if (viewportCurveVisibilityMask_ != 0
                && trackCurveVertexCount_ > 0)
            {
                const std::uint32_t verticesPerCurve = trackVerticesPerCurve_;
                vkCmdBindVertexBuffers(
                    commandBuffer_,
                    0,
                    1,
                    &trackCurveVertexBuffer_,
                    &vertexOffset
                );

                constexpr std::uint32_t curveBits[viewportCurveCount]{
                    viewportLeftRailCurve,
                    viewportRightRailCurve,
                    viewportCenterlineCurve,
                    viewportHeartlineCurve
                };

                for (const std::uint32_t curve : curveBits)
                {
                    if ((viewportCurveVisibilityMask_ & (1u << curve)) == 0)
                    {
                        continue;
                    }

                    vkCmdDraw(
                        commandBuffer_,
                        verticesPerCurve,
                        1,
                        curve * verticesPerCurve,
                        0
                    );
                }

                if (trackHighlightVertexCount_ > 0)
                {
                    constexpr std::array<float, 4> highlightColor{
                        1.0F, 0.82F, 0.12F, 0.78F
                    };
                    vkCmdPushConstants(
                        commandBuffer_,
                        pipelineLayout_,
                        VK_SHADER_STAGE_VERTEX_BIT,
                        sizeof(viewportViewProjection_),
                        sizeof(highlightColor),
                        highlightColor.data()
                    );

                    for (const std::uint32_t curve : curveBits)
                    {
                        if ((viewportCurveVisibilityMask_
                            & (1u << curve)) == 0)
                        {
                            continue;
                        }

                        vkCmdDraw(
                            commandBuffer_,
                            trackHighlightVertexCount_,
                            1,
                            curve * verticesPerCurve
                                + trackHighlightFirstVertex_,
                            0
                        );
                    }
                }
            }

            vkCmdEndRendering(commandBuffer_);

            VkImageMemoryBarrier toShaderReadBarrier{};
            toShaderReadBarrier.sType =
                VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            toShaderReadBarrier.srcAccessMask =
                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            toShaderReadBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            toShaderReadBarrier.oldLayout =
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            toShaderReadBarrier.newLayout =
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            toShaderReadBarrier.srcQueueFamilyIndex =
                VK_QUEUE_FAMILY_IGNORED;
            toShaderReadBarrier.dstQueueFamilyIndex =
                VK_QUEUE_FAMILY_IGNORED;
            toShaderReadBarrier.image = viewportImage_;
            toShaderReadBarrier.subresourceRange =
                toColorAttachmentBarrier.subresourceRange;

            vkCmdPipelineBarrier(
                commandBuffer_,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                0,
                0,
                nullptr,
                0,
                nullptr,
                1,
                &toShaderReadBarrier
            );
        }

        VkImageMemoryBarrier toColorAttachmentBarrier{};
        toColorAttachmentBarrier.sType =
            VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        toColorAttachmentBarrier.srcAccessMask = 0;
        toColorAttachmentBarrier.dstAccessMask =
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        toColorAttachmentBarrier.oldLayout =
            swapchainImageInitialized_[imageIndex] != 0
            ? VK_IMAGE_LAYOUT_PRESENT_SRC_KHR
            : VK_IMAGE_LAYOUT_UNDEFINED;
        toColorAttachmentBarrier.newLayout =
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toColorAttachmentBarrier.srcQueueFamilyIndex =
            VK_QUEUE_FAMILY_IGNORED;
        toColorAttachmentBarrier.dstQueueFamilyIndex =
            VK_QUEUE_FAMILY_IGNORED;
        toColorAttachmentBarrier.image = swapchainImages_[imageIndex];
        toColorAttachmentBarrier.subresourceRange.aspectMask =
            VK_IMAGE_ASPECT_COLOR_BIT;
        toColorAttachmentBarrier.subresourceRange.baseMipLevel = 0;
        toColorAttachmentBarrier.subresourceRange.levelCount = 1;
        toColorAttachmentBarrier.subresourceRange.baseArrayLayer = 0;
        toColorAttachmentBarrier.subresourceRange.layerCount = 1;

        const VkPipelineStageFlags sourceStage =
            swapchainImageInitialized_[imageIndex] != 0
            ? VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
            : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

        vkCmdPipelineBarrier(
            commandBuffer_,
            sourceStage,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &toColorAttachmentBarrier
        );

        VkRenderingAttachmentInfo colorAttachment{};
        colorAttachment.sType =
            VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
        colorAttachment.imageView = swapchainImageViews_[imageIndex];
        colorAttachment.imageLayout =
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.clearValue.color = {
            {0.070F, 0.075F, 0.080F, 1.0F}
        };

        VkRenderingInfo renderingInfo{};
        renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        renderingInfo.renderArea.extent = swapchainExtent_;
        renderingInfo.layerCount = 1;
        renderingInfo.colorAttachmentCount = 1;
        renderingInfo.pColorAttachments = &colorAttachment;

        vkCmdBeginRendering(commandBuffer_, &renderingInfo);

        if (renderCallback != nullptr)
        {
            renderCallback(commandBuffer_, userData);
        }

        vkCmdEndRendering(commandBuffer_);

        VkImageMemoryBarrier toPresentBarrier{};
        toPresentBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        toPresentBarrier.srcAccessMask =
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        toPresentBarrier.dstAccessMask = 0;
        toPresentBarrier.oldLayout =
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toPresentBarrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        toPresentBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toPresentBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toPresentBarrier.image = swapchainImages_[imageIndex];
        toPresentBarrier.subresourceRange =
            toColorAttachmentBarrier.subresourceRange;

        vkCmdPipelineBarrier(
            commandBuffer_,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &toPresentBarrier
        );

        result = vkEndCommandBuffer(commandBuffer_);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkEndCommandBuffer", result);
        }
    }

    void VulkanContext::drawFrame(
        const FrameRenderCallback renderCallback,
        void* const userData)
    {
        int width = 0;
        int height = 0;

        if (!SDL_GetWindowSizeInPixels(window_, &width, &height))
        {
            throw std::runtime_error(
                std::string("SDL_GetWindowSizeInPixels failed: ")
                + SDL_GetError()
            );
        }

        if (width == 0 || height == 0)
        {
            return;
        }

        waitForFrameCompletion();

        std::uint32_t imageIndex = 0;
        const VkResult acquireResult = vkAcquireNextImageKHR(
            device_,
            swapchain_,
            std::numeric_limits<std::uint64_t>::max(),
            imageAvailableSemaphore_,
            VK_NULL_HANDLE,
            &imageIndex
        );

        if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR)
        {
            recreateSwapchain();
            return;
        }

        if (acquireResult != VK_SUCCESS
            && acquireResult != VK_SUBOPTIMAL_KHR)
        {
            throwVulkanError("vkAcquireNextImageKHR", acquireResult);
        }

        VkResult result = vkResetFences(device_, 1, &frameFence_);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkResetFences", result);
        }

        result = vkResetCommandBuffer(commandBuffer_, 0);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkResetCommandBuffer", result);
        }

        recordDrawCommands(imageIndex, renderCallback, userData);

        const VkPipelineStageFlags waitStage =
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        const VkSemaphore renderFinishedSemaphore =
            renderFinishedSemaphores_[imageIndex];

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = &imageAvailableSemaphore_;
        submitInfo.pWaitDstStageMask = &waitStage;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer_;
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = &renderFinishedSemaphore;

        result = vkQueueSubmit(
            graphicsQueue_,
            1,
            &submitInfo,
            frameFence_
        );

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkQueueSubmit", result);
        }

        swapchainImageInitialized_[imageIndex] = 1;

        if (viewportImage_ != VK_NULL_HANDLE)
        {
            viewportImageInitialized_ = true;
        }

        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = &renderFinishedSemaphore;
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = &swapchain_;
        presentInfo.pImageIndices = &imageIndex;

        const VkResult presentResult = vkQueuePresentKHR(
            presentQueue_,
            &presentInfo
        );

        if (presentResult == VK_ERROR_OUT_OF_DATE_KHR
            || presentResult == VK_SUBOPTIMAL_KHR
            || acquireResult == VK_SUBOPTIMAL_KHR)
        {
            recreateSwapchain();
            return;
        }

        if (presentResult != VK_SUCCESS)
        {
            throwVulkanError("vkQueuePresentKHR", presentResult);
        }
    }

    void VulkanContext::recreateSwapchain()
    {
        int width = 0;
        int height = 0;

        if (!SDL_GetWindowSizeInPixels(window_, &width, &height))
        {
            throw std::runtime_error(
                std::string("SDL_GetWindowSizeInPixels failed: ")
                + SDL_GetError()
            );
        }

        if (width == 0 || height == 0)
        {
            return;
        }

        const VkResult result = vkDeviceWaitIdle(device_);

        if (result != VK_SUCCESS)
        {
            throwVulkanError("vkDeviceWaitIdle", result);
        }

        if (!createSwapchain())
        {
            return;
        }
    }

    void VulkanContext::destroyViewportTarget() noexcept
    {
        if (viewportImageView_ != VK_NULL_HANDLE)
        {
            vkDestroyImageView(device_, viewportImageView_, nullptr);
            viewportImageView_ = VK_NULL_HANDLE;
        }

        if (viewportDepthImageView_ != VK_NULL_HANDLE)
        {
            vkDestroyImageView(device_, viewportDepthImageView_, nullptr);
            viewportDepthImageView_ = VK_NULL_HANDLE;
        }

        if (viewportImage_ != VK_NULL_HANDLE)
        {
            vmaDestroyImage(
                allocator_,
                viewportImage_,
                viewportAllocation_
            );
            viewportImage_ = VK_NULL_HANDLE;
            viewportAllocation_ = VK_NULL_HANDLE;
        }

        if (viewportDepthImage_ != VK_NULL_HANDLE)
        {
            vmaDestroyImage(
                allocator_,
                viewportDepthImage_,
                viewportDepthAllocation_
            );
            viewportDepthImage_ = VK_NULL_HANDLE;
            viewportDepthAllocation_ = VK_NULL_HANDLE;
        }

        viewportExtent_ = {};
        viewportImageInitialized_ = false;
    }

    void VulkanContext::destroySwapchain() noexcept
    {
        for (const VkSemaphore semaphore : renderFinishedSemaphores_)
        {
            vkDestroySemaphore(device_, semaphore, nullptr);
        }
        renderFinishedSemaphores_.clear();

        for (const VkImageView imageView : swapchainImageViews_)
        {
            vkDestroyImageView(device_, imageView, nullptr);
        }
        swapchainImageViews_.clear();

        if (swapchain_ != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(device_, swapchain_, nullptr);
            swapchain_ = VK_NULL_HANDLE;
        }

        swapchainImages_.clear();
        swapchainImageInitialized_.clear();
        swapchainFormat_ = VK_FORMAT_UNDEFINED;
        swapchainExtent_ = {};
    }

    void VulkanContext::shutdown() noexcept
    {
        if (device_ != VK_NULL_HANDLE)
        {
            const VkResult result = vkDeviceWaitIdle(device_);

            if (result != VK_SUCCESS)
            {
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Error,
                    "VK",
                    "vkDeviceWaitIdle failed during shutdown with VkResult %d.",
                    static_cast<int>(result)
                );
            }

            if (frameFence_ != VK_NULL_HANDLE)
            {
                vkDestroyFence(device_, frameFence_, nullptr);
                frameFence_ = VK_NULL_HANDLE;
            }

            if (imageAvailableSemaphore_ != VK_NULL_HANDLE)
            {
                vkDestroySemaphore(
                    device_,
                    imageAvailableSemaphore_,
                    nullptr
                );
                imageAvailableSemaphore_ = VK_NULL_HANDLE;
            }

            commandBuffer_ = VK_NULL_HANDLE;

            if (commandPool_ != VK_NULL_HANDLE)
            {
                vkDestroyCommandPool(device_, commandPool_, nullptr);
                commandPool_ = VK_NULL_HANDLE;
            }

            destroySwapchain();
            destroyViewportTarget();

            if (graphicsPipeline_ != VK_NULL_HANDLE)
            {
                vkDestroyPipeline(device_, graphicsPipeline_, nullptr);
                graphicsPipeline_ = VK_NULL_HANDLE;
            }

            if (pipelineLayout_ != VK_NULL_HANDLE)
            {
                vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
                pipelineLayout_ = VK_NULL_HANDLE;
            }

            if (trackCurveVertexBuffer_ != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(
                    allocator_,
                    trackCurveVertexBuffer_,
                    trackCurveVertexAllocation_
                );
                trackCurveVertexBuffer_ = VK_NULL_HANDLE;
                trackCurveVertexAllocation_ = VK_NULL_HANDLE;
            }

            trackCurveVertexMappedData_ = nullptr;
            trackCurveVertexCapacity_ = 0;
            trackCurveVertexCount_ = 0;

            if (spareTrackCurveVertexBuffer_ != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(
                    allocator_,
                    spareTrackCurveVertexBuffer_,
                    spareTrackCurveVertexAllocation_
                );
                spareTrackCurveVertexBuffer_ = VK_NULL_HANDLE;
                spareTrackCurveVertexAllocation_ = VK_NULL_HANDLE;
            }

            spareTrackCurveVertexMappedData_ = nullptr;
            spareTrackCurveVertexCapacity_ = 0;

            if (staticVertexBuffer_ != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(
                    allocator_,
                    staticVertexBuffer_,
                    staticVertexAllocation_
                );
                staticVertexBuffer_ = VK_NULL_HANDLE;
                staticVertexAllocation_ = VK_NULL_HANDLE;
            }

            staticVertexMappedData_ = nullptr;
            staticVertexCapacity_ = 0;
            staticVertexCount_ = 0;
            trackCurveVertexCount_ = 0;
            trackVerticesPerCurve_ = 0;

            if (allocator_ != VK_NULL_HANDLE)
            {
                vmaDestroyAllocator(allocator_);
                allocator_ = VK_NULL_HANDLE;
            }

            vkDestroyDevice(device_, nullptr);
            device_ = VK_NULL_HANDLE;
        }

        graphicsQueue_ = VK_NULL_HANDLE;
        presentQueue_ = VK_NULL_HANDLE;
        physicalDevice_ = VK_NULL_HANDLE;

        if (surface_ != VK_NULL_HANDLE)
        {
            vkDestroySurfaceKHR(instance_, surface_, nullptr);
            surface_ = VK_NULL_HANDLE;
        }

#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
        if (debugMessenger_ != VK_NULL_HANDLE)
        {
            destroyDebugMessenger(instance_, debugMessenger_);
            debugMessenger_ = VK_NULL_HANDLE;
        }
#endif

        if (instance_ != VK_NULL_HANDLE)
        {
            vkDestroyInstance(instance_, nullptr);
            instance_ = VK_NULL_HANDLE;
        }

        window_ = nullptr;
        viewportViewProjection_ = {
            1.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F
        };
        swapchainGeneration_ = 0;
    }

    VkInstance VulkanContext::instance() const noexcept
    {
        return instance_;
    }

    VkPhysicalDevice VulkanContext::physicalDevice() const noexcept
    {
        return physicalDevice_;
    }

    VkDevice VulkanContext::device() const noexcept
    {
        return device_;
    }

    std::uint32_t VulkanContext::graphicsQueueFamily() const noexcept
    {
        return graphicsQueueFamily_;
    }

    VkQueue VulkanContext::graphicsQueue() const noexcept
    {
        return graphicsQueue_;
    }

    VkFormat VulkanContext::swapchainFormat() const noexcept
    {
        return swapchainFormat_;
    }

    std::uint32_t VulkanContext::swapchainImageCount() const noexcept
    {
        return static_cast<std::uint32_t>(swapchainImages_.size());
    }

    std::uint64_t VulkanContext::swapchainGeneration() const noexcept
    {
        return swapchainGeneration_;
    }

    VkExtent2D VulkanContext::viewportExtent() const noexcept
    {
        return viewportExtent_;
    }

    VkImageView VulkanContext::viewportImageView() const noexcept
    {
        return viewportImageView_;
    }
}
