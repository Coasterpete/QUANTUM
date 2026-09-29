#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/GroundSurface.hpp>
#include <quantum/renderer/SupportSolidGeometry.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::renderer
{
    using quantum::logging::LogLevel;
    using quantum::logging::logMessagef;

    namespace
    {
        // Mirrors the renderer's own error path so a Vulkan failure names the
        // operation that produced it instead of surfacing a bare result code.
        [[noreturn]] void fail(const char* operation, const VkResult result)
        {
            throw std::runtime_error(std::string(operation)
                + " failed with VkResult " + std::to_string(result));
        }

        // Timber texture slots, matching the support shader's set 1 bindings.
        constexpr std::array<std::string_view, 3> timberTextureAssets{
            timberAlbedoAsset, timberNormalAsset, timberRoughnessAsset};

        // The base color is authored neutral grayscale, so it uploads as sRGB
        // and the shader multiplies it by the authored tint. Normal and
        // roughness detail are linear data and must not be sRGB-decoded.
        constexpr std::array<VkFormat, 3> timberTextureFormats{
            VK_FORMAT_R8G8B8A8_SRGB,
            VK_FORMAT_R8G8B8A8_UNORM,
            VK_FORMAT_R8G8B8A8_UNORM};

        // The 1x1 built-in fallbacks keep timber drawable when a map is
        // missing: white detail so the authored tint passes through unchanged,
        // a flat normal decoding to +Z, and a neutral roughness multiplier.
        constexpr std::array<std::array<std::uint8_t, 4>, 3>
            fallbackTimberTexels{{
                {{255, 255, 255, 255}},
                {{128, 128, 255, 255}},
                {{255, 255, 255, 255}},
            }};
    }

    void VulkanContext::createSupportSolidResources()

    {
        try
        {
            // Set 1 for the timber maps. Set 0 is the shared environment set,
            // exactly as the ground surface uses it.
            const std::array bindings{
                VkDescriptorSetLayoutBinding{
                    0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                    VK_SHADER_STAGE_FRAGMENT_BIT},
                VkDescriptorSetLayoutBinding{
                    1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                    VK_SHADER_STAGE_FRAGMENT_BIT},
                VkDescriptorSetLayoutBinding{
                    2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                    VK_SHADER_STAGE_FRAGMENT_BIT}};
            VkDescriptorSetLayoutCreateInfo layoutInfo{};
            layoutInfo.sType =
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
            layoutInfo.pBindings = bindings.data();
            VkResult result = vkCreateDescriptorSetLayout(
                device_, &layoutInfo, nullptr,
                &supportSolidTextureDescriptorLayout_);
            if (result != VK_SUCCESS)
            {
                fail("vkCreateDescriptorSetLayout for solid supports", result);
            }

            const VkDescriptorPoolSize poolSize{
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                static_cast<std::uint32_t>(bindings.size())};
            VkDescriptorPoolCreateInfo poolInfo{};
            poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            poolInfo.poolSizeCount = 1;
            poolInfo.pPoolSizes = &poolSize;
            poolInfo.maxSets = 1;
            result = vkCreateDescriptorPool(
                device_, &poolInfo, nullptr,
                &supportSolidTextureDescriptorPool_);
            if (result != VK_SUCCESS)
            {
                fail("vkCreateDescriptorPool for solid supports", result);
            }

            VkDescriptorSetAllocateInfo setInfo{};
            setInfo.sType =
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            setInfo.descriptorPool = supportSolidTextureDescriptorPool_;
            setInfo.descriptorSetCount = 1;
            setInfo.pSetLayouts = &supportSolidTextureDescriptorLayout_;
            result = vkAllocateDescriptorSets(
                device_, &setInfo, &supportSolidTextureDescriptorSet_);
            if (result != VK_SUCCESS)
            {
                fail("vkAllocateDescriptorSets for solid supports", result);
            }

            // Timber repeats across members of every length, so it needs a
            // repeating, mip-filtered sampler. The ground surface already
            // built one for the same reason and outlives this pass, so it is
            // borrowed rather than duplicated.
            supportSolidTextureSampler_ = groundTextureSampler_;

            // A failure to read one bundled map must not leave solid timber
            // undrawable, so each slot publishes its neutral 1x1 fallback and
            // reports the failure once through the log.
            for (std::size_t slot = 0; slot < timberTextureAssets.size();
                ++slot)
            {
                const std::string identifier =
                    normalizeTimberTextureAssetIdentifier(
                        timberTextureAssets[slot]);
                supportSolidTextureIdentifiers_[slot] = identifier;
                GroundTextureImage image;
                try
                {
                    image = loadGroundTextureImage(
                        timberTextureAssetPath(identifier, runtimeAssetRoot()),
                        timberTextureFormats[slot]
                            == VK_FORMAT_R8G8B8A8_SRGB);
                }
                catch (const std::exception& exception)
                {
                    logMessagef(LogLevel::Error, "SUPPORT",
                        "%s Using the built-in neutral timber map for slot %zu.",
                        exception.what(), slot);
                    image = GroundTextureImage{};
                    image.width = 1;
                    image.height = 1;
                    image.mipLevels = 1;
                    image.srgb = timberTextureFormats[slot]
                        == VK_FORMAT_R8G8B8A8_SRGB;
                    const auto& texel = fallbackTimberTexels[slot];
                    image.rgba.assign(texel.begin(), texel.end());
                }
                uploadSolidSupportTextureImage(
                    image, timberTextureFormats[slot], slot);
            }

            createSupportSolidPipeline();
        }
        catch (...)
        {
            destroySupportSolidResources();
            throw;
        }
    }

    void VulkanContext::destroySupportSolidResources() noexcept
    {
        for (SolidSupportTextureImageResource& image
            : supportSolidTextureImages_)
        {
            destroySolidSupportTextureImage(image);
        }
        supportSolidTextureIdentifiers_.fill({});

        for (SupportSolidMeshResource& mesh : supportSolidMeshes_)
        {
            if (mesh.vertexBuffer != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(allocator_, mesh.vertexBuffer,
                    mesh.vertexAllocation);
                mesh.vertexBuffer = VK_NULL_HANDLE;
                mesh.vertexAllocation = VK_NULL_HANDLE;
            }
            if (mesh.triangleIndexBuffer != VK_NULL_HANDLE)
            {
                vmaDestroyBuffer(allocator_, mesh.triangleIndexBuffer,
                    mesh.triangleIndexAllocation);
                mesh.triangleIndexBuffer = VK_NULL_HANDLE;
                mesh.triangleIndexAllocation = VK_NULL_HANDLE;
            }
            mesh.vertexCount = 0;
            mesh.triangleIndexCount = 0;
            mesh.uploaded = false;
        }

        if (supportSolidInstanceBuffer_ != VK_NULL_HANDLE)
        {
            vmaDestroyBuffer(allocator_, supportSolidInstanceBuffer_,
                supportSolidInstanceAllocation_);
            supportSolidInstanceBuffer_ = VK_NULL_HANDLE;
            supportSolidInstanceAllocation_ = VK_NULL_HANDLE;
        }
        supportSolidInstanceMappedData_ = nullptr;
        supportSolidInstanceCapacity_ = 0;
        supportSolidDrawBatches_.clear();
        supportFoundationDrawBatches_.clear();

        // The timber sampler is borrowed from the ground surface, so it is
        // never destroyed here.
        supportSolidTextureSampler_ = VK_NULL_HANDLE;
        if (supportSolidTextureDescriptorPool_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorPool(device_,
                supportSolidTextureDescriptorPool_, nullptr);
            supportSolidTextureDescriptorPool_ = VK_NULL_HANDLE;
            supportSolidTextureDescriptorSet_ = VK_NULL_HANDLE;
        }
        if (supportSolidTextureDescriptorLayout_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device_,
                supportSolidTextureDescriptorLayout_, nullptr);
            supportSolidTextureDescriptorLayout_ = VK_NULL_HANDLE;
        }
        for (VkPipeline* const pipeline : {&supportSolidPipeline_,
                 &supportFoundationPipeline_})
        {
            if (*pipeline != VK_NULL_HANDLE)
            {
                vkDestroyPipeline(device_, *pipeline, nullptr);
                *pipeline = VK_NULL_HANDLE;
            }
        }
        for (VkPipelineLayout* const layout : {&supportSolidPipelineLayout_,
                 &supportFoundationPipelineLayout_})
        {
            if (*layout != VK_NULL_HANDLE)
            {
                vkDestroyPipelineLayout(device_, *layout, nullptr);
                *layout = VK_NULL_HANDLE;
            }
        }
    }
}
