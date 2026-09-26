#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/EnvironmentAssets.hpp>
#include <quantum/renderer/EnvironmentMap.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <glm/trigonometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    void check(const VkResult result, const char* const operation)
    {
        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(std::string(operation)
                + " failed with VkResult " + std::to_string(result));
        }
    }
}

namespace quantum::renderer
{
    void VulkanContext::uploadEnvironmentImageSet(
        const ProcessedEnvironment& processed,
        EnvironmentImageSet& destination)
    {
        // The caller always supplies an empty set, so a failure here can only
        // release what this call created.
        for (const EnvironmentImage& image : destination)
        {
            if (image.image != VK_NULL_HANDLE)
            {
                throw std::logic_error(
                    "The HDR environment upload target is already populated.");
            }
        }

        const auto upload = [this](const EnvironmentPixels& pixels,
            EnvironmentImage& target)
        {
            VkImageCreateInfo imageInfo{};
            imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            imageInfo.flags = pixels.layers == 6
                ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0;
            imageInfo.imageType = VK_IMAGE_TYPE_2D;
            imageInfo.format = VK_FORMAT_R32G32B32A32_SFLOAT;
            imageInfo.extent = {pixels.width, pixels.height, 1};
            imageInfo.mipLevels = pixels.mipLevels;
            imageInfo.arrayLayers = pixels.layers;
            imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT
                | VK_IMAGE_USAGE_SAMPLED_BIT;
            imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo allocationInfo{};
            allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
            check(vmaCreateImage(allocator_, &imageInfo, &allocationInfo,
                &target.image, &target.allocation, nullptr),
                "vmaCreateImage for HDR environment");

            VkImageViewCreateInfo viewInfo{};
            viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            viewInfo.image = target.image;
            viewInfo.viewType = pixels.layers == 6
                ? VK_IMAGE_VIEW_TYPE_CUBE : VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = imageInfo.format;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = pixels.mipLevels;
            viewInfo.subresourceRange.layerCount = pixels.layers;
            check(vkCreateImageView(device_, &viewInfo, nullptr, &target.view),
                "vkCreateImageView for HDR environment");

            const VkDeviceSize bytes = pixels.rgba.size() * sizeof(float);
            VkBufferCreateInfo stagingInfo{};
            stagingInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            stagingInfo.size = bytes;
            stagingInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            stagingInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            VmaAllocationCreateInfo stagingAllocationInfo{};
            stagingAllocationInfo.flags =
                VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            stagingAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
            VkBuffer staging = VK_NULL_HANDLE;
            VmaAllocation stagingAllocation = VK_NULL_HANDLE;
            VmaAllocationInfo mappedInfo{};
            VkCommandBuffer command = VK_NULL_HANDLE;
            try
            {
                check(vmaCreateBuffer(allocator_, &stagingInfo,
                    &stagingAllocationInfo, &staging, &stagingAllocation,
                    &mappedInfo), "vmaCreateBuffer for HDR upload");
                if (mappedInfo.pMappedData == nullptr)
                {
                    throw std::runtime_error(
                        "HDR staging buffer is unmapped.");
                }
                std::memcpy(mappedInfo.pMappedData, pixels.rgba.data(),
                    static_cast<std::size_t>(bytes));
                check(vmaFlushAllocation(allocator_, stagingAllocation,
                    0, bytes), "vmaFlushAllocation for HDR upload");

                VkCommandBufferAllocateInfo commandInfo{};
                commandInfo.sType =
                    VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                commandInfo.commandPool = commandPool_;
                commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                commandInfo.commandBufferCount = 1;
                check(vkAllocateCommandBuffers(device_, &commandInfo,
                    &command), "vkAllocateCommandBuffers for HDR upload");
                VkCommandBufferBeginInfo beginInfo{};
                beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(command, &beginInfo),
                    "vkBeginCommandBuffer for HDR upload");

                VkImageMemoryBarrier barrier{};
                barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.image = target.image;
                barrier.subresourceRange = viewInfo.subresourceRange;
                vkCmdPipelineBarrier(command,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr,
                    1, &barrier);

                std::vector<VkBufferImageCopy> regions;
                regions.reserve(pixels.layers * pixels.mipLevels);
                VkDeviceSize offset = 0;
                for (std::uint32_t layer = 0; layer < pixels.layers; ++layer)
                    for (std::uint32_t mip = 0; mip < pixels.mipLevels; ++mip)
                    {
                        const std::uint32_t width =
                            std::max(1u, pixels.width >> mip);
                        const std::uint32_t height =
                            std::max(1u, pixels.height >> mip);
                        VkBufferImageCopy region{};
                        region.bufferOffset = offset;
                        region.imageSubresource.aspectMask =
                            VK_IMAGE_ASPECT_COLOR_BIT;
                        region.imageSubresource.mipLevel = mip;
                        region.imageSubresource.baseArrayLayer = layer;
                        region.imageSubresource.layerCount = 1;
                        region.imageExtent = {width, height, 1};
                        regions.push_back(region);
                        offset += static_cast<VkDeviceSize>(width) * height
                            * 4 * sizeof(float);
                    }
                if (offset != bytes)
                {
                    throw std::runtime_error("HDR mip upload size mismatch");
                }
                vkCmdCopyBufferToImage(command, staging, target.image,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    static_cast<std::uint32_t>(regions.size()),
                    regions.data());

                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr,
                    0, nullptr, 1, &barrier);
                check(vkEndCommandBuffer(command),
                    "vkEndCommandBuffer for HDR upload");
                VkSubmitInfo submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
                submit.commandBufferCount = 1;
                submit.pCommandBuffers = &command;
                check(vkQueueSubmit(graphicsQueue_, 1, &submit, VK_NULL_HANDLE),
                    "vkQueueSubmit for HDR upload");
                check(vkQueueWaitIdle(graphicsQueue_),
                    "vkQueueWaitIdle for HDR upload");
            }
            catch (...)
            {
                if (command != VK_NULL_HANDLE)
                {
                    vkFreeCommandBuffers(device_, commandPool_, 1, &command);
                }
                vmaDestroyBuffer(allocator_, staging, stagingAllocation);
                destroyEnvironmentImage(target);
                throw;
            }
            vkFreeCommandBuffers(device_, commandPool_, 1, &command);
            vmaDestroyBuffer(allocator_, staging, stagingAllocation);
        };

        try
        {
            upload(processed.irradiance, destination[0]);
            upload(processed.specular, destination[1]);
            upload(processed.brdf, destination[2]);
            upload(processed.sky, destination[3]);
        }
        catch (...)
        {
            destroyEnvironmentImageSet(destination);
            throw;
        }
    }

    void VulkanContext::destroyEnvironmentImage(
        EnvironmentImage& image) noexcept
    {
        if (image.view != VK_NULL_HANDLE)
        {
            vkDestroyImageView(device_, image.view, nullptr);
            image.view = VK_NULL_HANDLE;
        }
        if (image.image != VK_NULL_HANDLE)
        {
            vmaDestroyImage(allocator_, image.image, image.allocation);
            image.image = VK_NULL_HANDLE;
            image.allocation = VK_NULL_HANDLE;
        }
    }

    void VulkanContext::destroyEnvironmentImageSet(
        EnvironmentImageSet& images) noexcept
    {
        for (EnvironmentImage& image : images)
        {
            destroyEnvironmentImage(image);
        }
    }

    void VulkanContext::createEnvironmentResources()
    {
        try
        {
            std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
            for (std::uint32_t index = 0; index < bindings.size(); ++index)
            {
                bindings[index].binding = index;
                bindings[index].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[index].descriptorCount = 1;
                bindings[index].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
            }
            VkDescriptorSetLayoutCreateInfo layoutInfo{};
            layoutInfo.sType =
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            layoutInfo.bindingCount =
                static_cast<std::uint32_t>(bindings.size());
            layoutInfo.pBindings = bindings.data();
            check(vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr,
                &environmentDescriptorLayout_),
                "vkCreateDescriptorSetLayout for HDR environment");

            VkDescriptorPoolSize poolSize{
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4};
            VkDescriptorPoolCreateInfo poolInfo{};
            poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            poolInfo.maxSets = 1;
            poolInfo.poolSizeCount = 1;
            poolInfo.pPoolSizes = &poolSize;
            check(vkCreateDescriptorPool(device_, &poolInfo, nullptr,
                &environmentDescriptorPool_),
                "vkCreateDescriptorPool for HDR environment");
            VkDescriptorSetAllocateInfo setInfo{};
            setInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            setInfo.descriptorPool = environmentDescriptorPool_;
            setInfo.descriptorSetCount = 1;
            setInfo.pSetLayouts = &environmentDescriptorLayout_;
            check(vkAllocateDescriptorSets(device_, &setInfo,
                &environmentDescriptorSet_),
                "vkAllocateDescriptorSets for HDR environment");

            VkSamplerCreateInfo samplerInfo{};
            samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            samplerInfo.magFilter = VK_FILTER_LINEAR;
            samplerInfo.minFilter = VK_FILTER_LINEAR;
            samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            samplerInfo.maxLod = 6.0F;
            check(vkCreateSampler(device_, &samplerInfo, nullptr,
                &environmentSampler_), "vkCreateSampler for HDR environment");

            // IBL preprocessing and upload are expensive, so the first
            // environment is prepared here, which keeps startup behavior
            // identical to Rendering M2. Later selections are prepared once on
            // demand and then retained by the cache.
            const std::string initial = environmentAsset_.empty()
                ? std::string(bundledEnvironmentAssets().front().identifier)
                : environmentAsset_;
            environmentDetail_.clear();
            try
            {
                setEnvironmentImages(loadEnvironmentImages(initial).images);
            }
            catch (const std::exception& error)
            {
                environmentDetail_ = error.what();
                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Warning, "VK",
                    "HDR environment '%s' unavailable: %s. Using constant "
                    "ambient.", initial.c_str(), error.what());
                environmentAvailable_ = false;
            }
            environmentAsset_ = initial;
        }
        catch (...)
        {
            destroyEnvironmentResources();
            throw;
        }
    }

    void VulkanContext::destroyEnvironmentResources() noexcept
    {
        if (environmentDescriptorPool_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorPool(device_, environmentDescriptorPool_, nullptr);
            environmentDescriptorPool_ = VK_NULL_HANDLE;
            environmentDescriptorSet_ = VK_NULL_HANDLE;
        }
        if (environmentSampler_ != VK_NULL_HANDLE)
        {
            vkDestroySampler(device_, environmentSampler_, nullptr);
            environmentSampler_ = VK_NULL_HANDLE;
        }
        // The cache is the sole owner of the images; environmentImages_ only
        // points at the published entry, so it must not be destroyed here.
        environmentImages_ = nullptr;
        for (auto& entry : environmentCache_)
        {
            if (entry.second.images[0].image != VK_NULL_HANDLE)
            {
                destroyEnvironmentImageSet(entry.second.images);
            }
        }
        environmentCache_.clear();
        if (environmentDescriptorLayout_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device_, environmentDescriptorLayout_,
                nullptr);
            environmentDescriptorLayout_ = VK_NULL_HANDLE;
        }
        environmentAvailable_ = false;
    }

    void VulkanContext::setEnvironment(const std::string_view identifier,
        const float rotationDegrees, const float lightingIntensity,
        const bool skyVisible)
    {
        if (!std::isfinite(rotationDegrees) || !std::isfinite(lightingIntensity)
            || lightingIntensity < 0.0F)
        {
            throw std::invalid_argument("Invalid HDR environment settings.");
        }
        const std::string accepted = validateEnvironmentAssetIdentifier(
            identifier);
        if (accepted != environmentAsset_)
        {
            environmentDetail_.clear();
            if (accepted.empty())
            {
                // "None" keeps the renderer's constant ambient term.
                environmentImages_ = nullptr;
                environmentAvailable_ = false;
            }
            else
            {
                try
                {
                    setEnvironmentImages(loadEnvironmentImages(accepted).images);
                }
                catch (const std::exception& error)
                {
                    // The failure is reported and the environment is treated as
                    // unavailable. A previously prepared sky stays cached and is
                    // never silently presented as the requested one.
                    environmentDetail_ = error.what();
                    environmentAvailable_ = false;
                    quantum::logging::logMessagef(
                        quantum::logging::LogLevel::Warning, "VK",
                        "HDR environment '%s' unavailable: %s. Using constant "
                        "ambient.", accepted.c_str(), error.what());
                }
            }
            environmentAsset_ = accepted;
        }
        environmentRotationRadians_ = glm::radians(
            std::fmod(std::fmod(rotationDegrees, 360.0F) + 360.0F, 360.0F));
        environmentIntensity_ = lightingIntensity;
        skyVisible_ = skyVisible;
    }

    Renderer::EnvironmentStatus VulkanContext::environmentStatus() const
    {
        return {environmentAsset_, environmentAvailable_, environmentDetail_};
    }

    const VulkanContext::PreparedEnvironment& VulkanContext::
        loadEnvironmentImages(const std::string& identifier)
    {
        if (const auto found = environmentCache_.find(identifier);
            found != environmentCache_.end())
        {
            return found->second;
        }
        const EnvironmentAsset* asset = findBundledEnvironmentAsset(identifier);
        if (asset == nullptr)
        {
            throw std::invalid_argument(
                "Unknown bundled HDR environment identifier: " + identifier);
        }

        PreparedEnvironment prepared;
        // Throws for a missing or malformed file; the caller decides whether
        // that is reported or falls back to constant ambient.
        uploadEnvironmentImageSet(
            preprocessEnvironment(environmentFilePath(asset->identifier)),
            prepared.images);
        quantum::logging::logMessagef(quantum::logging::LogLevel::Info, "VK",
            "Prepared HDR environment '%s'.", identifier.c_str());
        return environmentCache_.emplace(
            identifier, std::move(prepared)).first->second;
    }

    std::filesystem::path VulkanContext::environmentFilePath(
        const std::string& identifier) const
    {
        constexpr std::string_view assetsScheme = "assets://";
        if (!identifier.starts_with(assetsScheme))
        {
            throw std::invalid_argument(
                "Bundled HDR environments use package-relative identities: "
                + identifier);
        }
        return runtimeAssetRoot() / "assets"
            / std::filesystem::path(identifier.substr(assetsScheme.size()));
    }

    void VulkanContext::setEnvironmentImages(
        const EnvironmentImageSet& images)
    {
        if (images[0].image == VK_NULL_HANDLE)
        {
            throw std::logic_error(
                "Refusing to publish an empty HDR environment image set.");
        }
        // Candidate-then-publish, matching the ground-surface path: after the
        // descriptor set is repointed, no recorded or in-flight command buffer
        // may still reference the images it previously sampled. The set being
        // published is owned by the cache, which outlives every submission.
        waitForFrameCompletion();
        environmentImages_ = &images;

        // The image infos must outlive the batch they are passed in.
        std::array<VkDescriptorImageInfo, 4> imageInfos{};
        std::array<VkWriteDescriptorSet, 4> writes{};
        for (std::uint32_t index = 0; index < writes.size(); ++index)
        {
            imageInfos[index].sampler = environmentSampler_;
            imageInfos[index].imageView = environmentImages_->at(index).view;
            imageInfos[index].imageLayout =
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index].dstSet = environmentDescriptorSet_;
            writes[index].dstBinding = index;
            writes[index].descriptorCount = 1;
            writes[index].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[index].pImageInfo = &imageInfos[index];
        }
        vkUpdateDescriptorSets(device_,
            static_cast<std::uint32_t>(writes.size()), writes.data(),
            0, nullptr);
        environmentAvailable_ = true;
    }
}
