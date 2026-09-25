#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/EnvironmentMap.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    void check(const VkResult result, const char* operation)
    {
        if (result != VK_SUCCESS)
            throw std::runtime_error(std::string(operation)
                + " failed with VkResult " + std::to_string(result));
    }
}

namespace quantum::renderer
{
    void VulkanContext::createEnvironmentResources()
    {
        ProcessedEnvironment processed;
        try
        {
            processed = preprocessEnvironment(runtimeAssetRoot()
                / "assets/environment/rooitou_park_1k.hdr");
            environmentAvailable_ = true;
        }
        catch (const std::exception& error)
        {
            quantum::logging::logMessagef(logging::LogLevel::Warning, "VK",
                "HDR environment unavailable: %s. Using constant ambient.",
                error.what());
            const EnvironmentPixels black{1, 1, 6, 1,
                std::vector<float>(6 * 4, 0.0F)};
            processed.sky = black;
            processed.irradiance = black;
            processed.specular = black;
            processed.brdf = {1, 1, 1, 1, {0.0F, 0.0F, 0.0F, 1.0F}};
        }

        const auto upload = [this](const EnvironmentPixels& pixels,
            EnvironmentImage& destination)
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
                &destination.image, &destination.allocation, nullptr),
                "vmaCreateImage for HDR environment");

            VkImageViewCreateInfo viewInfo{};
            viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            viewInfo.image = destination.image;
            viewInfo.viewType = pixels.layers == 6
                ? VK_IMAGE_VIEW_TYPE_CUBE : VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = imageInfo.format;
            viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            viewInfo.subresourceRange.levelCount = pixels.mipLevels;
            viewInfo.subresourceRange.layerCount = pixels.layers;
            check(vkCreateImageView(device_, &viewInfo, nullptr,
                &destination.view), "vkCreateImageView for HDR environment");

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
            check(vmaCreateBuffer(allocator_, &stagingInfo,
                &stagingAllocationInfo, &staging, &stagingAllocation,
                &mappedInfo), "vmaCreateBuffer for HDR upload");
            VkCommandBuffer command = VK_NULL_HANDLE;
            try
            {
                if (mappedInfo.pMappedData == nullptr)
                    throw std::runtime_error("HDR staging buffer is unmapped");
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
                barrier.image = destination.image;
                barrier.subresourceRange = viewInfo.subresourceRange;
                vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
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
                    throw std::runtime_error("HDR mip upload size mismatch");
                vkCmdCopyBufferToImage(command, staging, destination.image,
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
                    vkFreeCommandBuffers(device_, commandPool_, 1, &command);
                vmaDestroyBuffer(allocator_, staging, stagingAllocation);
                throw;
            }
            vkFreeCommandBuffers(device_, commandPool_, 1, &command);
            vmaDestroyBuffer(allocator_, staging, stagingAllocation);
        };

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
            layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
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

            upload(processed.irradiance, environmentImages_[0]);
            upload(processed.specular, environmentImages_[1]);
            upload(processed.brdf, environmentImages_[2]);
            upload(processed.sky, environmentImages_[3]);

            std::array<VkDescriptorImageInfo, 4> images{};
            std::array<VkWriteDescriptorSet, 4> writes{};
            for (std::uint32_t index = 0; index < images.size(); ++index)
            {
                images[index].sampler = environmentSampler_;
                images[index].imageView = environmentImages_[index].view;
                images[index].imageLayout =
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[index].dstSet = environmentDescriptorSet_;
                writes[index].dstBinding = index;
                writes[index].descriptorCount = 1;
                writes[index].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[index].pImageInfo = &images[index];
            }
            vkUpdateDescriptorSets(device_,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);
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
        for (EnvironmentImage& image : environmentImages_)
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
        if (environmentDescriptorLayout_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device_, environmentDescriptorLayout_,
                nullptr);
            environmentDescriptorLayout_ = VK_NULL_HANDLE;
        }
        environmentAvailable_ = false;
    }
}
