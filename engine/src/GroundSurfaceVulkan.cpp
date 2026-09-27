#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/GroundSurface.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
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

    // The three built-in 1x1 maps keep the fragment shader free of per-map
    // availability flags: white albedo is neutral under the authored base
    // color, straight up is the horizontal surface normal, and a white
    // roughness multiplier leaves the authored scalar roughness unchanged.
    constexpr std::array<std::uint8_t, 4> fallbackAlbedoTexel{
        255, 255, 255, 255};
    constexpr std::array<std::uint8_t, 4> fallbackNormalTexel{
        128, 128, 255, 255};
    constexpr std::array<std::uint8_t, 4> fallbackRoughnessTexel{
        255, 255, 255, 255};

    quantum::renderer::GroundTextureImage fallbackGroundTextureImage(
        const std::uint32_t slot,
        const bool srgb)
    {
        const std::array<std::uint8_t, 4>& texel = slot == 1
            ? fallbackNormalTexel
            : (slot == 2 ? fallbackRoughnessTexel : fallbackAlbedoTexel);
        return {1, 1, 1, srgb,
            std::vector<std::uint8_t>(texel.begin(), texel.end())};
    }
}

namespace quantum::renderer
{
    void VulkanContext::createGroundSurfaceResources()
    {
        try
        {
            std::array<VkDescriptorSetLayoutBinding, 3> bindings{};
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
                &groundTextureDescriptorLayout_),
                "vkCreateDescriptorSetLayout for ground surface textures");

            VkDescriptorPoolSize poolSize{
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3};
            VkDescriptorPoolCreateInfo poolInfo{};
            poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            poolInfo.maxSets = 1;
            poolInfo.poolSizeCount = 1;
            poolInfo.pPoolSizes = &poolSize;
            check(vkCreateDescriptorPool(device_, &poolInfo, nullptr,
                &groundTextureDescriptorPool_),
                "vkCreateDescriptorPool for ground surface textures");
            VkDescriptorSetAllocateInfo setInfo{};
            setInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            setInfo.descriptorPool = groundTextureDescriptorPool_;
            setInfo.descriptorSetCount = 1;
            setInfo.pSetLayouts = &groundTextureDescriptorLayout_;
            check(vkAllocateDescriptorSets(device_, &setInfo,
                &groundTextureDescriptorSet_),
                "vkAllocateDescriptorSets for ground surface textures");

            // Tiling ground needs a repeating sampler with mip filtering; the
            // HDR environment's clamp-to-edge sampler is not reusable here.
            VkSamplerCreateInfo samplerInfo{};
            samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            samplerInfo.magFilter = VK_FILTER_LINEAR;
            samplerInfo.minFilter = VK_FILTER_LINEAR;
            samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            samplerInfo.maxLod = 16.0F;
            check(vkCreateSampler(device_, &samplerInfo, nullptr,
                &groundTextureSampler_),
                "vkCreateSampler for ground surface textures");

            uploadGroundTextureImage(
                GroundTextureImage{1, 1, 1, true,
                    std::vector<std::uint8_t>(
                        fallbackAlbedoTexel.begin(),
                        fallbackAlbedoTexel.end())},
                VK_FORMAT_R8G8B8A8_SRGB, groundTextureImages_[0]);
            uploadGroundTextureImage(
                GroundTextureImage{1, 1, 1, false,
                    std::vector<std::uint8_t>(
                        fallbackNormalTexel.begin(),
                        fallbackNormalTexel.end())},
                VK_FORMAT_R8G8B8A8_UNORM, groundTextureImages_[1]);
            uploadGroundTextureImage(
                GroundTextureImage{1, 1, 1, false,
                    std::vector<std::uint8_t>(
                        fallbackRoughnessTexel.begin(),
                        fallbackRoughnessTexel.end())},
                VK_FORMAT_R8G8B8A8_UNORM, groundTextureImages_[2]);

            // The image infos must outlive the batch, so they are declared
            // outside the loop and written in place.
            std::array<VkDescriptorImageInfo, 3> imageInfos{};
            std::array<VkWriteDescriptorSet, 3> writes{};
            for (std::uint32_t index = 0; index < writes.size(); ++index)
            {
                imageInfos[index].sampler = groundTextureSampler_;
                imageInfos[index].imageView = groundTextureImages_[index].view;
                imageInfos[index].imageLayout =
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[index].dstSet = groundTextureDescriptorSet_;
                writes[index].dstBinding = index;
                writes[index].descriptorCount = 1;
                writes[index].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[index].pImageInfo = &imageInfos[index];
            }
            vkUpdateDescriptorSets(device_,
                static_cast<std::uint32_t>(writes.size()), writes.data(),
                0, nullptr);

            // The quad is six indices over four vertices for the lifetime of the
            // context, so one retained allocation is rewritten in place.
            const GroundSurfaceMesh mesh =
                createGroundSurfaceMesh(groundSurface_);
            createGroundVertexBuffer(mesh.vertices);
            createGroundIndexBuffer(mesh.indices);
        }
        catch (...)
        {
            destroyGroundSurfaceResources();
            throw;
        }
    }

    void VulkanContext::destroyGroundSurfaceResources() noexcept
    {
        destroyGroundTextureImage(groundTextureImages_[0]);
        destroyGroundTextureImage(groundTextureImages_[1]);
        destroyGroundTextureImage(groundTextureImages_[2]);
        if (groundTextureSampler_ != VK_NULL_HANDLE)
        {
            vkDestroySampler(device_, groundTextureSampler_, nullptr);
            groundTextureSampler_ = VK_NULL_HANDLE;
        }
        if (groundTextureDescriptorPool_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorPool(device_, groundTextureDescriptorPool_,
                nullptr);
            groundTextureDescriptorPool_ = VK_NULL_HANDLE;
            groundTextureDescriptorSet_ = VK_NULL_HANDLE;
        }
        if (groundTextureDescriptorLayout_ != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(device_,
                groundTextureDescriptorLayout_, nullptr);
            groundTextureDescriptorLayout_ = VK_NULL_HANDLE;
        }
        if (groundVertexBuffer_ != VK_NULL_HANDLE)
        {
            vmaDestroyBuffer(allocator_, groundVertexBuffer_,
                groundVertexAllocation_);
            groundVertexBuffer_ = VK_NULL_HANDLE;
            groundVertexAllocation_ = VK_NULL_HANDLE;
        }
        if (groundIndexBuffer_ != VK_NULL_HANDLE)
        {
            vmaDestroyBuffer(allocator_, groundIndexBuffer_,
                groundIndexAllocation_);
            groundIndexBuffer_ = VK_NULL_HANDLE;
            groundIndexAllocation_ = VK_NULL_HANDLE;
        }
        groundVertexMappedData_ = nullptr;
        groundIndexMappedData_ = nullptr;
        groundVertexCount_ = 0;
        groundIndexCount_ = 0;
        groundTextureLoadStatuses_.fill({});
        groundTextureIdentifiers_.fill({});
    }

    void VulkanContext::destroyGroundTextureImage(
        GroundTextureImageResource& image) noexcept
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

    void VulkanContext::uploadGroundTextureImage(
        const GroundTextureImage& source,
        const VkFormat format,
        GroundTextureImageResource& destination)
    {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.format = format;
        imageInfo.extent = {source.width, source.height, 1};
        imageInfo.mipLevels = source.mipLevels;
        imageInfo.arrayLayers = 1;
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
            "vmaCreateImage for ground surface texture");

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = destination.image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = source.mipLevels;
        viewInfo.subresourceRange.layerCount = 1;
        check(vkCreateImageView(device_, &viewInfo, nullptr, &destination.view),
            "vkCreateImageView for ground surface texture");

        const VkDeviceSize bytes = source.rgba.size();
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
                &mappedInfo), "vmaCreateBuffer for ground texture upload");
            if (mappedInfo.pMappedData == nullptr)
            {
                throw std::runtime_error(
                    "The ground texture staging buffer is unmapped.");
            }
            std::memcpy(mappedInfo.pMappedData, source.rgba.data(),
                static_cast<std::size_t>(bytes));
            check(vmaFlushAllocation(allocator_, stagingAllocation, 0, bytes),
                "vmaFlushAllocation for ground texture upload");

            VkCommandBufferAllocateInfo commandInfo{};
            commandInfo.sType =
                VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            commandInfo.commandPool = commandPool_;
            commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            commandInfo.commandBufferCount = 1;
            check(vkAllocateCommandBuffers(device_, &commandInfo, &command),
                "vkAllocateCommandBuffers for ground texture upload");
            VkCommandBufferBeginInfo beginInfo{};
            beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            check(vkBeginCommandBuffer(command, &beginInfo),
                "vkBeginCommandBuffer for ground texture upload");

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
            regions.reserve(source.mipLevels);
            VkDeviceSize offset = 0;
            for (std::uint32_t mip = 0; mip < source.mipLevels; ++mip)
            {
                VkBufferImageCopy region{};
                region.bufferOffset = offset;
                region.imageSubresource.aspectMask =
                    VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.mipLevel = mip;
                region.imageSubresource.baseArrayLayer = 0;
                region.imageSubresource.layerCount = 1;
                region.imageExtent = {std::max(1u, source.width >> mip),
                    std::max(1u, source.height >> mip), 1};
                regions.push_back(region);
                offset += static_cast<VkDeviceSize>(
                    std::max(1u, source.width >> mip))
                    * std::max(1u, source.height >> mip) * 4;
            }
            if (offset != bytes)
            {
                throw std::runtime_error(
                    "The ground texture mip upload size does not match.");
            }
            vkCmdCopyBufferToImage(command, staging, destination.image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                static_cast<std::uint32_t>(regions.size()), regions.data());

            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr,
                0, nullptr, 1, &barrier);
            check(vkEndCommandBuffer(command),
                "vkEndCommandBuffer for ground texture upload");
            VkSubmitInfo submit{};
            submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            submit.commandBufferCount = 1;
            submit.pCommandBuffers = &command;
            check(vkQueueSubmit(graphicsQueue_, 1, &submit, VK_NULL_HANDLE),
                "vkQueueSubmit for ground texture upload");
            check(vkQueueWaitIdle(graphicsQueue_),
                "vkQueueWaitIdle for ground texture upload");
        }
        catch (...)
        {
            if (command != VK_NULL_HANDLE)
            {
                vkFreeCommandBuffers(device_, commandPool_, 1, &command);
            }
            vmaDestroyBuffer(allocator_, staging, stagingAllocation);
            // Only the resource this call created is released. Callers pass a
            // local candidate, so a failure can never destroy a live image.
            destroyGroundTextureImage(destination);
            throw;
        }
        vkFreeCommandBuffers(device_, commandPool_, 1, &command);
        vmaDestroyBuffer(allocator_, staging, stagingAllocation);
    }

    void VulkanContext::replaceGroundTexture(
        const std::uint32_t slot,
        const GroundTextureImage& image,
        const VkFormat format)
    {
        // Candidate-then-publish, matching the rest of the renderer: the new
        // image is built in a local resource, so a decode, allocation, or
        // upload failure leaves the currently published map bound. The
        // previous image is only released after the descriptor points at the
        // replacement and no in-flight frame can still sample it.
        GroundTextureImageResource candidate;
        uploadGroundTextureImage(image, format, candidate);
        waitForFrameCompletion();

        VkDescriptorImageInfo imageInfo{};
        imageInfo.sampler = groundTextureSampler_;
        imageInfo.imageView = candidate.view;
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = groundTextureDescriptorSet_;
        write.dstBinding = slot;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &imageInfo;
        vkUpdateDescriptorSets(device_, 1, &write, 0, nullptr);

        GroundTextureImageResource& published = groundTextureImages_[slot];
        destroyGroundTextureImage(published);
        published = candidate;
    }

    void VulkanContext::setGroundSurface(const GroundSurfaceSettings& settings)
    {
        if (allocator_ == VK_NULL_HANDLE)
        {
            throw std::logic_error(
                "VulkanContext cannot set the ground surface before initialization."
            );
        }
        // Validated on every call so a rejected value never becomes the
        // authoritative state, even on the per-frame push path below.
        validateGroundSurfaceSettings(settings);

        // The Editor pushes the same settings every frame, so the GPU work is
        // gated on the two properties that actually own GPU resources: the quad
        // geometry and the three texture identities. Everything else travels
        // through push constants at draw time.
        const bool geometryChanged =
            settings.elevation != groundSurface_.elevation
            || settings.sizeX != groundSurface_.sizeX
            || settings.sizeY != groundSurface_.sizeY
            || settings.uvTiling != groundSurface_.uvTiling;
        const bool texturesChanged =
            settings.albedoTexture != groundSurface_.albedoTexture
            || settings.normalTexture != groundSurface_.normalTexture
            || settings.roughnessTexture != groundSurface_.roughnessTexture;

        if (!geometryChanged && !texturesChanged)
        {
            groundSurface_.enabled = settings.enabled;
            groundSurface_.baseColor = settings.baseColor;
            groundSurface_.metallic = settings.metallic;
            groundSurface_.roughness = settings.roughness;
            return;
        }

        GroundSurfaceSettings accepted = settings;
        for (std::string* const identifier : {
            &accepted.albedoTexture, &accepted.normalTexture,
            &accepted.roughnessTexture})
        {
            if (!identifier->empty())
            {
                *identifier = normalizeGroundTextureAssetIdentifier(
                    *identifier);
            }
        }
        groundSurface_ = accepted;

        if (texturesChanged)
        {
            // The albedo slot is conventionally sRGB-encoded; normal and
            // roughness data is linear. A per-slot format is retained because
            // the sampler decodes sRGB in hardware.
            static constexpr std::array<VkFormat, 3> formats{
                VK_FORMAT_R8G8B8A8_SRGB,
                VK_FORMAT_R8G8B8A8_UNORM,
                VK_FORMAT_R8G8B8A8_UNORM};
            static constexpr std::array<bool, 3> srgbSlots{true, false, false};
            static constexpr std::array<const char*, 3> slotNames{
                "albedo", "normal", "roughness"};

            const std::array<std::string, 3> identifiers{
                accepted.albedoTexture, accepted.normalTexture,
                accepted.roughnessTexture};
            for (std::uint32_t slot = 0; slot < identifiers.size(); ++slot)
            {
                if (identifiers[slot] == groundTextureIdentifiers_[slot])
                {
                    continue;
                }
                GroundTextureLoadStatus status;
                status.requestedIdentifier = identifiers[slot];
                if (identifiers[slot].empty())
                {
                    // A custom map may already be bound, so clearing the slot
                    // must actively republish its neutral 1x1 fallback.
                    replaceGroundTexture(slot,
                        fallbackGroundTextureImage(slot, srgbSlots[slot]),
                        formats[slot]);
                    groundTextureLoadStatuses_[slot] = std::move(status);
                    groundTextureIdentifiers_[slot] = {};
                    continue;
                }

                const std::filesystem::path path = groundTextureAssetPath(
                    identifiers[slot], runtimeAssetRoot());
                try
                {
                    replaceGroundTexture(slot,
                        loadGroundTextureImage(path, srgbSlots[slot]),
                        formats[slot]);
                    status.state = GroundTextureLoadState::Loaded;
                }
                catch (const std::exception& error)
                {
                    // A failed map is reported and then replaced by the
                    // built-in neutral texel, so one broken file never removes
                    // the surface from the viewport.
                    status.state = classifyGroundTextureFailure(error.what());
                    status.usingFallback = true;
                    status.detail = error.what();
                    replaceGroundTexture(slot,
                        fallbackGroundTextureImage(slot, srgbSlots[slot]),
                        formats[slot]);
                }
                std::string detail;
                if (!status.detail.empty())
                {
                    detail = " - " + status.detail;
                }
                quantum::logging::logMessagef(
                    status.state == GroundTextureLoadState::Loaded
                        ? quantum::logging::LogLevel::Info
                        : quantum::logging::LogLevel::Warning,
                    "GROUND",
                    "Ground %s map '%s': %s%s%s",
                    slotNames[slot],
                    identifiers[slot].c_str(),
                    groundTextureLoadStateName(status.state),
                    status.usingFallback ? " (built-in fallback)" : "",
                    detail.c_str());
                groundTextureLoadStatuses_[slot] = std::move(status);
                groundTextureIdentifiers_[slot] = identifiers[slot];
            }
        }

        if (geometryChanged)
        {
            // The quad is six indices over four vertices, so the retained
            // allocations only need an in-place rewrite. Earlier submissions
            // can still be reading them, so the frame is drained first.
            const GroundSurfaceMesh mesh = createGroundSurfaceMesh(accepted);
            waitForFrameCompletion();
            writeGroundVertexBuffer(mesh.vertices);
            writeGroundIndexBuffer(mesh.indices);
        }
    }

    void VulkanContext::createGroundVertexBuffer(
        const std::span<const GroundSurfaceVertex> vertices)
    {
        const VkDeviceSize size =
            sizeof(GroundSurfaceVertex) * vertices.size();
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo allocationInfo{};
        allocationInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
        VmaAllocationInfo mappedInfo{};
        if (vmaCreateBuffer(allocator_, &bufferInfo, &allocationInfo,
            &groundVertexBuffer_, &groundVertexAllocation_,
            &mappedInfo) != VK_SUCCESS)
        {
            throw std::runtime_error(
                "vmaCreateBuffer failed for the ground surface vertices.");
        }
        groundVertexMappedData_ = mappedInfo.pMappedData;
        groundVertexCapacity_ = size;
        writeGroundVertexBuffer(vertices);
    }

    void VulkanContext::createGroundIndexBuffer(
        const std::span<const std::uint32_t> indices)
    {
        const VkDeviceSize size = sizeof(std::uint32_t) * indices.size();
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VmaAllocationCreateInfo allocationInfo{};
        allocationInfo.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
        VmaAllocationInfo mappedInfo{};
        if (vmaCreateBuffer(allocator_, &bufferInfo, &allocationInfo,
            &groundIndexBuffer_, &groundIndexAllocation_,
            &mappedInfo) != VK_SUCCESS)
        {
            throw std::runtime_error(
                "vmaCreateBuffer failed for the ground surface indices.");
        }
        groundIndexMappedData_ = mappedInfo.pMappedData;
        groundIndexCapacity_ = size;
        writeGroundIndexBuffer(indices);
    }

    void VulkanContext::writeGroundVertexBuffer(
        const std::span<const GroundSurfaceVertex> vertices)
    {
        const VkDeviceSize size =
            sizeof(GroundSurfaceVertex) * vertices.size();
        if (groundVertexMappedData_ == nullptr || vertices.empty()
            || size > groundVertexCapacity_)
        {
            throw std::logic_error(
                "The reusable ground surface vertex buffer is invalid or too small."
            );
        }
        std::memcpy(groundVertexMappedData_, vertices.data(),
            static_cast<std::size_t>(size));
        if (vmaFlushAllocation(allocator_, groundVertexAllocation_, 0, size)
            != VK_SUCCESS)
        {
            throw std::runtime_error(
                "vmaFlushAllocation failed for the ground surface vertices.");
        }
        groundVertexCount_ = static_cast<std::uint32_t>(vertices.size());
    }

    void VulkanContext::writeGroundIndexBuffer(
        const std::span<const std::uint32_t> indices)
    {
        const VkDeviceSize size = sizeof(std::uint32_t) * indices.size();
        if (groundIndexMappedData_ == nullptr || indices.empty()
            || size > groundIndexCapacity_)
        {
            throw std::logic_error(
                "The reusable ground surface index buffer is invalid or too small."
            );
        }
        std::memcpy(groundIndexMappedData_, indices.data(),
            static_cast<std::size_t>(size));
        if (vmaFlushAllocation(allocator_, groundIndexAllocation_, 0, size)
            != VK_SUCCESS)
        {
            throw std::runtime_error(
                "vmaFlushAllocation failed for the ground surface indices.");
        }
        groundIndexCount_ = static_cast<std::uint32_t>(indices.size());
    }

    std::optional<GroundTextureLoadStatus> VulkanContext::
        groundTextureLoadStatus(const std::string_view identifier) const
    {
        for (const GroundTextureLoadStatus& status :
            groundTextureLoadStatuses_)
        {
            if (!identifier.empty() && status.requestedIdentifier == identifier)
            {
                return status;
            }
        }
        return std::nullopt;
    }
}
