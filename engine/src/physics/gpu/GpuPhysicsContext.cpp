#include <quantum/physics/gpu/GpuPhysicsContext.hpp>

#include <quantum/engine/Logging.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace quantum::physics::gpu
{
    namespace
    {
        struct GpuTrackSample
        {
            double stationMeters = 0.0;
            double positionMeters[3] = {0.0, 0.0, 0.0};
            double tangent[3] = {1.0, 0.0, 0.0};
            double lateral[3] = {0.0, 1.0, 0.0};
            double up[3] = {0.0, 0.0, 1.0};
            double curvaturePerMeter[3] = {0.0, 0.0, 0.0};
        };
        // std430 GLSL: double (8) + 5x dvec3 (24 each) = 128. No extra padding.
        static_assert(sizeof(GpuTrackSample) == 128, "GpuTrackSample 128 std430");
        static_assert(alignof(GpuTrackSample) == 8, "GpuTrackSample align 8");
        static_assert(offsetof(GpuTrackSample, stationMeters) == 0, "station offset 0");
        static_assert(offsetof(GpuTrackSample, positionMeters) == 8, "position offset 8");
        static_assert(offsetof(GpuTrackSample, curvaturePerMeter) == 104, "curvature offset 104");

        struct SharedCoasterRecord
        {
            std::uint32_t trackOffset = 0;
            std::uint32_t trackCount = 0;
            std::uint32_t topology = 0;
            std::uint32_t _pad = 0;
            double length = 0.0;
        };
        // Must match GLSL GpuCoasterRecord std430: 4x uint (16) + double (8) = 24
        static_assert(sizeof(SharedCoasterRecord) == 24, "SharedCoasterRecord 24 std430");
        static_assert(alignof(SharedCoasterRecord) == 8, "SharedCoasterRecord align 8");
        static_assert(offsetof(SharedCoasterRecord, length) == 16, "CoasterRecord length offset 16");
        // Note: GLSL member is named trackLength to avoid shadowing builtin length(),
        // but offset is identical (16). C++ uses length for historical MAX compatibility.

        struct SharedGpuState
        {
            std::vector<GpuTrackSample> samples;
            std::vector<std::vector<GpuTrackSample>> perCoaster;
            std::vector<SharedCoasterRecord> records;
        };

        // Per-context CPU mirror for validation when no GPU is present.
        // M0 is single-threaded editor usage; protect with mutex for correctness
        // if multiple contexts are used from different threads (e.g., tests).
        [[nodiscard]] std::unordered_map<const void*, SharedGpuState>& sharedMap()
        {
            static std::unordered_map<const void*, SharedGpuState> map;
            return map;
        }
        [[nodiscard]] std::mutex& sharedMapMutex()
        {
            static std::mutex mutex;
            return mutex;
        }

        [[nodiscard]] bool finite3(const glm::dvec3& v) noexcept
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        [[nodiscard]] glm::dquat frameQuat(const glm::dvec3& t, const glm::dvec3& l, const glm::dvec3& u)
        {
            return glm::normalize(glm::quat_cast(glm::dmat3{t, l, u}));
        }

        [[nodiscard]] PhysicsTrackSample cpuSampleOne(
            const GpuTrackQuery& query,
            const std::vector<GpuTrackSample>& samples,
            std::uint32_t offset,
            std::uint32_t count,
            double length,
            std::uint32_t topology)
        {
            PhysicsTrackSample out{};
            out.location = query;

            double station = query.stationMeters;
            if (std::isfinite(length) && length > 0.0 && topology == 1u)
            {
                if (station < 0.0)
                {
                    station = std::fmod(station, length);
                    if (station < 0.0) station += length;
                }
                else if (station >= length)
                {
                    station = std::fmod(station, length);
                }
            }
            else
            {
                if (station < 0.0) station = 0.0;
                if (station > length) station = length;
            }
            out.location.stationMeters = station;

            if (count == 0 || offset + count > samples.size())
                return out;

            std::uint32_t low = offset;
            std::uint32_t high = offset + count;
            while (low < high)
            {
                const std::uint32_t mid = (low + high) >> 1;
                if (samples[mid].stationMeters < station)
                    low = mid + 1;
                else
                    high = mid;
            }
            const std::uint32_t upper = low;

            GpuTrackSample sm{};

            if (upper == offset)
            {
                sm = samples[offset];
            }
            else if (upper >= offset + count)
            {
                sm = samples[offset + count - 1];
            }
            else
            {
                const GpuTrackSample& before = samples[upper - 1];
                const GpuTrackSample& after = samples[upper];
                const double denom = after.stationMeters - before.stationMeters;
                double t = (denom > 0.0) ? (station - before.stationMeters) / denom : 0.0;
                t = std::clamp(t, 0.0, 1.0);

                sm.stationMeters = std::lerp(before.stationMeters, after.stationMeters, t);
                for (int i = 0; i < 3; ++i)
                {
                    sm.positionMeters[i] = std::lerp(before.positionMeters[i], after.positionMeters[i], t);
                    sm.curvaturePerMeter[i] = std::lerp(before.curvaturePerMeter[i], after.curvaturePerMeter[i], t);
                }
                glm::dvec3 bt{before.tangent[0], before.tangent[1], before.tangent[2]};
                glm::dvec3 bl{before.lateral[0], before.lateral[1], before.lateral[2]};
                glm::dvec3 bu{before.up[0], before.up[1], before.up[2]};
                glm::dvec3 at{after.tangent[0], after.tangent[1], after.tangent[2]};
                glm::dvec3 al{after.lateral[0], after.lateral[1], after.lateral[2]};
                glm::dvec3 au{after.up[0], after.up[1], after.up[2]};
                glm::dquat q0 = frameQuat(bt, bl, bu);
                glm::dquat q1 = frameQuat(at, al, au);
                if (glm::dot(q0, q1) < 0.0) q1 = -q1;
                glm::dquat q = glm::normalize(glm::slerp(q0, q1, t));
                glm::dmat3 m = glm::mat3_cast(q);
                sm.tangent[0] = m[0].x; sm.tangent[1] = m[0].y; sm.tangent[2] = m[0].z;
                sm.lateral[0] = m[1].x; sm.lateral[1] = m[1].y; sm.lateral[2] = m[1].z;
                sm.up[0] = m[2].x; sm.up[1] = m[2].y; sm.up[2] = m[2].z;
            }

            out.position[0] = sm.positionMeters[0];
            out.position[1] = sm.positionMeters[1];
            out.position[2] = sm.positionMeters[2];
            out.position[3] = 0.0;
            out.tangent[0] = sm.tangent[0];
            out.tangent[1] = sm.tangent[1];
            out.tangent[2] = sm.tangent[2];
            out.tangent[3] = 0.0;
            out.lateral[0] = sm.lateral[0];
            out.lateral[1] = sm.lateral[1];
            out.lateral[2] = sm.lateral[2];
            out.lateral[3] = 0.0;
            out.up[0] = sm.up[0];
            out.up[1] = sm.up[1];
            out.up[2] = sm.up[2];
            out.up[3] = 0.0;
            out.curvature[0] = sm.curvaturePerMeter[0];
            out.curvature[1] = sm.curvaturePerMeter[1];
            out.curvature[2] = sm.curvaturePerMeter[2];
            out.curvature[3] = 0.0;
            return out;
        }

        [[nodiscard]] VkBufferCreateInfo makeStorageBufferInfo(VkDeviceSize size) noexcept
        {
            VkBufferCreateInfo info{};
            info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            info.size = size;
            info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
            info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            return info;
        }

        [[nodiscard]] std::filesystem::path gpuShaderPath()
        {
            // Follow VulkanContext::shaderPath convention: <executableBase>/shaders/<file>
            const char* basePath = SDL_GetBasePath();
            if (basePath == nullptr)
                return std::filesystem::path("shaders") / "track_sample.comp.spv";
            std::filesystem::path p(basePath);
            return p / "shaders" / "track_sample.comp.spv";
        }

        [[nodiscard]] std::vector<std::uint32_t> readSpirvFile(const std::filesystem::path& path)
        {
            std::ifstream file(path, std::ios::binary | std::ios::ate);
            if (!file)
                throw std::runtime_error("Unable to open shader: " + path.string());
            const std::streampos end = file.tellg();
            if (end <= 0 || (static_cast<std::uint64_t>(end) % sizeof(std::uint32_t)) != 0)
                throw std::runtime_error("Shader invalid SPIR-V size: " + path.string());
            const std::size_t bytes = static_cast<std::size_t>(end);
            std::vector<std::uint32_t> code(bytes / sizeof(std::uint32_t));
            file.seekg(0);
            file.read(reinterpret_cast<char*>(code.data()), static_cast<std::streamsize>(bytes));
            if (!file)
                throw std::runtime_error("Unable to read shader: " + path.string());
            return code;
        }

        [[nodiscard]] VkShaderModule createShaderModuleLocal(VkDevice device, const std::vector<std::uint32_t>& code)
        {
            VkShaderModuleCreateInfo ci{};
            ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            ci.codeSize = code.size() * sizeof(std::uint32_t);
            ci.pCode = code.data();
            VkShaderModule mod = VK_NULL_HANDLE;
            const VkResult r = vkCreateShaderModule(device, &ci, nullptr, &mod);
            if (r != VK_SUCCESS)
                throw std::runtime_error("vkCreateShaderModule failed for track_sample.comp");
            return mod;
        }

        [[nodiscard]] double angleBetweenDeg(const glm::dvec3& a, const glm::dvec3& b) noexcept
        {
            const double la = glm::length(a);
            const double lb = glm::length(b);
            if (la == 0.0 || lb == 0.0) return 0.0;
            double c = glm::dot(a, b) / (la * lb);
            c = std::clamp(c, -1.0, 1.0);
            return glm::degrees(std::acos(c));
        }
    }



    GpuPhysicsContext::GpuPhysicsContext(renderer::VulkanContext& vulkan)
        : vulkan_(&vulkan)
        , device_(vulkan.device())
        , allocator_(vulkan.allocator())
        , computeQueue_(vulkan.graphicsQueue())
        , computeQueueFamily_(vulkan.graphicsQueueFamily())
        , shaderFloat64Enabled_(vulkan.shaderFloat64Enabled())
    {
        if (device_ == VK_NULL_HANDLE || allocator_ == VK_NULL_HANDLE)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext: no Vulkan device, CPU fallback only");
            return;
        }
        if (!shaderFloat64Enabled_)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext: shaderFloat64 not enabled, CPU fallback");
            return;
        }

        try
        {
            createCommandPool();
            createDescriptorResources();
            createPipeline();
            createSyncResources();
            createTimestampPool();
        }
        catch (const std::exception& e)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext GPU init failed (CPU fallback): %s", e.what());
        }
    }

    GpuPhysicsContext::GpuPhysicsContext(HeadlessHandles handles)
        : vulkan_(nullptr)
        , headless_(handles)
        , useHeadless_(true)
        , device_(handles.device)
        , allocator_(handles.allocator)
        , computeQueue_(handles.queue)
        , computeQueueFamily_(handles.queueFamily)
        , shaderFloat64Enabled_(handles.shaderFloat64Enabled)
    {
        if (device_ == VK_NULL_HANDLE || allocator_ == VK_NULL_HANDLE)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext(headless): no Vulkan device, CPU fallback only");
            return;
        }
        if (!shaderFloat64Enabled_)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext(headless): shaderFloat64 not enabled, CPU fallback");
            return;
        }
        try
        {
            createCommandPool();
            createDescriptorResources();
            createPipeline();
            createSyncResources();
            createTimestampPool();
        }
        catch (const std::exception& e)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Info, "VK", "GpuPhysicsContext(headless) GPU init failed (CPU fallback): %s", e.what());
        }
    }

    GpuPhysicsContext::HeadlessHandles GpuPhysicsContext::createHeadlessHandles()
    {
        HeadlessHandles h{};
        // Create minimal Vulkan instance without surface/extensions
        VkApplicationInfo app{};
        app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        app.pApplicationName = "QUANTUM-headless";
        app.apiVersion = VK_API_VERSION_1_3;
        VkInstanceCreateInfo ici{};
        ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        ici.pApplicationInfo = &app;
        VkInstance inst = VK_NULL_HANDLE;
        VkResult r = vkCreateInstance(&ici, nullptr, &inst);
        if (r != VK_SUCCESS)
            throw std::runtime_error("headless vkCreateInstance failed");
        h.instance = inst;
        h.ownsInstance = true;

        uint32_t pdCount = 0;
        r = vkEnumeratePhysicalDevices(inst, &pdCount, nullptr);
        if (pdCount == 0)
            throw std::runtime_error("headless: no physical devices");
        std::vector<VkPhysicalDevice> pds(pdCount);
        r = vkEnumeratePhysicalDevices(inst, &pdCount, pds.data());

        VkPhysicalDevice chosen = VK_NULL_HANDLE;
        uint32_t chosenQF = UINT32_MAX;
        bool chosenFloat64 = false;
        for (auto pd : pds)
        {
            VkPhysicalDeviceProperties props{};
            vkGetPhysicalDeviceProperties(pd, &props);
            if (props.apiVersion < VK_API_VERSION_1_3) continue;

            VkPhysicalDeviceVulkan13Features v13{};
            v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
            VkPhysicalDeviceFeatures2 f2{};
            f2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
            f2.pNext = &v13;
            vkGetPhysicalDeviceFeatures2(pd, &f2);
            if (v13.dynamicRendering != VK_TRUE
                || v13.maintenance4 != VK_TRUE) continue;
            if (f2.features.shaderFloat64 != VK_TRUE) continue;

            uint32_t qCount = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(pd, &qCount, nullptr);
            std::vector<VkQueueFamilyProperties> qProps(qCount);
            vkGetPhysicalDeviceQueueFamilyProperties(pd, &qCount, qProps.data());
            uint32_t gfx = UINT32_MAX;
            for (uint32_t i = 0; i < qCount; ++i)
                if (qProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) { gfx = i; break; }
            if (gfx == UINT32_MAX) continue;

            chosen = pd;
            chosenQF = gfx;
            chosenFloat64 = true;
            break;
        }
        if (chosen == VK_NULL_HANDLE)
            throw std::runtime_error("headless: no suitable physical device with shaderFloat64");

        h.physicalDevice = chosen;
        h.queueFamily = chosenQF;
        h.shaderFloat64Enabled = chosenFloat64;

        const float prio = 1.0f;
        VkDeviceQueueCreateInfo dq{};
        dq.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        dq.queueFamilyIndex = chosenQF;
        dq.queueCount = 1;
        dq.pQueuePriorities = &prio;

        VkPhysicalDeviceVulkan13Features v13e{};
        v13e.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
        v13e.dynamicRendering = VK_TRUE;
        v13e.maintenance4 = VK_TRUE;
        VkPhysicalDeviceFeatures2 f2e{};
        f2e.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        f2e.features.shaderFloat64 = VK_TRUE;
        f2e.pNext = &v13e;

        VkDeviceCreateInfo dci{};
        dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        dci.pNext = &f2e;
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &dq;

        VkDevice dev = VK_NULL_HANDLE;
        VkResult devRes = vkCreateDevice(chosen, &dci, nullptr, &dev);
        if (devRes != VK_SUCCESS)
            throw std::runtime_error("headless vkCreateDevice failed");
        h.device = dev;
        h.ownsDevice = true;
        VkQueue q = VK_NULL_HANDLE;
        vkGetDeviceQueue(dev, chosenQF, 0, &q);
        h.queue = q;

        VmaAllocatorCreateInfo aci{};
        aci.instance = inst;
        aci.physicalDevice = chosen;
        aci.device = dev;
        aci.vulkanApiVersion = VK_API_VERSION_1_3;
        VmaAllocator alloc = VK_NULL_HANDLE;
        VkResult vmaRes = vmaCreateAllocator(&aci, &alloc);
        if (vmaRes != VK_SUCCESS)
            throw std::runtime_error("headless vmaCreateAllocator failed");
        h.allocator = alloc;
        h.ownsAllocator = true;

        return h;
    }

    GpuPhysicsContext::~GpuPhysicsContext()
    {
        {
            std::lock_guard<std::mutex> lock(sharedMapMutex());
            sharedMap().erase(this);
        }

        if (device_ == VK_NULL_HANDLE)
        {
            // Even if device null, clean headless instance if owned
            if (useHeadless_ && headless_.ownsInstance && headless_.instance != VK_NULL_HANDLE)
                vkDestroyInstance(headless_.instance, nullptr);
            return;
        }

        vkDeviceWaitIdle(device_);

        for (auto& f : frames_)
        {
            // Query/Result are not persistently mapped (map/unmap per upload), readback is persistent (MAPPED_BIT) – destroy without unmap
            if (f.queryBuffer) vmaDestroyBuffer(allocator_, f.queryBuffer, f.queryAllocation);
            if (f.resultBuffer) vmaDestroyBuffer(allocator_, f.resultBuffer, f.resultAllocation);
            if (f.readbackBuffer) vmaDestroyBuffer(allocator_, f.readbackBuffer, f.readbackAllocation);
            f.readbackMapped = nullptr;
        }
        if (rigidBogie_.jobBuffer) vmaDestroyBuffer(allocator_, rigidBogie_.jobBuffer, rigidBogie_.jobAllocation);
        if (rigidBogie_.resultBuffer) vmaDestroyBuffer(allocator_, rigidBogie_.resultBuffer, rigidBogie_.resultAllocation);
        if (rigidBogie_.readbackBuffer) vmaDestroyBuffer(allocator_, rigidBogie_.readbackBuffer, rigidBogie_.readbackAllocation);
        rigidBogie_.readbackMapped = nullptr;
        if (trainPose_.definitionBuffer) vmaDestroyBuffer(allocator_, trainPose_.definitionBuffer, trainPose_.definitionAllocation);
        if (trainPose_.carBuffer) vmaDestroyBuffer(allocator_, trainPose_.carBuffer, trainPose_.carAllocation);
        if (trainPose_.connectionBuffer) vmaDestroyBuffer(allocator_, trainPose_.connectionBuffer, trainPose_.connectionAllocation);
        if (trainPose_.jobBuffer) vmaDestroyBuffer(allocator_, trainPose_.jobBuffer, trainPose_.jobAllocation);
        if (trainPose_.resultBuffer) vmaDestroyBuffer(allocator_, trainPose_.resultBuffer, trainPose_.resultAllocation);
        if (trainPose_.readbackBuffer) vmaDestroyBuffer(allocator_, trainPose_.readbackBuffer, trainPose_.readbackAllocation);
        trainPose_.readbackMapped = nullptr;
        if (validationFence_) vkDestroyFence(device_, validationFence_, nullptr);
        for (auto f : computeFences_) if (f) vkDestroyFence(device_, f, nullptr);
        if (commandPool_) vkDestroyCommandPool(device_, commandPool_, nullptr);
        if (computePipeline_) vkDestroyPipeline(device_, computePipeline_, nullptr);
        for (const VkPipeline pipeline : rigidBogiePipelines_)
            if (pipeline) vkDestroyPipeline(device_, pipeline, nullptr);
        if (trainPosePipeline_) vkDestroyPipeline(device_, trainPosePipeline_, nullptr);
        if (trainPosePipelineLayout_) vkDestroyPipelineLayout(device_, trainPosePipelineLayout_, nullptr);
        if (pipelineLayout_) vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
        if (trainPoseSetLayout_) vkDestroyDescriptorSetLayout(device_, trainPoseSetLayout_, nullptr);
        if (transientSetLayout_) vkDestroyDescriptorSetLayout(device_, transientSetLayout_, nullptr);
        if (persistentSetLayout_) vkDestroyDescriptorSetLayout(device_, persistentSetLayout_, nullptr);
        if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
        if (timestampPool_) vkDestroyQueryPool(device_, timestampPool_, nullptr);
        // Coaster record is persistent mapped (MAPPED_BIT) – destroy without explicit unmap
        if (coasterRecordBuffer_) vmaDestroyBuffer(allocator_, coasterRecordBuffer_, coasterRecordAllocation_);
        coasterRecordMapped_ = nullptr;
        if (trackBuffer_.buffer) vmaDestroyBuffer(allocator_, trackBuffer_.buffer, trackBuffer_.allocation);

        if (useHeadless_)
        {
            if (headless_.ownsAllocator && headless_.allocator != VK_NULL_HANDLE)
            {
                vmaDestroyAllocator(headless_.allocator);
                allocator_ = VK_NULL_HANDLE;
            }
            if (headless_.ownsDevice && headless_.device != VK_NULL_HANDLE)
            {
                vkDestroyDevice(headless_.device, nullptr);
                device_ = VK_NULL_HANDLE;
            }
            if (headless_.ownsInstance && headless_.instance != VK_NULL_HANDLE)
            {
                vkDestroyInstance(headless_.instance, nullptr);
            }
        }
    }

    void GpuPhysicsContext::uploadTrack(
        std::uint32_t coasterIndex,
        std::span<const coaster::TrackKinematicState> kinematics,
        double metersPerCoordinateUnit,
        coaster::TopologyKind topology,
        coaster::LayoutMode layoutMode)
    {
        // A failed replacement must never leave the previous track eligible
        // for production dispatch.
        trackUploaded_ = false;
        gpuTrackReady_ = false;
        lastSampleUsedGpu_ = false;

        // M0 invariant: single-coaster only. Multi-coaster is deferred to M1+.
        // Enforce explicitly rather than silently zeroing prior coaster lengths.
        if (coasterIndex != 0)
            throw std::invalid_argument("M0 GpuPhysicsContext supports only coasterIndex 0");
        if (!std::isfinite(metersPerCoordinateUnit) || metersPerCoordinateUnit <= 0.0)
            throw std::invalid_argument("metersPerCoordinateUnit must be positive finite");
        if (kinematics.size() < 2)
            throw std::invalid_argument("uploadTrack requires at least two samples");

        double prev = -1.0;
        for (size_t i = 0; i < kinematics.size(); ++i)
        {
            const auto& s = kinematics[i];
            if (!std::isfinite(s.distance) || !finite3(s.position) || !finite3(s.centerlineCurvature)
                || !finite3(s.frame.tangent) || !finite3(s.frame.lateral) || !finite3(s.frame.up))
                throw std::invalid_argument("kinematics sample must be finite");
            if (i == 0 && s.distance != 0.0) throw std::invalid_argument("first distance must be 0");
            if (i != 0 && s.distance <= prev) throw std::invalid_argument("distances must strictly increase");
            // Validate frame is orthonormal enough to form a rotation (finite already checks)
            prev = s.distance;
        }

        std::vector<GpuTrackSample> converted;
        converted.reserve(kinematics.size());
        for (const auto& s : kinematics)
        {
            GpuTrackSample gs{};
            gs.stationMeters = s.distance * metersPerCoordinateUnit;
            gs.positionMeters[0] = s.position.x * metersPerCoordinateUnit;
            gs.positionMeters[1] = s.position.y * metersPerCoordinateUnit;
            gs.positionMeters[2] = s.position.z * metersPerCoordinateUnit;
            gs.tangent[0] = s.frame.tangent.x;
            gs.tangent[1] = s.frame.tangent.y;
            gs.tangent[2] = s.frame.tangent.z;
            gs.lateral[0] = s.frame.lateral.x;
            gs.lateral[1] = s.frame.lateral.y;
            gs.lateral[2] = s.frame.lateral.z;
            gs.up[0] = s.frame.up.x;
            gs.up[1] = s.frame.up.y;
            gs.up[2] = s.frame.up.z;
            gs.curvaturePerMeter[0] = s.centerlineCurvature.x / metersPerCoordinateUnit;
            gs.curvaturePerMeter[1] = s.centerlineCurvature.y / metersPerCoordinateUnit;
            gs.curvaturePerMeter[2] = s.centerlineCurvature.z / metersPerCoordinateUnit;
            if (!std::isfinite(gs.stationMeters) || !std::isfinite(gs.positionMeters[0]))
                throw std::invalid_argument("SI conversion produced non-finite sample");
            converted.push_back(gs);
        }

        const double lengthMeters = converted.back().stationMeters;
        const std::uint32_t topologyValue = (topology == coaster::TopologyKind::ClosedCircuit) ? 1u : 0u;
        (void)layoutMode;

        // Update shared CPU mirror - M0 single-coaster path (coasterIndex == 0)
        {
            std::lock_guard<std::mutex> lock(sharedMapMutex());
            SharedGpuState& shared = sharedMap()[this];
            shared.perCoaster.assign(1, converted);
            shared.samples = converted;
            SharedCoasterRecord rec{};
            rec.trackOffset = 0;
            rec.trackCount = static_cast<std::uint32_t>(converted.size());
            rec.length = lengthMeters;
            rec.topology = topologyValue;
            shared.records.assign(1, rec);

            // Publish to header-visible host vectors (convert SharedCoasterRecord to inner CoasterRecord)
            coasterRecordsHost_.resize(1);
            coasterRecords_.resize(1);
            coasterRecordsHost_[0].trackOffset = 0;
            coasterRecordsHost_[0].trackCount = rec.trackCount;
            coasterRecordsHost_[0].topology = rec.topology;
            coasterRecordsHost_[0].length = rec.length;
            coasterRecords_[0] = coasterRecordsHost_[0];
            trackBuffer_.sampleCount = static_cast<std::uint32_t>(shared.samples.size());
        }

        trackUploaded_ = true;

        if (device_ != VK_NULL_HANDLE && allocator_ != VK_NULL_HANDLE && shaderFloat64Enabled_)
        {
            std::lock_guard<std::mutex> lock(sharedMapMutex());
            const SharedGpuState& shared = sharedMap().at(this);
            const bool trackRealloc = trackBuffer_.capacity < static_cast<VkDeviceSize>(shared.samples.size() * sizeof(GpuTrackSample));
            const bool coasterRealloc = coasterRecordCapacity_ < static_cast<VkDeviceSize>(shared.records.size() * sizeof(CoasterRecord));
            ensureTrackBufferCapacity(static_cast<std::uint32_t>(shared.samples.size()));
            ensureCoasterRecordCapacity(static_cast<std::uint32_t>(shared.records.size()));
            if (trackRealloc || coasterRealloc)
                updatePersistentDescriptors();

            if (trackBuffer_.buffer != VK_NULL_HANDLE && !shared.samples.empty())
            {
                void* mapped = nullptr;
                // Track buffer created with MAPPED_BIT; vmaMapMemory is still valid
                // and returns the persistent mapping. Unmap after memcpy.
                vmaMapMemory(allocator_, trackBuffer_.allocation, &mapped);
                std::memcpy(mapped, shared.samples.data(), shared.samples.size() * sizeof(GpuTrackSample));
                vmaFlushAllocation(allocator_, trackBuffer_.allocation, 0, VK_WHOLE_SIZE);
                vmaUnmapMemory(allocator_, trackBuffer_.allocation);
            }
            if (coasterRecordMapped_ && !shared.records.empty())
            {
                // GPU buffer layout matches SharedCoasterRecord (same 24 bytes, offset 16 for trackLength/length)
                std::memcpy(coasterRecordMapped_, shared.records.data(), shared.records.size() * sizeof(SharedCoasterRecord));
                vmaFlushAllocation(allocator_, coasterRecordAllocation_, 0, VK_WHOLE_SIZE);
            }
            // Ensure descriptors point at (potentially) new buffers even if not reallocated (first upload)
            updatePersistentDescriptors();
            gpuTrackReady_ = trackBuffer_.buffer != VK_NULL_HANDLE
                && coasterRecordBuffer_ != VK_NULL_HANDLE
                && persistentDescriptorSet_ != VK_NULL_HANDLE;
        }

        trackOffset_ = trackBuffer_.sampleCount;
    }

    void GpuPhysicsContext::sampleTrack(std::uint32_t slot, std::span<const GpuTrackQuery> queries)
    {
        lastSampleUsedGpu_ = false;
        if (queries.empty()) return;
        if (slot >= frames_.size()) throw std::out_of_range("slot out of range");
        currentSlot_ = slot;
        if (!gpuAvailable() || !gpuTrackReady_)
            return;
        // M1: real per-slot synchronous dispatch (fence-based). Mirrors sampleTrackGpu
        // but uses the requested slot's transient resources.
        updatePersistentDescriptors();
        ensureFrameBuffers(slot, queries.size());
        uploadQueries(slot, queries);
        updateFrameDescriptors(slot);

        auto& frame = frames_[slot];
        if (frame.queryBuffer == VK_NULL_HANDLE || frame.resultBuffer == VK_NULL_HANDLE) return;
        VkFence fence = computeFences_[slot];
        if (fence == VK_NULL_HANDLE) return;
        vkResetFences(device_, 1, &fence);
        VkCommandBuffer cmd = commandBuffers_[slot];
        vkResetCommandBuffer(cmd, 0);
        VkCommandBufferBeginInfo begin{};
        begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(cmd, &begin) != VK_SUCCESS) return;

        VkMemoryBarrier h2s{};
        h2s.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        h2s.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        h2s.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &h2s, 0, nullptr, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_);
        VkDescriptorSet sets[] = {persistentDescriptorSet_, frame.descriptorSet};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 2, sets, 0, nullptr);
        const std::uint32_t localSize = 64;
        const std::uint32_t groups = static_cast<std::uint32_t>((queries.size() + localSize - 1) / localSize);
        vkCmdDispatch(cmd, groups, 1, 1);

        VkMemoryBarrier s2t{};
        s2t.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        s2t.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        s2t.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &s2t, 0, nullptr, 0, nullptr);

        VkBufferCopy copy{};
        copy.size = queries.size() * sizeof(PhysicsTrackSample);
        vkCmdCopyBuffer(cmd, frame.resultBuffer, frame.readbackBuffer, 1, &copy);

        VkMemoryBarrier t2h{};
        t2h.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        t2h.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        t2h.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &t2h, 0, nullptr, 0, nullptr);

        vkEndCommandBuffer(cmd);
        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        vkQueueSubmit(computeQueue_, 1, &submit, fence);
        // M1 synchronous: wait so subsequent CPU readback is coherent (validation path will also wait)
        vkWaitForFences(device_, 1, &fence, VK_TRUE, 5'000'000'000ULL);
        lastSampleUsedGpu_ = true;
    }

    std::vector<PhysicsTrackSample> GpuPhysicsContext::sampleTrackForValidation(std::span<const GpuTrackQuery> queries)
    {
        if (!trackUploaded_)
            return {};

        std::vector<PhysicsTrackSample> out;
        out.reserve(queries.size());

        SharedGpuState shCopy;
        {
            std::lock_guard<std::mutex> lock(sharedMapMutex());
            auto it = sharedMap().find(this);
            if (it == sharedMap().end())
            {
                for (auto& q : queries)
                {
                    PhysicsTrackSample s{};
                    s.location = q;
                    out.push_back(s);
                }
                return out;
            }
            shCopy = it->second;
        }
        const SharedGpuState& sh = shCopy;

        for (const auto& q : queries)
        {
            if (q.coasterIndex >= sh.records.size())
            {
                PhysicsTrackSample s{};
                s.location = q;
                out.push_back(s);
                continue;
            }
            const SharedCoasterRecord& rec = sh.records[q.coasterIndex];
            out.push_back(cpuSampleOne(q, sh.samples, rec.trackOffset, rec.trackCount, rec.length, rec.topology));
        }
        return out;
    }

    void GpuPhysicsContext::createCommandPool()
    {
        VkCommandPoolCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        info.queueFamilyIndex = computeQueueFamily_;
        if (vkCreateCommandPool(device_, &info, nullptr, &commandPool_) != VK_SUCCESS)
            throw std::runtime_error("vkCreateCommandPool failed");
        VkCommandBufferAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc.commandPool = commandPool_;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = static_cast<std::uint32_t>(commandBuffers_.size());
        if (vkAllocateCommandBuffers(device_, &alloc, commandBuffers_.data()) != VK_SUCCESS)
            throw std::runtime_error("vkAllocateCommandBuffers failed");
    }

    void GpuPhysicsContext::createDescriptorResources()
    {
        VkDescriptorPoolSize poolSizes[] = {
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 13},
        };
        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.maxSets = 5;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = poolSizes;
        if (vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS)
            throw std::runtime_error("vkCreateDescriptorPool failed");

        VkDescriptorSetLayoutBinding pBindings[] = {
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        };
        VkDescriptorSetLayoutCreateInfo plInfo{};
        plInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        plInfo.bindingCount = 2;
        plInfo.pBindings = pBindings;
        if (vkCreateDescriptorSetLayout(device_, &plInfo, nullptr, &persistentSetLayout_) != VK_SUCCESS)
            throw std::runtime_error("vkCreateDescriptorSetLayout persistent failed");

        VkDescriptorSetLayoutBinding tBindings[] = {
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        };
        VkDescriptorSetLayoutCreateInfo tlInfo{};
        tlInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        tlInfo.bindingCount = 2;
        tlInfo.pBindings = tBindings;
        if (vkCreateDescriptorSetLayout(device_, &tlInfo, nullptr, &transientSetLayout_) != VK_SUCCESS)
            throw std::runtime_error("vkCreateDescriptorSetLayout transient failed");

        VkDescriptorSetLayoutBinding trainBindings[] = {
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        };
        VkDescriptorSetLayoutCreateInfo trainLayoutInfo{};
        trainLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        trainLayoutInfo.bindingCount = static_cast<std::uint32_t>(std::size(trainBindings));
        trainLayoutInfo.pBindings = trainBindings;
        if (vkCreateDescriptorSetLayout(device_, &trainLayoutInfo, nullptr,
                &trainPoseSetLayout_) != VK_SUCCESS)
            throw std::runtime_error("vkCreateDescriptorSetLayout train pose failed");

        allocateDescriptorSets();
    }

    void GpuPhysicsContext::createPipeline()
    {
        VkDescriptorSetLayout layouts[] = {persistentSetLayout_, transientSetLayout_};
        VkPipelineLayoutCreateInfo plInfo{};
        plInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        plInfo.setLayoutCount = 2;
        plInfo.pSetLayouts = layouts;
        if (vkCreatePipelineLayout(device_, &plInfo, nullptr, &pipelineLayout_) != VK_SUCCESS)
            throw std::runtime_error("vkCreatePipelineLayout failed");

        const VkDescriptorSetLayout trainLayouts[] = {
            persistentSetLayout_, trainPoseSetLayout_};
        VkPipelineLayoutCreateInfo trainPipelineLayoutInfo{};
        trainPipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        trainPipelineLayoutInfo.setLayoutCount = 2;
        trainPipelineLayoutInfo.pSetLayouts = trainLayouts;
        if (vkCreatePipelineLayout(device_, &trainPipelineLayoutInfo, nullptr,
                &trainPosePipelineLayout_) != VK_SUCCESS)
            throw std::runtime_error("vkCreatePipelineLayout train pose failed");

        const auto createComputePipeline = [&](const char* fileName,
                                               VkPipeline& pipeline,
                                               const std::uint32_t* localSize = nullptr,
                                               VkPipelineLayout layout = VK_NULL_HANDLE) -> bool
        {
            std::vector<std::filesystem::path> candidates;
            try
            {
                const auto root = std::filesystem::absolute(
                    std::filesystem::path(__FILE__).parent_path().parent_path()
                        .parent_path().parent_path().parent_path());
                candidates.push_back(root / "build" / "shaders" / fileName);
            }
            catch (...) {}
            try { candidates.push_back(std::filesystem::absolute(std::filesystem::path("build/shaders") / fileName)); } catch (...) {}
            try { candidates.push_back(std::filesystem::absolute(std::filesystem::path("shaders") / fileName)); } catch (...) {}
            if (SDL_WasInit(SDL_INIT_VIDEO))
            {
                try
                {
                    const auto samplePath = gpuShaderPath();
                    candidates.push_back(samplePath.parent_path() / fileName);
                }
                catch (...) {}
            }

            std::vector<std::uint32_t> code;
            for (const auto& path : candidates)
            {
                if (!std::filesystem::exists(path)) continue;
                code = readSpirvFile(path);
                break;
            }
            if (code.empty())
            {
                quantum::logging::logMessagef(quantum::logging::LogLevel::Info,
                    "VK", "GpuPhysicsContext: %s not found", fileName);
                return false;
            }

            VkShaderModule module = createShaderModuleLocal(device_, code);
            VkPipelineShaderStageCreateInfo stage{};
            stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
            stage.module = module;
            stage.pName = "main";
            VkSpecializationMapEntry localSizeEntry{};
            VkSpecializationInfo specialization{};
            if (localSize)
            {
                localSizeEntry.constantID = 0;
                localSizeEntry.offset = 0;
                localSizeEntry.size = sizeof(*localSize);
                specialization.mapEntryCount = 1;
                specialization.pMapEntries = &localSizeEntry;
                specialization.dataSize = sizeof(*localSize);
                specialization.pData = localSize;
                stage.pSpecializationInfo = &specialization;
            }
            VkComputePipelineCreateInfo info{};
            info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
            info.stage = stage;
            info.layout = layout != VK_NULL_HANDLE ? layout : pipelineLayout_;
            const VkResult result = vkCreateComputePipelines(
                device_, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline);
            vkDestroyShaderModule(device_, module, nullptr);
            if (result != VK_SUCCESS)
            {
                if (localSize)
                    return false;
                throw std::runtime_error(std::string("vkCreateComputePipelines failed for ") + fileName);
            }
            return true;
        };

        gpuPipelineReady_ = createComputePipeline(
            "track_sample.comp.spv", computePipeline_);
        for (std::size_t index = 0; index < rigidBogieLocalSizes_.size(); ++index)
            static_cast<void>(createComputePipeline(
                "rigid_bogie.comp.spv", rigidBogiePipelines_[index],
                &rigidBogieLocalSizes_[index]));
        rigidBogiePipelineReady_ = rigidBogiePipelines_[1] != VK_NULL_HANDLE;
        if (gpuPipelineReady_)
            quantum::logging::logMessagef(quantum::logging::LogLevel::Info,
                "VK", "GpuPhysicsContext: track sampling pipeline ready");
        if (rigidBogiePipelineReady_)
            quantum::logging::logMessagef(quantum::logging::LogLevel::Info,
                "VK", "GpuPhysicsContext: rigid-bogie prototype pipeline ready");
    }

    void GpuPhysicsContext::createTrainPosePipeline()
    {
        if (trainPosePipelineReady_ || trainPosePipeline_ != VK_NULL_HANDLE)
            return;
        std::vector<std::filesystem::path> candidates;
        try
        {
            const auto root = std::filesystem::absolute(
                std::filesystem::path(__FILE__).parent_path().parent_path()
                    .parent_path().parent_path().parent_path());
            candidates.push_back(root / "build" / "shaders"
                / "train_pose.comp.spv");
        }
        catch (...) {}
        try
        {
            candidates.push_back(std::filesystem::absolute(
                "build/shaders/train_pose.comp.spv"));
            candidates.push_back(std::filesystem::absolute(
                "shaders/train_pose.comp.spv"));
        }
        catch (...) {}
        std::vector<std::uint32_t> code;
        for (const auto& path : candidates)
        {
            if (!std::filesystem::exists(path)) continue;
            code = readSpirvFile(path);
            break;
        }
        if (code.empty())
            throw std::runtime_error("train_pose.comp.spv not found");
        VkShaderModule module = createShaderModuleLocal(device_, code);
        VkPipelineShaderStageCreateInfo stage{};
        stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stage.module = module;
        stage.pName = "main";
        VkComputePipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
        info.stage = stage;
        info.layout = trainPosePipelineLayout_;
#if defined(QUANTUM_ENABLE_VULKAN_VALIDATION)
        // The full parity shader triggers pathological optimization time on the
        // current NVIDIA driver. Debug validation needs executable code, not a
        // production timing binary; Release deliberately retains optimization.
        info.flags = VK_PIPELINE_CREATE_DISABLE_OPTIMIZATION_BIT;
#endif
        const VkResult result = vkCreateComputePipelines(device_,
            VK_NULL_HANDLE, 1, &info, nullptr, &trainPosePipeline_);
        vkDestroyShaderModule(device_, module, nullptr);
        if (result != VK_SUCCESS)
            throw std::runtime_error(
                "vkCreateComputePipelines failed for train_pose.comp.spv");
        trainPosePipelineReady_ = true;
        quantum::logging::logMessagef(quantum::logging::LogLevel::Info,
            "VK", "GpuPhysicsContext: train-pose residency prototype pipeline ready");
    }

    void GpuPhysicsContext::allocateDescriptorSets()
    {
        // Persistent set (track + coaster)
        VkDescriptorSetAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        alloc.descriptorPool = descriptorPool_;
        alloc.descriptorSetCount = 1;
        alloc.pSetLayouts = &persistentSetLayout_;
        if (vkAllocateDescriptorSets(device_, &alloc, &persistentDescriptorSet_) != VK_SUCCESS)
            throw std::runtime_error("vkAllocateDescriptorSets persistent failed");

        // Transient per-frame sets (query + result)
        for (auto& frame : frames_)
        {
            VkDescriptorSetAllocateInfo a{};
            a.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            a.descriptorPool = descriptorPool_;
            a.descriptorSetCount = 1;
            a.pSetLayouts = &transientSetLayout_;
            if (vkAllocateDescriptorSets(device_, &a, &frame.descriptorSet) != VK_SUCCESS)
                throw std::runtime_error("vkAllocateDescriptorSets transient failed");
        }
        VkDescriptorSetAllocateInfo rigidAlloc{};
        rigidAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        rigidAlloc.descriptorPool = descriptorPool_;
        rigidAlloc.descriptorSetCount = 1;
        rigidAlloc.pSetLayouts = &transientSetLayout_;
        if (vkAllocateDescriptorSets(device_, &rigidAlloc, &rigidBogie_.descriptorSet) != VK_SUCCESS)
            throw std::runtime_error("vkAllocateDescriptorSets rigid bogie failed");
        VkDescriptorSetAllocateInfo trainAlloc{};
        trainAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        trainAlloc.descriptorPool = descriptorPool_;
        trainAlloc.descriptorSetCount = 1;
        trainAlloc.pSetLayouts = &trainPoseSetLayout_;
        if (vkAllocateDescriptorSets(device_, &trainAlloc,
                &trainPose_.descriptorSet) != VK_SUCCESS)
            throw std::runtime_error("vkAllocateDescriptorSets train pose failed");
    }

    void GpuPhysicsContext::updatePersistentDescriptors()
    {
        if (persistentDescriptorSet_ == VK_NULL_HANDLE) return;
        if (trackBuffer_.buffer == VK_NULL_HANDLE || coasterRecordBuffer_ == VK_NULL_HANDLE) return;
        VkDescriptorBufferInfo trackInfo{};
        trackInfo.buffer = trackBuffer_.buffer;
        trackInfo.offset = 0;
        trackInfo.range = static_cast<VkDeviceSize>(trackBuffer_.sampleCount * sizeof(GpuTrackSample));
        if (trackInfo.range == 0) trackInfo.range = VK_WHOLE_SIZE;
        VkDescriptorBufferInfo coasterInfo{};
        coasterInfo.buffer = coasterRecordBuffer_;
        coasterInfo.offset = 0;
        coasterInfo.range = static_cast<VkDeviceSize>(coasterRecords_.size() * sizeof(CoasterRecord));
        if (coasterInfo.range == 0) coasterInfo.range = VK_WHOLE_SIZE;
        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = persistentDescriptorSet_;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &trackInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = persistentDescriptorSet_;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &coasterInfo;
        vkUpdateDescriptorSets(device_, 2, writes, 0, nullptr);
    }

    void GpuPhysicsContext::updateFrameDescriptors(std::uint32_t slot)
    {
        if (slot >= frames_.size()) return;
        auto& f = frames_[slot];
        if (f.descriptorSet == VK_NULL_HANDLE) return;
        if (f.queryBuffer == VK_NULL_HANDLE || f.resultBuffer == VK_NULL_HANDLE) return;
        if (f.queryCount == 0) return;
        VkDescriptorBufferInfo qInfo{};
        qInfo.buffer = f.queryBuffer;
        qInfo.offset = 0;
        qInfo.range = static_cast<VkDeviceSize>(f.queryCount * sizeof(GpuTrackQuery));
        VkDescriptorBufferInfo rInfo{};
        rInfo.buffer = f.resultBuffer;
        rInfo.offset = 0;
        rInfo.range = static_cast<VkDeviceSize>(f.queryCount * sizeof(PhysicsTrackSample));
        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = f.descriptorSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &qInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = f.descriptorSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &rInfo;
        vkUpdateDescriptorSets(device_, 2, writes, 0, nullptr);
    }

    void GpuPhysicsContext::createSyncResources()
    {
        VkFenceCreateInfo fi{};
        fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (auto& f : computeFences_)
            vkCreateFence(device_, &fi, nullptr, &f);
        vkCreateFence(device_, &fi, nullptr, &validationFence_);
    }

    void GpuPhysicsContext::createTimestampPool()
    {
        timestampSupported_ = false;
        const VkPhysicalDevice physicalDevice = useHeadless_
            ? headless_.physicalDevice : (vulkan_ ? vulkan_->physicalDevice() : VK_NULL_HANDLE);
        if (physicalDevice == VK_NULL_HANDLE) return;

        VkPhysicalDeviceSubgroupProperties subgroup{};
        subgroup.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_PROPERTIES;
        VkPhysicalDeviceProperties2 properties{};
        properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
        properties.pNext = &subgroup;
        vkGetPhysicalDeviceProperties2(physicalDevice, &properties);
        computeDeviceInfo_.subgroupSize = subgroup.subgroupSize;
        computeDeviceInfo_.maxWorkgroupSizeX = properties.properties.limits.maxComputeWorkGroupSize[0];
        computeDeviceInfo_.maxWorkgroupInvocations = properties.properties.limits.maxComputeWorkGroupInvocations;
        timestampPeriod_ = properties.properties.limits.timestampPeriod;
        computeDeviceInfo_.timestampPeriodNanoseconds = timestampPeriod_;

        std::uint32_t familyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &familyCount, nullptr);
        std::vector<VkQueueFamilyProperties> families(familyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &familyCount, families.data());
        if (computeQueueFamily_ >= families.size()) return;
        computeDeviceInfo_.timestampValidBits = families[computeQueueFamily_].timestampValidBits;
        if (computeDeviceInfo_.timestampValidBits == 0) return;

        VkQueryPoolCreateInfo queryInfo{};
        queryInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
        queryInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
        queryInfo.queryCount = 2;
        if (vkCreateQueryPool(device_, &queryInfo, nullptr, &timestampPool_) == VK_SUCCESS)
        {
            timestampSupported_ = true;
            computeDeviceInfo_.timestampsSupported = true;
        }
    }

    void GpuPhysicsContext::ensureTrackBufferCapacity(std::uint32_t requiredSamples)
    {
        const VkDeviceSize requiredBytes = static_cast<VkDeviceSize>(requiredSamples) * sizeof(GpuTrackSample);
        if (requiredBytes == 0) return;
        if (requiredBytes <= trackBuffer_.capacity) return;
        if (trackBuffer_.buffer) vmaDestroyBuffer(allocator_, trackBuffer_.buffer, trackBuffer_.allocation);
        VkBufferCreateInfo info = makeStorageBufferInfo(requiredBytes);
        VmaAllocationCreateInfo alloc{};
        alloc.usage = VMA_MEMORY_USAGE_AUTO;
        alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        VmaAllocationInfo aInfo{};
        vmaCreateBuffer(allocator_, &info, &alloc, &trackBuffer_.buffer, &trackBuffer_.allocation, &aInfo);
        trackBuffer_.capacity = info.size;
    }

    void GpuPhysicsContext::ensureCoasterRecordCapacity(std::uint32_t requiredCount)
    {
        const VkDeviceSize requiredBytes = static_cast<VkDeviceSize>(requiredCount) * sizeof(CoasterRecord);
        if (requiredBytes == 0) return;
        if (requiredBytes <= coasterRecordCapacity_) return;
        if (coasterRecordBuffer_) { vmaDestroyBuffer(allocator_, coasterRecordBuffer_, coasterRecordAllocation_); coasterRecordMapped_ = nullptr; }
        VkBufferCreateInfo info = makeStorageBufferInfo(requiredBytes);
        VmaAllocationCreateInfo alloc{};
        alloc.usage = VMA_MEMORY_USAGE_AUTO;
        alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo aInfo{};
        vmaCreateBuffer(allocator_, &info, &alloc, &coasterRecordBuffer_, &coasterRecordAllocation_, &aInfo);
        coasterRecordMapped_ = aInfo.pMappedData;
        coasterRecordCapacity_ = info.size;
    }

    void GpuPhysicsContext::ensureFrameBuffers(std::uint32_t slot, std::size_t queryCount)
    {
        if (slot >= frames_.size()) throw std::out_of_range("slot out of range");
        if (queryCount == 0) return;
        auto& frame = frames_[slot];
        frame.queryCount = static_cast<std::uint32_t>(queryCount);
        const VkDeviceSize queryBytes = static_cast<VkDeviceSize>(queryCount * sizeof(GpuTrackQuery));
        const VkDeviceSize resultBytes = static_cast<VkDeviceSize>(queryCount * sizeof(PhysicsTrackSample));

        // Query buffer (host-visible write) – exact size so shader .length() == queryCount
        VkDeviceSize qCap = 0;
        if (frame.queryBuffer != VK_NULL_HANDLE)
        {
            VmaAllocationInfo info{};
            vmaGetAllocationInfo(allocator_, frame.queryAllocation, &info);
            qCap = info.size;
        }
        if (queryBytes > qCap)
        {
            if (frame.queryBuffer) vmaDestroyBuffer(allocator_, frame.queryBuffer, frame.queryAllocation);
            VkBufferCreateInfo ci = makeStorageBufferInfo(queryBytes);
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_AUTO;
            aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
            VmaAllocationInfo aInfo{};
            vmaCreateBuffer(allocator_, &ci, &aci, &frame.queryBuffer, &frame.queryAllocation, &aInfo);
        }

        // Result buffer (storage) – exact size
        VkDeviceSize rCap = 0;
        if (frame.resultBuffer != VK_NULL_HANDLE)
        {
            VmaAllocationInfo info{};
            vmaGetAllocationInfo(allocator_, frame.resultAllocation, &info);
            rCap = info.size;
        }
        if (resultBytes > rCap)
        {
            if (frame.resultBuffer) vmaDestroyBuffer(allocator_, frame.resultBuffer, frame.resultAllocation);
            VkBufferCreateInfo ci = makeStorageBufferInfo(resultBytes);
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_AUTO;
            aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
            VmaAllocationInfo aInfo{};
            vmaCreateBuffer(allocator_, &ci, &aci, &frame.resultBuffer, &frame.resultAllocation, &aInfo);
        }

        // Readback buffer (host-visible read)
        VkDeviceSize rbCap = 0;
        if (frame.readbackBuffer != VK_NULL_HANDLE)
        {
            VmaAllocationInfo info{};
            vmaGetAllocationInfo(allocator_, frame.readbackAllocation, &info);
            rbCap = info.size;
        }
        if (resultBytes > rbCap)
        {
            // Readback is persistent mapped (MAPPED_BIT) – destroy without explicit unmap, exact size
            if (frame.readbackBuffer) vmaDestroyBuffer(allocator_, frame.readbackBuffer, frame.readbackAllocation);
            frame.readbackMapped = nullptr;
            VkBufferCreateInfo ci = makeStorageBufferInfo(resultBytes);
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_AUTO;
            aci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo aInfo{};
            vmaCreateBuffer(allocator_, &ci, &aci, &frame.readbackBuffer, &frame.readbackAllocation, &aInfo);
            frame.readbackMapped = aInfo.pMappedData;
        }

        updateFrameDescriptors(slot);
    }

    void GpuPhysicsContext::ensureRigidBogieBuffers(const std::size_t jobCount)
    {
        if (jobCount == 0 || jobCount <= rigidBogie_.capacityJobs) return;
        const VkDeviceSize jobBytes = jobCount * sizeof(GpuRigidBogieJob);
        const VkDeviceSize resultBytes = jobCount * sizeof(GpuRigidBogieResult);

        if (rigidBogie_.jobBuffer)
            vmaDestroyBuffer(allocator_, rigidBogie_.jobBuffer, rigidBogie_.jobAllocation);
        if (rigidBogie_.resultBuffer)
            vmaDestroyBuffer(allocator_, rigidBogie_.resultBuffer, rigidBogie_.resultAllocation);
        if (rigidBogie_.readbackBuffer)
            vmaDestroyBuffer(allocator_, rigidBogie_.readbackBuffer, rigidBogie_.readbackAllocation);
        rigidBogie_.readbackMapped = nullptr;

        VkBufferCreateInfo jobInfo = makeStorageBufferInfo(jobBytes);
        VmaAllocationCreateInfo jobAllocationInfo{};
        jobAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
        jobAllocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        if (vmaCreateBuffer(allocator_, &jobInfo, &jobAllocationInfo,
                &rigidBogie_.jobBuffer, &rigidBogie_.jobAllocation, nullptr) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate rigid-bogie job buffer");

        VkBufferCreateInfo resultInfo = makeStorageBufferInfo(resultBytes);
        VmaAllocationCreateInfo resultAllocationInfo{};
        resultAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
        if (vmaCreateBuffer(allocator_, &resultInfo, &resultAllocationInfo,
                &rigidBogie_.resultBuffer, &rigidBogie_.resultAllocation, nullptr) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate rigid-bogie result buffer");

        VmaAllocationCreateInfo readbackAllocationInfo{};
        readbackAllocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
        readbackAllocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo mappedInfo{};
        if (vmaCreateBuffer(allocator_, &resultInfo, &readbackAllocationInfo,
                &rigidBogie_.readbackBuffer, &rigidBogie_.readbackAllocation,
                &mappedInfo) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate rigid-bogie readback buffer");
        rigidBogie_.readbackMapped = mappedInfo.pMappedData;
        rigidBogie_.capacityJobs = jobCount;
    }

    void GpuPhysicsContext::updateRigidBogieDescriptors(const std::size_t jobCount)
    {
        if (rigidBogie_.descriptorSet == VK_NULL_HANDLE || jobCount == 0) return;
        VkDescriptorBufferInfo jobInfo{};
        jobInfo.buffer = rigidBogie_.jobBuffer;
        jobInfo.range = jobCount * sizeof(GpuRigidBogieJob);
        VkDescriptorBufferInfo resultInfo{};
        resultInfo.buffer = rigidBogie_.resultBuffer;
        resultInfo.range = jobCount * sizeof(GpuRigidBogieResult);
        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = rigidBogie_.descriptorSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo = &jobInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = rigidBogie_.descriptorSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &resultInfo;
        vkUpdateDescriptorSets(device_, 2, writes, 0, nullptr);
    }

    void GpuPhysicsContext::ensureTrainPoseBuffers(const std::size_t jobCount)
    {
        if (jobCount == 0 || jobCount <= trainPose_.capacityJobs) return;
        const VkDeviceSize jobBytes = jobCount * sizeof(GpuTrainPoseJob);
        const VkDeviceSize resultBytes = jobCount * sizeof(GpuTrainPoseResult);
        if (trainPose_.jobBuffer)
            vmaDestroyBuffer(allocator_, trainPose_.jobBuffer, trainPose_.jobAllocation);
        if (trainPose_.resultBuffer)
            vmaDestroyBuffer(allocator_, trainPose_.resultBuffer, trainPose_.resultAllocation);
        if (trainPose_.readbackBuffer)
            vmaDestroyBuffer(allocator_, trainPose_.readbackBuffer, trainPose_.readbackAllocation);
        trainPose_.readbackMapped = nullptr;

        VkBufferCreateInfo jobInfo = makeStorageBufferInfo(jobBytes);
        VmaAllocationCreateInfo uploadInfo{};
        uploadInfo.usage = VMA_MEMORY_USAGE_AUTO;
        uploadInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        if (vmaCreateBuffer(allocator_, &jobInfo, &uploadInfo,
                &trainPose_.jobBuffer, &trainPose_.jobAllocation, nullptr) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate train-pose job buffer");

        VkBufferCreateInfo resultInfo = makeStorageBufferInfo(resultBytes);
        VmaAllocationCreateInfo deviceInfo{};
        deviceInfo.usage = VMA_MEMORY_USAGE_AUTO;
        if (vmaCreateBuffer(allocator_, &resultInfo, &deviceInfo,
                &trainPose_.resultBuffer, &trainPose_.resultAllocation, nullptr) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate train-pose result buffer");

        VmaAllocationCreateInfo readbackInfo{};
        readbackInfo.usage = VMA_MEMORY_USAGE_AUTO;
        readbackInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
            | VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo mapped{};
        if (vmaCreateBuffer(allocator_, &resultInfo, &readbackInfo,
                &trainPose_.readbackBuffer, &trainPose_.readbackAllocation,
                &mapped) != VK_SUCCESS)
            throw std::runtime_error("Unable to allocate train-pose readback buffer");
        trainPose_.readbackMapped = mapped.pMappedData;
        trainPose_.capacityJobs = jobCount;
    }

    void GpuPhysicsContext::updateTrainPoseDescriptors(const std::size_t jobCount)
    {
        if (trainPose_.descriptorSet == VK_NULL_HANDLE || jobCount == 0
            || trainPose_.definitionBuffer == VK_NULL_HANDLE
            || trainPose_.carBuffer == VK_NULL_HANDLE
            || trainPose_.connectionBuffer == VK_NULL_HANDLE)
            return;
        VkDescriptorBufferInfo infos[5]{};
        infos[0].buffer = trainPose_.definitionBuffer;
        infos[0].range = trainPose_.definitions.size()
            * sizeof(GpuResidentTrainDefinition);
        infos[1].buffer = trainPose_.carBuffer;
        infos[1].range = trainPose_.cars.size() * sizeof(GpuResidentCarDefinition);
        infos[2].buffer = trainPose_.connectionBuffer;
        infos[2].range = std::max<std::size_t>(1, trainPose_.connections.size())
            * sizeof(GpuResidentConnectionDefinition);
        infos[3].buffer = trainPose_.jobBuffer;
        infos[3].range = jobCount * sizeof(GpuTrainPoseJob);
        infos[4].buffer = trainPose_.resultBuffer;
        infos[4].range = jobCount * sizeof(GpuTrainPoseResult);
        VkWriteDescriptorSet writes[5]{};
        for (std::uint32_t index = 0; index < 5; ++index)
        {
            writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index].dstSet = trainPose_.descriptorSet;
            writes[index].dstBinding = index;
            writes[index].descriptorCount = 1;
            writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[index].pBufferInfo = &infos[index];
        }
        vkUpdateDescriptorSets(device_, 5, writes, 0, nullptr);
    }

    void GpuPhysicsContext::uploadTrackData(std::span<const coaster::TrackKinematicState>, double) {}
    void GpuPhysicsContext::uploadQueries(std::uint32_t slot, std::span<const GpuTrackQuery> queries)
    {
        if (slot >= frames_.size()) throw std::out_of_range("slot out of range");
        if (queries.empty()) return;
        ensureFrameBuffers(slot, queries.size());
        auto& frame = frames_[slot];
        void* mapped = nullptr;
        vmaMapMemory(allocator_, frame.queryAllocation, &mapped);
        std::memcpy(mapped, queries.data(), queries.size() * sizeof(GpuTrackQuery));
        // Host-coherent memory: flush if non-coherent
        vmaFlushAllocation(allocator_, frame.queryAllocation, 0, VK_WHOLE_SIZE);
        vmaUnmapMemory(allocator_, frame.queryAllocation);
    }
    VkCommandBuffer GpuPhysicsContext::beginSingleTimeCommands() { return VK_NULL_HANDLE; }
    void GpuPhysicsContext::endSingleTimeCommands(VkCommandBuffer) {}

    bool GpuPhysicsContext::gpuAvailable() const noexcept
    {
        return device_ != VK_NULL_HANDLE && shaderFloat64Enabled_ && gpuPipelineReady_ && computePipeline_ != VK_NULL_HANDLE;
    }

    bool GpuPhysicsContext::hasUploadedTrack() const noexcept
    {
        return trackUploaded_;
    }

    bool GpuPhysicsContext::gpuTrackReady() const noexcept
    {
        return gpuTrackReady_;
    }

    bool GpuPhysicsContext::lastSampleUsedGpu() const noexcept
    {
        return lastSampleUsedGpu_;
    }

    bool GpuPhysicsContext::gpuRigidBogieReady() const noexcept
    {
        return device_ != VK_NULL_HANDLE && shaderFloat64Enabled_
            && rigidBogiePipelineReady_ && rigidBogiePipelines_[1] != VK_NULL_HANDLE
            && gpuTrackReady_;
    }

    GpuComputeDeviceInfo GpuPhysicsContext::computeDeviceInfo() const noexcept
    {
        return computeDeviceInfo_;
    }

    void GpuPhysicsContext::uploadTrainDefinition(
        const std::uint32_t trainDefinitionIndex,
        const TrainDefinition& definition)
    {
        validateTrainDefinition(definition);
        if (definition.cars.size() > gpuTrainPoseMaximumCarCount)
            throw std::invalid_argument(
                "GPU train-pose prototype supports at most eight cars");
        if (device_ == VK_NULL_HANDLE)
            return;
        createTrainPosePipeline();

        GpuResidentTrainDefinition resident;
        resident.firstCar = static_cast<std::uint32_t>(trainPose_.cars.size());
        resident.carCount = static_cast<std::uint32_t>(definition.cars.size());
        resident.firstConnection = static_cast<std::uint32_t>(
            trainPose_.connections.size());
        resident.connectionCount = static_cast<std::uint32_t>(
            definition.connections.size());
        if (trainDefinitionIndex >= trainPose_.definitions.size())
            trainPose_.definitions.resize(trainDefinitionIndex + 1);
        trainPose_.definitions[trainDefinitionIndex] = resident;

        const auto copy3 = [](double (&target)[4], const glm::dvec3& source)
        {
            target[0] = source.x;
            target[1] = source.y;
            target[2] = source.z;
            target[3] = 0.0;
        };
        for (std::size_t index = 0; index < definition.cars.size(); ++index)
        {
            const TrainCarDefinition& source = definition.cars[index];
            if (source.car.bogies.size() != 2)
                throw std::invalid_argument(
                    "GPU train-pose prototype requires exactly two bogies per car");
            GpuResidentCarDefinition car;
            car.sourceCarIndex = static_cast<std::uint32_t>(index);
            car.totalMassKilograms = totalCarMassKilograms(
                source.car, source.loadout);
            copy3(car.bogie0ReferencePositionMeters,
                source.car.bogies[0].referencePositionMeters);
            copy3(car.bogie1ReferencePositionMeters,
                source.car.bogies[1].referencePositionMeters);
            copy3(car.bodyDimensionsMeters, source.car.bodyDimensionsMeters);
            copy3(car.frontHitchPositionMeters,
                source.car.frontHitchPositionMeters);
            copy3(car.rearHitchPositionMeters,
                source.car.rearHitchPositionMeters);
            copy3(car.loadedCenterOfGravityMeters,
                loadedCarCenterOfGravityMeters(source.car, source.loadout));
            trainPose_.cars.push_back(car);
        }
        for (const InterCarConnectionDefinition& source : definition.connections)
            trainPose_.connections.push_back({source.rigidLengthMeters, 0.0});

        const auto replaceBuffer = [this](VkBuffer& buffer,
            VmaAllocation& allocation, const void* data, const VkDeviceSize bytes)
        {
            if (buffer) vmaDestroyBuffer(allocator_, buffer, allocation);
            VkBufferCreateInfo bufferInfo = makeStorageBufferInfo(bytes);
            VmaAllocationCreateInfo allocationInfo{};
            allocationInfo.usage = VMA_MEMORY_USAGE_AUTO;
            allocationInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
            if (vmaCreateBuffer(allocator_, &bufferInfo, &allocationInfo,
                    &buffer, &allocation, nullptr) != VK_SUCCESS)
                throw std::runtime_error("Unable to allocate resident train-definition buffer");
            void* mapped = nullptr;
            if (vmaMapMemory(allocator_, allocation, &mapped) != VK_SUCCESS)
                throw std::runtime_error("Unable to map resident train-definition buffer");
            std::memcpy(mapped, data, static_cast<std::size_t>(bytes));
            vmaFlushAllocation(allocator_, allocation, 0, bytes);
            vmaUnmapMemory(allocator_, allocation);
        };
        replaceBuffer(trainPose_.definitionBuffer, trainPose_.definitionAllocation,
            trainPose_.definitions.data(), trainPose_.definitions.size()
                * sizeof(GpuResidentTrainDefinition));
        replaceBuffer(trainPose_.carBuffer, trainPose_.carAllocation,
            trainPose_.cars.data(), trainPose_.cars.size()
                * sizeof(GpuResidentCarDefinition));
        const GpuResidentConnectionDefinition dummy{};
        replaceBuffer(trainPose_.connectionBuffer, trainPose_.connectionAllocation,
            trainPose_.connections.empty() ? static_cast<const void*>(&dummy)
                                           : trainPose_.connections.data(),
            std::max<std::size_t>(1, trainPose_.connections.size())
                * sizeof(GpuResidentConnectionDefinition));
    }

    bool GpuPhysicsContext::gpuTrainPoseReady() const noexcept
    {
        return device_ != VK_NULL_HANDLE && shaderFloat64Enabled_
            && trainPosePipelineReady_ && trainPosePipeline_ != VK_NULL_HANDLE
            && gpuTrackReady_ && trainPose_.definitionBuffer != VK_NULL_HANDLE;
    }

    std::vector<GpuRigidBogieResult> GpuPhysicsContext::solveRigidBogiesGpu(
        const std::span<const GpuRigidBogieJob> jobs,
        GpuRigidBogieBatchTimings* const timings,
        const std::uint32_t localSize)
    {
        using Clock = std::chrono::steady_clock;
        if (timings) *timings = {};
        if (jobs.empty()) return {};
        if (!gpuRigidBogieReady() || validationFence_ == VK_NULL_HANDLE)
            return {};
        const auto localSizeIterator = std::find(
            rigidBogieLocalSizes_.begin(), rigidBogieLocalSizes_.end(), localSize);
        if (localSizeIterator == rigidBogieLocalSizes_.end())
            throw std::invalid_argument("Unsupported rigid-bogie benchmark local size");
        const std::size_t pipelineIndex = static_cast<std::size_t>(
            localSizeIterator - rigidBogieLocalSizes_.begin());
        if (rigidBogiePipelines_[pipelineIndex] == VK_NULL_HANDLE)
            throw std::invalid_argument("Rigid-bogie local size is unsupported by this device");

        const auto totalBegin = Clock::now();
        const auto uploadBegin = totalBegin;
        ensureRigidBogieBuffers(jobs.size());
        updatePersistentDescriptors();
        updateRigidBogieDescriptors(jobs.size());
        void* mapped = nullptr;
        if (vmaMapMemory(allocator_, rigidBogie_.jobAllocation, &mapped) != VK_SUCCESS)
            throw std::runtime_error("Unable to map rigid-bogie job buffer");
        std::memcpy(mapped, jobs.data(), jobs.size_bytes());
        vmaFlushAllocation(allocator_, rigidBogie_.jobAllocation, 0, jobs.size_bytes());
        vmaUnmapMemory(allocator_, rigidBogie_.jobAllocation);
        const auto uploadEnd = Clock::now();

        const auto submitBegin = uploadEnd;
        vkResetFences(device_, 1, &validationFence_);
        VkCommandBuffer commandBuffer = commandBuffers_[0];
        vkResetCommandBuffer(commandBuffer, 0);
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
            throw std::runtime_error("Unable to begin rigid-bogie command buffer");
        if (timestampSupported_)
        {
            vkCmdResetQueryPool(commandBuffer, timestampPool_, 0, 2);
        }

        VkMemoryBarrier hostToShader{};
        hostToShader.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        hostToShader.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        hostToShader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_HOST_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &hostToShader,
            0, nullptr, 0, nullptr);
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
            rigidBogiePipelines_[pipelineIndex]);
        const VkDescriptorSet descriptorSets[] = {
            persistentDescriptorSet_, rigidBogie_.descriptorSet};
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
            pipelineLayout_, 0, 2, descriptorSets, 0, nullptr);
        if (timestampSupported_)
            vkCmdWriteTimestamp(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                timestampPool_, 0);
        vkCmdDispatch(commandBuffer,
            static_cast<std::uint32_t>((jobs.size() + localSize - 1) / localSize), 1, 1);
        if (timestampSupported_)
            vkCmdWriteTimestamp(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                timestampPool_, 1);

        VkMemoryBarrier shaderToTransfer{};
        shaderToTransfer.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        shaderToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        shaderToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &shaderToTransfer,
            0, nullptr, 0, nullptr);
        VkBufferCopy copy{};
        copy.size = jobs.size() * sizeof(GpuRigidBogieResult);
        vkCmdCopyBuffer(commandBuffer, rigidBogie_.resultBuffer,
            rigidBogie_.readbackBuffer, 1, &copy);
        VkMemoryBarrier transferToHost{};
        transferToHost.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        transferToHost.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        transferToHost.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &transferToHost,
            0, nullptr, 0, nullptr);
        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
            throw std::runtime_error("Unable to end rigid-bogie command buffer");
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        if (vkQueueSubmit(computeQueue_, 1, &submitInfo, validationFence_) != VK_SUCCESS)
            throw std::runtime_error("Unable to submit rigid-bogie dispatch");
        const auto submitEnd = Clock::now();

        const auto waitBegin = submitEnd;
        if (vkWaitForFences(device_, 1, &validationFence_, VK_TRUE,
                5'000'000'000ULL) != VK_SUCCESS)
            throw std::runtime_error("Rigid-bogie dispatch fence wait failed");
        const auto waitEnd = Clock::now();

        double gpuExecutionMicroseconds = 0.0;
        if (timestampSupported_)
        {
            std::uint64_t timestamps[2]{};
            if (vkGetQueryPoolResults(device_, timestampPool_, 0, 2,
                    sizeof(timestamps), timestamps, sizeof(std::uint64_t),
                    VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT) == VK_SUCCESS)
                gpuExecutionMicroseconds = static_cast<double>(timestamps[1] - timestamps[0])
                    * timestampPeriod_ / 1000.0;
        }

        const auto readbackBegin = waitEnd;
        vmaInvalidateAllocation(allocator_, rigidBogie_.readbackAllocation,
            0, copy.size);
        std::vector<GpuRigidBogieResult> results(jobs.size());
        if (rigidBogie_.readbackMapped)
            std::memcpy(results.data(), rigidBogie_.readbackMapped, copy.size);
        else
        {
            void* readback = nullptr;
            vmaMapMemory(allocator_, rigidBogie_.readbackAllocation, &readback);
            std::memcpy(results.data(), readback, copy.size);
            vmaUnmapMemory(allocator_, rigidBogie_.readbackAllocation);
        }
        const auto readbackEnd = Clock::now();
        if (timings)
        {
            const auto microseconds = [](const auto begin, const auto end)
            {
                return std::chrono::duration<double, std::micro>(end - begin).count();
            };
            timings->packingUploadMicroseconds = microseconds(uploadBegin, uploadEnd);
            timings->submitDispatchMicroseconds = microseconds(submitBegin, submitEnd);
            timings->fenceWaitMicroseconds = microseconds(waitBegin, waitEnd);
            timings->readbackMicroseconds = microseconds(readbackBegin, readbackEnd);
            timings->gpuExecutionMicroseconds = gpuExecutionMicroseconds;
            timings->totalMicroseconds = microseconds(totalBegin, readbackEnd);
        }
        return results;
    }

    std::vector<GpuTrainPoseResult> GpuPhysicsContext::solveTrainPosesGpu(
        const std::span<const GpuTrainPoseJob> jobs,
        GpuTrainPoseBatchTimings* const timings)
    {
        using Clock = std::chrono::steady_clock;
        if (timings) *timings = {};
        if (jobs.empty()) return {};
        if (!gpuTrainPoseReady() || validationFence_ == VK_NULL_HANDLE)
            return {};

        const auto totalBegin = Clock::now();
        const auto uploadBegin = totalBegin;
        ensureTrainPoseBuffers(jobs.size());
        updatePersistentDescriptors();
        updateTrainPoseDescriptors(jobs.size());
        void* mapped = nullptr;
        if (vmaMapMemory(allocator_, trainPose_.jobAllocation, &mapped) != VK_SUCCESS)
            throw std::runtime_error("Unable to map train-pose job buffer");
        std::memcpy(mapped, jobs.data(), jobs.size_bytes());
        vmaFlushAllocation(allocator_, trainPose_.jobAllocation, 0,
            jobs.size_bytes());
        vmaUnmapMemory(allocator_, trainPose_.jobAllocation);
        const auto uploadEnd = Clock::now();

        const auto submitBegin = uploadEnd;
        vkResetFences(device_, 1, &validationFence_);
        VkCommandBuffer commandBuffer = commandBuffers_[0];
        vkResetCommandBuffer(commandBuffer, 0);
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
            throw std::runtime_error("Unable to begin train-pose command buffer");
        if (timestampSupported_)
            vkCmdResetQueryPool(commandBuffer, timestampPool_, 0, 2);
        VkMemoryBarrier hostToShader{};
        hostToShader.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        hostToShader.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        hostToShader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_HOST_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &hostToShader,
            0, nullptr, 0, nullptr);
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
            trainPosePipeline_);
        const VkDescriptorSet descriptorSets[] = {
            persistentDescriptorSet_, trainPose_.descriptorSet};
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
            trainPosePipelineLayout_, 0, 2, descriptorSets, 0, nullptr);
        if (timestampSupported_)
            vkCmdWriteTimestamp(commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, timestampPool_, 0);
        vkCmdDispatch(commandBuffer, static_cast<std::uint32_t>(jobs.size()), 1, 1);
        if (timestampSupported_)
            vkCmdWriteTimestamp(commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, timestampPool_, 1);
        VkMemoryBarrier shaderToTransfer{};
        shaderToTransfer.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        shaderToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        shaderToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &shaderToTransfer,
            0, nullptr, 0, nullptr);
        VkBufferCopy copy{};
        copy.size = jobs.size() * sizeof(GpuTrainPoseResult);
        vkCmdCopyBuffer(commandBuffer, trainPose_.resultBuffer,
            trainPose_.readbackBuffer, 1, &copy);
        VkMemoryBarrier transferToHost{};
        transferToHost.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        transferToHost.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        transferToHost.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &transferToHost,
            0, nullptr, 0, nullptr);
        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
            throw std::runtime_error("Unable to end train-pose command buffer");
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        if (vkQueueSubmit(computeQueue_, 1, &submitInfo, validationFence_)
            != VK_SUCCESS)
            throw std::runtime_error("Unable to submit train-pose dispatch");
        const auto submitEnd = Clock::now();

        const auto waitBegin = submitEnd;
        if (vkWaitForFences(device_, 1, &validationFence_, VK_TRUE,
                5'000'000'000ULL) != VK_SUCCESS)
            throw std::runtime_error("Train-pose dispatch fence wait failed");
        const auto waitEnd = Clock::now();
        double gpuExecutionMicroseconds = 0.0;
        if (timestampSupported_)
        {
            std::uint64_t timestamps[2]{};
            if (vkGetQueryPoolResults(device_, timestampPool_, 0, 2,
                    sizeof(timestamps), timestamps, sizeof(std::uint64_t),
                    VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT)
                == VK_SUCCESS)
                gpuExecutionMicroseconds = static_cast<double>(
                    timestamps[1] - timestamps[0]) * timestampPeriod_ / 1000.0;
        }
        const auto readbackBegin = waitEnd;
        vmaInvalidateAllocation(allocator_, trainPose_.readbackAllocation,
            0, copy.size);
        std::vector<GpuTrainPoseResult> results(jobs.size());
        if (trainPose_.readbackMapped)
            std::memcpy(results.data(), trainPose_.readbackMapped, copy.size);
        else
        {
            void* readback = nullptr;
            vmaMapMemory(allocator_, trainPose_.readbackAllocation, &readback);
            std::memcpy(results.data(), readback, copy.size);
            vmaUnmapMemory(allocator_, trainPose_.readbackAllocation);
        }
        const auto readbackEnd = Clock::now();
        if (timings)
        {
            const auto microseconds = [](const auto begin, const auto end)
            {
                return std::chrono::duration<double, std::micro>(end - begin)
                    .count();
            };
            timings->packingUploadMicroseconds = microseconds(
                uploadBegin, uploadEnd);
            timings->submitDispatchMicroseconds = microseconds(
                submitBegin, submitEnd);
            timings->fenceWaitMicroseconds = microseconds(waitBegin, waitEnd);
            timings->readbackMicroseconds = microseconds(
                readbackBegin, readbackEnd);
            timings->gpuExecutionMicroseconds = gpuExecutionMicroseconds;
            timings->totalMicroseconds = microseconds(totalBegin, readbackEnd);
        }
        return results;
    }

    std::vector<PhysicsTrackSample> GpuPhysicsContext::sampleTrackGpu(std::span<const GpuTrackQuery> queries)
    {
        lastSampleUsedGpu_ = false;
        if (queries.empty()) return {};
        if (!trackUploaded_)
            return {};
        if (!gpuAvailable() || !gpuTrackReady_)
            return sampleTrackForValidation(queries);

        // M0 single-coaster check is already in shared state; validate queries refer to uploaded coaster
        const std::uint32_t slot = 0;
        // Use validationFence for synchronous M1 path
        const VkFence fence = validationFence_;
        if (fence == VK_NULL_HANDLE)
            return sampleTrackForValidation(queries);

        // Ensure track/coaster descriptors are up to date (in case track re-uploaded)
        updatePersistentDescriptors();

        // Ensure query/result buffers and upload queries
        ensureFrameBuffers(slot, queries.size());
        auto& frame = frames_[slot];
        uploadQueries(slot, queries);
        updateFrameDescriptors(slot);

        // Reset fence and record command buffer
        vkResetFences(device_, 1, &fence);
        VkCommandBuffer cmd = commandBuffers_[slot];
        vkResetCommandBuffer(cmd, 0);
        VkCommandBufferBeginInfo begin{};
        begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(cmd, &begin) != VK_SUCCESS)
            return sampleTrackForValidation(queries);

        // Host write -> shader read barrier
        VkMemoryBarrier hostToShader{};
        hostToShader.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        hostToShader.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        hostToShader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &hostToShader, 0, nullptr, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_);
        VkDescriptorSet sets[] = {persistentDescriptorSet_, frame.descriptorSet};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 2, sets, 0, nullptr);
        const std::uint32_t localSize = 64;
        const std::uint32_t groups = static_cast<std::uint32_t>((queries.size() + localSize - 1) / localSize);
        vkCmdDispatch(cmd, groups, 1, 1);

        // Shader write -> transfer read barrier for result copy
        VkMemoryBarrier shaderToTransfer{};
        shaderToTransfer.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        shaderToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        shaderToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &shaderToTransfer, 0, nullptr, 0, nullptr);

        VkBufferCopy copy{};
        copy.size = queries.size() * sizeof(PhysicsTrackSample);
        vkCmdCopyBuffer(cmd, frame.resultBuffer, frame.readbackBuffer, 1, &copy);

        // Transfer write -> host read barrier
        VkMemoryBarrier transferToHost{};
        transferToHost.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        transferToHost.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        transferToHost.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &transferToHost, 0, nullptr, 0, nullptr);

        if (vkEndCommandBuffer(cmd) != VK_SUCCESS)
            return sampleTrackForValidation(queries);

        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        if (vkQueueSubmit(computeQueue_, 1, &submit, fence) != VK_SUCCESS)
            return sampleTrackForValidation(queries);

        const VkResult wait = vkWaitForFences(device_, 1, &fence, VK_TRUE, 5'000'000'000ULL);
        if (wait != VK_SUCCESS)
        {
            vkDeviceWaitIdle(device_);
            return sampleTrackForValidation(queries);
        }

        // Invalidate readback if non-coherent, then copy out
        vmaInvalidateAllocation(allocator_, frame.readbackAllocation, 0, VK_WHOLE_SIZE);
        std::vector<PhysicsTrackSample> out(queries.size());
        if (frame.readbackMapped)
            std::memcpy(out.data(), frame.readbackMapped, copy.size);
        else
        {
            void* mapped = nullptr;
            vmaMapMemory(allocator_, frame.readbackAllocation, &mapped);
            std::memcpy(out.data(), mapped, copy.size);
            vmaUnmapMemory(allocator_, frame.readbackAllocation);
        }
        lastSampleUsedGpu_ = true;
        return out;
    }

    GpuPhysicsContext::GpuValidationResult GpuPhysicsContext::validateGpuAgainstCpu(std::span<const GpuTrackQuery> queries)
    {
        GpuValidationResult res{};
        res.queryCount = queries.size();
        if (queries.empty()) return res;
        if (!gpuAvailable())
            return res;

        auto gpu = sampleTrackGpu(queries);
        auto cpu = sampleTrackForValidation(queries);
        if (gpu.size() != cpu.size()) return res;
        res.gpuExecuted = lastSampleUsedGpu_;

        for (size_t i = 0; i < gpu.size(); ++i)
        {
            const auto& g = gpu[i];
            const auto& c = cpu[i];
            const double dStation = std::abs(g.location.stationMeters - c.location.stationMeters);
            res.maxStationError = std::max(res.maxStationError, dStation);

            glm::dvec3 gp{g.position[0], g.position[1], g.position[2]};
            glm::dvec3 cp{c.position[0], c.position[1], c.position[2]};
            res.maxPositionError = std::max(res.maxPositionError, glm::length(gp - cp));

            glm::dvec3 gt{g.tangent[0], g.tangent[1], g.tangent[2]};
            glm::dvec3 ct{c.tangent[0], c.tangent[1], c.tangent[2]};
            glm::dvec3 gl{g.lateral[0], g.lateral[1], g.lateral[2]};
            glm::dvec3 cl{c.lateral[0], c.lateral[1], c.lateral[2]};
            glm::dvec3 gu{g.up[0], g.up[1], g.up[2]};
            glm::dvec3 cu{c.up[0], c.up[1], c.up[2]};
            res.maxTangentAngleDeg = std::max(res.maxTangentAngleDeg, angleBetweenDeg(gt, ct));
            res.maxLateralAngleDeg = std::max(res.maxLateralAngleDeg, angleBetweenDeg(gl, cl));
            res.maxUpAngleDeg = std::max(res.maxUpAngleDeg, angleBetweenDeg(gu, cu));

            glm::dvec3 gcur{g.curvature[0], g.curvature[1], g.curvature[2]};
            glm::dvec3 ccur{c.curvature[0], c.curvature[1], c.curvature[2]};
            res.maxCurvatureError = std::max(res.maxCurvatureError, glm::length(gcur - ccur));
        }
        return res;
    }
}
