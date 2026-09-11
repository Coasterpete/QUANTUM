#pragma once

#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace quantum::physics::gpu
{
    struct GpuTrackQuery
    {
        std::uint32_t coasterIndex = 0;
        std::uint32_t path = 0;
        std::int32_t direction = 1;
        std::uint32_t _reserved = 0;
        double stationMeters = 0.0;
    };
    static_assert(sizeof(GpuTrackQuery) == 24, "GpuTrackQuery must be 24 bytes");
    static_assert(alignof(GpuTrackQuery) == 8, "GpuTrackQuery align 8");
    static_assert(offsetof(GpuTrackQuery, stationMeters) == 16, "GpuTrackQuery station offset 16 std430");

    struct PhysicsTrackSample
    {
        // Layout must match std430 in track_sample.comp:
        //   GpuTrackQuery (24) + 2x uint (8) = 32, then 5x double[4] (32 each) = 192.
        // C++ over-aligns each array to 32 to guarantee std430 stride 32, even though
        // GLSL base alignment is 8. Both produce stride 32 because 24+8=32.
        GpuTrackQuery location;
        std::uint32_t _pad0 = 0;
        std::uint32_t _pad1 = 0;
        alignas(32) double position[4] = {0.0, 0.0, 0.0, 0.0};
        alignas(32) double tangent[4] = {0.0, 0.0, 0.0, 0.0};
        alignas(32) double lateral[4] = {0.0, 0.0, 0.0, 0.0};
        alignas(32) double up[4] = {0.0, 0.0, 0.0, 0.0};
        alignas(32) double curvature[4] = {0.0, 0.0, 0.0, 0.0};
    };
    static_assert(sizeof(PhysicsTrackSample) == 192, "PhysicsTrackSample 192 bytes");
    static_assert(alignof(PhysicsTrackSample) == 32, "PhysicsTrackSample align 32 due to arrays");
    static_assert(offsetof(PhysicsTrackSample, _pad0) == 24, "PhysicsTrackSample pad0 offset");
    static_assert(offsetof(PhysicsTrackSample, position) == 32, "PhysicsTrackSample position offset 32 std430");
    static_assert(offsetof(PhysicsTrackSample, tangent) == 64, "PhysicsTrackSample tangent offset");
    static_assert(offsetof(PhysicsTrackSample, curvature) == 160, "PhysicsTrackSample curvature offset");

    enum class GpuRigidBogieStatus : std::uint32_t
    {
        Solved = 0,
        NominalOverextended = 1,
        NoLocalInterval = 2,
        NoFeasibleRoot = 3,
        DidNotConverge = 4,
        NonFinite = 5
    };

    struct GpuRigidBogieJob
    {
        std::uint32_t coasterIndex = 0;
        std::uint32_t path = 0;
        std::int32_t direction = 1;
        std::uint32_t jobId = 0;
        double referenceStationMeters = 0.0;
        double frontReferencePositionMeters[4] = {0.0, 0.0, 0.0, 0.0};
        double rearReferencePositionMeters[4] = {0.0, 0.0, 0.0, 0.0};
    };
    static_assert(sizeof(GpuRigidBogieJob) == 88, "GpuRigidBogieJob must be 88 bytes std430");
    static_assert(alignof(GpuRigidBogieJob) == 8, "GpuRigidBogieJob align 8");
    static_assert(offsetof(GpuRigidBogieJob, coasterIndex) == 0, "GpuRigidBogieJob coaster offset 0");
    static_assert(offsetof(GpuRigidBogieJob, path) == 4, "GpuRigidBogieJob path offset 4");
    static_assert(offsetof(GpuRigidBogieJob, direction) == 8, "GpuRigidBogieJob direction offset 8");
    static_assert(offsetof(GpuRigidBogieJob, jobId) == 12, "GpuRigidBogieJob id offset 12");
    static_assert(offsetof(GpuRigidBogieJob, referenceStationMeters) == 16, "GpuRigidBogieJob station offset 16");
    static_assert(offsetof(GpuRigidBogieJob, frontReferencePositionMeters) == 24, "GpuRigidBogieJob front offset 24");
    static_assert(offsetof(GpuRigidBogieJob, rearReferencePositionMeters) == 56, "GpuRigidBogieJob rear offset 56");

    struct GpuRigidBogieResult
    {
        std::uint32_t jobId = 0;
        GpuRigidBogieStatus status = GpuRigidBogieStatus::NonFinite;
        std::uint32_t refinementIterations = 0;
        std::uint32_t bracketExpansions = 0;
        double frontStationMeters = 0.0;
        double rearStationMeters = 0.0;
        double finalResidualMeters = 0.0;
    };
    static_assert(sizeof(GpuRigidBogieResult) == 40, "GpuRigidBogieResult must be 40 bytes std430");
    static_assert(alignof(GpuRigidBogieResult) == 8, "GpuRigidBogieResult align 8");
    static_assert(offsetof(GpuRigidBogieResult, jobId) == 0, "GpuRigidBogieResult id offset 0");
    static_assert(offsetof(GpuRigidBogieResult, status) == 4, "GpuRigidBogieResult status offset 4");
    static_assert(offsetof(GpuRigidBogieResult, refinementIterations) == 8, "GpuRigidBogieResult refinement offset 8");
    static_assert(offsetof(GpuRigidBogieResult, bracketExpansions) == 12, "GpuRigidBogieResult expansion offset 12");
    static_assert(offsetof(GpuRigidBogieResult, frontStationMeters) == 16, "GpuRigidBogieResult front offset 16");
    static_assert(offsetof(GpuRigidBogieResult, rearStationMeters) == 24, "GpuRigidBogieResult rear offset 24");
    static_assert(offsetof(GpuRigidBogieResult, finalResidualMeters) == 32, "GpuRigidBogieResult residual offset 32");

    struct GpuRigidBogieBatchTimings
    {
        double packingUploadMicroseconds = 0.0;
        double submitDispatchMicroseconds = 0.0;
        double fenceWaitMicroseconds = 0.0;
        double readbackMicroseconds = 0.0;
        double gpuExecutionMicroseconds = 0.0;
        double totalMicroseconds = 0.0;
    };

    inline constexpr std::size_t gpuTrainPoseMaximumCarCount = 8;
    inline constexpr std::size_t gpuTrainPoseMaximumConnectionCount =
        gpuTrainPoseMaximumCarCount - 1;

    enum class GpuTrainPoseStatus : std::uint32_t
    {
        Solved = 0,
        InvalidJob = 1,
        NonFinite = 2,
        OpenTrackPlacement = 3,
        RigidNominalOverextended = 4,
        RigidNoLocalInterval = 5,
        RigidNoFeasibleRoot = 6,
        RigidDidNotConverge = 7,
        ConnectorNoSearchInterval = 8,
        ConnectorNoLegalPlacement = 9,
        ConnectorOpenEndpoint = 10,
        ConnectorCannotClose = 11,
        ConnectorClosureError = 12,
        PoseConstructionError = 13,
        MassPropertiesError = 14
    };

    struct GpuTrainPoseJob
    {
        std::uint32_t coasterIndex = 0;
        std::uint32_t path = 0;
        std::int32_t direction = 1;
        std::uint32_t jobId = 0;
        std::uint32_t trainDefinitionIndex = 0;
        std::uint32_t _reserved0 = 0;
        double referenceStationMeters = 0.0;
    };
    static_assert(sizeof(GpuTrainPoseJob) == 32);
    static_assert(alignof(GpuTrainPoseJob) == 8);
    static_assert(offsetof(GpuTrainPoseJob, referenceStationMeters) == 24);

    struct GpuResidentTrainDefinition
    {
        std::uint32_t firstCar = 0;
        std::uint32_t carCount = 0;
        std::uint32_t firstConnection = 0;
        std::uint32_t connectionCount = 0;
    };
    static_assert(sizeof(GpuResidentTrainDefinition) == 16);

    struct GpuResidentCarDefinition
    {
        std::uint32_t sourceCarIndex = 0;
        std::uint32_t _reserved0 = 0;
        std::uint32_t _reserved1 = 0;
        std::uint32_t _reserved2 = 0;
        double totalMassKilograms = 0.0;
        double _massPadding = 0.0;
        double bogie0ReferencePositionMeters[4]{};
        double bogie1ReferencePositionMeters[4]{};
        double bodyDimensionsMeters[4]{};
        double frontHitchPositionMeters[4]{};
        double rearHitchPositionMeters[4]{};
        double loadedCenterOfGravityMeters[4]{};
    };
    static_assert(sizeof(GpuResidentCarDefinition) == 224);
    static_assert(offsetof(GpuResidentCarDefinition,
        bogie0ReferencePositionMeters) == 32);
    static_assert(offsetof(GpuResidentCarDefinition,
        loadedCenterOfGravityMeters) == 192);

    struct GpuResidentConnectionDefinition
    {
        double rigidLengthMeters = 0.0;
        double _reserved = 0.0;
    };
    static_assert(sizeof(GpuResidentConnectionDefinition) == 16);

    struct GpuTrainCarPose
    {
        std::uint32_t carIndex = 0;
        std::uint32_t frontBogieDefinitionIndex = 0;
        std::uint32_t rearBogieDefinitionIndex = 0;
        std::uint32_t _reserved = 0;
        double referenceStationMeters = 0.0;
        double frontBogieStationMeters = 0.0;
        double rearBogieStationMeters = 0.0;
        double totalMassKilograms = 0.0;
        double bodyWorldPositionMeters[4]{};
        double bodyTangent[4]{};
        double bodyLateral[4]{};
        double bodyUp[4]{};
        double bodyOrientationWxyz[4]{};
        double localCenterOfGravityMeters[4]{};
        double worldCenterOfGravityMeters[4]{};
        double frontHitchWorldPositionMeters[4]{};
        double rearHitchWorldPositionMeters[4]{};
        double frontBogieWorldPositionMeters[4]{};
        double frontBogieTrackTangent[4]{};
        double frontBogieTrackLateral[4]{};
        double frontBogieTrackUp[4]{};
        double frontBogieRelativeOrientationWxyz[4]{};
        double rearBogieWorldPositionMeters[4]{};
        double rearBogieTrackTangent[4]{};
        double rearBogieTrackLateral[4]{};
        double rearBogieTrackUp[4]{};
        double rearBogieRelativeOrientationWxyz[4]{};
        double frontBogieRelativeYawRadians = 0.0;
        double rearBogieRelativeYawRadians = 0.0;
    };
    static_assert(sizeof(GpuTrainCarPose) == 672);
    static_assert(offsetof(GpuTrainCarPose, bodyWorldPositionMeters) == 48);
    static_assert(offsetof(GpuTrainCarPose,
        frontBogieRelativeYawRadians) == 656);

    struct GpuTrainConnectionPose
    {
        std::uint32_t connectionIndex = 0;
        std::uint32_t leadingCarIndex = 0;
        std::uint32_t followingCarIndex = 0;
        std::uint32_t solverIterationCount = 0;
        std::uint32_t usedExhaustiveSearchFallback = 0;
        std::uint32_t candidateEvaluationCount = 0;
        std::uint32_t rigidRefinementCount = 0;
        std::uint32_t _reserved = 0;
        double authoredRigidLengthMeters = 0.0;
        double actualEndpointDistanceMeters = 0.0;
        double signedLengthResidualMeters = 0.0;
        double finalBracketSizeMeters = 0.0;
        double leadingEndpointWorldPositionMeters[4]{};
        double followingEndpointWorldPositionMeters[4]{};
        double worldDirection[4]{};
        double directionInLeadingBody[4]{};
        double directionInFollowingBody[4]{};
        double followingBodyRelativeOrientationWxyz[4]{};
        double relativeYawPitchRollRadians[4]{};
    };
    static_assert(sizeof(GpuTrainConnectionPose) == 288);
    static_assert(offsetof(GpuTrainConnectionPose,
        authoredRigidLengthMeters) == 32);
    static_assert(offsetof(GpuTrainConnectionPose,
        leadingEndpointWorldPositionMeters) == 64);

    struct GpuTrainPoseResult
    {
        std::uint32_t jobId = 0;
        GpuTrainPoseStatus status = GpuTrainPoseStatus::InvalidJob;
        std::uint32_t carCount = 0;
        std::uint32_t connectionCount = 0;
        std::uint32_t failureCarIndex = 0;
        std::uint32_t failureConnectionIndex = 0;
        std::uint32_t rigidSolveCount = 0;
        std::uint32_t rigidRefinementCount = 0;
        std::uint32_t rigidBracketExpansionCount = 0;
        std::uint32_t connectorCandidateEvaluationCount = 0;
        std::uint32_t connectorRefinementCount = 0;
        std::uint32_t connectorFallbackCount = 0;
        std::uint32_t connectorCandidateMinimum = 0;
        std::uint32_t connectorCandidateMaximum = 0;
        std::uint32_t connectorRefinementMinimum = 0;
        std::uint32_t connectorRefinementMaximum = 0;
        double referenceStationMeters = 0.0;
        double totalMassKilograms = 0.0;
        double maximumAbsoluteConnectorResidualMeters = 0.0;
        double _reservedDouble = 0.0;
        double aggregateWorldCenterOfGravityMeters[4]{};
        GpuTrainCarPose cars[gpuTrainPoseMaximumCarCount]{};
        GpuTrainConnectionPose
            connections[gpuTrainPoseMaximumConnectionCount]{};
    };
    static_assert(offsetof(GpuTrainPoseResult, referenceStationMeters) == 64);
    static_assert(offsetof(GpuTrainPoseResult,
        aggregateWorldCenterOfGravityMeters) == 96);
    static_assert(offsetof(GpuTrainPoseResult, cars) == 128);
    static_assert(sizeof(GpuTrainPoseResult) == 7'520);

    using GpuTrainPoseBatchTimings = GpuRigidBogieBatchTimings;

    struct GpuComputeDeviceInfo
    {
        std::uint32_t subgroupSize = 0;
        std::uint32_t maxWorkgroupSizeX = 0;
        std::uint32_t maxWorkgroupInvocations = 0;
        std::uint32_t timestampValidBits = 0;
        bool timestampsSupported = false;
        double timestampPeriodNanoseconds = 0.0;
    };

    class GpuPhysicsContext
    {
    public:
        struct HeadlessHandles
        {
            VkInstance instance = VK_NULL_HANDLE;
            VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
            VkDevice device = VK_NULL_HANDLE;
            VmaAllocator allocator = VK_NULL_HANDLE;
            VkQueue queue = VK_NULL_HANDLE;
            std::uint32_t queueFamily = 0;
            bool shaderFloat64Enabled = false;
            bool ownsInstance = false;
            bool ownsDevice = false;
            bool ownsAllocator = false;
        };
        explicit GpuPhysicsContext(renderer::VulkanContext& vulkan);
        explicit GpuPhysicsContext(HeadlessHandles handles);
        ~GpuPhysicsContext();

        GpuPhysicsContext(const GpuPhysicsContext&) = delete;
        GpuPhysicsContext& operator=(const GpuPhysicsContext&) = delete;

        // Factory for headless compute-only validation (no SDL window).
        [[nodiscard]] static HeadlessHandles createHeadlessHandles();

        // M0: single-coaster only (coasterIndex must be 0). Multi-coaster is deferred to M1.
        // Throws std::invalid_argument if coasterIndex != 0.
        void uploadTrack(
            std::uint32_t coasterIndex,
            std::span<const coaster::TrackKinematicState> kinematics,
            double metersPerCoordinateUnit,
            coaster::TopologyKind topology,
            coaster::LayoutMode layoutMode);

        // M0: no real GPU dispatch. Validates slot and records currentSlot_ for
        // future M1 async path. Use sampleTrackForValidation for CPU reference results.
        void sampleTrack(std::uint32_t slot, std::span<const GpuTrackQuery> queries);

        // CPU reference sampler matching track_sample.comp logic for M0 validation.
        // NOTE: CPU uses quaternion slerp (glm::slerp) for frame interpolation;
        //       GLSL uses mix + Gram-Schmidt (nlerp) due to lack of portable dquat slerp.
        //       The mismatch is documented and deferred to M1 for exact equivalence.
        [[nodiscard]] std::vector<PhysicsTrackSample> sampleTrackForValidation(
            std::span<const GpuTrackQuery> queries);

        // M1: real GPU dispatch (synchronous). Returns GPU results when available;
        // falls back to CPU when device/shaderFloat64/pipeline unavailable.
        [[nodiscard]] bool gpuAvailable() const noexcept;
        [[nodiscard]] bool hasUploadedTrack() const noexcept;
        [[nodiscard]] bool gpuTrackReady() const noexcept;
        [[nodiscard]] bool lastSampleUsedGpu() const noexcept;
        [[nodiscard]] std::vector<PhysicsTrackSample> sampleTrackGpu(
            std::span<const GpuTrackQuery> queries);
        [[nodiscard]] bool gpuRigidBogieReady() const noexcept;
        [[nodiscard]] GpuComputeDeviceInfo computeDeviceInfo() const noexcept;
        [[nodiscard]] std::vector<GpuRigidBogieResult> solveRigidBogiesGpu(
            std::span<const GpuRigidBogieJob> jobs,
            GpuRigidBogieBatchTimings* timings = nullptr,
            std::uint32_t localSize = 64);
        void uploadTrainDefinition(
            std::uint32_t trainDefinitionIndex,
            const TrainDefinition& definition);
        [[nodiscard]] bool gpuTrainPoseReady() const noexcept;
        [[nodiscard]] std::vector<GpuTrainPoseResult> solveTrainPosesGpu(
            std::span<const GpuTrainPoseJob> jobs,
            GpuTrainPoseBatchTimings* timings = nullptr);
        // Validates GPU vs CPU for given queries, returns max errors. Throws on GPU
        // unavailable. Uses same tolerances as GpuPhysicsTrackSamplingValidation test.
        struct GpuValidationResult
        {
            bool gpuExecuted = false;
            std::size_t queryCount = 0;
            double maxStationError = 0.0;
            double maxPositionError = 0.0;
            double maxTangentAngleDeg = 0.0;
            double maxLateralAngleDeg = 0.0;
            double maxUpAngleDeg = 0.0;
            double maxCurvatureError = 0.0;
        };
        [[nodiscard]] GpuValidationResult validateGpuAgainstCpu(
            std::span<const GpuTrackQuery> queries);

    private:
        struct TrackBuffer
        {
            VkBuffer buffer = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkDeviceSize capacity = 0;
            std::uint32_t sampleCount = 0;
        };

        struct CoasterRecord
        {
            std::uint32_t trackOffset = 0;
            std::uint32_t trackCount = 0;
            std::uint32_t topology = 0;
            std::uint32_t _pad = 0;
            double length = 0.0;
        };
        static_assert(sizeof(CoasterRecord) == 24, "CoasterRecord must be 24 bytes std430");
        static_assert(alignof(CoasterRecord) == 8, "CoasterRecord align 8");
        static_assert(offsetof(CoasterRecord, length) == 16, "CoasterRecord length offset 16");

        struct TransientFrameData
        {
            VkBuffer queryBuffer = VK_NULL_HANDLE;
            VmaAllocation queryAllocation = VK_NULL_HANDLE;
            VkBuffer resultBuffer = VK_NULL_HANDLE;
            VmaAllocation resultAllocation = VK_NULL_HANDLE;
            VkBuffer readbackBuffer = VK_NULL_HANDLE;
            VmaAllocation readbackAllocation = VK_NULL_HANDLE;
            void* readbackMapped = nullptr;
            VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
            std::uint32_t queryCount = 0;
        };

        struct RigidBogieData
        {
            VkBuffer jobBuffer = VK_NULL_HANDLE;
            VmaAllocation jobAllocation = VK_NULL_HANDLE;
            VkBuffer resultBuffer = VK_NULL_HANDLE;
            VmaAllocation resultAllocation = VK_NULL_HANDLE;
            VkBuffer readbackBuffer = VK_NULL_HANDLE;
            VmaAllocation readbackAllocation = VK_NULL_HANDLE;
            void* readbackMapped = nullptr;
            VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
            VkDeviceSize capacityJobs = 0;
        };

        struct TrainPoseData
        {
            VkBuffer definitionBuffer = VK_NULL_HANDLE;
            VmaAllocation definitionAllocation = VK_NULL_HANDLE;
            VkBuffer carBuffer = VK_NULL_HANDLE;
            VmaAllocation carAllocation = VK_NULL_HANDLE;
            VkBuffer connectionBuffer = VK_NULL_HANDLE;
            VmaAllocation connectionAllocation = VK_NULL_HANDLE;
            VkBuffer jobBuffer = VK_NULL_HANDLE;
            VmaAllocation jobAllocation = VK_NULL_HANDLE;
            VkBuffer resultBuffer = VK_NULL_HANDLE;
            VmaAllocation resultAllocation = VK_NULL_HANDLE;
            VkBuffer readbackBuffer = VK_NULL_HANDLE;
            VmaAllocation readbackAllocation = VK_NULL_HANDLE;
            void* readbackMapped = nullptr;
            VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
            VkDeviceSize capacityJobs = 0;
            std::vector<GpuResidentTrainDefinition> definitions;
            std::vector<GpuResidentCarDefinition> cars;
            std::vector<GpuResidentConnectionDefinition> connections;
        };

        renderer::VulkanContext* vulkan_ = nullptr;
        HeadlessHandles headless_;
        bool useHeadless_ = false;
        VkDevice device_ = VK_NULL_HANDLE;
        VmaAllocator allocator_ = VK_NULL_HANDLE;
        VkQueue computeQueue_ = VK_NULL_HANDLE;
        std::uint32_t computeQueueFamily_ = 0;
        bool shaderFloat64Enabled_ = false;

        TrackBuffer trackBuffer_;
        std::vector<CoasterRecord> coasterRecords_;
        std::vector<CoasterRecord> coasterRecordsHost_;
        VkBuffer coasterRecordBuffer_ = VK_NULL_HANDLE;
        VmaAllocation coasterRecordAllocation_ = VK_NULL_HANDLE;
        void* coasterRecordMapped_ = nullptr;
        VkDeviceSize coasterRecordCapacity_ = 0;

        VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
        VkDescriptorSetLayout persistentSetLayout_ = VK_NULL_HANDLE;
        VkDescriptorSet persistentDescriptorSet_ = VK_NULL_HANDLE;
        VkDescriptorSetLayout transientSetLayout_ = VK_NULL_HANDLE;
        std::array<TransientFrameData, 2> frames_{};
        VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline computePipeline_ = VK_NULL_HANDLE;
        static constexpr std::array<std::uint32_t, 4> rigidBogieLocalSizes_{32, 64, 128, 256};
        std::array<VkPipeline, rigidBogieLocalSizes_.size()> rigidBogiePipelines_{};
        RigidBogieData rigidBogie_;
        VkDescriptorSetLayout trainPoseSetLayout_ = VK_NULL_HANDLE;
        VkPipelineLayout trainPosePipelineLayout_ = VK_NULL_HANDLE;
        VkPipeline trainPosePipeline_ = VK_NULL_HANDLE;
        TrainPoseData trainPose_;

        VkCommandPool commandPool_ = VK_NULL_HANDLE;
        std::array<VkCommandBuffer, 2> commandBuffers_{};
        std::array<VkFence, 2> computeFences_{};
        VkFence validationFence_ = VK_NULL_HANDLE;

        VkQueryPool timestampPool_ = VK_NULL_HANDLE;
        bool timestampSupported_ = false;
        double timestampPeriod_ = 0.0;
        GpuComputeDeviceInfo computeDeviceInfo_{};

        std::uint32_t trackOffset_ = 0;
        std::uint32_t currentSlot_ = 0;
        bool gpuPipelineReady_ = false;
        bool rigidBogiePipelineReady_ = false;
        bool trainPosePipelineReady_ = false;
        bool trackUploaded_ = false;
        bool gpuTrackReady_ = false;
        bool lastSampleUsedGpu_ = false;

        void createCommandPool();
        void createDescriptorResources();
        void createPipeline();
        void createTrainPosePipeline();
        void createSyncResources();
        void createTimestampPool();
        void ensureTrackBufferCapacity(std::uint32_t requiredSamples);
        void ensureCoasterRecordCapacity(std::uint32_t requiredCount);
        void ensureFrameBuffers(std::uint32_t slot, std::size_t queryCount);
        void ensureRigidBogieBuffers(std::size_t jobCount);
        void updateRigidBogieDescriptors(std::size_t jobCount);
        void ensureTrainPoseBuffers(std::size_t jobCount);
        void updateTrainPoseDescriptors(std::size_t jobCount);
        void allocateDescriptorSets();
        void updatePersistentDescriptors();
        void updateFrameDescriptors(std::uint32_t slot);
        void uploadTrackData(std::span<const coaster::TrackKinematicState> kinematics,
                             double metersPerCoordinateUnit);
        void uploadQueries(std::uint32_t slot, std::span<const GpuTrackQuery> queries);
        VkCommandBuffer beginSingleTimeCommands();
        void endSingleTimeCommands(VkCommandBuffer cmd);
    };
}
