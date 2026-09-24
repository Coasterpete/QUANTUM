#pragma once

#include <quantum/physics/CarPose.hpp>
#include <quantum/renderer/FrameSynchronizationTelemetry.hpp>

#include <cstddef>
#include <cstdint>

namespace quantum::editor
{
    struct FrameBlockingEvents
    {
        bool viewportResized = false;
        bool imguiBackendRecreated = false;
        bool windowMinimized = false;
        bool windowRestored = false;
        bool focusLost = false;
        bool focusRegained = false;
        bool trackBufferMutation = false;
        bool hardwareAssetReload = false;
        bool modalOrFileDialog = false;
    };

    // One rendered frame's lightweight CPU-side measurements. Durations are
    // milliseconds and deliberately exclude GPU timestamp-query data.
    struct FramePerformanceSample
    {
        std::uint64_t frameId = 0;
        double rawSimulationDeltaMilliseconds = 0.0;
        double accumulatorBeforeMilliseconds = 0.0;
        double accumulatorAfterIncomingMilliseconds = 0.0;
        double accumulatorRemainingMilliseconds = 0.0;
        double discardedWallTimeMilliseconds = 0.0;
        std::size_t requestedPhysicsStepCount = 0;
        double frameTimeMilliseconds = 0.0;
        std::size_t fixedPhysicsStepCount = 0;
        bool maximumPhysicsStepsHit = false;
        double minimumPhysicsStepMilliseconds = 0.0;
        double averagePhysicsStepMilliseconds = 0.0;
        double maximumPhysicsStepMilliseconds = 0.0;
        double physicsMilliseconds = 0.0;
        std::size_t consecutiveCatchUpFrameCount = 0;
        double eventPumpMilliseconds = 0.0;
        double preSimulationCpuMilliseconds = 0.0;
        double frameStartToSimulationMilliseconds = 0.0;
        double mainThreadFrameMilliseconds = 0.0;
        double interpolationMilliseconds = 0.0;
        double renderPoseSolveMilliseconds = 0.0;
        std::size_t renderPoseSolveCount = 0;
        std::size_t renderPoseFailureCount = 0;
        double previewVertexPreparationMilliseconds = 0.0;
        std::uint64_t simulationStartingTick = 0;
        std::uint64_t simulationEndingTick = 0;
        double simulationStartingStationMeters = 0.0;
        double simulationEndingStationMeters = 0.0;
        double startingSignedVelocityMetersPerSecond = 0.0;
        double endingSignedVelocityMetersPerSecond = 0.0;
        double minimumAbsoluteVelocityMetersPerSecond = 0.0;
        std::size_t zeroVelocityStepCount = 0;
        std::size_t zeroSpeedTransitionCount = 0;
        std::size_t rollbackStartCount = 0;
        std::size_t rollbackStepCount = 0;
        bool boundaryStopped = false;
        double gpuPreviewSamplingMilliseconds = 0.0;
        double gpuPreviewPreparationMilliseconds = 0.0;
        double gpuPreviewCommandRecordingMilliseconds = 0.0;
        double gpuPreviewQueueSubmitMilliseconds = 0.0;
        double gpuPreviewFenceWaitMilliseconds = 0.0;
        double gpuPreviewReadbackMilliseconds = 0.0;
        double gpuPreviewValidationMilliseconds = 0.0;
        std::size_t gpuPreviewQueryCount = 0;
        std::size_t gpuPreviewDispatchCount = 0;
        std::size_t gpuPreviewFallbackCount = 0;
        double previewVertexPublishMilliseconds = 0.0;
        double previewFrameSlotUpdateMilliseconds = 0.0;
        double previewFrameSlotWaitMilliseconds = 0.0;
        double deferredBufferReclaimMilliseconds = 0.0;
        double drawFrameCpuMilliseconds = 0.0;
        double acquireCallMilliseconds = 0.0;
        double presentCallMilliseconds = 0.0;
        double gpuExecutionMilliseconds = 0.0;
        std::size_t deferredBufferCountBeforeReclaim = 0;
        std::uint64_t deferredBufferBytesBeforeReclaim = 0;
        std::size_t reclaimedBufferCount = 0;
        bool gpuTimingAvailable = false;
        bool previewStreamUpdated = false;
        bool swapchainRecreated = false;
        bool synchronousReadback = false;
        FrameBlockingEvents blockingEvents;
        renderer::FrameSynchronizationTelemetry synchronization;
        physics::TrainSolveCounters solverCounters;
    };
}
