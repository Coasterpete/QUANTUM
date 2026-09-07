#pragma once

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
        double interpolationMilliseconds = 0.0;
        double renderPoseSolveMilliseconds = 0.0;
        std::size_t renderPoseSolveCount = 0;
        std::size_t renderPoseFailureCount = 0;
        double previewVertexPreparationMilliseconds = 0.0;
        double previewVertexPublishMilliseconds = 0.0;
        double previewFrameSlotUpdateMilliseconds = 0.0;
        double previewFrameSlotWaitMilliseconds = 0.0;
        double drawFrameCpuMilliseconds = 0.0;
        double acquireCallMilliseconds = 0.0;
        double presentCallMilliseconds = 0.0;
        bool previewStreamUpdated = false;
        bool swapchainRecreated = false;
        bool synchronousReadback = false;
        FrameBlockingEvents blockingEvents;
        renderer::FrameSynchronizationTelemetry synchronization;
    };
}
