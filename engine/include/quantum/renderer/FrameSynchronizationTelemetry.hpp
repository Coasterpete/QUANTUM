#pragma once

#include <cstdint>

namespace quantum::renderer
{
    // CPU observations only. Times share steady_clock's millisecond epoch;
    // they are not GPU timestamps. IDs identify resources without owning them.
    struct FrameSubmissionTelemetry
    {
        std::uint64_t drawId = 0;
        std::uint64_t swapchainGeneration = 0;
        std::uint32_t frameSlot = 0;
        std::uint32_t imageIndex = UINT32_MAX;
        // renderFinishedSemaphores_[imageIndex] in this swapchain generation.
        bool fenceReset = false;
        bool submitted = false;
        bool presentCalled = false;
        std::int32_t acquireResult = 0;
        std::int32_t resetResult = 0;
        std::int32_t submitResult = 0;
        std::int32_t presentResult = 0;
        double resetBeginMilliseconds = 0.0;
        double resetEndMilliseconds = 0.0;
        double recordCommandsMilliseconds = 0.0;
        double submitBeginMilliseconds = 0.0;
        double submitEndMilliseconds = 0.0;
        double presentBeginMilliseconds = 0.0;
        double presentEndMilliseconds = 0.0;
    };

    struct FrameSynchronizationTelemetry
    {
        FrameSubmissionTelemetry current;
        // With two frames in flight this normally belongs to draw N-2,
        // not the immediately preceding draw N-1.
        FrameSubmissionTelemetry waitedSubmission;
        std::int32_t fenceStatusBeforeWait = 0;
        double fenceStatusCallMilliseconds = 0.0;
        double waitBeginMilliseconds = 0.0;
        double waitEndMilliseconds = 0.0;
    };
}
