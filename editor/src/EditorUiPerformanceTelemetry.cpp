#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/EditorStyle.hpp>
#include <quantum/editor/SimulationPreview.hpp>

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cstddef>

namespace quantum::editor
{
    void EditorUi::drawPerformanceTelemetry()
    {
        if (!performanceTelemetryWindowOpen_)
        {
            return;
        }

        if (!ImGui::Begin(
                "Performance Telemetry",
                &performanceTelemetryWindowOpen_,
                ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::End();
            return;
        }

        if (performanceHistoryCount_ == 0)
        {
            ImGui::TextDisabled("Waiting for the first rendered frame...");
            ImGui::End();
            return;
        }

        std::size_t displayedCount = 0;
        double displayedDurationMilliseconds = 0.0;
        while (displayedCount < performanceHistoryCount_
            && displayedDurationMilliseconds < 2000.0)
        {
            const std::size_t index =
                (performanceHistoryNext_ + performanceHistoryCapacity - 1
                    - displayedCount)
                % performanceHistoryCapacity;
            displayedDurationMilliseconds += std::max(
                performanceHistory_[index].frameTimeMilliseconds,
                0.0);
            ++displayedCount;
        }

        const std::size_t currentIndex =
            (performanceHistoryNext_ + performanceHistoryCapacity - 1)
            % performanceHistoryCapacity;
        const FramePerformanceSample& current =
            performanceHistory_[currentIndex];
        const FramePerformanceSample* previous = nullptr;
        if (performanceHistoryCount_ > 1)
        {
            const std::size_t previousIndex =
                (performanceHistoryNext_ + performanceHistoryCapacity - 2)
                % performanceHistoryCapacity;
            previous = &performanceHistory_[previousIndex];
        }
        const std::size_t oldestIndex =
            (performanceHistoryNext_ + performanceHistoryCapacity
                - displayedCount)
            % performanceHistoryCapacity;

        double summedFrameMilliseconds = 0.0;
        for (std::size_t offset = 0; offset < displayedCount; ++offset)
        {
            const std::size_t index =
                (oldestIndex + offset) % performanceHistoryCapacity;
            summedFrameMilliseconds +=
                performanceHistory_[index].frameTimeMilliseconds;
        }
        const double averageFrameMilliseconds = displayedCount > 0
            ? summedFrameMilliseconds / static_cast<double>(displayedCount)
            : 0.0;
        const double currentFramesPerSecond =
            current.frameTimeMilliseconds > 0.0
            ? 1000.0 / current.frameTimeMilliseconds
            : 0.0;
        const double averageFramesPerSecond = averageFrameMilliseconds > 0.0
            ? 1000.0 / averageFrameMilliseconds
            : 0.0;
        const double previewMilliseconds =
            current.previewVertexPreparationMilliseconds
            + current.previewVertexPublishMilliseconds
            + current.previewFrameSlotUpdateMilliseconds
            + current.previewFrameSlotWaitMilliseconds;

        ImGui::TextDisabled(
            "Latest completed frame (displayed one frame later)");
        ImGui::Text(
            "Render: %.1f FPS current | %.1f FPS 2 s avg",
            currentFramesPerSecond,
            averageFramesPerSecond);
        ImGui::Text(
            "Frame: %.3f ms current | %.3f ms 2 s avg",
            current.frameTimeMilliseconds,
            averageFrameMilliseconds);

        editorHeading("Catch-up / Spike Diagnostics", fonts_);
        const bool largeCatchUp = current.requestedPhysicsStepCount
            >= SimulationPreview::catchUpStepThreshold;
        if (largeCatchUp)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, palette::warning);
        }
        ImGui::Text(
            "Frame %llu | raw incoming delta %.3f ms",
            static_cast<unsigned long long>(current.frameId),
            current.rawSimulationDeltaMilliseconds);
        ImGui::Text(
            "Requested / executed: %zu / %zu%s",
            current.requestedPhysicsStepCount,
            current.fixedPhysicsStepCount,
            current.maximumPhysicsStepsHit ? " | maximum hit" : "");
        if (largeCatchUp)
        {
            ImGui::PopStyleColor();
        }
        ImGui::Text(
            "Accumulator ms: %.3f before | %.3f after input | %.3f remaining",
            current.accumulatorBeforeMilliseconds,
            current.accumulatorAfterIncomingMilliseconds,
            current.accumulatorRemainingMilliseconds);
        ImGui::Text(
            "Discarded wall time (cap/interruption): %.3f ms | consecutive >= %zu-step frames: %zu",
            current.discardedWallTimeMilliseconds,
            SimulationPreview::catchUpStepThreshold,
            current.consecutiveCatchUpFrameCount);
        ImGui::Text(
            "Physics step min / avg / max: %.3f / %.3f / %.3f ms | total %.3f ms",
            current.minimumPhysicsStepMilliseconds,
            current.averagePhysicsStepMilliseconds,
            current.maximumPhysicsStepMilliseconds,
            current.physicsMilliseconds);
        ImGui::Text(
            "Current pre-simulation: events %.3f ms | after events %.3f ms | total %.3f ms",
            current.eventPumpMilliseconds,
            current.preSimulationCpuMilliseconds,
            current.frameStartToSimulationMilliseconds);
        ImGui::Text(
            "Current UI/window tags: resize %s | ImGui recreation %s | minimize/restore %s/%s | focus lost/regained %s/%s",
            current.blockingEvents.viewportResized ? "yes" : "no",
            current.blockingEvents.imguiBackendRecreated ? "yes" : "no",
            current.blockingEvents.windowMinimized ? "yes" : "no",
            current.blockingEvents.windowRestored ? "yes" : "no",
            current.blockingEvents.focusLost ? "yes" : "no",
            current.blockingEvents.focusRegained ? "yes" : "no");
        ImGui::Text(
            "Current work tags: track mutation %s | hardware reload %s | modal/file dialog %s",
            current.blockingEvents.trackBufferMutation ? "yes" : "no",
            current.blockingEvents.hardwareAssetReload ? "yes" : "no",
            current.blockingEvents.modalOrFileDialog ? "yes" : "no");

        editorHeading("Previous-frame blocking summary", fonts_);
        if (previous != nullptr)
        {
            ImGui::Text(
                "Frame %llu: fence %.3f | acquire %.3f | present %.3f | drawFrame %.3f ms",
                static_cast<unsigned long long>(previous->frameId),
                previous->previewFrameSlotWaitMilliseconds,
                previous->acquireCallMilliseconds,
                previous->presentCallMilliseconds,
                previous->drawFrameCpuMilliseconds);
            ImGui::Text(
                "Pre-simulation: events %.3f | after events %.3f | total %.3f ms",
                previous->eventPumpMilliseconds,
                previous->preSimulationCpuMilliseconds,
                previous->frameStartToSimulationMilliseconds);
            ImGui::Text(
                "Known tags: swapchain recreation %s | synchronous readback %s",
                previous->swapchainRecreated ? "yes" : "no",
                previous->synchronousReadback ? "yes" : "no");
            ImGui::Text(
                "UI/window tags: viewport resize %s | ImGui recreation %s | minimize/restore %s/%s | focus lost/regained %s/%s",
                previous->blockingEvents.viewportResized ? "yes" : "no",
                previous->blockingEvents.imguiBackendRecreated ? "yes" : "no",
                previous->blockingEvents.windowMinimized ? "yes" : "no",
                previous->blockingEvents.windowRestored ? "yes" : "no",
                previous->blockingEvents.focusLost ? "yes" : "no",
                previous->blockingEvents.focusRegained ? "yes" : "no");
            ImGui::Text(
                "Work tags: track mutation %s | hardware reload %s | modal/file dialog %s",
                previous->blockingEvents.trackBufferMutation ? "yes" : "no",
                previous->blockingEvents.hardwareAssetReload ? "yes" : "no",
                previous->blockingEvents.modalOrFileDialog ? "yes" : "no");
        }
        else
        {
            ImGui::TextDisabled("No preceding completed frame yet.");
        }
        ImGui::TextDisabled(
            "Frame N input follows the prior ImGui interval; current pre-simulation work before NewFrame may also contribute.");
        ImGui::TextDisabled(
            "Unavailable: independent retirement/upload/readback subphase durations; their enclosing CPU phase or tag is shown.");

        ImGui::Separator();
        ImGui::Text(
            "Physics: %zu step(s) @ 240 Hz | %.3f ms CPU",
            current.fixedPhysicsStepCount,
            current.physicsMilliseconds);
        ImGui::Text(
            "Interpolation: %.3f ms | render pose %.3f ms | %zu solve(s) | %zu failure(s)",
            current.interpolationMilliseconds,
            current.renderPoseSolveMilliseconds,
            current.renderPoseSolveCount,
            current.renderPoseFailureCount);
        ImGui::Text(
            "Preview path + slot wait: %.3f ms | updated: %s",
            previewMilliseconds,
            current.previewStreamUpdated ? "yes" : "no");
        ImGui::TextDisabled(
            "  vertices %.3f | publish %.3f | slot upload %.3f | frame/preview slot wait %.3f ms",
            current.previewVertexPreparationMilliseconds,
            current.previewVertexPublishMilliseconds,
            current.previewFrameSlotUpdateMilliseconds,
            current.previewFrameSlotWaitMilliseconds);
        ImGui::Text(
            "drawFrame CPU: %.3f ms",
            current.drawFrameCpuMilliseconds);
        ImGui::Text(
            "Acquire call: %.3f ms | Present call: %.3f ms",
            current.acquireCallMilliseconds,
            current.presentCallMilliseconds);
        ImGui::TextDisabled(
            "Present mode: FIFO (VSync). Acquire/present are CPU call times, not display latency.");

        struct PlotData
        {
            enum class Metric
            {
                RawDelta,
                ExecutedSteps,
                MaximumStep,
                PreSimulation
            };

            const FramePerformanceSample* samples = nullptr;
            std::size_t capacity = 0;
            std::size_t oldestIndex = 0;
            Metric metric = Metric::RawDelta;
        };
        const auto plotValue = [](void* data, const int offset) -> float
        {
            const auto& plot = *static_cast<const PlotData*>(data);
            const FramePerformanceSample& sample = plot.samples[
                (plot.oldestIndex + static_cast<std::size_t>(offset))
                    % plot.capacity];
            switch (plot.metric)
            {
            case PlotData::Metric::RawDelta:
                return static_cast<float>(
                    sample.rawSimulationDeltaMilliseconds);
            case PlotData::Metric::ExecutedSteps:
                return static_cast<float>(sample.fixedPhysicsStepCount);
            case PlotData::Metric::MaximumStep:
                return static_cast<float>(
                    sample.maximumPhysicsStepMilliseconds);
            case PlotData::Metric::PreSimulation:
                return static_cast<float>(
                    sample.frameStartToSimulationMilliseconds);
            }
            return 0.0F;
        };

        PlotData plot{
            performanceHistory_.data(),
            performanceHistoryCapacity,
            oldestIndex,
            PlotData::Metric::RawDelta};
        const ImVec2 plotSize{360.0F * editorPresentationScale(), 48.0F};
        editorHeading("Rolling ~2 seconds", fonts_);
        ImGui::PlotLines(
            "Raw delta ms",
            plotValue,
            &plot,
            static_cast<int>(displayedCount),
            0,
            nullptr,
            0.0F,
            FLT_MAX,
            plotSize);
        plot.metric = PlotData::Metric::ExecutedSteps;
        ImGui::PlotLines(
            "Executed steps",
            plotValue,
            &plot,
            static_cast<int>(displayedCount),
            0,
            nullptr,
            0.0F,
            FLT_MAX,
            plotSize);
        plot.metric = PlotData::Metric::MaximumStep;
        ImGui::PlotLines(
            "Max physics step ms",
            plotValue,
            &plot,
            static_cast<int>(displayedCount),
            0,
            nullptr,
            0.0F,
            FLT_MAX,
            plotSize);
        plot.metric = PlotData::Metric::PreSimulation;
        ImGui::PlotLines(
            "Pre-simulation ms",
            plotValue,
            &plot,
            static_cast<int>(displayedCount),
            0,
            nullptr,
            0.0F,
            FLT_MAX,
            plotSize);

        ImGui::End();
    }
}
