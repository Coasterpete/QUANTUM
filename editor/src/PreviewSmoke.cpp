#include <quantum/editor/PreviewSmoke.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace
{
    using quantum::editor::FramePerformanceSample;
    using quantum::editor::PreviewSmokeSpikeRecord;

    [[nodiscard]] std::expected<double, std::string> parsePositiveDouble(
        const std::string_view value,
        const std::string_view option)
    {
        double parsed = 0.0;
        const auto result = std::from_chars(
            value.data(), value.data() + value.size(), parsed);
        if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
            || !std::isfinite(parsed) || parsed <= 0.0)
        {
            return std::unexpected(
                std::string(option) + " requires a finite value greater than zero.");
        }
        return parsed;
    }

    [[nodiscard]] std::expected<std::size_t, std::string> parsePositiveSize(
        const std::string_view value,
        const std::string_view option)
    {
        std::size_t parsed = 0;
        const auto result = std::from_chars(
            value.data(), value.data() + value.size(), parsed);
        if (result.ec != std::errc{} || result.ptr != value.data() + value.size()
            || parsed == 0)
        {
            return std::unexpected(
                std::string(option) + " requires a positive integer.");
        }
        return parsed;
    }

    [[nodiscard]] double spikeSeverity(
        const PreviewSmokeSpikeRecord& spike,
        const quantum::editor::PreviewSmokeOptions& options) noexcept
    {
        return std::max(
            std::max(spike.current.frameTimeMilliseconds,
                spike.current.previewFrameSlotWaitMilliseconds)
                / options.spikeFrameMilliseconds,
            static_cast<double>(spike.current.fixedPhysicsStepCount)
                / static_cast<double>(options.spikeStepThreshold));
    }

    [[nodiscard]] std::vector<std::string> blockingTags(
        const FramePerformanceSample& sample)
    {
        std::vector<std::string> tags;
        tags.reserve(11);
        if (sample.swapchainRecreated) tags.emplace_back("swapchain_recreation");
        if (sample.blockingEvents.imguiBackendRecreated)
            tags.emplace_back("imgui_vulkan_backend_recreation");
        if (sample.blockingEvents.viewportResized)
            tags.emplace_back("viewport_resize_or_retirement");
        if (sample.synchronousReadback)
            tags.emplace_back("synchronous_readback");
        if (sample.blockingEvents.trackBufferMutation)
            tags.emplace_back("track_buffer_mutation");
        if (sample.blockingEvents.hardwareAssetReload)
            tags.emplace_back("hardware_reload");
        if (sample.blockingEvents.modalOrFileDialog)
            tags.emplace_back("dialog");
        if (sample.blockingEvents.windowMinimized)
            tags.emplace_back("window_minimized");
        if (sample.blockingEvents.windowRestored)
            tags.emplace_back("window_restored");
        if (sample.blockingEvents.focusLost) tags.emplace_back("focus_lost");
        if (sample.blockingEvents.focusRegained)
            tags.emplace_back("focus_regained");
        return tags;
    }

    [[nodiscard]] nlohmann::json submissionJson(
        const quantum::renderer::FrameSubmissionTelemetry& submission)
    {
        return {
            {"draw_id", submission.drawId},
            {"frame_slot", submission.frameSlot},
            {"swapchain_generation", submission.swapchainGeneration},
            {"image_index", submission.imageIndex},
            {"render_finished_semaphore_image_index", submission.imageIndex},
            {"fence_reset", submission.fenceReset},
            {"submitted", submission.submitted},
            {"present_called", submission.presentCalled},
            {"acquire_result", submission.acquireResult},
            {"reset_result", submission.resetResult},
            {"submit_result", submission.submitResult},
            {"present_result", submission.presentResult},
            {"reset_begin_ms", submission.resetBeginMilliseconds},
            {"reset_end_ms", submission.resetEndMilliseconds},
            {"record_commands_cpu_ms", submission.recordCommandsMilliseconds},
            {"submit_begin_ms", submission.submitBeginMilliseconds},
            {"submit_end_ms", submission.submitEndMilliseconds},
            {"present_begin_ms", submission.presentBeginMilliseconds},
            {"present_end_ms", submission.presentEndMilliseconds}
        };
    }

    [[nodiscard]] nlohmann::json synchronizationJson(
        const quantum::renderer::FrameSynchronizationTelemetry& synchronization)
    {
        return {
            {"current_submission", submissionJson(synchronization.current)},
            {"waited_submission", submissionJson(synchronization.waitedSubmission)},
            {"fence_status_before_wait", synchronization.fenceStatusBeforeWait},
            {"fence_status_call_ms", synchronization.fenceStatusCallMilliseconds},
            {"wait_begin_ms", synchronization.waitBeginMilliseconds},
            {"wait_end_ms", synchronization.waitEndMilliseconds}
        };
    }

    [[nodiscard]] nlohmann::json spikeJson(
        const PreviewSmokeSpikeRecord& spike)
    {
        const FramePerformanceSample& current = spike.current;
        const FramePerformanceSample& previous = spike.previous;
        return {
            {"frame_id", current.frameId},
            {"synchronization", synchronizationJson(current.synchronization)},
            {"raw_incoming_delta_ms", current.rawSimulationDeltaMilliseconds},
            {"frame_interval_ms", current.frameTimeMilliseconds},
            {"accumulator_before_input_ms", current.accumulatorBeforeMilliseconds},
            {"accumulator_after_clamp_ms", current.accumulatorAfterIncomingMilliseconds},
            {"accumulator_remaining_ms", current.accumulatorRemainingMilliseconds},
            {"discarded_wall_time_ms", current.discardedWallTimeMilliseconds},
            {"requested_steps", current.requestedPhysicsStepCount},
            {"executed_steps", current.fixedPhysicsStepCount},
            {"maximum_step_limit_hit", current.maximumPhysicsStepsHit},
            {"physics_step_cpu_ms", {
                {"minimum", current.minimumPhysicsStepMilliseconds},
                {"average", current.averagePhysicsStepMilliseconds},
                {"maximum", current.maximumPhysicsStepMilliseconds}}},
            {"total_physics_cpu_ms", current.physicsMilliseconds},
            {"interpolation_cpu_ms", current.interpolationMilliseconds},
            {"current_pre_simulation_cpu_ms", current.preSimulationCpuMilliseconds},
            {"event_pump_ms", current.eventPumpMilliseconds},
            {"frame_start_to_simulation_ms", current.frameStartToSimulationMilliseconds},
            {"draw_frame_cpu_ms", current.drawFrameCpuMilliseconds},
            {"frame_slot_fence_wait_ms", current.previewFrameSlotWaitMilliseconds},
            {"acquire_cpu_ms", current.acquireCallMilliseconds},
            {"present_cpu_ms", current.presentCallMilliseconds},
            {"previous_frame", spike.hasPrevious ? nlohmann::json{
                {"frame_id", previous.frameId},
                {"synchronization", synchronizationJson(previous.synchronization)},
                {"raw_incoming_delta_ms", previous.rawSimulationDeltaMilliseconds},
                {"executed_steps", previous.fixedPhysicsStepCount},
                {"total_physics_cpu_ms", previous.physicsMilliseconds},
                {"interpolation_cpu_ms", previous.interpolationMilliseconds},
                {"event_pump_ms", previous.eventPumpMilliseconds},
                {"pre_simulation_cpu_ms", previous.preSimulationCpuMilliseconds},
                {"frame_slot_fence_wait_ms", previous.previewFrameSlotWaitMilliseconds},
                {"acquire_cpu_ms", previous.acquireCallMilliseconds},
                {"present_cpu_ms", previous.presentCallMilliseconds},
                {"draw_frame_cpu_ms", previous.drawFrameCpuMilliseconds},
                {"blocking_tags", blockingTags(previous)}} : nlohmann::json(nullptr)},
            {"current_blocking_tags", blockingTags(current)},
            {"next_frame", spike.hasNext ? nlohmann::json{
                {"frame_id", spike.next.frameId},
                {"raw_incoming_delta_ms", spike.next.rawSimulationDeltaMilliseconds},
                {"executed_steps", spike.next.fixedPhysicsStepCount},
                {"total_physics_cpu_ms", spike.next.physicsMilliseconds}}
                : nlohmann::json(nullptr)},
            {"consecutive_catch_up_frames", current.consecutiveCatchUpFrameCount}
        };
    }

    [[nodiscard]] std::filesystem::path defaultOutputBase()
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm local{};
#ifdef _WIN32
        localtime_s(&local, &time);
#else
        localtime_r(&time, &local);
#endif
        std::ostringstream name;
        name << "preview-smoke-" << std::put_time(&local, "%Y%m%d-%H%M%S");
        return std::filesystem::path{"build"} / "diagnostics" / name.str();
    }

    [[nodiscard]] std::filesystem::path outputBase(
        const quantum::editor::PreviewSmokeOptions& options)
    {
        std::filesystem::path base = options.outputBasePath.empty()
            ? defaultOutputBase() : options.outputBasePath;
        if (base.extension() == ".json" || base.extension() == ".txt")
            base.replace_extension();
        return base;
    }
}

namespace quantum::editor
{
    std::expected<std::optional<PreviewSmokeOptions>, std::string>
    parsePreviewSmokeArguments(
        const std::span<const std::string_view> arguments)
    {
        const auto mode = std::find(
            arguments.begin(), arguments.end(), "--dev-preview-smoke");
        if (mode == arguments.end()) return std::optional<PreviewSmokeOptions>{};

        PreviewSmokeOptions options;
        for (std::size_t index = 0; index < arguments.size(); ++index)
        {
            const std::string_view argument = arguments[index];
            if (argument == "--log-level")
            {
                if (++index >= arguments.size())
                    return std::unexpected("--log-level requires a value.");
                continue;
            }
            if (argument.starts_with("--log-level=")) continue;
            if (argument == "--repeat")
            {
                options.repeat = true;
                continue;
            }
            if (argument == "--dev-preview-smoke")
            {
                if (++index >= arguments.size() || arguments[index].starts_with("--"))
                    return std::unexpected(
                        "--dev-preview-smoke requires a .quantum document path.");
                options.documentPath = arguments[index];
                continue;
            }

            const auto requireValue = [&](const std::string_view name)
                -> std::expected<std::string_view, std::string>
            {
                if (index + 1 >= arguments.size())
                    return std::unexpected(std::string(name) + " requires a value.");
                return arguments[++index];
            };

            if (argument == "--duration" || argument == "--spike-frame-ms"
                || argument == "--spike-steps" || argument == "--output")
            {
                const auto value = requireValue(argument);
                if (!value) return std::unexpected(value.error());
                if (argument == "--output")
                {
                    if (value->empty())
                        return std::unexpected("--output requires a non-empty path.");
                    options.outputBasePath = *value;
                }
                else if (argument == "--spike-steps")
                {
                    const auto parsed = parsePositiveSize(*value, argument);
                    if (!parsed) return std::unexpected(parsed.error());
                    options.spikeStepThreshold = *parsed;
                }
                else
                {
                    const auto parsed = parsePositiveDouble(*value, argument);
                    if (!parsed) return std::unexpected(parsed.error());
                    if (argument == "--duration") options.durationSeconds = *parsed;
                    else options.spikeFrameMilliseconds = *parsed;
                }
                continue;
            }
            return std::unexpected(
                "Unknown preview smoke option '" + std::string(argument) + "'.");
        }

        if (options.documentPath.empty())
            return std::unexpected(
                "--dev-preview-smoke requires a .quantum document path.");
        return std::optional<PreviewSmokeOptions>{std::move(options)};
    }

    std::expected<void, std::string> validatePreviewSmokeInput(
        const PreviewSmokeOptions& options)
    {
        if (options.documentPath.extension() != ".quantum")
            return std::unexpected(
                "Preview smoke input must be a .quantum document.");
        if (!std::filesystem::is_regular_file(options.documentPath))
            return std::unexpected(
                "Preview smoke document does not exist: "
                + options.documentPath.string());
        return {};
    }

    PreviewSmokeCollector::PreviewSmokeCollector(
        const PreviewSmokeOptions& options)
        : options_(options)
    {
        frameIntervalsMilliseconds_.reserve(maximumPercentileSamples);
        totals_.documentPath = options.documentPath;
        totals_.requestedDurationSeconds = options.durationSeconds;
#ifdef NDEBUG
        totals_.buildConfiguration = "Release";
#else
        totals_.buildConfiguration = "Debug";
#endif
    }

    void PreviewSmokeCollector::record(
        const FramePerformanceSample& sample) noexcept
    {
        for (std::size_t index = 0; index < spikeCount_; ++index)
        {
            if (!spikes_[index].hasNext)
            {
                spikes_[index].next = sample;
                spikes_[index].hasNext = true;
            }
        }
        ++totals_.renderedFrameCount;
        totals_.simulationFixedStepCount += sample.fixedPhysicsStepCount;
        if (sample.frameTimeMilliseconds > 0.0)
        {
            if (frameIntervalsMilliseconds_.size() < maximumPercentileSamples)
                frameIntervalsMilliseconds_.push_back(sample.frameTimeMilliseconds);
            else
                ++totals_.percentileSamplesDropped;
            totals_.averageFrameMilliseconds += sample.frameTimeMilliseconds;
            totals_.maximumFrameMilliseconds = std::max(
                totals_.maximumFrameMilliseconds, sample.frameTimeMilliseconds);
        }

        totals_.maximumStepsInRenderedFrame = std::max(
            totals_.maximumStepsInRenderedFrame, sample.fixedPhysicsStepCount);
        totals_.averagePhysicsMillisecondsPerFrame += sample.physicsMilliseconds;
        totals_.maximumPhysicsMillisecondsPerFrame = std::max(
            totals_.maximumPhysicsMillisecondsPerFrame, sample.physicsMilliseconds);
        totals_.maximumFixedStepMilliseconds = std::max(
            totals_.maximumFixedStepMilliseconds,
            sample.maximumPhysicsStepMilliseconds);
        totals_.averageFixedStepMilliseconds +=
            sample.averagePhysicsStepMilliseconds
            * static_cast<double>(sample.fixedPhysicsStepCount);
        if (sample.maximumPhysicsStepsHit) ++totals_.maximumStepsHitFrameCount;
        totals_.averageInterpolationMilliseconds += sample.interpolationMilliseconds;
        totals_.maximumInterpolationMilliseconds = std::max(
            totals_.maximumInterpolationMilliseconds,
            sample.interpolationMilliseconds);
        totals_.interpolationSolveFailureCount += sample.renderPoseFailureCount;

        const double previewUpload = sample.previewVertexPreparationMilliseconds
            + sample.previewVertexPublishMilliseconds
            + sample.previewFrameSlotUpdateMilliseconds;
        totals_.averagePreviewUploadMilliseconds += previewUpload;
        totals_.maximumPreviewUploadMilliseconds = std::max(
            totals_.maximumPreviewUploadMilliseconds, previewUpload);
        totals_.averagePreviewSlotWaitMilliseconds +=
            sample.previewFrameSlotWaitMilliseconds;
        totals_.maximumPreviewSlotWaitMilliseconds = std::max(
            totals_.maximumPreviewSlotWaitMilliseconds,
            sample.previewFrameSlotWaitMilliseconds);
        totals_.averageDrawFrameMilliseconds += sample.drawFrameCpuMilliseconds;
        totals_.maximumDrawFrameMilliseconds = std::max(
            totals_.maximumDrawFrameMilliseconds, sample.drawFrameCpuMilliseconds);
        totals_.averageFrameSlotFenceWaitMilliseconds +=
            sample.previewFrameSlotWaitMilliseconds;
        totals_.maximumFrameSlotFenceWaitMilliseconds = std::max(
            totals_.maximumFrameSlotFenceWaitMilliseconds,
            sample.previewFrameSlotWaitMilliseconds);
        totals_.averageAcquireMilliseconds += sample.acquireCallMilliseconds;
        totals_.maximumAcquireMilliseconds = std::max(
            totals_.maximumAcquireMilliseconds, sample.acquireCallMilliseconds);
        totals_.averagePresentMilliseconds += sample.presentCallMilliseconds;
        totals_.maximumPresentMilliseconds = std::max(
            totals_.maximumPresentMilliseconds, sample.presentCallMilliseconds);

        totals_.largestRawDeltaMilliseconds = std::max(
            totals_.largestRawDeltaMilliseconds,
            sample.rawSimulationDeltaMilliseconds);
        totals_.averageRawDeltaMilliseconds += sample.rawSimulationDeltaMilliseconds;
        totals_.maximumRequestedStepCount = std::max(
            totals_.maximumRequestedStepCount, sample.requestedPhysicsStepCount);
        totals_.maximumExecutedStepCount = std::max(
            totals_.maximumExecutedStepCount, sample.fixedPhysicsStepCount);
        totals_.maximumConsecutiveCatchUpFrameCount = std::max(
            totals_.maximumConsecutiveCatchUpFrameCount,
            sample.consecutiveCatchUpFrameCount);
        totals_.totalDiscardedWallTimeMilliseconds +=
            sample.discardedWallTimeMilliseconds;
        if (sample.discardedWallTimeMilliseconds > 0.0)
            ++totals_.discardedWallTimeFrameCount;

        if (sample.frameTimeMilliseconds >= options_.spikeFrameMilliseconds
            || sample.previewFrameSlotWaitMilliseconds >= options_.spikeFrameMilliseconds
            || sample.fixedPhysicsStepCount >= options_.spikeStepThreshold)
        {
            PreviewSmokeSpikeRecord candidate;
            candidate.current = sample;
            candidate.hasPrevious = previousSample_.has_value();
            if (candidate.hasPrevious) candidate.previous = *previousSample_;
            if (spikeCount_ < spikes_.size())
                spikes_[spikeCount_++] = candidate;
            else
            {
                auto weakest = spikes_.begin();
                for (auto it = spikes_.begin() + 1; it != spikes_.end(); ++it)
                    if (spikeSeverity(*it, options_) < spikeSeverity(*weakest, options_))
                        weakest = it;
                if (spikeSeverity(candidate, options_) > spikeSeverity(*weakest, options_))
                    *weakest = candidate;
            }
        }
        previousSample_ = sample;
    }

    double PreviewSmokeCollector::percentile(
        const std::span<const double> values,
        const double percentileFraction)
    {
        if (values.empty()) return 0.0;
        std::vector<double> sorted(values.begin(), values.end());
        std::sort(sorted.begin(), sorted.end());
        const double clamped = std::clamp(percentileFraction, 0.0, 1.0);
        const std::size_t rank = static_cast<std::size_t>(
            std::ceil(clamped * static_cast<double>(sorted.size())));
        return sorted[std::max<std::size_t>(rank, 1) - 1];
    }

    PreviewSmokeReport PreviewSmokeCollector::finish(
        const double actualElapsedSeconds,
        const bool playbackCompletedNormally,
        const bool previewOrPhysicsFailure,
        std::string failureMessage) const
    {
        PreviewSmokeReport report = totals_;
        report.actualElapsedSeconds = actualElapsedSeconds;
        report.playbackCompletedNormally = playbackCompletedNormally;
        report.previewOrPhysicsFailure = previewOrPhysicsFailure;
        report.failureMessage = std::move(failureMessage);
        const double frames = static_cast<double>(report.renderedFrameCount);
        const double intervals = static_cast<double>(
            frameIntervalsMilliseconds_.size() + report.percentileSamplesDropped);
        if (intervals > 0.0)
        {
            report.averageFrameMilliseconds /= intervals;
            report.averageFramesPerSecond = report.averageFrameMilliseconds > 0.0
                ? 1000.0 / report.averageFrameMilliseconds : 0.0;
            report.minimumFramesPerSecond = report.maximumFrameMilliseconds > 0.0
                ? 1000.0 / report.maximumFrameMilliseconds : 0.0;
        }
        report.p95FrameMilliseconds = percentile(
            frameIntervalsMilliseconds_, 0.95);
        report.p99FrameMilliseconds = percentile(
            frameIntervalsMilliseconds_, 0.99);
        if (frames > 0.0)
        {
            report.averageStepsPerRenderedFrame =
                static_cast<double>(report.simulationFixedStepCount) / frames;
            report.averagePhysicsMillisecondsPerFrame /= frames;
            report.averageInterpolationMilliseconds /= frames;
            report.averagePreviewUploadMilliseconds /= frames;
            report.averagePreviewSlotWaitMilliseconds /= frames;
            report.averageDrawFrameMilliseconds /= frames;
            report.averageFrameSlotFenceWaitMilliseconds /= frames;
            report.averageAcquireMilliseconds /= frames;
            report.averagePresentMilliseconds /= frames;
            report.averageRawDeltaMilliseconds /= frames;
        }
        report.averageFixedStepMilliseconds = report.simulationFixedStepCount > 0
            ? totals_.averageFixedStepMilliseconds
                / static_cast<double>(report.simulationFixedStepCount)
            : 0.0;
        report.spikes.assign(spikes_.begin(), spikes_.begin() + spikeCount_);
        std::sort(report.spikes.begin(), report.spikes.end(),
            [&](const auto& left, const auto& right)
            {
                const double leftSeverity = spikeSeverity(left, options_);
                const double rightSeverity = spikeSeverity(right, options_);
                return leftSeverity != rightSeverity
                    ? leftSeverity > rightSeverity
                    : left.current.frameId < right.current.frameId;
            });
        return report;
    }

    PreviewSmokeReportPaths writePreviewSmokeReports(
        const PreviewSmokeReport& report,
        const PreviewSmokeOptions& options)
    {
        const std::filesystem::path base = outputBase(options);
        PreviewSmokeReportPaths paths{base.string() + ".json", base.string() + ".txt"};
        if (!base.parent_path().empty())
            std::filesystem::create_directories(base.parent_path());

        nlohmann::json spikes = nlohmann::json::array();
        for (const auto& spike : report.spikes) spikes.push_back(spikeJson(spike));
        const nlohmann::json json = {
            {"run", {
                {"document_path", report.documentPath.string()},
                {"build_configuration", report.buildConfiguration},
                {"requested_duration_seconds", report.requestedDurationSeconds},
                {"actual_elapsed_wall_seconds", report.actualElapsedSeconds},
                {"rendered_frame_count", report.renderedFrameCount},
                {"simulation_fixed_step_count", report.simulationFixedStepCount},
                {"playback_completed_normally", report.playbackCompletedNormally},
                {"preview_or_physics_failure", report.previewOrPhysicsFailure},
                {"failure_message", report.failureMessage}}},
            {"frame_performance", {
                {"average_fps", report.averageFramesPerSecond},
                {"average_frame_ms", report.averageFrameMilliseconds},
                {"maximum_frame_ms", report.maximumFrameMilliseconds},
                {"p95_frame_ms", report.p95FrameMilliseconds},
                {"p99_frame_ms", report.p99FrameMilliseconds},
                {"minimum_fps", report.minimumFramesPerSecond},
                {"percentile_samples_dropped", report.percentileSamplesDropped}}},
            {"physics", {
                {"total_steps", report.simulationFixedStepCount},
                {"average_steps_per_rendered_frame", report.averageStepsPerRenderedFrame},
                {"maximum_steps_in_rendered_frame", report.maximumStepsInRenderedFrame},
                {"average_cpu_ms_per_frame", report.averagePhysicsMillisecondsPerFrame},
                {"maximum_cpu_ms_per_frame", report.maximumPhysicsMillisecondsPerFrame},
                {"average_fixed_step_cpu_ms", report.averageFixedStepMilliseconds},
                {"maximum_fixed_step_cpu_ms", report.maximumFixedStepMilliseconds},
                {"maximum_steps_hit_frame_count", report.maximumStepsHitFrameCount}}},
            {"interpolation", {
                {"average_cpu_ms", report.averageInterpolationMilliseconds},
                {"maximum_cpu_ms", report.maximumInterpolationMilliseconds},
                {"solve_failure_count", report.interpolationSolveFailureCount}}},
            {"preview_renderer", {
                {"average_preview_upload_ms", report.averagePreviewUploadMilliseconds},
                {"maximum_preview_upload_ms", report.maximumPreviewUploadMilliseconds},
                {"average_preview_slot_wait_ms", report.averagePreviewSlotWaitMilliseconds},
                {"maximum_preview_slot_wait_ms", report.maximumPreviewSlotWaitMilliseconds},
                {"average_draw_frame_cpu_ms", report.averageDrawFrameMilliseconds},
                {"maximum_draw_frame_cpu_ms", report.maximumDrawFrameMilliseconds},
                {"average_frame_slot_fence_wait_ms", report.averageFrameSlotFenceWaitMilliseconds},
                {"maximum_frame_slot_fence_wait_ms", report.maximumFrameSlotFenceWaitMilliseconds},
                {"average_acquire_cpu_ms", report.averageAcquireMilliseconds},
                {"maximum_acquire_cpu_ms", report.maximumAcquireMilliseconds},
                {"average_present_cpu_ms", report.averagePresentMilliseconds},
                {"maximum_present_cpu_ms", report.maximumPresentMilliseconds}}},
            {"catch_up_raw_delta", {
                {"largest_raw_incoming_delta_ms", report.largestRawDeltaMilliseconds},
                {"average_raw_delta_ms", report.averageRawDeltaMilliseconds},
                {"maximum_requested_step_count", report.maximumRequestedStepCount},
                {"maximum_executed_step_count", report.maximumExecutedStepCount},
                {"maximum_consecutive_catch_up_frame_count", report.maximumConsecutiveCatchUpFrameCount},
                {"total_discarded_wall_time_ms", report.totalDiscardedWallTimeMilliseconds},
                {"discarded_wall_time_frame_count", report.discardedWallTimeFrameCount}}},
            {"spikes", std::move(spikes)}
        };

        std::ofstream jsonOutput(paths.json, std::ios::binary);
        if (!jsonOutput || !(jsonOutput << std::setw(2) << json << '\n'))
            throw std::runtime_error(
                "Could not write preview smoke JSON report: " + paths.json.string());

        std::ofstream textOutput(paths.text, std::ios::binary);
        textOutput << std::fixed << std::setprecision(3)
            << "QUANTUM Developer Preview Smoke Test\n"
            << "Result: " << (report.previewOrPhysicsFailure
                || !report.playbackCompletedNormally ? "FAILED" : "PASSED") << '\n'
            << "Document: " << report.documentPath.string() << '\n'
            << "Build: " << report.buildConfiguration << '\n'
            << "Duration: " << report.actualElapsedSeconds << " s / "
            << report.requestedDurationSeconds << " s requested\n"
            << "Frames / fixed steps: " << report.renderedFrameCount << " / "
            << report.simulationFixedStepCount << '\n'
            << "FPS avg/min: " << report.averageFramesPerSecond << " / "
            << report.minimumFramesPerSecond << '\n'
            << "Frame ms avg/max/p95/p99: " << report.averageFrameMilliseconds
            << " / " << report.maximumFrameMilliseconds << " / "
            << report.p95FrameMilliseconds << " / " << report.p99FrameMilliseconds << '\n'
            << "Physics ms/frame avg/max: "
            << report.averagePhysicsMillisecondsPerFrame << " / "
            << report.maximumPhysicsMillisecondsPerFrame << '\n'
            << "Physics step ms avg/max: " << report.averageFixedStepMilliseconds
            << " / " << report.maximumFixedStepMilliseconds << '\n'
            << "Catch-up requested/executed max: "
            << report.maximumRequestedStepCount << " / "
            << report.maximumExecutedStepCount << '\n'
            << "Discarded wall time: " << report.totalDiscardedWallTimeMilliseconds
            << " ms across " << report.discardedWallTimeFrameCount << " frame(s)\n"
            << "Retained spikes: " << report.spikes.size() << '\n';
        if (!report.failureMessage.empty())
            textOutput << "Failure: " << report.failureMessage << '\n';
        for (const auto& spike : report.spikes)
            textOutput << "  frame " << spike.current.frameId << ": frame "
                << spike.current.frameTimeMilliseconds << " ms, raw delta "
                << spike.current.rawSimulationDeltaMilliseconds << " ms, steps "
                << spike.current.requestedPhysicsStepCount << "/"
                << spike.current.fixedPhysicsStepCount
                << ", fence " << spike.current.previewFrameSlotWaitMilliseconds
                << " ms (slot " << spike.current.synchronization.current.frameSlot
                << ", submission "
                << spike.current.synchronization.waitedSubmission.drawId
                << ")\n";
        if (!textOutput)
            throw std::runtime_error(
                "Could not write preview smoke text report: " + paths.text.string());
        return paths;
    }
}
