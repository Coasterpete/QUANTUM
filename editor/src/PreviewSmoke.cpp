#include <quantum/editor/PreviewSmoke.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace
{
    using quantum::editor::FramePerformanceSample;
    using quantum::editor::PreviewSmokeSpikeRecord;
    using quantum::editor::PreviewSmokeTimeSlice;

    constexpr double previewSmokeTimeSliceMilliseconds = 5'000.0;

    void addSolveCounters(
        quantum::physics::TrainSolveCounters& destination,
        const quantum::physics::TrainSolveCounters& source) noexcept
    {
        destination.solveTrainPoseCalls += source.solveTrainPoseCalls;
        destination.solveCarGeometryCalls += source.solveCarGeometryCalls;
        destination.rigidBogieSolveCalls += source.rigidBogieSolveCalls;
        destination.rigidBogieRefinementIterations +=
            source.rigidBogieRefinementIterations;
        destination.rigidBogieBracketExpansions +=
            source.rigidBogieBracketExpansions;
        destination.connectionCandidateEvaluations +=
            source.connectionCandidateEvaluations;
        destination.connectorSolveCalls += source.connectorSolveCalls;
        destination.connectorRefinementIterations +=
            source.connectorRefinementIterations;
        destination.connectorFallbackUses += source.connectorFallbackUses;
        destination.trackSampleCalls += source.trackSampleCalls;
        destination.intervalHintMisses += source.intervalHintMisses;
        destination.openBoundaryPoseAttempts +=
            source.openBoundaryPoseAttempts;
        destination.openBoundaryPoseFailures +=
            source.openBoundaryPoseFailures;
        destination.openBoundaryRefinementIterations +=
            source.openBoundaryRefinementIterations;
        destination.solveTrainPoseNanoseconds +=
            source.solveTrainPoseNanoseconds;
        destination.solveCarGeometryNanoseconds +=
            source.solveCarGeometryNanoseconds;
        destination.rigidBogieSolveNanoseconds +=
            source.rigidBogieSolveNanoseconds;
        destination.connectionCandidateNanoseconds +=
            source.connectionCandidateNanoseconds;
        destination.trackSampleNanoseconds += source.trackSampleNanoseconds;
    }

    [[nodiscard]] double ratio(
        const double numerator, const std::uint64_t denominator) noexcept
    {
        return denominator > 0
            ? numerator / static_cast<double>(denominator)
            : 0.0;
    }

    [[nodiscard]] nlohmann::json timeSliceJson(
        const PreviewSmokeTimeSlice& slice)
    {
        const double elapsedSeconds = slice.frameMilliseconds / 1'000.0;
        return {
            {"begin_seconds", slice.beginSeconds},
            {"end_seconds", slice.endSeconds},
            {"rendered_frames", slice.renderedFrameCount},
            {"fixed_steps", slice.fixedStepCount},
            {"average_fps", elapsedSeconds > 0.0
                ? static_cast<double>(slice.renderedFrameCount) / elapsedSeconds
                : 0.0},
            {"average_steps_per_frame", ratio(
                static_cast<double>(slice.fixedStepCount),
                slice.renderedFrameCount)},
            {"average_fixed_step_cpu_ms", ratio(
                slice.fixedStepMilliseconds, slice.fixedStepCount)},
            {"maximum_fixed_step_cpu_ms", slice.maximumFixedStepMilliseconds},
            {"average_physics_cpu_ms_per_frame", ratio(
                slice.physicsMilliseconds, slice.renderedFrameCount)},
            {"average_gpu_execution_ms", ratio(
                slice.gpuExecutionMilliseconds, slice.gpuTimingSampleCount)},
            {"solver_per_fixed_step", {
                {"solve_train_pose_calls", ratio(
                    static_cast<double>(slice.solverCounters.solveTrainPoseCalls),
                    slice.fixedStepCount)},
                {"rigid_bogie_solve_calls", ratio(
                    static_cast<double>(slice.solverCounters.rigidBogieSolveCalls),
                    slice.fixedStepCount)},
                {"rigid_bogie_refinement_iterations", ratio(
                    static_cast<double>(slice.solverCounters
                        .rigidBogieRefinementIterations),
                    slice.fixedStepCount)},
                {"connection_candidate_evaluations", ratio(
                    static_cast<double>(slice.solverCounters
                        .connectionCandidateEvaluations),
                    slice.fixedStepCount)},
                {"connector_refinement_iterations", ratio(
                    static_cast<double>(slice.solverCounters
                        .connectorRefinementIterations),
                    slice.fixedStepCount)},
                {"track_sample_calls", ratio(
                    static_cast<double>(slice.solverCounters.trackSampleCalls),
                    slice.fixedStepCount)},
                {"interval_hint_misses", ratio(
                    static_cast<double>(slice.solverCounters.intervalHintMisses),
                    slice.fixedStepCount)}}},
            {"solver_average_cpu_ms_per_call", {
                {"solve_train_pose", ratio(
                    static_cast<double>(slice.solverCounters
                        .solveTrainPoseNanoseconds) / 1'000'000.0,
                    slice.solverCounters.solveTrainPoseCalls)},
                {"solve_car_geometry", ratio(
                    static_cast<double>(slice.solverCounters
                        .solveCarGeometryNanoseconds) / 1'000'000.0,
                    slice.solverCounters.solveCarGeometryCalls)},
                {"rigid_bogie_solve", ratio(
                    static_cast<double>(slice.solverCounters
                        .rigidBogieSolveNanoseconds) / 1'000'000.0,
                    slice.solverCounters.rigidBogieSolveCalls)},
                {"connection_candidate", ratio(
                    static_cast<double>(slice.solverCounters
                        .connectionCandidateNanoseconds) / 1'000'000.0,
                    slice.solverCounters.connectionCandidateEvaluations)},
                {"track_sample", ratio(
                    static_cast<double>(slice.solverCounters
                        .trackSampleNanoseconds) / 1'000'000.0,
                    slice.solverCounters.trackSampleCalls)}}}
        };
    }

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
            {"frames_in_flight", synchronization.framesInFlight},
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
            {"main_thread_frame_ms", current.mainThreadFrameMilliseconds},
            {"gpu_execution_ms", current.gpuTimingAvailable
                ? nlohmann::json(current.gpuExecutionMilliseconds)
                : nlohmann::json(nullptr)},
            {"accumulator_before_input_ms", current.accumulatorBeforeMilliseconds},
            {"accumulator_after_clamp_ms", current.accumulatorAfterIncomingMilliseconds},
            {"accumulator_remaining_ms", current.accumulatorRemainingMilliseconds},
            {"discarded_wall_time_ms", current.discardedWallTimeMilliseconds},
            {"requested_steps", current.requestedPhysicsStepCount},
            {"executed_steps", current.fixedPhysicsStepCount},
            {"simulation_tick", {
                {"begin", current.simulationStartingTick},
                {"end", current.simulationEndingTick}}},
            {"station_m", {
                {"begin", current.simulationStartingStationMeters},
                {"end", current.simulationEndingStationMeters}}},
            {"signed_velocity_mps", {
                {"begin", current.startingSignedVelocityMetersPerSecond},
                {"end", current.endingSignedVelocityMetersPerSecond},
                {"minimum_absolute",
                    current.minimumAbsoluteVelocityMetersPerSecond}}},
            {"zero_velocity_steps", current.zeroVelocityStepCount},
            {"zero_speed_transitions", current.zeroSpeedTransitionCount},
            {"rollback_starts", current.rollbackStartCount},
            {"rollback_steps", current.rollbackStepCount},
            {"boundary_stopped", current.boundaryStopped},
            {"maximum_step_limit_hit", current.maximumPhysicsStepsHit},
            {"physics_step_cpu_ms", {
                {"minimum", current.minimumPhysicsStepMilliseconds},
                {"average", current.averagePhysicsStepMilliseconds},
                {"maximum", current.maximumPhysicsStepMilliseconds}}},
            {"total_physics_cpu_ms", current.physicsMilliseconds},
            {"gpu_preview_sampling_cpu_ms",
                current.gpuPreviewSamplingMilliseconds},
            {"gpu_preview_validation_cpu_ms",
                current.gpuPreviewValidationMilliseconds},
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

    [[nodiscard]] nlohmann::json frameTraceJson(
        const FramePerformanceSample& sample)
    {
        const auto& counters = sample.solverCounters;
        return {
            {"frame_id", sample.frameId},
            {"simulation_tick_begin", sample.simulationStartingTick},
            {"simulation_tick_end", sample.simulationEndingTick},
            {"station_begin_m", sample.simulationStartingStationMeters},
            {"station_end_m", sample.simulationEndingStationMeters},
            {"signed_velocity_begin_mps",
                sample.startingSignedVelocityMetersPerSecond},
            {"signed_velocity_end_mps",
                sample.endingSignedVelocityMetersPerSecond},
            {"minimum_absolute_velocity_mps",
                sample.minimumAbsoluteVelocityMetersPerSecond},
            {"zero_velocity_steps", sample.zeroVelocityStepCount},
            {"zero_speed_transitions", sample.zeroSpeedTransitionCount},
            {"rollback_starts", sample.rollbackStartCount},
            {"rollback_steps", sample.rollbackStepCount},
            {"boundary_stopped", sample.boundaryStopped},
            {"raw_delta_ms", sample.rawSimulationDeltaMilliseconds},
            {"frame_interval_ms", sample.frameTimeMilliseconds},
            {"requested_steps", sample.requestedPhysicsStepCount},
            {"executed_steps", sample.fixedPhysicsStepCount},
            {"fixed_step_cpu_ms", {
                {"minimum", sample.minimumPhysicsStepMilliseconds},
                {"average", sample.averagePhysicsStepMilliseconds},
                {"maximum", sample.maximumPhysicsStepMilliseconds}}},
            {"physics_and_gpu_preview_cpu_ms", sample.physicsMilliseconds},
            {"event_pump_cpu_ms", sample.eventPumpMilliseconds},
            {"pre_simulation_cpu_ms", sample.preSimulationCpuMilliseconds},
            {"interpolation_cpu_ms", sample.interpolationMilliseconds},
            {"render_pose_solve_cpu_ms", sample.renderPoseSolveMilliseconds},
            {"render_pose_solves", sample.renderPoseSolveCount},
            {"gpu_preview", {
                {"sampling_cpu_ms", sample.gpuPreviewSamplingMilliseconds},
                {"preparation_cpu_ms",
                    sample.gpuPreviewPreparationMilliseconds},
                {"command_recording_cpu_ms",
                    sample.gpuPreviewCommandRecordingMilliseconds},
                {"queue_submit_cpu_ms",
                    sample.gpuPreviewQueueSubmitMilliseconds},
                {"fence_wait_cpu_ms",
                    sample.gpuPreviewFenceWaitMilliseconds},
                {"readback_cpu_ms",
                    sample.gpuPreviewReadbackMilliseconds},
                {"validation_cpu_ms", sample.gpuPreviewValidationMilliseconds},
                {"queries", sample.gpuPreviewQueryCount},
                {"dispatches", sample.gpuPreviewDispatchCount},
                {"fallbacks", sample.gpuPreviewFallbackCount}}},
            {"renderer", {
                {"synchronization", synchronizationJson(sample.synchronization)},
                {"draw_frame_cpu_ms", sample.drawFrameCpuMilliseconds},
                {"preview_slot_update_cpu_ms", sample.previewFrameSlotUpdateMilliseconds},
                {"gpu_execution_ms", sample.gpuTimingAvailable
                    ? nlohmann::json(sample.gpuExecutionMilliseconds)
                    : nlohmann::json(nullptr)},
                {"fence_wait_ms", sample.previewFrameSlotWaitMilliseconds},
                {"deferred_buffer_reclaim_cpu_ms",
                    sample.deferredBufferReclaimMilliseconds},
                {"acquire_cpu_ms", sample.acquireCallMilliseconds},
                {"present_cpu_ms", sample.presentCallMilliseconds},
                {"deferred_buffer_count_before_reclaim",
                    sample.deferredBufferCountBeforeReclaim},
                {"deferred_buffer_bytes_before_reclaim",
                    sample.deferredBufferBytesBeforeReclaim},
                {"reclaimed_buffer_count", sample.reclaimedBufferCount}}},
            {"solver", {
                {"solve_train_pose_calls", counters.solveTrainPoseCalls},
                {"rigid_bogie_solve_calls", counters.rigidBogieSolveCalls},
                {"rigid_bogie_refinement_iterations",
                    counters.rigidBogieRefinementIterations},
                {"connection_candidate_evaluations",
                    counters.connectionCandidateEvaluations},
                {"connector_refinement_iterations",
                    counters.connectorRefinementIterations},
                {"connector_fallback_uses", counters.connectorFallbackUses},
                {"track_sample_calls", counters.trackSampleCalls},
                {"open_boundary_pose_attempts",
                    counters.openBoundaryPoseAttempts},
                {"open_boundary_pose_failures",
                    counters.openBoundaryPoseFailures},
                {"open_boundary_refinement_iterations",
                    counters.openBoundaryRefinementIterations}}}
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
            if (argument == "--stopped-preview")
            {
                options.stoppedPreview = true;
                continue;
            }
            if (argument == "--simulator")
            {
                options.simulator = true;
                continue;
            }
            if (argument == "--mode-cycle")
            {
                options.modeCycle = true;
                continue;
            }
            if (argument == "--camera-orbit")
            {
                options.cameraOrbit = true;
                continue;
            }
            if (argument == "--msaa-off")
            {
                options.msaaOff = true;
                continue;
            }
            if (argument == "--transition-drag")
            {
                options.transitionDrag = true;
                continue;
            }
            if (argument == "--resize-window")
            {
                options.resizeWindow = true;
                continue;
            }
            if (argument == "--disable-gpu-preview-sampling")
            {
                options.disableGpuPreviewSampling = true;
                continue;
            }
            if (argument == "--enable-gpu-validation")
            {
                options.enableGpuValidation = true;
                continue;
            }
            if (argument == "--frame-trace")
            {
                options.captureFrameTrace = true;
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
                || argument == "--spike-steps" || argument == "--output"
                || argument == "--region-style-edit")
            {
                const auto value = requireValue(argument);
                if (!value) return std::unexpected(value.error());
                if (argument == "--region-style-edit")
                {
                    if (*value == "hardware-spacing")
                    {
                        options.regionStyleEdit =
                            PreviewSmokeRegionStyleEdit::HardwareSpacing;
                    }
                    else if (*value == "rail-material")
                    {
                        options.regionStyleEdit =
                            PreviewSmokeRegionStyleEdit::RailMaterial;
                    }
                    else if (*value == "rail-spacing")
                    {
                        options.regionStyleEdit =
                            PreviewSmokeRegionStyleEdit::RailCenterSpacing;
                    }
                    else
                    {
                        return std::unexpected(
                            "--region-style-edit requires hardware-spacing, "
                            "rail-material, or rail-spacing.");
                    }
                }
                else if (argument == "--output")
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
        if (options_.captureFrameTrace)
        {
            frameTrace_.push_back(sample);
        }
        for (std::size_t index = 0; index < spikeCount_; ++index)
        {
            if (!spikes_[index].hasNext)
            {
                spikes_[index].next = sample;
                spikes_[index].hasNext = true;
            }
        }

        ++currentTimeSlice_.renderedFrameCount;
        currentTimeSlice_.fixedStepCount += sample.fixedPhysicsStepCount;
        currentTimeSlice_.frameMilliseconds += sample.frameTimeMilliseconds;
        currentTimeSlice_.physicsMilliseconds += sample.physicsMilliseconds;
        currentTimeSlice_.fixedStepMilliseconds +=
            sample.averagePhysicsStepMilliseconds
            * static_cast<double>(sample.fixedPhysicsStepCount);
        currentTimeSlice_.maximumFixedStepMilliseconds = std::max(
            currentTimeSlice_.maximumFixedStepMilliseconds,
            sample.maximumPhysicsStepMilliseconds);
        if (sample.gpuTimingAvailable)
        {
            currentTimeSlice_.gpuExecutionMilliseconds +=
                sample.gpuExecutionMilliseconds;
            ++currentTimeSlice_.gpuTimingSampleCount;
        }
        addSolveCounters(currentTimeSlice_.solverCounters, sample.solverCounters);
        currentTimeSlice_.endSeconds = currentTimeSlice_.beginSeconds
            + currentTimeSlice_.frameMilliseconds / 1'000.0;
        if (currentTimeSlice_.frameMilliseconds
            >= previewSmokeTimeSliceMilliseconds)
        {
            timeSlices_.push_back(currentTimeSlice_);
            currentTimeSlice_ = {};
            currentTimeSlice_.beginSeconds = timeSlices_.back().endSeconds;
            currentTimeSlice_.endSeconds = currentTimeSlice_.beginSeconds;
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
            if (sample.frameTimeMilliseconds > 16.667)
                ++totals_.framesOver16Milliseconds;
            if (sample.frameTimeMilliseconds > 33.333)
                ++totals_.framesOver33Milliseconds;
        }

        totals_.averageMainThreadFrameMilliseconds +=
            sample.mainThreadFrameMilliseconds;
        totals_.maximumMainThreadFrameMilliseconds = std::max(
            totals_.maximumMainThreadFrameMilliseconds,
            sample.mainThreadFrameMilliseconds);
        totals_.averageEventPumpMilliseconds += sample.eventPumpMilliseconds;
        totals_.maximumEventPumpMilliseconds = std::max(
            totals_.maximumEventPumpMilliseconds, sample.eventPumpMilliseconds);
        totals_.averagePreSimulationMilliseconds +=
            sample.preSimulationCpuMilliseconds;
        totals_.maximumPreSimulationMilliseconds = std::max(
            totals_.maximumPreSimulationMilliseconds,
            sample.preSimulationCpuMilliseconds);
        if (sample.blockingEvents.viewportResized)
            ++totals_.viewportResizeCount;
        if (sample.swapchainRecreated)
            ++totals_.swapchainRecreationCount;

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
        if (sample.gpuTimingAvailable)
        {
            ++totals_.gpuTimingSampleCount;
            totals_.averageGpuExecutionMilliseconds +=
                sample.gpuExecutionMilliseconds;
            totals_.maximumGpuExecutionMilliseconds = std::max(
                totals_.maximumGpuExecutionMilliseconds,
                sample.gpuExecutionMilliseconds);
        }
        totals_.maximumDeferredBufferCount = std::max(
            totals_.maximumDeferredBufferCount,
            sample.deferredBufferCountBeforeReclaim);
        totals_.maximumDeferredBufferBytes = std::max(
            totals_.maximumDeferredBufferBytes,
            sample.deferredBufferBytesBeforeReclaim);
        totals_.totalReclaimedBufferCount += sample.reclaimedBufferCount;

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

        addSolveCounters(totals_.solverCounters, sample.solverCounters);

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
        if (!frameIntervalsMilliseconds_.empty())
        {
            std::vector<double> descending = frameIntervalsMilliseconds_;
            std::sort(descending.begin(), descending.end(), std::greater<>());
            const std::size_t slowCount = std::max<std::size_t>(1,
                static_cast<std::size_t>(std::ceil(
                    static_cast<double>(descending.size()) * 0.01)));
            const double slowTotal = std::accumulate(
                descending.begin(), descending.begin() + slowCount, 0.0);
            report.onePercentLowFramesPerSecond =
                1000.0 / (slowTotal / static_cast<double>(slowCount));
        }
        if (frames > 0.0)
        {
            report.averageMainThreadFrameMilliseconds /= frames;
            report.averageEventPumpMilliseconds /= frames;
            report.averagePreSimulationMilliseconds /= frames;
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
        if (report.gpuTimingSampleCount > 0)
        {
            report.averageGpuExecutionMilliseconds /=
                static_cast<double>(report.gpuTimingSampleCount);
        }
        report.averageFixedStepMilliseconds = report.simulationFixedStepCount > 0
            ? totals_.averageFixedStepMilliseconds
                / static_cast<double>(report.simulationFixedStepCount)
            : 0.0;
        report.timeSlices = timeSlices_;
        if (currentTimeSlice_.renderedFrameCount > 0)
            report.timeSlices.push_back(currentTimeSlice_);
        report.spikes.assign(spikes_.begin(), spikes_.begin() + spikeCount_);
        report.frameTrace = frameTrace_;
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
        nlohmann::json timeSlices = nlohmann::json::array();
        for (const auto& slice : report.timeSlices)
            timeSlices.push_back(timeSliceJson(slice));
        nlohmann::json frameTrace = nlohmann::json::array();
        for (const auto& sample : report.frameTrace)
            frameTrace.push_back(frameTraceJson(sample));
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
                {"one_percent_low_fps", report.onePercentLowFramesPerSecond},
                {"frames_over_16_667_ms", report.framesOver16Milliseconds},
                {"frames_over_33_333_ms", report.framesOver33Milliseconds},
                {"percentile_samples_dropped", report.percentileSamplesDropped}}},
            {"cpu_frame", {
                {"average_main_thread_ms", report.averageMainThreadFrameMilliseconds},
                {"maximum_main_thread_ms", report.maximumMainThreadFrameMilliseconds},
                {"average_event_pump_ms", report.averageEventPumpMilliseconds},
                {"maximum_event_pump_ms", report.maximumEventPumpMilliseconds},
                {"average_pre_simulation_ms", report.averagePreSimulationMilliseconds},
                {"maximum_pre_simulation_ms", report.maximumPreSimulationMilliseconds}}},
            {"resize", {
                {"viewport_resize_count", report.viewportResizeCount},
                {"swapchain_recreation_count", report.swapchainRecreationCount}}},
            {"physics", {
                {"total_steps", report.simulationFixedStepCount},
                {"average_steps_per_rendered_frame", report.averageStepsPerRenderedFrame},
                {"maximum_steps_in_rendered_frame", report.maximumStepsInRenderedFrame},
                {"average_cpu_ms_per_frame", report.averagePhysicsMillisecondsPerFrame},
                {"maximum_cpu_ms_per_frame", report.maximumPhysicsMillisecondsPerFrame},
                {"average_fixed_step_cpu_ms", report.averageFixedStepMilliseconds},
                {"maximum_fixed_step_cpu_ms", report.maximumFixedStepMilliseconds},
                {"maximum_steps_hit_frame_count", report.maximumStepsHitFrameCount}}},
            {"solver_counters", {
                {"solve_train_pose_calls", report.solverCounters.solveTrainPoseCalls},
                {"solve_car_geometry_calls", report.solverCounters.solveCarGeometryCalls},
                {"rigid_bogie_solve_calls", report.solverCounters.rigidBogieSolveCalls},
                {"rigid_bogie_refinement_iterations", report.solverCounters.rigidBogieRefinementIterations},
                {"rigid_bogie_bracket_expansions", report.solverCounters.rigidBogieBracketExpansions},
                {"connection_candidate_evaluations", report.solverCounters.connectionCandidateEvaluations},
                {"connector_refinement_iterations", report.solverCounters.connectorRefinementIterations},
                {"connector_fallback_uses", report.solverCounters.connectorFallbackUses},
                {"track_sample_calls", report.solverCounters.trackSampleCalls},
                {"interval_hint_misses", report.solverCounters.intervalHintMisses},
                {"open_boundary_pose_attempts",
                    report.solverCounters.openBoundaryPoseAttempts},
                {"open_boundary_pose_failures",
                    report.solverCounters.openBoundaryPoseFailures},
                {"open_boundary_refinement_iterations",
                    report.solverCounters.openBoundaryRefinementIterations}}},
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
            {"gpu_frame", {
                {"sample_count", report.gpuTimingSampleCount},
                {"average_execution_ms", report.averageGpuExecutionMilliseconds},
                {"maximum_execution_ms", report.maximumGpuExecutionMilliseconds}}},
            {"deferred_buffers", {
                {"maximum_pending_count", report.maximumDeferredBufferCount},
                {"maximum_pending_bytes", report.maximumDeferredBufferBytes},
                {"total_reclaimed_count", report.totalReclaimedBufferCount}}},
            {"catch_up_raw_delta", {
                {"largest_raw_incoming_delta_ms", report.largestRawDeltaMilliseconds},
                {"average_raw_delta_ms", report.averageRawDeltaMilliseconds},
                {"maximum_requested_step_count", report.maximumRequestedStepCount},
                {"maximum_executed_step_count", report.maximumExecutedStepCount},
                {"maximum_consecutive_catch_up_frame_count", report.maximumConsecutiveCatchUpFrameCount},
                {"total_discarded_wall_time_ms", report.totalDiscardedWallTimeMilliseconds},
                {"discarded_wall_time_frame_count", report.discardedWallTimeFrameCount}}},
            {"time_slices", std::move(timeSlices)},
            {"frame_trace", std::move(frameTrace)},
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
            << "FPS 1% low: " << report.onePercentLowFramesPerSecond << '\n'
            << "Frame ms avg/max/p95/p99: " << report.averageFrameMilliseconds
            << " / " << report.maximumFrameMilliseconds << " / "
            << report.p95FrameMilliseconds << " / " << report.p99FrameMilliseconds << '\n'
            << "Frames >16.667/>33.333 ms: "
            << report.framesOver16Milliseconds << " / "
            << report.framesOver33Milliseconds << '\n'
            << "Main thread ms avg/max: "
            << report.averageMainThreadFrameMilliseconds << " / "
            << report.maximumMainThreadFrameMilliseconds << '\n'
            << "Event pump ms avg/max: "
            << report.averageEventPumpMilliseconds << " / "
            << report.maximumEventPumpMilliseconds << '\n'
            << "Pre-simulation ms avg/max: "
            << report.averagePreSimulationMilliseconds << " / "
            << report.maximumPreSimulationMilliseconds << '\n'
            << "Viewport resizes / swapchain recreations: "
            << report.viewportResizeCount << " / "
            << report.swapchainRecreationCount << '\n'
            << "GPU execution ms avg/max (samples): "
            << report.averageGpuExecutionMilliseconds << " / "
            << report.maximumGpuExecutionMilliseconds << " ("
            << report.gpuTimingSampleCount << ")\n"
            << "Deferred buffers max count/bytes, total reclaimed: "
            << report.maximumDeferredBufferCount << " / "
            << report.maximumDeferredBufferBytes << " / "
            << report.totalReclaimedBufferCount << '\n'
            << "Physics ms/frame avg/max: "
            << report.averagePhysicsMillisecondsPerFrame << " / "
            << report.maximumPhysicsMillisecondsPerFrame << '\n'
            << "Physics step ms avg/max: " << report.averageFixedStepMilliseconds
            << " / " << report.maximumFixedStepMilliseconds << '\n'
            << "Solver calls: solve=" << report.solverCounters.solveTrainPoseCalls
            << " geometry=" << report.solverCounters.solveCarGeometryCalls
            << " rigidsolve=" << report.solverCounters.rigidBogieSolveCalls
            << " refinement=" << report.solverCounters.rigidBogieRefinementIterations
            << " brackexp=" << report.solverCounters.rigidBogieBracketExpansions
            << " candidates=" << report.solverCounters.connectionCandidateEvaluations
            << " connref=" << report.solverCounters.connectorRefinementIterations
            << " fallbacks=" << report.solverCounters.connectorFallbackUses
            << " track_samples=" << report.solverCounters.trackSampleCalls
            << " hintmisses=" << report.solverCounters.intervalHintMisses << '\n'
            << "Open-boundary pose attempts/failures/refinements: "
            << report.solverCounters.openBoundaryPoseAttempts << " / "
            << report.solverCounters.openBoundaryPoseFailures << " / "
            << report.solverCounters.openBoundaryRefinementIterations << '\n'
            << "Catch-up requested/executed max: "
            << report.maximumRequestedStepCount << " / "
            << report.maximumExecutedStepCount << '\n'
            << "Discarded wall time: " << report.totalDiscardedWallTimeMilliseconds
            << " ms across " << report.discardedWallTimeFrameCount << " frame(s)\n"
            << "Five-second time slices: " << report.timeSlices.size() << '\n'
            << "Retained spikes: " << report.spikes.size() << '\n';
        if (!report.frameTrace.empty())
        {
            textOutput << "Frame trace samples: " << report.frameTrace.size()
                << '\n';
        }
        for (const auto& slice : report.timeSlices)
        {
            const double elapsedSeconds = slice.frameMilliseconds / 1'000.0;
            textOutput << "  " << slice.beginSeconds << "-" << slice.endSeconds
                << " s: "
                << (elapsedSeconds > 0.0
                    ? static_cast<double>(slice.renderedFrameCount)
                        / elapsedSeconds
                    : 0.0)
                << " FPS, "
                << ratio(static_cast<double>(slice.fixedStepCount),
                    slice.renderedFrameCount)
                << " steps/frame, "
                << ratio(slice.fixedStepMilliseconds, slice.fixedStepCount)
                << " ms/step, "
                << ratio(slice.physicsMilliseconds, slice.renderedFrameCount)
                << " physics ms/frame\n";
        }
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
