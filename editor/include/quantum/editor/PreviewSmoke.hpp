#pragma once

#include <quantum/editor/FramePerformanceTelemetry.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::editor
{
    struct PreviewSmokeOptions
    {
        std::filesystem::path documentPath;
        std::filesystem::path outputBasePath;
        double durationSeconds = 20.0;
        double spikeFrameMilliseconds = 33.3;
        std::size_t spikeStepThreshold = 8;
        bool repeat = false;
    };

    [[nodiscard]] std::expected<std::optional<PreviewSmokeOptions>, std::string>
    parsePreviewSmokeArguments(std::span<const std::string_view> arguments);

    [[nodiscard]] std::expected<void, std::string>
    validatePreviewSmokeInput(const PreviewSmokeOptions& options);

    struct PreviewSmokeSpikeRecord
    {
        FramePerformanceSample current;
        FramePerformanceSample previous;
        bool hasPrevious = false;
        FramePerformanceSample next;
        bool hasNext = false;
    };

    struct PreviewSmokeReport
    {
        std::filesystem::path documentPath;
        std::string buildConfiguration;
        double requestedDurationSeconds = 0.0;
        double actualElapsedSeconds = 0.0;
        std::uint64_t renderedFrameCount = 0;
        std::uint64_t simulationFixedStepCount = 0;
        bool playbackCompletedNormally = false;
        bool previewOrPhysicsFailure = false;
        std::string failureMessage;

        double averageFramesPerSecond = 0.0;
        double averageFrameMilliseconds = 0.0;
        double maximumFrameMilliseconds = 0.0;
        double p95FrameMilliseconds = 0.0;
        double p99FrameMilliseconds = 0.0;
        double minimumFramesPerSecond = 0.0;

        double averageStepsPerRenderedFrame = 0.0;
        std::size_t maximumStepsInRenderedFrame = 0;
        double averagePhysicsMillisecondsPerFrame = 0.0;
        double maximumPhysicsMillisecondsPerFrame = 0.0;
        double averageFixedStepMilliseconds = 0.0;
        double maximumFixedStepMilliseconds = 0.0;
        std::uint64_t maximumStepsHitFrameCount = 0;

        double averageInterpolationMilliseconds = 0.0;
        double maximumInterpolationMilliseconds = 0.0;
        std::uint64_t interpolationSolveFailureCount = 0;

        double averagePreviewUploadMilliseconds = 0.0;
        double maximumPreviewUploadMilliseconds = 0.0;
        double averagePreviewSlotWaitMilliseconds = 0.0;
        double maximumPreviewSlotWaitMilliseconds = 0.0;
        double averageDrawFrameMilliseconds = 0.0;
        double maximumDrawFrameMilliseconds = 0.0;
        double averageFrameSlotFenceWaitMilliseconds = 0.0;
        double maximumFrameSlotFenceWaitMilliseconds = 0.0;
        double averageAcquireMilliseconds = 0.0;
        double maximumAcquireMilliseconds = 0.0;
        double averagePresentMilliseconds = 0.0;
        double maximumPresentMilliseconds = 0.0;

        double largestRawDeltaMilliseconds = 0.0;
        double averageRawDeltaMilliseconds = 0.0;
        std::size_t maximumRequestedStepCount = 0;
        std::size_t maximumExecutedStepCount = 0;
        std::size_t maximumConsecutiveCatchUpFrameCount = 0;
        double totalDiscardedWallTimeMilliseconds = 0.0;
        std::uint64_t discardedWallTimeFrameCount = 0;

        std::uint64_t percentileSamplesDropped = 0;
        std::vector<PreviewSmokeSpikeRecord> spikes;
    };

    class PreviewSmokeCollector
    {
    public:
        static constexpr std::size_t maximumRetainedSpikes = 32;
        static constexpr std::size_t maximumPercentileSamples = 1'000'000;

        explicit PreviewSmokeCollector(const PreviewSmokeOptions& options);
        void record(const FramePerformanceSample& sample) noexcept;

        [[nodiscard]] PreviewSmokeReport finish(
            double actualElapsedSeconds,
            bool playbackCompletedNormally,
            bool previewOrPhysicsFailure,
            std::string failureMessage = {}) const;

        [[nodiscard]] static double percentile(
            std::span<const double> values,
            double percentileFraction);

    private:
        PreviewSmokeOptions options_;
        std::vector<double> frameIntervalsMilliseconds_;
        std::array<PreviewSmokeSpikeRecord, maximumRetainedSpikes> spikes_{};
        std::size_t spikeCount_ = 0;
        std::optional<FramePerformanceSample> previousSample_;
        PreviewSmokeReport totals_;
    };

    struct PreviewSmokeReportPaths
    {
        std::filesystem::path json;
        std::filesystem::path text;
    };

    [[nodiscard]] PreviewSmokeReportPaths writePreviewSmokeReports(
        const PreviewSmokeReport& report,
        const PreviewSmokeOptions& options);
}
