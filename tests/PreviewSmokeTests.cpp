#include <quantum/editor/PreviewSmoke.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    void require(const bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string& message)
    {
        if (std::abs(actual - expected) > tolerance)
            throw std::runtime_error(message + ": got "
                + std::to_string(actual) + ", expected "
                + std::to_string(expected));
    }

    std::filesystem::path fixturePath()
    {
        const auto path = std::filesystem::temp_directory_path()
            / "quantum-preview-smoke-arguments.quantum";
        std::ofstream(path) << "{}";
        return path;
    }

    auto parse(const std::vector<std::string_view>& arguments)
    {
        return quantum::editor::parsePreviewSmokeArguments(arguments);
    }

    void cliParsingAndDefaults()
    {
        const auto fixture = fixturePath();
        const std::string fixtureString = fixture.string();
        const auto result = parse({
            "--dev-preview-smoke", fixtureString});
        require(result.has_value() && result->has_value(),
            "smoke arguments parse");
        requireNear((**result).durationSeconds, 20.0, 0.0,
            "default duration");
        requireNear((**result).spikeFrameMilliseconds, 33.3, 0.0,
            "default frame threshold");
        require((**result).spikeStepThreshold == 8,
            "default step threshold");
        require(!(**result).repeat, "repeat is opt-in");
        const auto repeated = parse({
            "--dev-preview-smoke", fixtureString, "--repeat"});
        require(repeated && repeated->has_value() && (**repeated).repeat,
            "developer repeat flag parses without a value");

        const auto custom = parse({
            "--log-level", "debug", "--dev-preview-smoke", fixtureString,
            "--duration", "2.5", "--spike-frame-ms", "25",
            "--spike-steps", "6", "--output", "reports/run.json"});
        require(custom && custom->has_value(), "custom arguments parse");
        requireNear((**custom).durationSeconds, 2.5, 0.0,
            "custom duration");
        requireNear((**custom).spikeFrameMilliseconds, 25.0, 0.0,
            "custom frame threshold");
        require((**custom).spikeStepThreshold == 6,
            "custom step threshold");
        require((**custom).outputBasePath == "reports/run.json",
            "custom output path");
        std::filesystem::remove(fixture);
    }

    void cliFailuresAndNormalPath()
    {
        const auto normal = parse({"--log-level", "info"});
        require(normal && !normal->has_value(),
            "flag absence leaves normal application path untouched");
        const auto missing = parse({
            "--dev-preview-smoke", "missing.quantum"});
        require(missing && missing->has_value()
                && !quantum::editor::validatePreviewSmokeInput(**missing),
            "missing document rejected before application startup");

        const auto fixture = fixturePath();
        const std::string fixtureString = fixture.string();
        for (const std::vector<std::string_view> arguments : {
                std::vector<std::string_view>{
                    "--dev-preview-smoke", fixtureString, "--duration", "0"},
                std::vector<std::string_view>{
                    "--dev-preview-smoke", fixtureString, "--duration", "nan"},
                std::vector<std::string_view>{
                    "--dev-preview-smoke", fixtureString, "--spike-frame-ms", "-1"},
                std::vector<std::string_view>{
                    "--dev-preview-smoke", fixtureString, "--spike-steps", "0"}})
            require(!parse(arguments), "invalid numeric option rejected");
        std::filesystem::remove(fixture);
    }

    void aggregationAndPercentiles()
    {
        quantum::editor::PreviewSmokeOptions options;
        options.documentPath = "fixture.quantum";
        options.spikeFrameMilliseconds = 1000.0;
        options.spikeStepThreshold = 1000;
        quantum::editor::PreviewSmokeCollector collector(options);

        quantum::editor::FramePerformanceSample first;
        first.frameId = 1;
        first.frameTimeMilliseconds = 10.0;
        first.rawSimulationDeltaMilliseconds = 11.0;
        first.fixedPhysicsStepCount = 2;
        first.requestedPhysicsStepCount = 3;
        first.averagePhysicsStepMilliseconds = 0.25;
        first.maximumPhysicsStepMilliseconds = 0.4;
        first.physicsMilliseconds = 1.0;
        first.interpolationMilliseconds = 0.2;
        first.previewVertexPreparationMilliseconds = 0.1;
        first.previewVertexPublishMilliseconds = 0.2;
        first.previewFrameSlotUpdateMilliseconds = 0.3;
        first.previewFrameSlotWaitMilliseconds = 0.4;
        first.drawFrameCpuMilliseconds = 1.5;
        first.acquireCallMilliseconds = 0.5;
        first.presentCallMilliseconds = 0.6;
        collector.record(first);

        auto second = first;
        second.frameId = 2;
        second.frameTimeMilliseconds = 20.0;
        second.rawSimulationDeltaMilliseconds = 21.0;
        second.fixedPhysicsStepCount = 4;
        second.requestedPhysicsStepCount = 5;
        second.averagePhysicsStepMilliseconds = 0.5;
        second.maximumPhysicsStepMilliseconds = 0.8;
        second.physicsMilliseconds = 3.0;
        second.interpolationMilliseconds = 0.4;
        second.discardedWallTimeMilliseconds = 2.0;
        second.maximumPhysicsStepsHit = true;
        second.renderPoseFailureCount = 1;
        collector.record(second);

        const auto report = collector.finish(1.0, true, false);
        require(report.renderedFrameCount == 2, "frame count");
        require(report.simulationFixedStepCount == 6, "step count");
        requireNear(report.averageFrameMilliseconds, 15.0, 1.0e-12,
            "average frame interval");
        requireNear(report.p95FrameMilliseconds, 20.0, 1.0e-12,
            "p95 nearest-rank");
        requireNear(report.p99FrameMilliseconds, 20.0, 1.0e-12,
            "p99 nearest-rank");
        requireNear(report.averageStepsPerRenderedFrame, 3.0, 1.0e-12,
            "average steps per frame");
        requireNear(report.averagePhysicsMillisecondsPerFrame, 2.0, 1.0e-12,
            "average physics CPU per frame");
        requireNear(report.averageFixedStepMilliseconds, 2.5 / 6.0,
            1.0e-12, "weighted fixed-step average");
        require(report.maximumStepsHitFrameCount == 1,
            "maximum-steps hit count");
        require(report.interpolationSolveFailureCount == 1,
            "interpolation failure count");
        requireNear(report.totalDiscardedWallTimeMilliseconds, 2.0,
            1.0e-12, "discarded time sum");
    }

    void spikeRetentionIsBounded()
    {
        quantum::editor::PreviewSmokeOptions options;
        options.documentPath = "fixture.quantum";
        options.spikeFrameMilliseconds = 1.0;
        options.spikeStepThreshold = 1;
        quantum::editor::PreviewSmokeCollector collector(options);
        for (std::uint64_t frame = 1; frame <= 40; ++frame)
        {
            quantum::editor::FramePerformanceSample sample;
            sample.frameId = frame;
            sample.frameTimeMilliseconds = static_cast<double>(frame);
            sample.fixedPhysicsStepCount = 1;
            collector.record(sample);
        }
        const auto report = collector.finish(1.0, true, false);
        require(report.spikes.size()
                == quantum::editor::PreviewSmokeCollector::maximumRetainedSpikes,
            "spike storage is bounded");
        require(report.spikes.front().current.frameId == 40,
            "worst spike sorts first");
        require(report.spikes.back().current.frameId == 9,
            "collector retains the worst 32 frames");
        require(report.spikes.front().hasPrevious
                && report.spikes.front().previous.frameId == 39,
            "spike retains explicit causal previous frame");
        require(!report.spikes.front().hasNext
            && report.spikes.back().hasNext
            && report.spikes.back().next.frameId == 10,
            "spikes retain the following frame when one was recorded");
    }

    void reportsAreWrittenOnceAtFinish()
    {
        quantum::editor::PreviewSmokeOptions options;
        options.documentPath = "fixture.quantum";
        options.outputBasePath = std::filesystem::temp_directory_path()
            / "quantum-preview-smoke-report-test";
        quantum::editor::PreviewSmokeCollector collector(options);
        quantum::editor::FramePerformanceSample sample;
        sample.frameId = 1;
        sample.frameTimeMilliseconds = 16.0;
        sample.previewFrameSlotWaitMilliseconds = 479.0;
        sample.synchronization.current.drawId = 3;
        sample.synchronization.current.frameSlot = 0;
        sample.synchronization.waitedSubmission.drawId = 1;
        sample.synchronization.waitedSubmission.frameSlot = 0;
        sample.synchronization.waitedSubmission.imageIndex = 2;
        sample.synchronization.waitedSubmission.submitted = true;
        collector.record(sample);
        const auto report = collector.finish(0.02, true, false);
        require(report.spikes.size() == 1,
            "a fence-only stall is retained before its following long delta");
        require(report.spikes[0].current.synchronization.waitedSubmission.drawId == 1,
            "retain the actual waited slot submission, not just the previous frame");
        const auto paths = quantum::editor::writePreviewSmokeReports(
            report, options);
        require(std::filesystem::is_regular_file(paths.json)
                && std::filesystem::is_regular_file(paths.text),
            "both report formats written");
        std::ifstream jsonInput(paths.json);
        const std::string json{
            std::istreambuf_iterator<char>(jsonInput),
            std::istreambuf_iterator<char>()};
        require(json.find("\"frame_performance\"") != std::string::npos
                && json.find("\"spikes\"") != std::string::npos,
            "JSON contains structured summary and spikes");
        require(json.find("\"waited_submission\"") != std::string::npos
                && json.find("\"render_finished_semaphore_image_index\"") != std::string::npos,
            "JSON includes causal fence and semaphore identity");
        jsonInput.close();
        std::filesystem::remove(paths.json);
        std::filesystem::remove(paths.text);
    }
}

int main()
{
    try
    {
        cliParsingAndDefaults();
        cliFailuresAndNormalPath();
        aggregationAndPercentiles();
        spikeRetentionIsBounded();
        reportsAreWrittenOnceAtFinish();
        std::cout << "Preview smoke tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Preview smoke test failure: " << error.what() << '\n';
        return 1;
    }
}
