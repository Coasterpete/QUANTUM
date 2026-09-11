#include <quantum/physics/CarPose.hpp>
#include <quantum/physics/gpu/GpuPhysicsContext.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::LayoutMode;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;
    using quantum::physics::BogieDefinition;
    using quantum::physics::CarDefinition;
    using quantum::physics::CompiledPhysicsTrack;
    using quantum::physics::TrackLocation;
    using quantum::physics::TrainSolveCounters;
    using quantum::physics::TravelDirection;
    using quantum::physics::gpu::GpuPhysicsContext;
    using quantum::physics::gpu::GpuRigidBogieBatchTimings;
    using quantum::physics::gpu::GpuRigidBogieJob;
    using quantum::physics::gpu::GpuRigidBogieResult;
    using quantum::physics::gpu::GpuRigidBogieStatus;

    constexpr std::array<std::size_t, 14> batchSizes{
        1, 8, 16, 32, 60, 63, 64, 65, 75, 96, 128, 256, 512, 1024};

    struct Fixture
    {
        std::string name;
        std::vector<TrackKinematicState> samples;
        TopologyKind topology = TopologyKind::OpenLinear;
    };

    struct CpuResult
    {
        GpuRigidBogieResult result;
        double elapsedMicroseconds = 0.0;
    };

    [[nodiscard]] CurveFrame frameForTangent(const glm::dvec3 tangent)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        const glm::dvec3 lateral{0.0, 1.0, 0.0};
        return {unit, lateral, glm::normalize(glm::cross(unit, lateral))};
    }

    [[nodiscard]] Fixture straightFixture(const double length = 40.0)
    {
        const CurveFrame frame = frameForTangent({1.0, 0.0, 0.0});
        return {"straight", {
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {length, {length, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}}},
            TopologyKind::OpenLinear};
    }

    [[nodiscard]] Fixture horizontalCircleFixture(
        const double radius = 25.0,
        const double sweep = 2.0 * std::numbers::pi,
        const TopologyKind topology = TopologyKind::ClosedCircuit)
    {
        constexpr int segmentCount = 720;
        Fixture result{"horizontal-circle", {}, topology};
        result.samples.reserve(segmentCount + 1);
        for (int index = 0; index <= segmentCount; ++index)
        {
            const double angle = sweep * static_cast<double>(index) / segmentCount;
            result.samples.push_back({radius * angle,
                {radius * std::sin(angle), radius * (1.0 - std::cos(angle)), 0.0},
                {{std::cos(angle), std::sin(angle), 0.0},
                    {-std::sin(angle), std::cos(angle), 0.0}, {0.0, 0.0, 1.0}},
                {-std::sin(angle) / radius, std::cos(angle) / radius, 0.0}});
        }
        return result;
    }

    [[nodiscard]] Fixture verticalArcFixture(const bool crest)
    {
        constexpr double radius = 24.0;
        constexpr double startAngle = -0.7;
        constexpr double endAngle = 0.7;
        constexpr int segmentCount = 512;
        const double verticalSign = crest ? 1.0 : -1.0;
        Fixture result{crest ? "vertical-crest" : "vertical-valley", {},
            TopologyKind::OpenLinear};
        result.samples.reserve(segmentCount + 1);
        for (int index = 0; index <= segmentCount; ++index)
        {
            const double angle = startAngle + (endAngle - startAngle)
                * static_cast<double>(index) / segmentCount;
            const glm::dvec3 tangent{std::cos(angle), 0.0,
                -verticalSign * std::sin(angle)};
            result.samples.push_back({radius * (angle - startAngle),
                {radius * std::sin(angle), 0.0,
                    verticalSign * radius * std::cos(angle)},
                {tangent, {0.0, 1.0, 0.0},
                    glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})},
                {-std::sin(angle) / radius, 0.0,
                    -verticalSign * std::cos(angle) / radius}});
        }
        return result;
    }

    [[nodiscard]] Fixture nominalOverextensionFixture()
    {
        Fixture result = straightFixture(10.0);
        result.name = "nominal-overextension";
        result.samples.back().position = {100.0, 0.0, 0.0};
        return result;
    }

    [[nodiscard]] GpuRigidBogieJob makeJob(
        const std::uint32_t id,
        const double station,
        const int direction = 1,
        const glm::dvec3 front = {1.0, 0.0, 0.0},
        const glm::dvec3 rear = {-1.0, 0.0, 0.0})
    {
        GpuRigidBogieJob job{};
        job.jobId = id;
        job.direction = direction;
        job.referenceStationMeters = station;
        for (int component = 0; component < 3; ++component)
        {
            job.frontReferencePositionMeters[component] = front[component];
            job.rearReferencePositionMeters[component] = rear[component];
        }
        return job;
    }

    [[nodiscard]] CarDefinition carFor(const GpuRigidBogieJob& job)
    {
        CarDefinition car;
        car.bogies = {
            BogieDefinition{{job.frontReferencePositionMeters[0],
                job.frontReferencePositionMeters[1],
                job.frontReferencePositionMeters[2]}},
            BogieDefinition{{job.rearReferencePositionMeters[0],
                job.rearReferencePositionMeters[1],
                job.rearReferencePositionMeters[2]}}};
        return car;
    }

    [[nodiscard]] GpuRigidBogieStatus statusForException(
        const std::string_view message)
    {
        if (message.find("nominal bogie stations") != std::string_view::npos)
            return GpuRigidBogieStatus::NominalOverextended;
        if (message.find("local rigid-bogie station interval") != std::string_view::npos)
            return GpuRigidBogieStatus::NoLocalInterval;
        if (message.find("cannot be placed within") != std::string_view::npos)
            return GpuRigidBogieStatus::NoFeasibleRoot;
        if (message.find("did not converge") != std::string_view::npos)
            return GpuRigidBogieStatus::DidNotConverge;
        return GpuRigidBogieStatus::NonFinite;
    }

    [[nodiscard]] CpuResult solveCpu(
        const CompiledPhysicsTrack& track,
        const GpuRigidBogieJob& job)
    {
        CpuResult output;
        output.result.jobId = job.jobId;
        const CarDefinition car = carFor(job);
        const TrackLocation location{quantum::physics::primaryTrackPathId,
            job.referenceStationMeters,
            job.direction < 0 ? TravelDirection::DecreasingStation
                              : TravelDirection::IncreasingStation};
        TrainSolveCounters counters;
        const auto begin = std::chrono::steady_clock::now();
        try
        {
            const auto pose = quantum::physics::detail::solveCarPoseForValidatedDefinition(
                track, car, location, {}, &counters);
            output.result.status = GpuRigidBogieStatus::Solved;
            output.result.refinementIterations = static_cast<std::uint32_t>(
                counters.rigidBogieRefinementIterations);
            output.result.bracketExpansions = static_cast<std::uint32_t>(
                counters.rigidBogieBracketExpansions);
            output.result.frontStationMeters =
                pose.frontBogie().location().stationMeters;
            output.result.rearStationMeters =
                pose.rearBogie().location().stationMeters;
            const glm::dvec3 authoredFront{
                job.frontReferencePositionMeters[0],
                job.frontReferencePositionMeters[1],
                job.frontReferencePositionMeters[2]};
            const glm::dvec3 authoredRear{
                job.rearReferencePositionMeters[0],
                job.rearReferencePositionMeters[1],
                job.rearReferencePositionMeters[2]};
            output.result.finalResidualMeters = glm::length(
                pose.frontBogie().worldPositionMeters()
                    - pose.rearBogie().worldPositionMeters())
                - glm::length(authoredFront - authoredRear);
        }
        catch (const std::exception& error)
        {
            output.result.status = statusForException(error.what());
            output.result.refinementIterations = static_cast<std::uint32_t>(
                counters.rigidBogieRefinementIterations);
            output.result.bracketExpansions = static_cast<std::uint32_t>(
                counters.rigidBogieBracketExpansions);
        }
        output.elapsedMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - begin).count();
        return output;
    }

    void upload(GpuPhysicsContext& gpu, const Fixture& fixture)
    {
        gpu.uploadTrack(0, fixture.samples, 1.0, fixture.topology,
            fixture.topology == TopologyKind::ClosedCircuit
                ? LayoutMode::Circuit : LayoutMode::Shuttle);
    }

    struct ValidationTotals
    {
        std::size_t jobs = 0;
        std::size_t statusMismatches = 0;
        std::size_t refinementDifferences = 0;
        std::size_t expansionDifferences = 0;
        double maxFrontError = 0.0;
        double maxRearError = 0.0;
        double maxResidualError = 0.0;
        std::vector<std::uint32_t> gpuRefinements;
    };

    void compareBatch(
        GpuPhysicsContext& gpu,
        const Fixture& fixture,
        const std::vector<GpuRigidBogieJob>& jobs,
        ValidationTotals& totals,
        const std::uint32_t localSize = 64)
    {
        upload(gpu, fixture);
        const auto gpuResults = gpu.solveRigidBogiesGpu(jobs, nullptr, localSize);
        if (gpuResults.size() != jobs.size())
            throw std::runtime_error("GPU rigid-bogie dispatch returned wrong result count");
        for (std::size_t index = 0; index < jobs.size(); ++index)
        {
            const CpuResult cpu = solveCpu(
                CompiledPhysicsTrack{fixture.samples, 1.0, fixture.topology}, jobs[index]);
            const auto& actual = gpuResults[index];
            ++totals.jobs;
            totals.statusMismatches += actual.status != cpu.result.status;
            totals.refinementDifferences +=
                actual.refinementIterations != cpu.result.refinementIterations;
            totals.expansionDifferences +=
                actual.bracketExpansions != cpu.result.bracketExpansions;
            totals.gpuRefinements.push_back(actual.refinementIterations);
            if (actual.status == GpuRigidBogieStatus::Solved
                && cpu.result.status == GpuRigidBogieStatus::Solved)
            {
                totals.maxFrontError = std::max(totals.maxFrontError,
                    std::abs(actual.frontStationMeters - cpu.result.frontStationMeters));
                totals.maxRearError = std::max(totals.maxRearError,
                    std::abs(actual.rearStationMeters - cpu.result.rearStationMeters));
                totals.maxResidualError = std::max(totals.maxResidualError,
                    std::abs(actual.finalResidualMeters - cpu.result.finalResidualMeters));
            }
        }
    }

    [[nodiscard]] const char* statusName(const GpuRigidBogieStatus status)
    {
        switch (status)
        {
        case GpuRigidBogieStatus::Solved: return "Solved";
        case GpuRigidBogieStatus::NominalOverextended: return "NominalOverextended";
        case GpuRigidBogieStatus::NoLocalInterval: return "NoLocalInterval";
        case GpuRigidBogieStatus::NoFeasibleRoot: return "NoFeasibleRoot";
        case GpuRigidBogieStatus::DidNotConverge: return "DidNotConverge";
        case GpuRigidBogieStatus::NonFinite: return "NonFinite";
        }
        return "Unknown";
    }

    void reportCase(
        GpuPhysicsContext& gpu,
        const Fixture& fixture,
        const GpuRigidBogieJob& job)
    {
        upload(gpu, fixture);
        const auto gpuResult = gpu.solveRigidBogiesGpu({&job, 1}).front();
        const auto cpuResult = solveCpu(
            CompiledPhysicsTrack{fixture.samples, 1.0, fixture.topology}, job).result;
        std::printf("CASE name=%s cpu=%s gpu=%s cpu_refine=%u gpu_refine=%u "
            "cpu_expand=%u gpu_expand=%u gpu_residual=%.17g\n",
            fixture.name.c_str(), statusName(cpuResult.status),
            statusName(gpuResult.status), cpuResult.refinementIterations,
            gpuResult.refinementIterations, cpuResult.bracketExpansions,
            gpuResult.bracketExpansions, gpuResult.finalResidualMeters);
    }

    [[nodiscard]] std::vector<GpuRigidBogieJob> ordinaryJobs(
        const std::size_t count,
        const double trackLength)
    {
        std::vector<GpuRigidBogieJob> jobs;
        jobs.reserve(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            const double station = std::fmod(
                3.0 + 0.371 * static_cast<double>(index), trackLength);
            jobs.push_back(makeJob(static_cast<std::uint32_t>(index), station,
                index % 5 == 0 ? -1 : 1));
        }
        return jobs;
    }

    void printDistribution(std::vector<std::uint32_t> values)
    {
        std::sort(values.begin(), values.end());
        const double mean = std::accumulate(values.begin(), values.end(), 0.0)
            / static_cast<double>(values.size());
        const auto percentile = [&](const double p)
        {
            return values[static_cast<std::size_t>(
                std::ceil(p * values.size()) - 1.0)];
        };
        std::printf("GPU_ITERATIONS min=%u median=%u mean=%.3f p95=%u max=%u\n",
            values.front(), percentile(0.5), mean, percentile(0.95), values.back());
    }

    void validate(GpuPhysicsContext& gpu)
    {
        ValidationTotals totals;
        const Fixture circle = horizontalCircleFixture();
        const double circleLength = circle.samples.back().distance;
        for (const std::size_t count : batchSizes)
            compareBatch(gpu, circle, ordinaryJobs(count, circleLength), totals);
        for (const std::uint32_t localSize : {32u, 128u, 256u})
            compareBatch(gpu, circle, ordinaryJobs(256, circleLength), totals,
                localSize);

        compareBatch(gpu, straightFixture(), {
            makeJob(10'001, 20.0),
            makeJob(10'002, 20.0, -1),
            makeJob(10'003, 0.0),
            makeJob(10'004, 40.0),
            makeJob(10'005, 20.0, 1, {1.3, 0.4, 0.6}, {-1.1, -0.2, 0.1})}, totals);
        compareBatch(gpu, verticalArcFixture(false), {
            makeJob(11'001, 16.8), makeJob(11'002, 0.0)}, totals);
        compareBatch(gpu, verticalArcFixture(true), {
            makeJob(12'001, 16.8), makeJob(12'002, 0.0)}, totals);
        compareBatch(gpu, circle, {
            makeJob(13'001, circleLength - 0.25),
            makeJob(13'002, 0.25, -1)}, totals);
        compareBatch(gpu, nominalOverextensionFixture(), {
            makeJob(14'001, 5.0)}, totals);
        compareBatch(gpu, straightFixture(1.0), {
            makeJob(15'001, 0.5)}, totals);
        const Fixture shortCircuit = horizontalCircleFixture(
            0.5, 2.0 * std::numbers::pi, TopologyKind::ClosedCircuit);
        compareBatch(gpu, shortCircuit, {
            makeJob(16'001, 0.5)}, totals);
        compareBatch(gpu, straightFixture(), {
            makeJob(17'001, std::numeric_limits<double>::quiet_NaN())}, totals);

        reportCase(gpu, nominalOverextensionFixture(), makeJob(14'001, 5.0));
        reportCase(gpu, straightFixture(1.0), makeJob(15'001, 0.5));
        reportCase(gpu, shortCircuit, makeJob(16'001, 0.5));
        reportCase(gpu, verticalArcFixture(true), makeJob(12'002, 0.0));
        reportCase(gpu, straightFixture(),
            makeJob(17'001, std::numeric_limits<double>::quiet_NaN()));

        std::printf("VALIDATION jobs=%zu status_mismatches=%zu max_front=%.17g "
            "max_rear=%.17g max_residual=%.17g refinement_differences=%zu "
            "expansion_differences=%zu\n",
            totals.jobs, totals.statusMismatches, totals.maxFrontError,
            totals.maxRearError, totals.maxResidualError,
            totals.refinementDifferences, totals.expansionDifferences);
        printDistribution(std::move(totals.gpuRefinements));
        if (totals.statusMismatches != 0 || totals.maxFrontError > 2.0e-8
            || totals.maxRearError > 2.0e-8 || totals.maxResidualError > 2.0e-9)
            throw std::runtime_error("CPU/GPU rigid-bogie numerical equivalence failed");
    }

    template<typename Function>
    [[nodiscard]] double averageMicroseconds(
        const std::size_t iterations, Function&& function)
    {
        const auto begin = std::chrono::steady_clock::now();
        for (std::size_t iteration = 0; iteration < iterations; ++iteration)
            function();
        return std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - begin).count() / iterations;
    }

    void benchmark(GpuPhysicsContext& gpu)
    {
        const Fixture fixture = horizontalCircleFixture();
        upload(gpu, fixture);
        const CompiledPhysicsTrack cpuTrack{
            fixture.samples, 1.0, fixture.topology};
        std::printf("PERFORMANCE ABI upload_bytes_per_job=%zu readback_bytes_per_job=%zu\n",
            sizeof(GpuRigidBogieJob), sizeof(GpuRigidBogieResult));
        const auto device = gpu.computeDeviceInfo();
        std::printf("DEVICE subgroup=%u max_workgroup_x=%u max_invocations=%u timestamp_bits=%u timestamp_period_ns=%.3f\n",
            device.subgroupSize, device.maxWorkgroupSizeX,
            device.maxWorkgroupInvocations, device.timestampValidBits,
            device.timestampPeriodNanoseconds);
        for (const std::uint32_t localSize : {32u, 64u, 128u, 256u})
        {
        std::printf("LOCAL_SIZE %u\n", localSize);
        std::printf("%6s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %9s\n",
            "jobs", "CPU_us", "CPU_us/j", "GPU_exec", "upload", "submit",
            "wait", "readback", "GPU_us", "GPU_us/j", "GPU_j/s", "speedup");
        for (const std::size_t count : batchSizes)
        {
            const auto jobs = ordinaryJobs(count, fixture.samples.back().distance);
            std::vector<CarDefinition> cars;
            std::vector<TrackLocation> locations;
            cars.reserve(jobs.size());
            locations.reserve(jobs.size());
            for (const auto& job : jobs)
            {
                cars.push_back(carFor(job));
                locations.push_back({quantum::physics::primaryTrackPathId,
                    job.referenceStationMeters,
                    job.direction < 0 ? TravelDirection::DecreasingStation
                                      : TravelDirection::IncreasingStation});
            }
            const auto warmupResults = gpu.solveRigidBogiesGpu(jobs, nullptr, localSize);
            if (localSize == 32 && count == 65)
            {
                for (std::size_t index = 0; index < warmupResults.size(); ++index)
                    if (warmupResults[index].refinementIterations > 1)
                        std::printf("DIVERGENT_JOB index=%zu station=%.6f direction=%d refinements=%u expansions=%u\n",
                            index, jobs[index].referenceStationMeters, jobs[index].direction,
                            warmupResults[index].refinementIterations,
                            warmupResults[index].bracketExpansions);
            }
            const std::size_t iterations = count <= 128 ? 100 : 25;
            const double cpuMicroseconds = averageMicroseconds(iterations, [&]
            {
                for (std::size_t index = 0; index < jobs.size(); ++index)
                {
                    static_cast<void>(quantum::physics::detail::
                        solveCarPoseForValidatedDefinition(
                            cpuTrack, cars[index], locations[index], {}, nullptr));
                }
            });
            GpuRigidBogieBatchTimings accumulated{};
            for (std::size_t iteration = 0; iteration < iterations; ++iteration)
            {
                GpuRigidBogieBatchTimings current;
                static_cast<void>(gpu.solveRigidBogiesGpu(jobs, &current, localSize));
                accumulated.packingUploadMicroseconds += current.packingUploadMicroseconds;
                accumulated.submitDispatchMicroseconds += current.submitDispatchMicroseconds;
                accumulated.fenceWaitMicroseconds += current.fenceWaitMicroseconds;
                accumulated.readbackMicroseconds += current.readbackMicroseconds;
                accumulated.gpuExecutionMicroseconds += current.gpuExecutionMicroseconds;
                accumulated.totalMicroseconds += current.totalMicroseconds;
            }
            accumulated.packingUploadMicroseconds /= iterations;
            accumulated.submitDispatchMicroseconds /= iterations;
            accumulated.fenceWaitMicroseconds /= iterations;
            accumulated.readbackMicroseconds /= iterations;
            accumulated.gpuExecutionMicroseconds /= iterations;
            accumulated.totalMicroseconds /= iterations;
            std::printf("%6zu %10.2f %10.3f %10.0f %10.2f %10.2f %10.2f "
                "%10.2f %10.2f %10.3f %10.0f %9.3f\n", count, cpuMicroseconds,
                cpuMicroseconds / count, accumulated.gpuExecutionMicroseconds,
                accumulated.packingUploadMicroseconds,
                accumulated.submitDispatchMicroseconds,
                accumulated.fenceWaitMicroseconds,
                accumulated.readbackMicroseconds,
                accumulated.totalMicroseconds,
                accumulated.totalMicroseconds / count,
                1.0e6 * count / accumulated.totalMicroseconds,
                cpuMicroseconds / accumulated.totalMicroseconds);
        }
        }
    }
}

int main(const int argc, char** argv)
{
    try
    {
        auto handles = GpuPhysicsContext::createHeadlessHandles();
        GpuPhysicsContext gpu{std::move(handles)};
        if (!gpu.gpuAvailable())
        {
            std::printf("GPU RIGID-BOGIE PROTOTYPE SKIPPED: fp64 GPU unavailable\n");
            return 0;
        }
        const Fixture initial = straightFixture();
        upload(gpu, initial);
        if (!gpu.gpuRigidBogieReady())
            throw std::runtime_error("rigid-bogie compute pipeline unavailable");
        validate(gpu);
        if (argc > 1 && std::string_view{argv[1]} == "--benchmark")
            benchmark(gpu);
        std::printf("GpuRigidBogiePrototypeTests PASSED\n");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "GpuRigidBogiePrototypeTests FAILED: %s\n", error.what());
        return 1;
    }
}
