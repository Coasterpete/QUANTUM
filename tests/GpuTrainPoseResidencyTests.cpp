#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/physics/gpu/GpuPhysicsContext.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::LayoutMode;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;
    using namespace quantum::physics;
    using namespace quantum::physics::gpu;

    struct Fixture
    {
        std::string name;
        std::vector<TrackKinematicState> samples;
        TopologyKind topology = TopologyKind::OpenLinear;
    };

    struct Totals
    {
        std::size_t poses = 0;
        std::size_t statusMismatches = 0;
        std::size_t counterDifferences = 0;
        std::size_t fallbackDifferences = 0;
        double maxStation = 0.0;
        double maxPosition = 0.0;
        double maxOrientationRadians = 0.0;
        double maxConnectorResidual = 0.0;
        double maxCenterOfGravity = 0.0;
        bool sawExhaustive = false;
        bool sawTangent = false;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    [[nodiscard]] CurveFrame frameForTangent(
        const glm::dvec3 tangent, const double bank = 0.0)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        const CurveFrame base{unit, {0.0, 1.0, 0.0},
            glm::normalize(glm::cross(unit, glm::dvec3{0.0, 1.0, 0.0}))};
        return quantum::geometry::applyRoll(base, bank);
    }

    [[nodiscard]] Fixture straightFixture(const double length = 200.0)
    {
        const CurveFrame frame = frameForTangent({1.0, 0.0, 0.0});
        return {"straight", {
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {length, {length, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}}},
            TopologyKind::OpenLinear};
    }

    [[nodiscard]] Fixture circleFixture(
        const double radius = 25.0, const bool varyingBank = false)
    {
        constexpr int count = 720;
        Fixture result{"horizontal-circle", {}, TopologyKind::ClosedCircuit};
        result.samples.reserve(count + 1);
        for (int index = 0; index <= count; ++index)
        {
            const double angle = 2.0 * std::numbers::pi * index / count;
            CurveFrame frame{{std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0}, {0.0, 0.0, 1.0}};
            if (varyingBank)
                frame = quantum::geometry::applyRoll(
                    frame, 0.35 * std::sin(2.0 * angle));
            result.samples.push_back({radius * angle,
                {radius * std::sin(angle),
                    radius * (1.0 - std::cos(angle)), 0.0}, frame,
                {-std::sin(angle) / radius,
                    std::cos(angle) / radius, 0.0}});
        }
        return result;
    }

    [[nodiscard]] Fixture verticalFixture(const bool crest)
    {
        constexpr double radius = 24.0;
        constexpr double start = -0.9;
        constexpr double end = 0.9;
        constexpr int count = 600;
        const double sign = crest ? 1.0 : -1.0;
        Fixture result{crest ? "vertical-crest" : "vertical-valley", {},
            TopologyKind::OpenLinear};
        for (int index = 0; index <= count; ++index)
        {
            const double angle = start + (end - start) * index / count;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, -sign * std::sin(angle)};
            result.samples.push_back({radius * (angle - start),
                {radius * std::sin(angle), 0.0,
                    sign * radius * std::cos(angle)},
                {tangent, {0.0, 1.0, 0.0},
                    glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})},
                {-std::sin(angle) / radius, 0.0,
                    -sign * std::cos(angle) / radius}});
        }
        return result;
    }

    [[nodiscard]] CarDefinition carDefinition()
    {
        CarDefinition car;
        car.dryMassKilograms = 800.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.65};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.25};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.25};
        car.bogies = {BogieDefinition{{-1.15, 0.0, 0.0}},
            BogieDefinition{{1.15, 0.0, 0.0}}};
        return car;
    }

    [[nodiscard]] TrainDefinition trainOf(
        const std::size_t count = 4, const double connectorLength = 0.5)
    {
        TrainDefinition train;
        for (std::size_t index = 0; index < count; ++index)
        {
            train.cars.push_back({carDefinition(),
                CarLoadout{200.0, {0.2, 0.0, 0.85}}});
            if (index != 0) train.connections.push_back({connectorLength});
        }
        return train;
    }

    [[nodiscard]] TrackLocation locationFor(
        const GpuTrainPoseJob& job)
    {
        return {primaryTrackPathId, job.referenceStationMeters,
            job.direction < 0 ? TravelDirection::DecreasingStation
                              : TravelDirection::IncreasingStation};
    }

    [[nodiscard]] glm::dvec3 vector3(const double (&value)[4])
    {
        return {value[0], value[1], value[2]};
    }

    [[nodiscard]] glm::dquat quaternion(const double (&value)[4])
    {
        return {value[0], value[1], value[2], value[3]};
    }

    [[nodiscard]] double orientationError(
        glm::dquat left, glm::dquat right)
    {
        left = glm::normalize(left);
        right = glm::normalize(right);
        const double cosine = std::clamp(
            std::abs(glm::dot(left, right)), 0.0, 1.0);
        return 2.0 * std::acos(cosine);
    }

    void upload(GpuPhysicsContext& gpu, const Fixture& fixture)
    {
        gpu.uploadTrack(0, fixture.samples, 1.0, fixture.topology,
            fixture.topology == TopologyKind::ClosedCircuit
                ? LayoutMode::Circuit : LayoutMode::Shuttle);
    }

    void compareSolved(
        const TrainPose& cpu,
        const TrainSolveCounters& cpuCounters,
        const GpuTrainPoseResult& gpu,
        Totals& totals)
    {
        ++totals.poses;
        totals.statusMismatches += gpu.status != GpuTrainPoseStatus::Solved;
        require(gpu.status == GpuTrainPoseStatus::Solved,
            "GPU train-pose status mismatch");
        require(gpu.carCount == cpu.carCount()
                && gpu.connectionCount == cpu.connectionCount(),
            "GPU train-pose shape mismatch");
        for (std::size_t index = 0; index < cpu.carCount(); ++index)
        {
            const CarPose& expected = cpu.cars()[index].carPose();
            const GpuTrainCarPose& actual = gpu.cars[index];
            totals.maxStation = std::max({totals.maxStation,
                std::abs(actual.referenceStationMeters
                    - expected.referenceLocation().stationMeters),
                std::abs(actual.frontBogieStationMeters
                    - expected.frontBogie().location().stationMeters),
                std::abs(actual.rearBogieStationMeters
                    - expected.rearBogie().location().stationMeters)});
            for (const auto& pair : {
                std::pair{vector3(actual.bodyWorldPositionMeters),
                    expected.bodyWorldPositionMeters()},
                std::pair{vector3(actual.worldCenterOfGravityMeters),
                    expected.worldCenterOfGravityMeters()},
                std::pair{vector3(actual.frontHitchWorldPositionMeters),
                    expected.frontHitchWorldPositionMeters()},
                std::pair{vector3(actual.rearHitchWorldPositionMeters),
                    expected.rearHitchWorldPositionMeters()},
                std::pair{vector3(actual.frontBogieWorldPositionMeters),
                    expected.frontBogie().worldPositionMeters()},
                std::pair{vector3(actual.rearBogieWorldPositionMeters),
                    expected.rearBogie().worldPositionMeters()}})
                totals.maxPosition = std::max(
                    totals.maxPosition, glm::length(pair.first - pair.second));
            totals.maxCenterOfGravity = std::max(totals.maxCenterOfGravity,
                glm::length(vector3(actual.worldCenterOfGravityMeters)
                    - expected.worldCenterOfGravityMeters()));
            totals.maxOrientationRadians = std::max(
                totals.maxOrientationRadians,
                orientationError(quaternion(actual.bodyOrientationWxyz),
                    expected.bodyOrientation()));
        }
        for (std::size_t index = 0; index < cpu.connectionCount(); ++index)
        {
            const auto& expected = cpu.connections()[index];
            const auto& actual = gpu.connections[index];
            totals.maxConnectorResidual = std::max(
                totals.maxConnectorResidual,
                std::abs(actual.signedLengthResidualMeters
                    - expected.signedLengthResidualMeters()));
            totals.fallbackDifferences +=
                (actual.usedExhaustiveSearchFallback != 0)
                != expected.usedExhaustiveSearchFallback();
            totals.sawExhaustive |= expected.usedExhaustiveSearchFallback();
            totals.sawTangent |= expected.usedExhaustiveSearchFallback()
                && expected.authoredRigidLengthMeters() == 0.0;
        }
        totals.maxCenterOfGravity = std::max(totals.maxCenterOfGravity,
            glm::length(vector3(gpu.aggregateWorldCenterOfGravityMeters)
                - cpu.aggregateWorldCenterOfGravityMeters()));
        totals.counterDifferences += gpu.rigidSolveCount
                    != cpuCounters.rigidBogieSolveCalls
            || gpu.rigidRefinementCount
                    != cpuCounters.rigidBogieRefinementIterations
            || gpu.rigidBracketExpansionCount
                    != cpuCounters.rigidBogieBracketExpansions
            || gpu.connectorCandidateEvaluationCount
                    != cpuCounters.connectionCandidateEvaluations
            || gpu.connectorRefinementCount
                    != cpuCounters.connectorRefinementIterations
            || gpu.connectorFallbackCount
                    != cpuCounters.connectorFallbackUses;
    }

    void validateFixture(
        GpuPhysicsContext& gpu,
        const Fixture& fixture,
        const TrainDefinition& train,
        const std::vector<GpuTrainPoseJob>& jobs,
        Totals& totals)
    {
        upload(gpu, fixture);
        gpu.uploadTrainDefinition(0, train);
        const CompiledPhysicsTrack track{
            fixture.samples, 1.0, fixture.topology};
        const auto results = gpu.solveTrainPosesGpu(jobs);
        require(results.size() == jobs.size(), "GPU pose result count");
        for (std::size_t index = 0; index < jobs.size(); ++index)
        {
            TrainSolveCounters counters;
            const TrainPose cpu = solveTrainPose(
                track, train, locationFor(jobs[index]), &counters);
            compareSolved(cpu, counters, results[index], totals);
        }
    }

    [[nodiscard]] GpuTrainPoseJob job(
        const std::uint32_t id, const double station, const int direction = 1)
    {
        GpuTrainPoseJob result;
        result.jobId = id;
        result.referenceStationMeters = station;
        result.direction = direction;
        return result;
    }

    void validate(GpuPhysicsContext& gpu)
    {
        Totals totals;
        validateFixture(gpu, straightFixture(), trainOf(),
            {job(1, 40.0), job(2, 150.0, -1)}, totals);
        validateFixture(gpu, circleFixture(), trainOf(),
            {job(3, 30.0), job(4, 1.0), job(5, 100.0, -1)}, totals);
        validateFixture(gpu, circleFixture(25.0, true), trainOf(),
            {job(6, 30.0)}, totals);
        validateFixture(gpu, verticalFixture(false), trainOf(3),
            {job(7, 25.0)}, totals);
        validateFixture(gpu, verticalFixture(true), trainOf(3),
            {job(8, 25.0)}, totals);
        validateFixture(gpu, straightFixture(), trainOf(2, 0.0),
            {job(9, 60.0)}, totals);

        const Fixture open = straightFixture(40.0);
        upload(gpu, open);
        gpu.uploadTrainDefinition(0, trainOf(3));
        const auto openResult = gpu.solveTrainPosesGpu(
            std::array{job(10, 5.0)}).front();
        require(openResult.status == GpuTrainPoseStatus::ConnectorOpenEndpoint
                || openResult.status == GpuTrainPoseStatus::OpenTrackPlacement,
            "open-track boundary status");

        TrainDefinition impossible = trainOf(2, 0.0);
        impossible.cars[1].car.frontHitchPositionMeters.y = 1.0;
        gpu.uploadTrainDefinition(0, impossible);
        const auto impossibleResult = gpu.solveTrainPosesGpu(
            std::array{job(11, 20.0)}).front();
        require(impossibleResult.status
                == GpuTrainPoseStatus::ConnectorCannotClose,
            "connector failure status");

        gpu.uploadTrainDefinition(0, trainOf());
        const auto invalidResult = gpu.solveTrainPosesGpu(std::array{
            job(12, std::numeric_limits<double>::quiet_NaN())}).front();
        require(invalidResult.status == GpuTrainPoseStatus::InvalidJob,
            "non-finite input status");

        std::printf("VALIDATION poses=%zu status_mismatches=%zu "
            "max_station=%.17g max_position=%.17g max_orientation_rad=%.17g "
            "max_connector_residual=%.17g max_cog=%.17g "
            "counter_differences=%zu fallback_differences=%zu "
            "exhaustive=%s tangent=%s\n",
            totals.poses, totals.statusMismatches, totals.maxStation,
            totals.maxPosition, totals.maxOrientationRadians,
            totals.maxConnectorResidual, totals.maxCenterOfGravity,
            totals.counterDifferences, totals.fallbackDifferences,
            totals.sawExhaustive ? "yes" : "no",
            totals.sawTangent ? "yes" : "no");
        require(totals.statusMismatches == 0, "status equivalence");
        require(totals.maxStation <= 2.0e-8, "station equivalence");
        require(totals.maxPosition <= 2.0e-8, "position equivalence");
        require(totals.maxOrientationRadians <= 2.0e-7,
            "orientation equivalence");
        require(totals.maxConnectorResidual <= 2.0e-9,
            "connector residual equivalence");
        require(totals.maxCenterOfGravity <= 2.0e-8,
            "center-of-gravity equivalence");
        require(totals.counterDifferences == 0, "counter equivalence");
        require(totals.fallbackDifferences == 0, "fallback equivalence");
    }

    [[nodiscard]] GpuTrainPoseBatchTimings averageGpu(
        GpuPhysicsContext& gpu,
        const std::vector<GpuTrainPoseJob>& jobs,
        const std::size_t iterations)
    {
        static_cast<void>(gpu.solveTrainPosesGpu(jobs));
        GpuTrainPoseBatchTimings total;
        for (std::size_t iteration = 0; iteration < iterations; ++iteration)
        {
            GpuTrainPoseBatchTimings one;
            const auto result = gpu.solveTrainPosesGpu(jobs, &one);
            require(std::all_of(result.begin(), result.end(), [](const auto& value)
                { return value.status == GpuTrainPoseStatus::Solved; }),
                "benchmark pose solve");
            total.packingUploadMicroseconds += one.packingUploadMicroseconds;
            total.submitDispatchMicroseconds += one.submitDispatchMicroseconds;
            total.fenceWaitMicroseconds += one.fenceWaitMicroseconds;
            total.readbackMicroseconds += one.readbackMicroseconds;
            total.gpuExecutionMicroseconds += one.gpuExecutionMicroseconds;
            total.totalMicroseconds += one.totalMicroseconds;
        }
        total.packingUploadMicroseconds /= iterations;
        total.submitDispatchMicroseconds /= iterations;
        total.fenceWaitMicroseconds /= iterations;
        total.readbackMicroseconds /= iterations;
        total.gpuExecutionMicroseconds /= iterations;
        total.totalMicroseconds /= iterations;
        return total;
    }

    void benchmark(GpuPhysicsContext& gpu)
    {
        const Fixture fixture = circleFixture();
        const TrainDefinition train = trainOf();
        const CompiledPhysicsTrack track{
            fixture.samples, 1.0, fixture.topology};
        upload(gpu, fixture);
        gpu.uploadTrainDefinition(0, train);
        const std::vector<GpuTrainPoseJob> base{
            job(1, 30.0), job(2, 29.99), job(3, 30.01), job(4, 30.02),
            job(5, 35.0), job(6, 40.0), job(7, 45.0), job(8, 50.0)};
        constexpr std::size_t iterations = 50;
        std::printf("PERFORMANCE result_bytes_per_job=%zu job_bytes=%zu\n",
            sizeof(GpuTrainPoseResult), sizeof(GpuTrainPoseJob));
        for (const std::size_t count : {1u, 3u, 4u, 8u})
        {
            const std::vector<GpuTrainPoseJob> jobs(
                base.begin(), base.begin() + count);
            const auto cpuBegin = std::chrono::steady_clock::now();
            for (std::size_t iteration = 0; iteration < iterations; ++iteration)
                for (const auto& current : jobs)
                    static_cast<void>(solveTrainPose(
                        track, train, locationFor(current)));
            const double cpu = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - cpuBegin).count()
                / iterations;
            const auto timing = averageGpu(gpu, jobs, iterations);
            std::printf("PERF jobs=%zu cpu_us=%.3f gpu_exec_us=%.3f "
                "upload_us=%.3f submit_us=%.3f wait_us=%.3f "
                "readback_us=%.3f total_us=%.3f speedup=%.3f\n",
                count, cpu, timing.gpuExecutionMicroseconds,
                timing.packingUploadMicroseconds,
                timing.submitDispatchMicroseconds,
                timing.fenceWaitMicroseconds, timing.readbackMicroseconds,
                timing.totalMicroseconds, cpu / timing.totalMicroseconds);
        }

        TrainDynamicsState state;
        state.generalizedReferenceLocation = locationFor(base.front());
        state.signedVelocityMetersPerSecond = 12.0;
        state.runState = FollowerRunState::Running;
        const TrainStepResult seedStep = stepTrain(track, train, {}, state);
        const GpuTrainPoseJob committed = job(20,
            seedStep.state.generalizedReferenceLocation.stationMeters);
        const auto gpuThree = averageGpu(gpu,
            {base[0], base[1], base[2]}, iterations);
        const auto gpuCommitted = averageGpu(gpu, {committed}, iterations);
        TrainSolveCounters counters;
        const auto stepBegin = std::chrono::steady_clock::now();
        for (std::size_t iteration = 0; iteration < iterations; ++iteration)
            static_cast<void>(stepTrain(track, train, {}, state, {}, {}, &counters));
        const double cpuStep = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - stepBegin).count() / iterations;
        const double cpuPose = static_cast<double>(
            counters.solveTrainPoseNanoseconds) / 1000.0 / iterations;
        const double cpuNonPose = std::max(0.0, cpuStep - cpuPose);
        const double modeled = gpuThree.totalMicroseconds + cpuNonPose
            + gpuCommitted.totalMicroseconds;
        std::printf("STEP_MODEL cpu_step_us=%.3f cpu_pose_us=%.3f "
            "cpu_non_pose_us=%.3f gpu_three_us=%.3f gpu_committed_us=%.3f "
            "modeled_gpu_step_us=%.3f speedup=%.3f\n",
            cpuStep, cpuPose, cpuNonPose, gpuThree.totalMicroseconds,
            gpuCommitted.totalMicroseconds, modeled, cpuStep / modeled);
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
            std::printf("GPU TRAIN-POSE PROTOTYPE SKIPPED: fp64 GPU unavailable\n");
            return 0;
        }
        upload(gpu, straightFixture());
        gpu.uploadTrainDefinition(0, trainOf());
        require(gpu.gpuTrainPoseReady(), "train-pose pipeline unavailable");
        validate(gpu);
        if (argc > 1 && std::string_view{argv[1]} == "--benchmark")
            benchmark(gpu);
        std::printf("GpuTrainPoseResidencyTests PASSED\n");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "GpuTrainPoseResidencyTests FAILED: %s\n",
            error.what());
        return 1;
    }
}
