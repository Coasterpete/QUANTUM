#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/physics/gpu/GpuPhysicsContext.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <numbers>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::TrackKinematicState;
    using quantum::physics::gpu::GpuPhysicsContext;
    using quantum::physics::gpu::GpuTrackQuery;

    [[nodiscard]] std::vector<TrackKinematicState> circleTrack()
    {
        constexpr std::size_t sampleCount = 720;
        constexpr double radiusMeters = 25.0;
        std::vector<TrackKinematicState> samples;
        samples.reserve(sampleCount + 1);
        for (std::size_t index = 0; index <= sampleCount; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / sampleCount;
            samples.push_back({
                radiusMeters * angle,
                {radiusMeters * std::sin(angle),
                    radiusMeters * (1.0 - std::cos(angle)), 0.0},
                {{std::cos(angle), std::sin(angle), 0.0},
                    {-std::sin(angle), std::cos(angle), 0.0},
                    {0.0, 0.0, 1.0}},
                {-std::sin(angle) / radiusMeters,
                    std::cos(angle) / radiusMeters, 0.0}
            });
        }
        return samples;
    }

    [[nodiscard]] std::vector<GpuTrackQuery> queries(
        const std::size_t count,
        const double trackLengthMeters)
    {
        std::vector<GpuTrackQuery> result(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            result[index].direction = 1;
            result[index].stationMeters = std::fmod(
                0.371 * static_cast<double>(index), trackLengthMeters);
        }
        return result;
    }

    template<typename Function>
    [[nodiscard]] double averageMicroseconds(
        const std::size_t iterations,
        Function&& function)
    {
        const auto begin = std::chrono::steady_clock::now();
        for (std::size_t iteration = 0; iteration < iterations; ++iteration)
        {
            function();
        }
        return std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - begin).count()
            / static_cast<double>(iterations);
    }
}

int main()
{
    GpuPhysicsContext::HeadlessHandles handles;
    try
    {
        handles = GpuPhysicsContext::createHeadlessHandles();
    }
    catch (const std::exception& error)
    {
        std::printf("GPU sampling performance benchmark skipped: %s\n",
            error.what());
        return 0;
    }

    GpuPhysicsContext gpu{std::move(handles)};
    const auto track = circleTrack();
    gpu.uploadTrack(0, track, 1.0,
        quantum::coaster::TopologyKind::ClosedCircuit,
        quantum::coaster::LayoutMode::Circuit);
    if (!gpu.gpuAvailable() || !gpu.gpuTrackReady())
    {
        std::printf(
            "GPU sampling performance benchmark skipped: fp64 GPU path unavailable\n");
        return 0;
    }

    std::printf("QUANTUM synchronous GPU track-sampling benchmark\n");
    std::printf("%10s %16s %16s %16s\n",
        "queries", "GPU us/dispatch", "CPU us/batch", "GPU ns/query");
    for (const std::size_t count : {1u, 2u, 6u, 24u, 64u, 256u, 1024u})
    {
        const auto batch = queries(count, track.back().distance);
        static_cast<void>(gpu.sampleTrackGpu(batch));
        const std::size_t iterations = count <= 64 ? 200 : 50;
        const double gpuMicroseconds = averageMicroseconds(
            iterations,
            [&] { static_cast<void>(gpu.sampleTrackGpu(batch)); });
        const double cpuMicroseconds = averageMicroseconds(
            iterations,
            [&] { static_cast<void>(gpu.sampleTrackForValidation(batch)); });
        std::printf("%10zu %16.2f %16.2f %16.2f\n",
            count,
            gpuMicroseconds,
            cpuMicroseconds,
            1'000.0 * gpuMicroseconds / static_cast<double>(count));
    }
    return 0;
}
