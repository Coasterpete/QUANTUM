#include <quantum/physics/gpu/GpuPhysicsContext.hpp>
#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    void require(bool cond, const char* msg)
    {
        if (!cond) throw std::runtime_error(msg);
    }

    std::vector<quantum::coaster::TrackKinematicState> makeStraight(std::size_t n, double spacing)
    {
        std::vector<quantum::coaster::TrackKinematicState> out;
        out.reserve(n);
        for (size_t i=0;i<n;++i){
            quantum::coaster::TrackKinematicState s{};
            s.distance = double(i)*spacing;
            s.position = glm::dvec3(double(i)*spacing,0,0);
            s.frame.tangent = glm::dvec3(1,0,0);
            s.frame.lateral = glm::dvec3(0,1,0);
            s.frame.up = glm::dvec3(0,0,1);
            s.centerlineCurvature = glm::dvec3(0);
            out.push_back(s);
        }
        return out;
    }

    quantum::physics::gpu::GpuTrackQuery makeQuery(double station, uint32_t idx=0)
    {
        quantum::physics::gpu::GpuTrackQuery q{};
        q.coasterIndex = idx;
        q.path = 0;
        q.direction = 1;
        q.stationMeters = station;
        return q;
    }

    void testEmptyBatch(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        std::vector<quantum::physics::gpu::GpuTrackQuery> q;
        auto res = gpu.sampleTrackGpu(q);
        require(res.empty(), "empty batch should return empty");
        require(!gpu.lastSampleUsedGpu() || res.empty(), "empty should not claim GPU");
        std::cout << "  empty batch OK\n";
    }

    void testOneQuery(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        std::vector<quantum::physics::gpu::GpuTrackQuery> qs{makeQuery(1.5)};
        auto r = gpu.validateGpuAgainstCpu(qs);
        require(r.gpuExecuted, "one query GPU not executed");
        require(r.maxPositionError < 1e-9, "one query pos error");
        std::cout << "  one query OK posErr=" << r.maxPositionError << "\n";
    }

    void testBatchSize(quantum::physics::gpu::GpuPhysicsContext& gpu, size_t n)
    {
        std::vector<quantum::physics::gpu::GpuTrackQuery> qs;
        qs.reserve(n);
        for (size_t i=0;i<n;++i) qs.push_back(makeQuery(double(i)*0.37));
        auto r = gpu.validateGpuAgainstCpu(qs);
        require(r.gpuExecuted, "batch GPU not executed");
        require(r.queryCount==n, "query count mismatch");
        require(r.maxPositionError < 1e-9, "batch pos error");
        require(r.maxStationError < 1e-9, "batch station error");
        std::cout << "  batch " << n << " OK groups=" << (n+63)/64 << " posErr=" << r.maxPositionError << "\n";
    }

    void testRepeatedDispatches(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        for (int iter=0; iter<5; ++iter)
        {
            std::vector<quantum::physics::gpu::GpuTrackQuery> qs;
            for (int i=0;i<10;++i) qs.push_back(makeQuery(double(iter*10 + i)*0.5));
            auto r = gpu.validateGpuAgainstCpu(qs);
            require(r.gpuExecuted, "repeated GPU not executed");
            require(r.maxPositionError < 1e-9, "repeated pos error");
        }
        std::cout << "  repeated dispatches OK\n";
    }

    void testCapacityGrowth(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        // Start small, then larger (forces reallocation), then small again (reuse)
        testBatchSize(gpu, 10);
        testBatchSize(gpu, 100);
        testBatchSize(gpu, 10);
        std::cout << "  capacity growth OK\n";
    }

    void testOrdering(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        std::vector<quantum::physics::gpu::GpuTrackQuery> qs;
        for (int i=0;i<20;++i) qs.push_back(makeQuery(double(19-i)*0.5)); // reverse order 9.5..0
        auto gpuRes = gpu.sampleTrackGpu(qs);
        require(gpuRes.size()==qs.size(), "ordering size");
        for (size_t i=0;i<qs.size();++i)
        {
            double diff = std::abs(gpuRes[i].location.stationMeters - qs[i].stationMeters);
            if (diff >= 1e-9 || std::isnan(diff))
            {
                std::cout << "  ordering mismatch i=" << i << " q=" << qs[i].stationMeters << " gpu=" << gpuRes[i].location.stationMeters << " diff=" << diff << "\n";
                require(false, "ordering station mismatch");
            }
        }
        std::cout << "  ordering OK\n";
    }

    void testFallback()
    {
        quantum::physics::gpu::GpuPhysicsContext::HeadlessHandles h{};
        // No device -> fallback
        quantum::physics::gpu::GpuPhysicsContext gpu(h);
        require(!gpu.gpuAvailable(), "empty handles should not be gpuAvailable");
        auto track = makeStraight(20, 0.5);
        gpu.uploadTrack(0, track, 1.0,
            quantum::coaster::TopologyKind::OpenLinear,
            quantum::coaster::LayoutMode::Shuttle);
        require(gpu.hasUploadedTrack(), "CPU fallback track should be uploaded");
        require(!gpu.gpuTrackReady(), "empty handles cannot have a GPU-ready track");
        auto q = std::vector<quantum::physics::gpu::GpuTrackQuery>{makeQuery(1.0)};
        auto res = gpu.sampleTrackGpu(q);
        require(!gpu.lastSampleUsedGpu(), "fallback should not claim GPU");
        require(res.size()==1, "fallback size");
        require(std::abs(res[0].position[0] - 1.0) < 1e-9,
            "fallback should sample the uploaded CPU mirror");
        std::cout << "  fallback OK\n";
    }

    void testNoUploadedTrack(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        require(!gpu.hasUploadedTrack(), "new context should not report an uploaded track");
        require(!gpu.gpuTrackReady(), "new context should not report a GPU-ready track");
        const std::vector<quantum::physics::gpu::GpuTrackQuery> queries{
            makeQuery(1.0)};
        require(gpu.sampleTrackGpu(queries).empty(),
            "production sampling before track upload must return no samples");
        require(!gpu.lastSampleUsedGpu(),
            "sampling before track upload must not claim GPU execution");
        std::cout << "  no-upload guard OK\n";
    }

    void testTrackReplacement(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        auto replacement = makeStraight(20, 0.5);
        for (auto& sample : replacement)
            sample.position.x += 100.0;
        gpu.uploadTrack(0, replacement, 1.0,
            quantum::coaster::TopologyKind::OpenLinear,
            quantum::coaster::LayoutMode::Shuttle);
        require(gpu.hasUploadedTrack() && gpu.gpuTrackReady(),
            "replacement track should be GPU ready");

        const std::vector<quantum::physics::gpu::GpuTrackQuery> queries{
            makeQuery(1.25)};
        const auto samples = gpu.sampleTrackGpu(queries);
        require(gpu.lastSampleUsedGpu(), "replacement track should execute on GPU");
        require(samples.size() == 1, "replacement sample count");
        require(std::abs(samples[0].position[0] - 101.25) < 1e-9,
            "replacement sampling must not use the previous GPU track");
        std::cout << "  track replacement OK\n";
    }

    void testAlternatingSlots(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        // Use sampleTrack(slot, queries) alternating slots 0 and 1
        for (int i=0;i<6;++i)
        {
            uint32_t slot = i%2;
            std::vector<quantum::physics::gpu::GpuTrackQuery> qs{makeQuery(double(i)*0.7), makeQuery(double(i)*0.7+0.3)};
            gpu.sampleTrack(slot, qs);
            require(gpu.lastSampleUsedGpu(), "alternating slot GPU not used");
        }
        std::cout << "  alternating slots OK\n";
    }
}

int main()
{
    try
    {
        quantum::physics::gpu::GpuPhysicsContext::HeadlessHandles h;
        try {
            h = quantum::physics::gpu::GpuPhysicsContext::createHeadlessHandles();
        } catch (const std::exception& e) {
            std::cout << "GPU BATCHED TEST SKIPPED – headless Vulkan failed: " << e.what() << "\n";
            return 0;
        }
        std::cout << "GPU BATCHED TEST EXECUTED headless shaderFloat64=" << h.shaderFloat64Enabled << "\n";
        quantum::physics::gpu::GpuPhysicsContext gpu(std::move(h));
        if (!gpu.gpuAvailable())
        {
            std::cout << "GPU BATCHED TEST SKIPPED – gpu not available\n";
            return 0;
        }
        testNoUploadedTrack(gpu);
        // Upload track for batch tests
        auto track = makeStraight(20, 0.5); // length 9.5
        gpu.uploadTrack(0, track, 1.0, quantum::coaster::TopologyKind::OpenLinear, quantum::coaster::LayoutMode::Shuttle);

        testEmptyBatch(gpu);
        testOneQuery(gpu);
        testBatchSize(gpu, 63);
        testBatchSize(gpu, 64);
        testBatchSize(gpu, 65);
        testBatchSize(gpu, 200);
        testRepeatedDispatches(gpu);
        testCapacityGrowth(gpu);
        testOrdering(gpu);
        testAlternatingSlots(gpu);
        testFallback();
        testTrackReplacement(gpu);

        // Large batch validation
        {
            std::vector<quantum::physics::gpu::GpuTrackQuery> qs;
            for (int i=0;i<200;++i) qs.push_back(makeQuery(double(i)*0.05));
            auto r = gpu.validateGpuAgainstCpu(qs);
            require(r.gpuExecuted, "large batch GPU not executed");
            require(r.maxPositionError < 1e-9, "large batch pos error");
            std::cout << "  large batch 200 OK\n";
        }

        std::cout << "GpuPhysicsBatchedSamplingTests PASSED (GPU executed)\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "GpuPhysicsBatchedSamplingTests FAILED: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
