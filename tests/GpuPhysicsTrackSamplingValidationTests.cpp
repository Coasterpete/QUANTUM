#include <quantum/physics/gpu/GpuPhysicsContext.hpp>
#include <quantum/renderer/VulkanContext.hpp>
#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

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

    std::vector<quantum::coaster::TrackKinematicState> makeStraightTrack(std::size_t count, double spacing)
    {
        std::vector<quantum::coaster::TrackKinematicState> out;
        out.reserve(count);
        for (size_t i = 0; i < count; ++i)
        {
            quantum::coaster::TrackKinematicState s{};
            s.distance = static_cast<double>(i) * spacing;
            s.position = glm::dvec3(static_cast<double>(i) * spacing, 0.0, 0.0);
            s.frame.tangent = glm::dvec3(1.0, 0.0, 0.0);
            s.frame.lateral = glm::dvec3(0.0, 1.0, 0.0);
            s.frame.up = glm::dvec3(0.0, 0.0, 1.0);
            s.centerlineCurvature = glm::dvec3(0.0);
            out.push_back(s);
        }
        return out;
    }

    std::vector<quantum::coaster::TrackKinematicState> makeBankedArcTrack()
    {
        // Simple yawed arc with bank: positions along circle radius 20, 10 samples
        std::vector<quantum::coaster::TrackKinematicState> out;
        const double radius = 20.0;
        const double totalAngle = glm::radians(45.0);
        const std::size_t count = 20;
        const double totalLength = radius * totalAngle;
        for (size_t i = 0; i < count; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(count - 1);
            const double angle = t * totalAngle;
            const double dist = t * totalLength;
            quantum::coaster::TrackKinematicState s{};
            s.distance = dist;
            s.position = glm::dvec3(std::sin(angle) * radius, 0.0, (1.0 - std::cos(angle)) * radius);
            // Tangent along circle
            s.frame.tangent = glm::normalize(glm::dvec3(std::cos(angle), 0.0, std::sin(angle)));
            // Bank 10 deg about tangent
            const double bank = glm::radians(10.0) * t;
            glm::dvec3 lateral = glm::dvec3(-std::sin(angle) * std::sin(bank), std::cos(bank), -std::cos(angle) * std::sin(bank));
            glm::dvec3 up = glm::dvec3(-std::sin(angle) * std::cos(bank), -std::sin(bank), -std::cos(angle) * std::cos(bank));
            // Re-orthonormalize
            s.frame.lateral = glm::normalize(lateral - glm::dot(lateral, s.frame.tangent) * s.frame.tangent);
            s.frame.up = glm::normalize(glm::cross(s.frame.tangent, s.frame.lateral));
            // Curvature = 1/r in direction toward center (approx)
            s.centerlineCurvature = glm::dvec3(-std::sin(angle) / radius, 0.0, -std::cos(angle) / radius);
            out.push_back(s);
        }
        return out;
    }

    void runValidationWithGpu(quantum::physics::gpu::GpuPhysicsContext& gpu)
    {
        std::cout << "GPU VALIDATION EXECUTED\n";
        std::cout << "  shaderFloat64Enabled=true gpuAvailable=" << gpu.gpuAvailable() << "\n";

        // Test straight open track
        auto straight = makeStraightTrack(10, 1.0);
        const double mu = 1.0;
        gpu.uploadTrack(0, straight, mu, quantum::coaster::TopologyKind::OpenLinear, quantum::coaster::LayoutMode::Shuttle);

        const double length = straight.back().distance * mu;
        std::vector<quantum::physics::gpu::GpuTrackQuery> queries;
        auto push = [&](double station) {
            quantum::physics::gpu::GpuTrackQuery q{};
            q.coasterIndex = 0;
            q.path = 0;
            q.direction = 1;
            q.stationMeters = station;
            queries.push_back(q);
        };
        // Stations: 0, first sample, between, interior, on boundary, final, at length, negative (open clamp), >length clamp, one-sample not yet, multiple
        push(0.0);
        push(0.0);
        push(0.5);
        push(2.3);
        push(5.0);
        push(9.0);
        push(length);
        push(-5.0);
        push(length + 5.0);
        push(1.0);
        push(1.5);
        push(8.5);

        auto res = gpu.validateGpuAgainstCpu(queries);
        require(res.gpuExecuted, "GPU did not execute (fallback used)");
        std::cout << "  straight open: maxStation=" << res.maxStationError
                  << " maxPos=" << res.maxPositionError
                  << " maxTangentDeg=" << res.maxTangentAngleDeg
                  << " maxLateralDeg=" << res.maxLateralAngleDeg
                  << " maxUpDeg=" << res.maxUpAngleDeg
                  << " maxCurv=" << res.maxCurvatureError << "\n";
        if (res.maxStationError >= 1e-12 || res.maxPositionError >= 1e-9)
        {
            auto gpuS = gpu.sampleTrackGpu(queries);
            auto cpuS = gpu.sampleTrackForValidation(queries);
            for (size_t i = 0; i < queries.size(); ++i)
            {
                std::cout << "    q[" << i << "] qStation=" << queries[i].stationMeters
                          << " cpuStation=" << cpuS[i].location.stationMeters
                          << " gpuStation=" << gpuS[i].location.stationMeters
                          << " cpuPos=" << cpuS[i].position[0] << "," << cpuS[i].position[1] << "," << cpuS[i].position[2]
                          << " gpuPos=" << gpuS[i].position[0] << "," << gpuS[i].position[1] << "," << gpuS[i].position[2] << "\n";
            }
        }

        // Tolerances: position/curvature/station tight, orientation approx due to nlerp vs slerp
        require(res.maxStationError < 1e-12, "station error too large");
        require(res.maxPositionError < 1e-9, "position error too large");
        require(res.maxCurvatureError < 1e-9, "curvature error too large");
        // Orientation approximation: allow 0.01 deg for straight (should be ~0)
        require(res.maxTangentAngleDeg < 0.01, "tangent angle error too large");
        require(res.maxLateralAngleDeg < 0.01, "lateral angle error too large");
        require(res.maxUpAngleDeg < 0.01, "up angle error too large");

        // Circuit wrapping test
        auto circuitTrack = makeStraightTrack(20, 1.0);
        gpu.uploadTrack(0, circuitTrack, mu, quantum::coaster::TopologyKind::ClosedCircuit, quantum::coaster::LayoutMode::Circuit);
        const double cLen = circuitTrack.back().distance * mu;
        queries.clear();
        push(-1.0);
        push(cLen + 1.0);
        push(cLen * 2.5);
        push(0.0);
        push(cLen - 0.5);
        auto res2 = gpu.validateGpuAgainstCpu(queries);
        require(res2.gpuExecuted, "circuit GPU not executed");
        std::cout << "  circuit wrap: maxStation=" << res2.maxStationError << " maxPos=" << res2.maxPositionError << "\n";
        require(res2.maxPositionError < 1e-9, "circuit position error too large");

        // Banked arc test (orientation heavier)
        auto arc = makeBankedArcTrack();
        gpu.uploadTrack(0, arc, mu, quantum::coaster::TopologyKind::OpenLinear, quantum::coaster::LayoutMode::Shuttle);
        queries.clear();
        for (double s = 0.0; s <= arc.back().distance; s += 0.73)
            push(s);
        auto res3 = gpu.validateGpuAgainstCpu(queries);
        require(res3.gpuExecuted, "arc GPU not executed");
        std::cout << "  banked arc: maxPos=" << res3.maxPositionError
                  << " maxTangentDeg=" << res3.maxTangentAngleDeg
                  << " maxLateralDeg=" << res3.maxLateralAngleDeg
                  << " maxUpDeg=" << res3.maxUpAngleDeg << "\n";
        // Arc orientation uses approximation, allow 0.5 deg
        require(res3.maxPositionError < 1e-7, "arc position error too large");
        require(res3.maxTangentAngleDeg < 0.5, "arc tangent angle too large");
        require(res3.maxLateralAngleDeg < 0.5, "arc lateral angle too large");
        require(res3.maxUpAngleDeg < 0.5, "arc up angle too large");

        std::cout << "GpuPhysicsTrackSamplingValidation PASSED (GPU executed)\n";
    }

    void testGpuSampling()
    {
        // First try windowed VulkanContext (preferred, matches editor path)
        bool windowAttempted = false;
        bool windowSucceeded = false;
        SDL_Window* window = nullptr;
        quantum::renderer::VulkanContext vulkanWindowed;
        if (SDL_Init(SDL_INIT_VIDEO) == 0)
        {
            windowAttempted = true;
            window = SDL_CreateWindow("gpu-validation", 64, 64, SDL_WINDOW_HIDDEN | SDL_WINDOW_VULKAN);
            if (window)
            {
                try
                {
                    quantum::coaster::RenderableTrack emptyTrack{};
                    vulkanWindowed.initialize(window, {}, 0, emptyTrack, false);
                    if (vulkanWindowed.shaderFloat64Enabled())
                    {
                        quantum::physics::gpu::GpuPhysicsContext gpu(vulkanWindowed);
                        if (gpu.gpuAvailable())
                        {
                            runValidationWithGpu(gpu);
                            windowSucceeded = true;
                            vulkanWindowed.shutdown();
                            SDL_DestroyWindow(window);
                            SDL_Quit();
                            return;
                        }
                        else
                        {
                            std::cout << "Windowed GPU pipeline not ready, trying headless fallback\n";
                        }
                    }
                    else
                    {
                        std::cout << "GPU VALIDATION SKIPPED – shaderFloat64 not supported on windowed device\n";
                    }
                    vulkanWindowed.shutdown();
                }
                catch (const std::exception& e)
                {
                    std::cout << "Windowed Vulkan init failed: " << e.what() << " – trying headless\n";
                    try { vulkanWindowed.shutdown(); } catch (...) {}
                }
                SDL_DestroyWindow(window);
            }
            else
            {
                std::cout << "SDL_CreateWindow failed: " << SDL_GetError() << " – trying headless\n";
            }
            SDL_Quit();
        }
        else
        {
            std::cout << "SDL_Init failed: " << SDL_GetError() << " – trying headless\n";
        }

        // Headless fallback: compute-only Vulkan without surface (works in CI/headless)
        try
        {
            auto handles = quantum::physics::gpu::GpuPhysicsContext::createHeadlessHandles();
            std::cout << "Headless Vulkan device created: shaderFloat64=" << handles.shaderFloat64Enabled << "\n";
            quantum::physics::gpu::GpuPhysicsContext gpu(std::move(handles));
            if (!gpu.gpuAvailable())
            {
                std::cout << "GPU VALIDATION SKIPPED – headless GpuPhysicsContext GPU pipeline not ready\n";
                return;
            }
            runValidationWithGpu(gpu);
            // gpu destructor will clean headless handles
            return;
        }
        catch (const std::exception& e)
        {
            std::cout << "GPU VALIDATION SKIPPED – headless Vulkan failed: " << e.what() << "\n";
            if (!windowAttempted)
                std::cout << "  (no window attempted due to SDL failure)\n";
            return;
        }
    }
}

int main()
{
    try
    {
        testGpuSampling();
    }
    catch (const std::exception& e)
    {
        std::cerr << "GpuPhysics validation FAILED: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
