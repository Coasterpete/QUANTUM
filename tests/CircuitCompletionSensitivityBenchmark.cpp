#include <quantum/coaster/detail/CircuitCompletionDetail.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
    using quantum::coaster::detail::CircuitCompletionEndpoint;
    using quantum::coaster::detail::CircuitCompletionIntegrationSchedule;
    using quantum::coaster::detail::CircuitCompletionParameterVector;
    using quantum::coaster::detail::CircuitCompletionSensitivityResult;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;

    template<typename Operation>
    [[nodiscard]] double medianMicroseconds(
        Operation&& operation,
        const std::size_t samples,
        const std::size_t iterationsPerSample,
        double& sink
    )
    {
        std::vector<double> measurements;
        measurements.reserve(samples);
        for (std::size_t sample = 0; sample < samples; ++sample)
        {
            const auto begin = std::chrono::steady_clock::now();
            for (std::size_t iteration = 0;
                 iteration < iterationsPerSample;
                 ++iteration)
            {
                sink += operation();
            }
            const auto end = std::chrono::steady_clock::now();
            const double elapsedMicroseconds =
                std::chrono::duration<double, std::micro>(end - begin).count();
            measurements.push_back(
                elapsedMicroseconds
                / static_cast<double>(iterationsPerSample)
            );
        }

        std::sort(measurements.begin(), measurements.end());
        return measurements[measurements.size() / 2];
    }
}

int main()
{
    constexpr double connectorLength = 40.0;
    constexpr std::size_t sampleCount = 15;
    constexpr std::size_t iterationsPerSample = 5;
    const CircuitCompletionParameterVector parameters{
        -0.16, 0.21, -0.09,
        0.24, -0.18, 0.13,
        -0.11, 0.17, -0.20
    };
    const CurveFrame frame = applyLocalYaw(
        applyLocalPitch(
            applyRoll(
                CurveFrame{
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0}
                },
                0.37
            ),
            -0.29
        ),
        0.41
    );
    const CircuitCompletionEndpoint start{
        {2.75, -4.5, 1.25},
        frame.tangent,
        frame.up
    };
    const CircuitCompletionIntegrationSchedule schedule =
        quantum::coaster::detail::makeCircuitCompletionIntegrationSchedule(
            parameters,
            connectorLength
        );

    double sink = 0.0;
    for (std::size_t warmup = 0; warmup < 3; ++warmup)
    {
        const CircuitCompletionSensitivityResult result =
            quantum::coaster::detail::
                evaluateCircuitCompletionEndpointSensitivities(
                    start,
                    parameters,
                    connectorLength,
                    schedule
                );
        sink += result.endpointSensitivity.endpoint.position.x;
    }

    const double nominalMicroseconds = medianMicroseconds(
        [&]()
        {
            const CircuitCompletionEndpoint endpoint =
                quantum::coaster::detail::
                    evaluateCircuitCompletionFullCoupledEndpoint(
                        start,
                        parameters,
                        connectorLength,
                        schedule
                    );
            return endpoint.position.x + endpoint.tangent.y;
        },
        sampleCount,
        iterationsPerSample,
        sink
    );
    const double augmentedMicroseconds = medianMicroseconds(
        [&]()
        {
            const CircuitCompletionSensitivityResult result =
                quantum::coaster::detail::
                    evaluateCircuitCompletionEndpointSensitivities(
                        start,
                        parameters,
                        connectorLength,
                        schedule
                    );
            return result.endpointSensitivity.endpoint.position.x
                + result.residualJacobian[4][7];
        },
        sampleCount,
        iterationsPerSample,
        sink
    );
    const double ninePerturbedMicroseconds = medianMicroseconds(
        [&]()
        {
            double result = 0.0;
            for (std::size_t parameter = 0;
                 parameter < parameters.size();
                 ++parameter)
            {
                CircuitCompletionParameterVector perturbed = parameters;
                perturbed[parameter] += 1.0e-7;
                const CircuitCompletionEndpoint endpoint =
                    quantum::coaster::detail::
                        evaluateCircuitCompletionProductionEndpoint(
                            start,
                            perturbed,
                            connectorLength
                        );
                result += endpoint.position.x + endpoint.up.z;
            }
            return result;
        },
        sampleCount,
        iterationsPerSample,
        sink
    );

    const double augmentedCost =
        augmentedMicroseconds / nominalMicroseconds;
    const double augmentedVsPerturbed =
        augmentedMicroseconds / ninePerturbedMicroseconds;
    const double predictedSolverSpeedup = 1.0 / (
        0.185 + 0.815 * (augmentedCost / 9.0)
    );

    volatile double retainedSink = sink;
    static_cast<void>(retainedSink);

    std::cout << std::setprecision(17);
    std::cout << "[BENCHMARK] nominal-full-coupled-us="
              << nominalMicroseconds << '\n';
    std::cout << "[BENCHMARK] augmented-sensitivity-us="
              << augmentedMicroseconds << '\n';
    std::cout << "[BENCHMARK] nine-production-perturbations-us="
              << ninePerturbedMicroseconds << '\n';
    std::cout << "[BENCHMARK] c=" << augmentedCost << '\n';
    std::cout << "[BENCHMARK] augmented-vs-nine-perturbations="
              << augmentedVsPerturbed << '\n';
    std::cout << "[BENCHMARK] predicted-solver-speedup="
              << predictedSolverSpeedup << '\n';
    return std::isfinite(sink) ? 0 : 1;
}
