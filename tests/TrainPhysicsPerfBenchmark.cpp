#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrainPhysics.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;
    using namespace quantum::physics;

    constexpr std::size_t benchmarkStepCount = 1'000;

    [[nodiscard]] CurveFrame frameForTangent(const glm::dvec3& tangent)
    {
        const glm::dvec3 unit = glm::normalize(tangent);
        return {
            unit,
            {0.0, 1.0, 0.0},
            glm::normalize(glm::cross(unit, glm::dvec3{0.0, 1.0, 0.0}))
        };
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const double lengthMeters = 500.0)
    {
        const CurveFrame frame = frameForTangent({1.0, 0.0, 0.0});
        const std::vector<TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {lengthMeters, lengthMeters * frame.tangent, frame, {0.0, 0.0, 0.0}}
        };
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalCircleTrack(
        const double radiusMeters)
    {
        constexpr int sampleCount = 720;
        std::vector<TrackKinematicState> samples;
        samples.reserve(sampleCount + 1);
        for (int index = 0; index <= sampleCount; ++index)
        {
            const double angle = 2.0 * std::numbers::pi
                * static_cast<double>(index) / sampleCount;
            const CurveFrame frame{
                {std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0},
                {0.0, 0.0, 1.0}
            };
            samples.push_back({
                radiusMeters * angle,
                {
                    radiusMeters * std::sin(angle),
                    radiusMeters * (1.0 - std::cos(angle)),
                    0.0
                },
                frame,
                {
                    -std::sin(angle) / radiusMeters,
                    std::cos(angle) / radiusMeters,
                    0.0
                }
            });
        }
        return {samples, 1.0, TopologyKind::ClosedCircuit};
    }

    [[nodiscard]] CompiledPhysicsTrack verticalArcTrack(
        const bool crest,
        const double radiusMeters = 24.0)
    {
        constexpr double startAngle = -0.9;
        constexpr double endAngle = 0.9;
        constexpr int sampleCount = 600;
        std::vector<TrackKinematicState> samples;
        samples.reserve(sampleCount + 1);
        for (int index = 0; index <= sampleCount; ++index)
        {
            const double angle = startAngle
                + (endAngle - startAngle)
                    * static_cast<double>(index) / sampleCount;
            const double verticalSign = crest ? 1.0 : -1.0;
            const glm::dvec3 tangent{
                std::cos(angle), 0.0, -verticalSign * std::sin(angle)};
            const CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            samples.push_back({
                radiusMeters * (angle - startAngle),
                {
                    radiusMeters * std::sin(angle),
                    0.0,
                    verticalSign * radiusMeters * std::cos(angle)
                },
                frame,
                {
                    -std::sin(angle) / radiusMeters,
                    0.0,
                    -verticalSign * std::cos(angle) / radiusMeters
                }
            });
        }
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CarDefinition carDefinition()
    {
        CarDefinition car;
        car.dryMassKilograms = 800.0;
        car.dryCenterOfGravityMeters = {0.0, 0.0, 0.65};
        car.bodyDimensionsMeters = {4.0, 1.35, 1.4};
        car.frontHitchPositionMeters = {2.0, 0.0, 0.2};
        car.rearHitchPositionMeters = {-2.0, 0.0, 0.2};
        car.bogies = {
            BogieDefinition{{1.15, 0.0, 0.0}},
            BogieDefinition{{-1.15, 0.0, 0.0}}
        };
        return car;
    }

    [[nodiscard]] TrainDefinition fourCarTrain()
    {
        TrainDefinition train;
        for (std::size_t index = 0; index < 4; ++index)
        {
            train.cars.push_back({
                carDefinition(),
                CarLoadout{200.0, {0.0, 0.0, 0.9}}
            });
            if (index != 0)
            {
                train.connections.push_back({0.5});
            }
        }
        train.resistance.constantMechanicalForceNewtons = 500.0;
        train.resistance.linearResistanceCoefficientNewtonSecondsPerMeter = 50.0;
        train.resistance.airDensityKilogramsPerCubicMeter = 1.225;
        train.resistance.dragAreaSquareMeters = 2.5;
        train.resistance.rollingResistanceCoefficient = 0.01;
        validateTrainDefinition(train);
        return train;
    }

    [[nodiscard]] TrainDynamicsState dynamicsState(const double stationMeters)
    {
        TrainDynamicsState state;
        state.generalizedReferenceLocation = {
            primaryTrackPathId,
            stationMeters,
            TravelDirection::IncreasingStation
        };
        state.signedVelocityMetersPerSecond = 20.0;
        state.runState = FollowerRunState::Running;
        return state;
    }

    struct BenchmarkCase
    {
        std::string_view name;
        CompiledPhysicsTrack track;
        double stationMeters = 0.0;
    };

    struct BenchmarkResult
    {
        std::string_view name;
        TrainSolveCounters counters;
        double averageStepMilliseconds = 0.0;
    };

    [[nodiscard]] BenchmarkResult runBenchmark(
        const BenchmarkCase& benchmark,
        const TrainDefinition& train)
    {
        const TrainDynamicsState initialState = dynamicsState(
            benchmark.stationMeters);
        TrainSolveCounters counters;
        const auto begin = std::chrono::steady_clock::now();
        for (std::size_t index = 0; index < benchmarkStepCount; ++index)
        {
            // Each sample starts from the same valid state so the benchmark
            // compares geometry cost without an open-track boundary search.
            static_cast<void>(stepTrain(
                benchmark.track,
                train,
                PhysicsEnvironment{},
                initialState,
                FixedStepSettings{},
                {},
                &counters));
        }
        const double elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count();
        return {
            benchmark.name,
            counters,
            elapsedMilliseconds / static_cast<double>(benchmarkStepCount)
        };
    }

    [[nodiscard]] double perStep(const std::uint64_t total) noexcept
    {
        return static_cast<double>(total)
            / static_cast<double>(benchmarkStepCount);
    }

    [[nodiscard]] double ratio(
        const std::uint64_t baseline,
        const std::uint64_t comparison) noexcept
    {
        return baseline == 0
            ? 0.0
            : static_cast<double>(comparison) / static_cast<double>(baseline);
    }

    void printCounterRow(
        const char* name,
        const std::uint64_t straight,
        const std::uint64_t curved)
    {
        if (straight == 0)
        {
            std::printf("%-38s %14.2f %14.2f %14s\n",
                name, perStep(straight), perStep(curved), "n/a");
            return;
        }
        std::printf("%-38s %14.2f %14.2f %14.2f\n",
            name, perStep(straight), perStep(curved), ratio(straight, curved));
    }

    [[nodiscard]] bool verifyDeterministicWorkCounts(
        const std::array<BenchmarkResult, 6>& results)
    {
        const std::uint64_t expectedPoseSolves = 4 * benchmarkStepCount;
        for (const BenchmarkResult& result : results)
        {
            if (result.counters.solveTrainPoseCalls != expectedPoseSolves)
            {
                std::fprintf(stderr,
                    "%.*s: expected four solveTrainPose calls per step.\n",
                    static_cast<int>(result.name.size()), result.name.data());
                return false;
            }
        }
        if (results.front().counters.rigidBogieRefinementIterations != 0
            || results.front().counters.rigidBogieBracketExpansions != 0)
        {
            std::fprintf(stderr,
                "Straight baseline unexpectedly performed rigid-bogie refinement.\n");
            return false;
        }
        return true;
    }
}

int main()
{
    const TrainDefinition train = fourCarTrain();
    const std::array<BenchmarkCase, 6> benchmarks{
        BenchmarkCase{"Straight", straightTrack(), 100.0},
        BenchmarkCase{"Circle R=50", horizontalCircleTrack(50.0), 100.0},
        BenchmarkCase{"Circle R=25", horizontalCircleTrack(25.0), 50.0},
        BenchmarkCase{"Circle R=15", horizontalCircleTrack(15.0), 30.0},
        BenchmarkCase{"Vertical Valley R=24", verticalArcTrack(false), 25.0},
        BenchmarkCase{"Vertical Crest R=24", verticalArcTrack(true), 25.0}
    };

    std::array<BenchmarkResult, benchmarks.size()> results;
    for (std::size_t index = 0; index < benchmarks.size(); ++index)
    {
        results[index] = runBenchmark(benchmarks[index], train);
    }

    std::printf("QUANTUM Train Physics Performance Benchmark\n");
    std::printf("4-car train, fixed 240 Hz step, %zu independent samples per case\n\n",
        benchmarkStepCount);
    std::printf("%-30s %18s\n", "Case", "Average CPU step");
    for (const BenchmarkResult& result : results)
    {
        std::printf("%-30.*s %15.4f ms\n",
            static_cast<int>(result.name.size()), result.name.data(),
            result.averageStepMilliseconds);
    }

    const TrainSolveCounters& straight = results[0].counters;
    const TrainSolveCounters& curved = results[2].counters;
    std::printf("\nCircle R=25 work comparison\n");
    std::printf("%-38s %14s %14s %14s\n",
        "Counter", "Straight/step", "Curved/step", "Ratio");
    printCounterRow("solveTrainPoseCalls",
        straight.solveTrainPoseCalls, curved.solveTrainPoseCalls);
    printCounterRow("solveCarGeometryCalls",
        straight.solveCarGeometryCalls, curved.solveCarGeometryCalls);
    printCounterRow("rigidBogieSolveCalls",
        straight.rigidBogieSolveCalls, curved.rigidBogieSolveCalls);
    printCounterRow("rigidBogieRefinementIterations",
        straight.rigidBogieRefinementIterations,
        curved.rigidBogieRefinementIterations);
    printCounterRow("rigidBogieBracketExpansions",
        straight.rigidBogieBracketExpansions,
        curved.rigidBogieBracketExpansions);
    printCounterRow("connectionCandidateEvaluations",
        straight.connectionCandidateEvaluations,
        curved.connectionCandidateEvaluations);
    printCounterRow("connectorRefinementIterations",
        straight.connectorRefinementIterations,
        curved.connectorRefinementIterations);
    printCounterRow("connectorFallbackUses",
        straight.connectorFallbackUses, curved.connectorFallbackUses);
    printCounterRow("trackSampleCalls",
        straight.trackSampleCalls, curved.trackSampleCalls);
    printCounterRow("intervalHintMisses",
        straight.intervalHintMisses, curved.intervalHintMisses);
    std::printf("\nVertical crest rigid-bogie bracket expansions: %.2f /step\n",
        perStep(results.back().counters.rigidBogieBracketExpansions));

    return verifyDeterministicWorkCounts(results) ? 0 : 1;
}
