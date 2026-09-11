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
        double averageProfiledStepMilliseconds = 0.0;
    };

    [[nodiscard]] BenchmarkResult runBenchmark(
        const BenchmarkCase& benchmark,
        const TrainDefinition& train)
    {
        const TrainDynamicsState initialState = dynamicsState(
            benchmark.stationMeters);
        for (std::size_t index = 0; index < 20; ++index)
        {
            static_cast<void>(stepTrain(
                benchmark.track,
                train,
                PhysicsEnvironment{},
                initialState,
                FixedStepSettings{}));
        }
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
                {}));
        }
        const double elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count();

        TrainSolveCounters counters;
        const auto profiledBegin = std::chrono::steady_clock::now();
        for (std::size_t index = 0; index < benchmarkStepCount; ++index)
        {
            static_cast<void>(stepTrain(
                benchmark.track,
                train,
                PhysicsEnvironment{},
                initialState,
                FixedStepSettings{},
                {},
                &counters));
        }
        const double profiledElapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - profiledBegin).count();
        return {
            benchmark.name,
            counters,
            elapsedMilliseconds / static_cast<double>(benchmarkStepCount),
            profiledElapsedMilliseconds
                / static_cast<double>(benchmarkStepCount)
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

    [[nodiscard]] double millisecondsPerStep(
        const std::uint64_t nanoseconds) noexcept
    {
        return static_cast<double>(nanoseconds)
            / (1'000'000.0 * static_cast<double>(benchmarkStepCount));
    }

    [[nodiscard]] double averagePerCall(
        const std::uint64_t total,
        const std::uint64_t calls) noexcept
    {
        return calls == 0
            ? 0.0
            : static_cast<double>(total) / static_cast<double>(calls);
    }

    void printProfile(const BenchmarkResult& result)
    {
        const TrainSolveCounters& counters = result.counters;
        const std::uint64_t minimumIterations =
            counters.rigidBogieSolveCalls == 0
            ? 0 : counters.rigidBogieRefinementIterationsMinimum;
        const std::uint64_t minimumSamples =
            counters.rigidBogieSolveCalls == 0
            ? 0 : counters.rigidBogieTrackSamplesMinimum;
        std::printf("\n%.*s detailed profile (per step)\n",
            static_cast<int>(result.name.size()), result.name.data());
        std::printf("  uninstrumented step:       %9.4f ms\n",
            result.averageStepMilliseconds);
        std::printf("  instrumented step:         %9.4f ms\n",
            result.averageProfiledStepMilliseconds);
        std::printf("  solveTrainPose inclusive:  %9.4f ms\n",
            millisecondsPerStep(counters.solveTrainPoseNanoseconds));
        std::printf("  solveCarGeometry inclusive:%9.4f ms\n",
            millisecondsPerStep(counters.solveCarGeometryNanoseconds));
        std::printf("  rigid-bogie inclusive:     %9.4f ms\n",
            millisecondsPerStep(counters.rigidBogieSolveNanoseconds));
        std::printf("  connector candidates:      %9.4f ms\n",
            millisecondsPerStep(counters.connectionCandidateNanoseconds));
        std::printf("  track sampling:            %9.4f ms\n",
            millisecondsPerStep(counters.trackSampleNanoseconds));
        std::printf("  rigid refinements/call:    %6llu / %6.2f / %6llu min/avg/max\n",
            static_cast<unsigned long long>(minimumIterations),
            averagePerCall(counters.rigidBogieRefinementIterations,
                counters.rigidBogieSolveCalls),
            static_cast<unsigned long long>(
                counters.rigidBogieRefinementIterationsMaximum));
        std::printf("  rigid track samples/call:  %6llu / %6.2f / %6llu min/avg/max\n",
            static_cast<unsigned long long>(minimumSamples),
            averagePerCall(counters.rigidBogieTrackSamples,
                counters.rigidBogieSolveCalls),
            static_cast<unsigned long long>(
                counters.rigidBogieTrackSamplesMaximum));
        std::printf("  connector candidates/step: %9.2f\n",
            perStep(counters.connectionCandidateEvaluations));
        const std::uint64_t minimumConnectorCandidates =
            counters.connectorSolveCalls == 0
            ? 0 : counters.connectorCandidateEvaluationsMinimum;
        const std::uint64_t minimumConnectorRefinements =
            counters.connectorSolveCalls == 0
            ? 0 : counters.connectorRefinementIterationsMinimum;
        std::printf("  candidates/connector:      %6llu / %6.2f / %6llu min/avg/max\n",
            static_cast<unsigned long long>(minimumConnectorCandidates),
            averagePerCall(counters.connectionCandidateEvaluations,
                counters.connectorSolveCalls),
            static_cast<unsigned long long>(
                counters.connectorCandidateEvaluationsMaximum));
        std::printf("  refinements/connector:     %6llu / %6.2f / %6llu min/avg/max\n",
            static_cast<unsigned long long>(minimumConnectorRefinements),
            averagePerCall(counters.connectorRefinementIterations,
                counters.connectorSolveCalls),
            static_cast<unsigned long long>(
                counters.connectorRefinementIterationsMaximum));
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

    for (const BenchmarkResult& result : results)
    {
        printProfile(result);
    }

    return verifyDeterministicWorkCounts(results) ? 0 : 1;
}
