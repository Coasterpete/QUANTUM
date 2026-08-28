#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/detail/CircuitCompletionDetail.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::CircuitCompletionFailure;
    using quantum::coaster::CircuitCompletionResult;
    using quantum::coaster::CircuitCompletionSettings;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::LayoutMode;
    using quantum::coaster::RegionKind;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TopologyTolerances;
    using quantum::coaster::detail::CircuitCompletionJacobianStrategy;
    using quantum::coaster::detail::CircuitCompletionLmDiagnostics;
    using quantum::coaster::detail::CircuitCompletionLmSeedDiagnostics;
    using quantum::coaster::detail::CircuitCompletionLmWorkCounts;
    using quantum::coaster::detail::CircuitCompletionParameterVector;

    constexpr double radiansToDegrees =
        180.0 / 3.14159265358979323846;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    [[nodiscard]] bool sameWorkCounts(
        const CircuitCompletionLmWorkCounts& first,
        const CircuitCompletionLmWorkCounts& second)
    {
        return first.jacobianConstructions == second.jacobianConstructions
            && first.connectorIntegrations == second.connectorIntegrations
            && first.sensitivityTraversals
                == second.sensitivityTraversals
            && first.trialIntegrations == second.trialIntegrations
            && first.dampingTrials == second.dampingTrials
            && first.rejectedDampingTrials
                == second.rejectedDampingTrials
            && first.clampedParameters == second.clampedParameters;
    }

    [[nodiscard]] AuthoredTrack makeStraightTrack(const double length)
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        track.setLayoutMode(LayoutMode::Circuit);
        quantum::coaster::setSectionLength(track.section(0), length);
        return track;
    }

    struct SolveObservation
    {
        CircuitCompletionResult result;
        CircuitCompletionLmDiagnostics diagnostics;
        double elapsedMilliseconds = 0.0;
    };

    [[nodiscard]] SolveObservation solve(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings,
        const CircuitCompletionJacobianStrategy strategy)
    {
        using Clock = std::chrono::steady_clock;
        SolveObservation observation;
        const auto start = Clock::now();
        observation.result = quantum::coaster::detail::
            completeCircuitCandidateWithJacobianStrategy(
                track,
                settings,
                strategy,
                &observation.diagnostics);
        const auto end = Clock::now();
        observation.elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(end - start).count();
        return observation;
    }

    [[nodiscard]] std::string classification(
        const CircuitCompletionResult& result)
    {
        return result.success
            ? "Success"
            : quantum::coaster::circuitCompletionFailureLabel(
                result.failureReason);
    }

    [[nodiscard]] const GeometricSection& connectorProfiles(
        const CircuitCompletionResult& result)
    {
        return result.completedTrack
            .section(result.completedTrack.sectionCount() - 1)
            .rateProfileRegion().rateProfiles;
    }

    [[nodiscard]] std::array<double, 9> profileKnots(
        const CircuitCompletionResult& result)
    {
        const GeometricSection& profiles = connectorProfiles(result);
        return {
            profiles.pitch.segments[0].transition.valueBegin,
            profiles.pitch.segments[0].transition.valueEnd,
            profiles.pitch.segments[1].transition.valueEnd,
            profiles.yaw.segments[0].transition.valueBegin,
            profiles.yaw.segments[0].transition.valueEnd,
            profiles.yaw.segments[1].transition.valueEnd,
            profiles.roll.segments[0].transition.valueBegin,
            profiles.roll.segments[0].transition.valueEnd,
            profiles.roll.segments[1].transition.valueEnd};
    }

    void validateSuccessfulResult(const SolveObservation& observation)
    {
        require(observation.result.success, "Expected a successful result");
        const auto topology = quantum::coaster::computeTrackTopology(
            observation.result.completedTrack);
        constexpr TopologyTolerances tolerances;
        require(
            topology.kind == TopologyKind::ClosedCircuit,
            "Successful result is not independently ClosedCircuit");
        require(
            topology.diagnostics.positionalGap
                <= tolerances.closureGapTolerance,
            "Successful result exceeds the position tolerance");
        require(
            topology.diagnostics.tangentMismatchDegrees
                <= tolerances.angleTolerance,
            "Successful result exceeds the tangent tolerance");
        require(
            topology.diagnostics.frameMismatchDegrees
                <= tolerances.angleTolerance,
            "Successful result exceeds the frame tolerance");

        const auto states = quantum::coaster::integrateAuthoredTrack(
            observation.result.completedTrack,
            0.25);
        require(!states.empty(), "Successful result has no centerline states");
        for (const RiderLocalGeometryState& state : states)
        {
            require(
                std::isfinite(state.distance)
                    && std::isfinite(state.position.x)
                    && std::isfinite(state.position.y)
                    && std::isfinite(state.position.z)
                    && std::isfinite(state.frame.tangent.x)
                    && std::isfinite(state.frame.tangent.y)
                    && std::isfinite(state.frame.tangent.z)
                    && std::isfinite(state.frame.lateral.x)
                    && std::isfinite(state.frame.lateral.y)
                    && std::isfinite(state.frame.lateral.z)
                    && std::isfinite(state.frame.up.x)
                    && std::isfinite(state.frame.up.y)
                    && std::isfinite(state.frame.up.z),
                "Successful result contains non-finite geometry");
        }
        for (const double parameter : profileKnots(observation.result))
        {
            require(
                std::isfinite(parameter),
                "Successful result contains a non-finite profile knot");
        }
        require(
            profileKnots(observation.result)
                == observation.diagnostics.finalParameters,
            "Generated profile knots do not match selected LM parameters");
    }

    void verifyCandidateRanking(const SolveObservation& observation)
    {
        if (observation.diagnostics.seeds.empty())
        {
            return;
        }
        require(
            observation.diagnostics.selectedSeedIndex
                < observation.diagnostics.seeds.size(),
            "Selected seed index is outside the reached seed set");

        const auto& selected = observation.diagnostics.seeds[
            observation.diagnostics.selectedSeedIndex];
        const bool anyConverged = std::ranges::any_of(
            observation.diagnostics.seeds,
            [](const CircuitCompletionLmSeedDiagnostics& seed)
            {
                return seed.converged;
            });
        require(
            !anyConverged || selected.converged,
            "A non-converged candidate outranked a converged candidate");

        if (!anyConverged)
        {
            const double minimumRms = std::ranges::min(
                observation.diagnostics.seeds,
                {},
                &CircuitCompletionLmSeedDiagnostics::finalResidualRms)
                .finalResidualRms;
            require(
                selected.finalResidualRms == minimumRms,
                "Failed candidate ranking did not select the lowest RMS");
        }
    }

    void printParameters(const CircuitCompletionParameterVector& parameters)
    {
        std::cout << '[';
        for (std::size_t index = 0; index < parameters.size(); ++index)
        {
            std::cout << (index == 0 ? "" : ",") << parameters[index];
        }
        std::cout << ']';
    }

    void printResidualProgression(
        const CircuitCompletionLmSeedDiagnostics& seed)
    {
        std::cout << '[';
        constexpr std::size_t stride = 20;
        for (std::size_t index = 0;
             index < seed.residualProgression.size(); ++index)
        {
            if (index != 0
                && index + 1 != seed.residualProgression.size()
                && index % stride != 0)
            {
                continue;
            }
            std::cout << (index == 0 ? "" : ",")
                      << index << ':' << seed.residualProgression[index];
        }
        std::cout << ']';
    }

    void printObservation(
        const std::string_view fixtureName,
        const SolveObservation& observation)
    {
        const auto& diagnostics = observation.diagnostics;
        const auto& work = diagnostics.work;
        std::cout << "[SOLVE] fixture=" << fixtureName
                  << " strategy="
                  << quantum::coaster::detail::
                        circuitCompletionJacobianStrategyLabel(
                            diagnostics.strategy)
                  << " classification=" << classification(observation.result)
                  << " seeds=" << diagnostics.seeds.size()
                  << " selected-seed=" << diagnostics.selectedSeedIndex
                  << " selected-iterations=" << observation.result.iterationCount
                  << " global-iterations=" << diagnostics.totalIterationsSpent
                  << " jacobians=" << work.jacobianConstructions
                  << " damping-trials=" << work.dampingTrials
                  << " rejected-trials=" << work.rejectedDampingTrials
                  << " connector-integrations=" << work.connectorIntegrations
                  << " sensitivity-traversals=" << work.sensitivityTraversals
                  << " trial-integrations=" << work.trialIntegrations
                  << " clamps=" << work.clampedParameters
                  << " final-rms=" << diagnostics.finalResidualRms
                  << " position=" << observation.result.finalPositionalGap
                  << " tangent-deg="
                  << observation.result.finalTangentErrorDegrees
                  << " frame-deg=" << observation.result.finalFrameErrorDegrees
                  << " runtime-ms=" << observation.elapsedMilliseconds
                  << " topology="
                  << (observation.result.success
                        ? "ClosedCircuit"
                        : "NoCandidateTrack")
                  << " params=";
        printParameters(diagnostics.finalParameters);
        std::cout << '\n';

        for (const CircuitCompletionLmSeedDiagnostics& seed : diagnostics.seeds)
        {
            std::cout << "[SEED] fixture=" << fixtureName
                      << " strategy="
                      << quantum::coaster::detail::
                            circuitCompletionJacobianStrategyLabel(
                                diagnostics.strategy)
                      << " index=" << seed.seedIndex
                      << " converged=" << seed.converged
                      << " iterations=" << seed.iterationCount
                      << " jacobians=" << seed.work.jacobianConstructions
                      << " damping-trials=" << seed.work.dampingTrials
                      << " rejected=" << seed.work.rejectedDampingTrials
                      << " trial-integrations=" << seed.work.trialIntegrations
                      << " final-rms=" << seed.finalResidualRms
                      << " initial=";
            printParameters(seed.initialParameters);
            std::cout
                      << " progression=";
            printResidualProgression(seed);
            std::cout << '\n';
        }

        if (observation.result.success)
        {
            std::cout << "[PROFILE] fixture=" << fixtureName
                      << " strategy="
                      << quantum::coaster::detail::
                            circuitCompletionJacobianStrategyLabel(
                                diagnostics.strategy)
                      << " length="
                      << observation.result.completedTrack.section(
                            observation.result.completedTrack.sectionCount() - 1)
                            .length
                      << " knots=";
            printParameters(profileKnots(observation.result));
            std::cout << '\n';
        }
    }

    struct GeometryComparison
    {
        double maximumCenterlineDisplacement = 0.0;
        double rmsCenterlineDisplacement = 0.0;
        double maximumTangentAngleDegrees = 0.0;
        double maximumLateralAngleDegrees = 0.0;
        double maximumUpAngleDegrees = 0.0;
        double maximumPitchRateDifference = 0.0;
        double maximumYawRateDifference = 0.0;
        double maximumRollRateDifference = 0.0;
        double maximumCurvatureDifference = 0.0;
        double finiteDifferencePeakCurvature = 0.0;
        double sensitivityPeakCurvature = 0.0;
        double peakRadiusDifference = 0.0;
        std::string classification;
    };

    [[nodiscard]] double angleDegrees(
        const glm::dvec3& first,
        const glm::dvec3& second)
    {
        return std::acos(glm::clamp(glm::dot(first, second), -1.0, 1.0))
            * radiansToDegrees;
    }

    [[nodiscard]] GeometryComparison compareInteriorGeometry(
        const SolveObservation& finiteDifference,
        const SolveObservation& sensitivity,
        const double sourceLength,
        const double connectorLength)
    {
        require(
            finiteDifference.result.success && sensitivity.result.success,
            "Interior geometry comparison requires two successful results");
        const auto finiteDifferenceStates =
            quantum::coaster::integrateAuthoredTrack(
                finiteDifference.result.completedTrack,
                0.25);
        const auto sensitivityStates =
            quantum::coaster::integrateAuthoredTrack(
                sensitivity.result.completedTrack,
                0.25);
        require(
            finiteDifferenceStates.size() == sensitivityStates.size(),
            "A/B centerlines use different matched sample counts");

        GeometryComparison comparison;
        double squaredDisplacement = 0.0;
        std::size_t connectorSampleCount = 0;
        for (std::size_t index = 0;
             index < finiteDifferenceStates.size(); ++index)
        {
            const auto& first = finiteDifferenceStates[index];
            const auto& second = sensitivityStates[index];
            require(
                std::abs(first.distance - second.distance) <= 1.0e-12,
                "A/B centerline samples are not distance matched");
            if (first.distance + 1.0e-12 < sourceLength)
            {
                continue;
            }
            const double displacement =
                glm::length(first.position - second.position);
            comparison.maximumCenterlineDisplacement = std::max(
                comparison.maximumCenterlineDisplacement,
                displacement);
            squaredDisplacement += displacement * displacement;
            ++connectorSampleCount;
            comparison.maximumTangentAngleDegrees = std::max(
                comparison.maximumTangentAngleDegrees,
                angleDegrees(first.frame.tangent, second.frame.tangent));
            comparison.maximumLateralAngleDegrees = std::max(
                comparison.maximumLateralAngleDegrees,
                angleDegrees(first.frame.lateral, second.frame.lateral));
            comparison.maximumUpAngleDegrees = std::max(
                comparison.maximumUpAngleDegrees,
                angleDegrees(first.frame.up, second.frame.up));
        }
        comparison.rmsCenterlineDisplacement = std::sqrt(
            squaredDisplacement / static_cast<double>(connectorSampleCount));

        const GeometricSection& finiteDifferenceProfiles =
            connectorProfiles(finiteDifference.result);
        const GeometricSection& sensitivityProfiles =
            connectorProfiles(sensitivity.result);
        constexpr std::size_t rateSampleCount = 161;
        for (std::size_t index = 0; index < rateSampleCount; ++index)
        {
            const double distance = connectorLength
                * static_cast<double>(index)
                / static_cast<double>(rateSampleCount - 1);
            const auto first = quantum::coaster::evaluateGeometricSection(
                finiteDifferenceProfiles,
                connectorLength,
                distance);
            const auto second = quantum::coaster::evaluateGeometricSection(
                sensitivityProfiles,
                connectorLength,
                distance);
            comparison.maximumPitchRateDifference = std::max(
                comparison.maximumPitchRateDifference,
                std::abs(first.pitch - second.pitch));
            comparison.maximumYawRateDifference = std::max(
                comparison.maximumYawRateDifference,
                std::abs(first.yaw - second.yaw));
            comparison.maximumRollRateDifference = std::max(
                comparison.maximumRollRateDifference,
                std::abs(first.roll - second.roll));
            const double firstCurvature = std::hypot(
                first.pitch,
                first.yaw);
            const double secondCurvature = std::hypot(
                second.pitch,
                second.yaw);
            comparison.maximumCurvatureDifference = std::max(
                comparison.maximumCurvatureDifference,
                std::abs(firstCurvature - secondCurvature));
            comparison.finiteDifferencePeakCurvature = std::max(
                comparison.finiteDifferencePeakCurvature,
                firstCurvature);
            comparison.sensitivityPeakCurvature = std::max(
                comparison.sensitivityPeakCurvature,
                secondCurvature);
        }
        comparison.peakRadiusDifference = std::abs(
            1.0 / comparison.finiteDifferencePeakCurvature
            - 1.0 / comparison.sensitivityPeakCurvature);

        const bool negligible =
            comparison.maximumCenterlineDisplacement <= 1.0e-6
            && comparison.maximumTangentAngleDegrees <= 1.0e-5
            && comparison.maximumUpAngleDegrees <= 1.0e-5
            && comparison.maximumPitchRateDifference <= 1.0e-8
            && comparison.maximumYawRateDifference <= 1.0e-8
            && comparison.maximumRollRateDifference <= 1.0e-8;
        const bool material =
            comparison.maximumCenterlineDisplacement > 0.5
            || comparison.maximumTangentAngleDegrees > 2.0
            || comparison.maximumLateralAngleDegrees > 2.0
            || comparison.maximumUpAngleDegrees > 2.0
            || comparison.maximumCurvatureDifference > 0.02;
        comparison.classification = negligible
            ? "Negligible"
            : material
                ? "Materially different - engineering review needed"
                : "Different but valid";
        return comparison;
    }

    void printGeometry(
        const std::string_view fixture,
        const GeometryComparison& comparison)
    {
        std::cout << "[GEOMETRY] fixture=" << fixture
                  << " classification=" << comparison.classification
                  << " max-centerline="
                  << comparison.maximumCenterlineDisplacement
                  << " rms-centerline="
                  << comparison.rmsCenterlineDisplacement
                  << " max-tangent-deg="
                  << comparison.maximumTangentAngleDegrees
                  << " max-lateral-deg="
                  << comparison.maximumLateralAngleDegrees
                  << " max-up-deg=" << comparison.maximumUpAngleDegrees
                  << " max-pitch-rate="
                  << comparison.maximumPitchRateDifference
                  << " max-yaw-rate="
                  << comparison.maximumYawRateDifference
                  << " max-roll-rate="
                  << comparison.maximumRollRateDifference
                  << " max-curvature="
                  << comparison.maximumCurvatureDifference
                  << " fd-peak-curvature="
                  << comparison.finiteDifferencePeakCurvature
                  << " sensitivity-peak-curvature="
                  << comparison.sensitivityPeakCurvature
                  << " peak-radius-difference="
                  << comparison.peakRadiusDifference << '\n';
    }

    struct NullSpaceSummary
    {
        double totalStepNorm = 0.0;
        double totalNullSpaceStepNorm = 0.0;
        std::size_t minimumRank = 9;
        std::size_t maximumRank = 0;
        double maximumParameterNorm = 0.0;
        std::uint64_t clampedParameters = 0;
    };

    [[nodiscard]] NullSpaceSummary summarizeNullSpace(
        const SolveObservation& observation)
    {
        NullSpaceSummary summary;
        for (const auto& seed : observation.diagnostics.seeds)
        {
            for (const auto& iteration : seed.iterations)
            {
                summary.totalStepNorm += iteration.proposedStepNorm;
                summary.totalNullSpaceStepNorm +=
                    iteration.approximateNullSpaceStepNorm;
                summary.minimumRank = std::min(
                    summary.minimumRank,
                    iteration.numericalRank);
                summary.maximumRank = std::max(
                    summary.maximumRank,
                    iteration.numericalRank);
                summary.maximumParameterNorm = std::max(
                    summary.maximumParameterNorm,
                    iteration.parameterNorm);
                summary.clampedParameters += iteration.clampedParameters;
            }
        }
        return summary;
    }

    void printNullSpaceComparison(
        const std::string_view fixture,
        const SolveObservation& finiteDifference,
        const SolveObservation& sensitivity)
    {
        const NullSpaceSummary first = summarizeNullSpace(finiteDifference);
        const NullSpaceSummary second = summarizeNullSpace(sensitivity);
        const double firstFraction = first.totalStepNorm > 0.0
            ? first.totalNullSpaceStepNorm / first.totalStepNorm
            : 0.0;
        const double secondFraction = second.totalStepNorm > 0.0
            ? second.totalNullSpaceStepNorm / second.totalStepNorm
            : 0.0;
        const char* motion = secondFraction < firstFraction * 0.8
            ? "less"
            : secondFraction > firstFraction * 1.25
                ? "greater"
                : "similar";
        std::cout << "[NULLSPACE] fixture=" << fixture
                  << " fd-rank=" << first.minimumRank << '-'
                  << first.maximumRank
                  << " sensitivity-rank=" << second.minimumRank << '-'
                  << second.maximumRank
                  << " fd-null-fraction=" << firstFraction
                  << " sensitivity-null-fraction=" << secondFraction
                  << " sensitivity-motion=" << motion
                  << " fd-max-param-norm=" << first.maximumParameterNorm
                  << " sensitivity-max-param-norm="
                  << second.maximumParameterNorm
                  << " fd-clamps=" << first.clampedParameters
                  << " sensitivity-clamps=" << second.clampedParameters
                  << '\n';
    }

    void requireDeterministic(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings,
        const CircuitCompletionJacobianStrategy strategy,
        const SolveObservation& reference,
        const std::string_view label)
    {
        for (int repetition = 1; repetition < 3; ++repetition)
        {
            const SolveObservation repeated = solve(track, settings, strategy);
            require(
                repeated.result.success == reference.result.success
                    && repeated.result.failureReason
                        == reference.result.failureReason,
                std::string(label) + " classification is not deterministic");
            require(
                repeated.diagnostics.selectedSeedIndex
                    == reference.diagnostics.selectedSeedIndex
                    && repeated.diagnostics.finalParameters
                        == reference.diagnostics.finalParameters,
                std::string(label) + " selected candidate is not deterministic");
            require(
                repeated.diagnostics.totalIterationsSpent
                    == reference.diagnostics.totalIterationsSpent
                    && sameWorkCounts(
                        repeated.diagnostics.work,
                        reference.diagnostics.work),
                std::string(label) + " solver path is not deterministic");
            if (reference.result.success)
            {
                require(
                    profileKnots(repeated.result)
                        == profileKnots(reference.result),
                    std::string(label) + " profiles are not deterministic");
                const auto topology = quantum::coaster::computeTrackTopology(
                    repeated.result.completedTrack);
                require(
                    topology.kind == TopologyKind::ClosedCircuit
                        && topology.diagnostics.positionalGap
                            == reference.result.finalPositionalGap
                        && topology.diagnostics.tangentMismatchDegrees
                            == reference.result.finalTangentErrorDegrees
                        && topology.diagnostics.frameMismatchDegrees
                            == reference.result.finalFrameErrorDegrees,
                    std::string(label) + " topology is not deterministic");
            }
        }
        std::cout << "[DETERMINISM] fixture=" << label
                  << " strategy="
                  << quantum::coaster::detail::
                        circuitCompletionJacobianStrategyLabel(strategy)
                  << " repetitions=3 exact=yes\n";
    }

    struct FixturePair
    {
        std::string name;
        double sourceLength = 0.0;
        double connectorLength = 0.0;
        SolveObservation finiteDifference;
        SolveObservation sensitivity;
    };

    [[nodiscard]] FixturePair runFixturePair(
        const std::string& name,
        const double sourceLength,
        const double connectorLength)
    {
        const AuthoredTrack track = makeStraightTrack(sourceLength);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = connectorLength;
        FixturePair pair;
        pair.name = name;
        pair.sourceLength = sourceLength;
        pair.connectorLength = connectorLength;
        pair.finiteDifference = solve(
            track,
            settings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        pair.sensitivity = solve(
            track,
            settings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        printObservation(name, pair.finiteDifference);
        printObservation(name, pair.sensitivity);
        verifyCandidateRanking(pair.finiteDifference);
        verifyCandidateRanking(pair.sensitivity);
        return pair;
    }

    void testProductionDefaultUsesSensitivity(
        const FixturePair& canonical)
    {
        const AuthoredTrack track = makeStraightTrack(15.0);
        CircuitCompletionSettings settings;
        settings.preferredConnectorLength = 40.0;
        const CircuitCompletionResult production =
            quantum::coaster::completeCircuitCandidate(track, settings);
        require(
            production.success == canonical.sensitivity.result.success
                && production.failureReason
                    == canonical.sensitivity.result.failureReason
                && production.iterationCount
                    == canonical.sensitivity.result.iterationCount
                && production.finalPositionalGap
                    == canonical.sensitivity.result.finalPositionalGap
                && profileKnots(production)
                    == profileKnots(canonical.sensitivity.result),
            "Production entry point does not exactly select sensitivities");
        std::cout << "[PRODUCTION] default-strategy=Sensitivity exact=yes\n";
    }

    void testJacobianWorkAccounting(
        const std::array<const FixturePair*, 5>& fixtures)
    {
        for (const FixturePair* fixture : fixtures)
        {
            const CircuitCompletionLmWorkCounts& finiteDifference =
                fixture->finiteDifference.diagnostics.work;
            const CircuitCompletionLmWorkCounts& sensitivity =
                fixture->sensitivity.diagnostics.work;
            const std::uint64_t finiteDifferenceNominalIntegrations =
                fixture->finiteDifference.diagnostics.seeds.size();
            const std::uint64_t sensitivityNominalIntegrations =
                fixture->sensitivity.diagnostics.seeds.size();

            require(
                finiteDifference.sensitivityTraversals == 0
                    && finiteDifference.connectorIntegrations
                        == finiteDifferenceNominalIntegrations
                            + finiteDifference.trialIntegrations
                            + 9 * finiteDifference.jacobianConstructions,
                fixture->name
                    + " finite-difference work accounting changed");
            require(
                sensitivity.sensitivityTraversals
                        == sensitivity.jacobianConstructions
                    && sensitivity.connectorIntegrations
                        == sensitivityNominalIntegrations
                            + sensitivity.trialIntegrations,
                fixture->name + " sensitivity work accounting changed");
        }

        std::cout << "[PRODUCTION-WORK] nominal-per-seed=yes"
                  << " sensitivity-per-jacobian=yes"
                  << " finite-difference-perturbations=none\n";
    }

    void testFixturePairs(
        const FixturePair& canonical,
        const FixturePair& nonconvergent,
        const FixturePair& additional,
        const FixturePair& focusedTwentyForty,
        const FixturePair& focusedThirtySixty)
    {
        require(
            canonical.finiteDifference.result.success,
            "Canonical FD fixture regressed");
        require(
            canonical.sensitivity.result.success,
            "Canonical sensitivity fixture did not converge");
        validateSuccessfulResult(canonical.finiteDifference);
        validateSuccessfulResult(canonical.sensitivity);

        require(
            !nonconvergent.finiteDifference.result.success
                && nonconvergent.finiteDifference.result.failureReason
                    == CircuitCompletionFailure::DidNotConverge,
            "30m/40m FD classification changed");
        require(
            !nonconvergent.sensitivity.result.success
                && nonconvergent.sensitivity.result.failureReason
                    == CircuitCompletionFailure::DidNotConverge,
            "30m/40m sensitivity classification changed");

        require(
            additional.finiteDifference.result.success,
            "20m/60m established FD success regressed");
        require(
            additional.sensitivity.result.success,
            "20m/60m sensitivity regressed to failure");
        validateSuccessfulResult(additional.finiteDifference);
        validateSuccessfulResult(additional.sensitivity);

        for (const FixturePair* pair : {
                 &focusedTwentyForty,
                 &focusedThirtySixty})
        {
            require(
                pair->finiteDifference.result.success,
                pair->name + " established FD success regressed");
            validateSuccessfulResult(pair->finiteDifference);
            require(
                pair->sensitivity.result.success,
                pair->name
                    + " sensitivity basin-access seed did not converge");
            validateSuccessfulResult(pair->sensitivity);
        }

        std::cout << "[FIXTURES] 15m/40m=Equivalent-success"
                  << " 30m/40m="
                  << (nonconvergent.sensitivity.result.success
                        ? "Classification-changed"
                        : "Equivalent-failure")
                  << " 20m/60m=Equivalent-success"
                  << " 20m/40m="
                  << (focusedTwentyForty.sensitivity.result.success
                        ? "Equivalent-success"
                        : "Classification-changed")
                  << " 30m/60m="
                  << (focusedThirtySixty.sensitivity.result.success
                        ? "Equivalent-success"
                        : "Classification-changed") << '\n';
    }

    void testSeedBudgetReachability(
        const FixturePair& canonical,
        const FixturePair& nonconvergent,
        const FixturePair& additional,
        const FixturePair& focusedTwentyForty,
        const FixturePair& focusedThirtySixty)
    {
        const auto& finiteDifferenceSeeds =
            nonconvergent.finiteDifference.diagnostics.seeds;
        const auto& sensitivitySeeds =
            nonconvergent.sensitivity.diagnostics.seeds;
        require(
            finiteDifferenceSeeds.size() == 5
                && sensitivitySeeds.size() == 5
                && nonconvergent.finiteDifference.diagnostics
                        .totalIterationsSpent == 600
                && nonconvergent.sensitivity.diagnostics
                        .totalIterationsSpent == 600,
            "30m/40m did not consume the expected five-seed budget");
        for (std::size_t index = 0;
             index < finiteDifferenceSeeds.size(); ++index)
        {
            require(
                finiteDifferenceSeeds[index].initialParameters
                    == sensitivitySeeds[index].initialParameters,
                "Seed family depends on the Jacobian strategy");
            require(
                finiteDifferenceSeeds[index].iterationCount == 120
                    && sensitivitySeeds[index].iterationCount == 120,
                "Budget-exhaustion fixture did not spend 120 iterations per seed");
        }

        constexpr double inverseLength = 1.0 / 40.0;
        constexpr double pitchBiasAngle = 0.025;
        for (const auto& reflectedSeed :
             std::array<std::pair<std::size_t, double>, 2>{{
                 {2, 1.0},
                 {3, -1.0}}})
        {
            CircuitCompletionParameterVector expected{};
            expected[0] =
                reflectedSeed.second * pitchBiasAngle * inverseLength;
            expected[1] = expected[0];
            expected[2] = expected[0];
            expected[3] = -8.0 * inverseLength;
            expected[5] = 8.0 * inverseLength;
            require(
                sensitivitySeeds[reflectedSeed.first].initialParameters
                    == expected,
                "Basin-access seed values changed");
        }
        for (const FixturePair* successful : {
                 &canonical,
                 &additional,
                 &focusedTwentyForty,
                 &focusedThirtySixty})
        {
            require(
                successful->finiteDifference.diagnostics.selectedSeedIndex
                    <= 1,
                successful->name
                    + " established FD success depended on a starved seed");
        }

        std::cout << "[BASIN-SEEDS] jacobian-independent=yes"
                  << " pitch-angle=0.025 pair=reflected"
                  << " insertion=after-first-yaw-fallback"
                  << " budget-reachable=0-4"
                  << " original-starved=old-3,old-4"
                  << " established-success-dependency=none\n";
    }

    struct OverrideCases
    {
        SolveObservation successfulFiniteDifference;
        SolveObservation successfulSensitivity;
        SolveObservation difficultFiniteDifference;
        SolveObservation difficultSensitivity;
    };

    [[nodiscard]] OverrideCases testInitialParamOverride()
    {
        OverrideCases cases;
        CircuitCompletionSettings successfulSettings;
        successfulSettings.preferredConnectorLength = 40.0;
        successfulSettings.initialParamOverride = std::array<double, 6>{
            0.0,
            0.0,
            16.79 / 40.0,
            -16.79 / 40.0,
            0.0,
            0.0};
        const AuthoredTrack successfulTrack = makeStraightTrack(15.0);
        cases.successfulFiniteDifference = solve(
            successfulTrack,
            successfulSettings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        cases.successfulSensitivity = solve(
            successfulTrack,
            successfulSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        require(cases.successfulFiniteDifference.result.success,
            "Known FD override did not succeed");
        require(cases.successfulSensitivity.result.success,
            "Known sensitivity override did not succeed");
        require(
            cases.successfulFiniteDifference.diagnostics.seeds.size() == 1
                && cases.successfulSensitivity.diagnostics.seeds.size() == 1,
            "Override path unexpectedly used fallback seeds");
        validateSuccessfulResult(cases.successfulFiniteDifference);
        validateSuccessfulResult(cases.successfulSensitivity);

        CircuitCompletionSettings difficultSettings;
        difficultSettings.preferredConnectorLength = 40.0;
        difficultSettings.initialParamOverride = std::array<double, 6>{
            0.1,
            0.1,
            -0.1,
            -0.1,
            0.05,
            -0.05};
        const AuthoredTrack difficultTrack = makeStraightTrack(30.0);
        cases.difficultFiniteDifference = solve(
            difficultTrack,
            difficultSettings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        cases.difficultSensitivity = solve(
            difficultTrack,
            difficultSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        require(
            cases.difficultFiniteDifference.diagnostics.seeds.size() == 1
                && cases.difficultSensitivity.diagnostics.seeds.size() == 1,
            "Difficult override unexpectedly used fallback seeds");
        verifyCandidateRanking(cases.difficultFiniteDifference);
        verifyCandidateRanking(cases.difficultSensitivity);

        CircuitCompletionSettings invalidSettings;
        invalidSettings.preferredConnectorLength = 40.0;
        invalidSettings.initialParamOverride = std::array<double, 6>{
            std::numeric_limits<double>::quiet_NaN(),
            0.0, 0.0, 0.0, 0.0, 0.0};
        for (const CircuitCompletionJacobianStrategy strategy : {
                 CircuitCompletionJacobianStrategy::FiniteDifference,
                 CircuitCompletionJacobianStrategy::Sensitivity})
        {
            const SolveObservation invalid = solve(
                successfulTrack,
                invalidSettings,
                strategy);
            require(
                !invalid.result.success
                    && invalid.result.failureReason
                        == CircuitCompletionFailure::InvalidInput
                    && invalid.diagnostics.seeds.empty(),
                "Invalid override did not fail safely before LM");
        }

        printObservation("override-success-FD", cases.successfulFiniteDifference);
        printObservation("override-success-sensitivity",
            cases.successfulSensitivity);
        printObservation("override-difficult-FD", cases.difficultFiniteDifference);
        printObservation("override-difficult-sensitivity",
            cases.difficultSensitivity);
        std::cout << "[OVERRIDE] successful=both-success difficult="
                  << classification(cases.difficultFiniteDifference.result)
                  << '/'
                  << classification(cases.difficultSensitivity.result)
                  << " invalid=both-InvalidInput fallback-seeds=no\n";
        return cases;
    }

    void testFailureSafety()
    {
        const AuthoredTrack straight = makeStraightTrack(15.0);
        for (const CircuitCompletionJacobianStrategy strategy : {
                 CircuitCompletionJacobianStrategy::FiniteDifference,
                 CircuitCompletionJacobianStrategy::Sensitivity})
        {
            CircuitCompletionSettings invalidLength;
            invalidLength.preferredConnectorLength = -1.0;
            const SolveObservation negative = solve(
                straight,
                invalidLength,
                strategy);
            require(
                negative.result.failureReason
                    == CircuitCompletionFailure::InvalidInput,
                "Negative connector length did not fail safely");

            invalidLength.preferredConnectorLength =
                std::numeric_limits<double>::quiet_NaN();
            const SolveObservation nan = solve(
                straight,
                invalidLength,
                strategy);
            require(
                nan.result.failureReason == CircuitCompletionFailure::InvalidInput,
                "NaN connector length did not fail safely");

            AuthoredTrack shuttle = straight;
            shuttle.setLayoutMode(LayoutMode::Shuttle);
            const SolveObservation shuttleResult = solve(
                shuttle,
                {},
                strategy);
            require(
                shuttleResult.result.failureReason
                    == CircuitCompletionFailure::ShuttleLayout,
                "Shuttle topology did not fail safely");

            const AuthoredTrack empty;
            const SolveObservation emptyResult = solve(empty, {}, strategy);
            require(
                emptyResult.result.failureReason
                    == CircuitCompletionFailure::InvalidInput,
                "Empty input did not fail safely");

            AuthoredTrack geometry = makeStraightTrack(15.0);
            geometry.section(0).kind = RegionKind::Geometry;
            const SolveObservation geometryResult = solve(
                geometry,
                {},
                strategy);
            require(
                geometryResult.result.failureReason
                    == CircuitCompletionFailure::InvalidInput,
                "Unsupported geometry did not fail safely");
        }
        std::cout << "[FAILURE-SAFETY] invalid-length invalid-override"
                  << " shuttle empty unsupported-geometry budget-exhaustion=pass\n";
    }

    struct TimingSummary
    {
        double minimumMilliseconds = 0.0;
        double medianMilliseconds = 0.0;
        double maximumMilliseconds = 0.0;
    };

    struct TimingPair
    {
        TimingSummary finiteDifference;
        TimingSummary sensitivity;
    };

    [[nodiscard]] TimingSummary summarizeTimings(std::vector<double> values)
    {
        std::ranges::sort(values);
        return {
            values.front(),
            values[values.size() / 2],
            values.back()};
    }

    [[nodiscard]] double timedSolveMilliseconds(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings,
        const CircuitCompletionJacobianStrategy strategy)
    {
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        const CircuitCompletionResult result = quantum::coaster::detail::
            completeCircuitCandidateWithJacobianStrategy(
                track,
                settings,
                strategy,
                nullptr);
        const auto end = Clock::now();
        if (result.success)
        {
            volatile double benchmarkSink = result.finalPositionalGap;
            (void)benchmarkSink;
        }
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    [[nodiscard]] double timedProductionSolveMilliseconds(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings)
    {
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        const CircuitCompletionResult result =
            quantum::coaster::completeCircuitCandidate(track, settings);
        const auto end = Clock::now();
        volatile double benchmarkSink = result.finalPositionalGap;
        (void)benchmarkSink;
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    [[nodiscard]] TimingSummary benchmarkProductionSolve(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings,
        const std::size_t repetitions)
    {
        (void)timedProductionSolveMilliseconds(track, settings);
        std::vector<double> samples;
        samples.reserve(repetitions);
        for (std::size_t repetition = 0;
             repetition < repetitions; ++repetition)
        {
            samples.push_back(
                timedProductionSolveMilliseconds(track, settings));
        }
        return summarizeTimings(std::move(samples));
    }

    [[nodiscard]] TimingPair benchmarkSolvePair(
        const AuthoredTrack& track,
        const CircuitCompletionSettings& settings,
        const std::size_t repetitions)
    {
        (void)timedSolveMilliseconds(
            track,
            settings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        (void)timedSolveMilliseconds(
            track,
            settings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        std::vector<double> finiteDifferenceSamples;
        std::vector<double> sensitivitySamples;
        finiteDifferenceSamples.reserve(repetitions);
        sensitivitySamples.reserve(repetitions);
        for (std::size_t repetition = 0;
             repetition < repetitions; ++repetition)
        {
            const auto measureFiniteDifference = [&]
            {
                finiteDifferenceSamples.push_back(timedSolveMilliseconds(
                    track,
                    settings,
                    CircuitCompletionJacobianStrategy::FiniteDifference));
            };
            const auto measureSensitivity = [&]
            {
                sensitivitySamples.push_back(timedSolveMilliseconds(
                    track,
                    settings,
                    CircuitCompletionJacobianStrategy::Sensitivity));
            };
            if (repetition % 2 == 0)
            {
                measureFiniteDifference();
                measureSensitivity();
            }
            else
            {
                measureSensitivity();
                measureFiniteDifference();
            }
        }
        return {
            summarizeTimings(std::move(finiteDifferenceSamples)),
            summarizeTimings(std::move(sensitivitySamples))};
    }

    [[nodiscard]] double timedFocusedWorkload(
        const CircuitCompletionJacobianStrategy strategy)
    {
        using Clock = std::chrono::steady_clock;
        const std::array<std::array<double, 2>, 5> fixtures{{
            {15.0, 40.0},
            {30.0, 40.0},
            {20.0, 40.0},
            {20.0, 60.0},
            {30.0, 60.0}}};
        const auto start = Clock::now();
        for (const auto& fixture : fixtures)
        {
            CircuitCompletionSettings settings;
            settings.preferredConnectorLength = fixture[1];
            const CircuitCompletionResult result = quantum::coaster::detail::
                completeCircuitCandidateWithJacobianStrategy(
                    makeStraightTrack(fixture[0]),
                    settings,
                    strategy,
                    nullptr);
            volatile double benchmarkSink = result.finalPositionalGap;
            (void)benchmarkSink;
        }
        const auto end = Clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    [[nodiscard]] TimingPair benchmarkFocusedWorkloadPair(
        const std::size_t repetitions)
    {
        (void)timedFocusedWorkload(
            CircuitCompletionJacobianStrategy::FiniteDifference);
        (void)timedFocusedWorkload(
            CircuitCompletionJacobianStrategy::Sensitivity);
        std::vector<double> finiteDifferenceSamples;
        std::vector<double> sensitivitySamples;
        finiteDifferenceSamples.reserve(repetitions);
        sensitivitySamples.reserve(repetitions);
        for (std::size_t repetition = 0;
             repetition < repetitions; ++repetition)
        {
            const auto measureFiniteDifference = [&]
            {
                finiteDifferenceSamples.push_back(timedFocusedWorkload(
                    CircuitCompletionJacobianStrategy::FiniteDifference));
            };
            const auto measureSensitivity = [&]
            {
                sensitivitySamples.push_back(timedFocusedWorkload(
                    CircuitCompletionJacobianStrategy::Sensitivity));
            };
            if (repetition % 2 == 0)
            {
                measureFiniteDifference();
                measureSensitivity();
            }
            else
            {
                measureSensitivity();
                measureFiniteDifference();
            }
        }
        return {
            summarizeTimings(std::move(finiteDifferenceSamples)),
            summarizeTimings(std::move(sensitivitySamples))};
    }

    [[nodiscard]] double timedProductionFocusedWorkload()
    {
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        for (const auto& fixture : std::array<std::array<double, 2>, 5>{{
                 {15.0, 40.0},
                 {30.0, 40.0},
                 {20.0, 40.0},
                 {20.0, 60.0},
                 {30.0, 60.0}}})
        {
            CircuitCompletionSettings settings;
            settings.preferredConnectorLength = fixture[1];
            const CircuitCompletionResult result =
                quantum::coaster::completeCircuitCandidate(
                    makeStraightTrack(fixture[0]),
                    settings);
            volatile double benchmarkSink = result.finalPositionalGap;
            (void)benchmarkSink;
        }
        const auto end = Clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    [[nodiscard]] TimingSummary benchmarkProductionFocusedWorkload(
        const std::size_t repetitions)
    {
        (void)timedProductionFocusedWorkload();
        std::vector<double> samples;
        samples.reserve(repetitions);
        for (std::size_t repetition = 0;
             repetition < repetitions; ++repetition)
        {
            samples.push_back(timedProductionFocusedWorkload());
        }
        return summarizeTimings(std::move(samples));
    }

    CircuitCompletionLmWorkCounts& operator+=(
        CircuitCompletionLmWorkCounts& left,
        const CircuitCompletionLmWorkCounts& right)
    {
        left.jacobianConstructions += right.jacobianConstructions;
        left.connectorIntegrations += right.connectorIntegrations;
        left.sensitivityTraversals += right.sensitivityTraversals;
        left.trialIntegrations += right.trialIntegrations;
        left.dampingTrials += right.dampingTrials;
        left.rejectedDampingTrials += right.rejectedDampingTrials;
        left.clampedParameters += right.clampedParameters;
        return left;
    }

    [[nodiscard]] CircuitCompletionLmWorkCounts focusedWorkCounts(
        const CircuitCompletionJacobianStrategy strategy)
    {
        CircuitCompletionLmWorkCounts total;
        for (const auto& fixture : std::array<std::array<double, 2>, 5>{{
                 {15.0, 40.0},
                 {30.0, 40.0},
                 {20.0, 40.0},
                 {20.0, 60.0},
                 {30.0, 60.0}}})
        {
            CircuitCompletionSettings settings;
            settings.preferredConnectorLength = fixture[1];
            total += solve(
                makeStraightTrack(fixture[0]),
                settings,
                strategy)
                .diagnostics.work;
        }
        return total;
    }

    void printBenchmark(
        const std::string_view fixture,
        const TimingPair& timings,
        const CircuitCompletionLmWorkCounts& finiteDifferenceWork,
        const CircuitCompletionLmWorkCounts& sensitivityWork,
        const std::size_t repetitions)
    {
        std::cout << "[BENCHMARK] fixture=" << fixture
                  << " repetitions=" << repetitions
                  << " fd-min-ms="
                  << timings.finiteDifference.minimumMilliseconds
                  << " fd-median-ms="
                  << timings.finiteDifference.medianMilliseconds
                  << " fd-max-ms="
                  << timings.finiteDifference.maximumMilliseconds
                  << " sensitivity-min-ms="
                  << timings.sensitivity.minimumMilliseconds
                  << " sensitivity-median-ms="
                  << timings.sensitivity.medianMilliseconds
                  << " sensitivity-max-ms="
                  << timings.sensitivity.maximumMilliseconds
                  << " speedup="
                  << timings.finiteDifference.medianMilliseconds
                        / timings.sensitivity.medianMilliseconds
                  << " fd-iterations="
                  << finiteDifferenceWork.jacobianConstructions
                  << " sensitivity-iterations="
                  << sensitivityWork.jacobianConstructions
                  << " fd-jacobians="
                  << finiteDifferenceWork.jacobianConstructions
                  << " sensitivity-jacobians="
                  << sensitivityWork.jacobianConstructions
                  << " fd-integrations="
                  << finiteDifferenceWork.connectorIntegrations
                  << " sensitivity-integrations="
                  << sensitivityWork.connectorIntegrations
                  << " fd-sensitivity-traversals="
                  << finiteDifferenceWork.sensitivityTraversals
                  << " sensitivity-sensitivity-traversals="
                  << sensitivityWork.sensitivityTraversals
                  << " fd-trial-integrations="
                  << finiteDifferenceWork.trialIntegrations
                  << " sensitivity-trial-integrations="
                  << sensitivityWork.trialIntegrations
                  << " fd-damping-trials=" << finiteDifferenceWork.dampingTrials
                  << " sensitivity-damping-trials="
                  << sensitivityWork.dampingTrials << '\n';
    }

    void printProductionBenchmark(
        const std::string_view fixture,
        const TimingSummary& timings,
        const CircuitCompletionLmWorkCounts& work,
        const std::size_t repetitions)
    {
        std::cout << "[PRODUCTION-BENCHMARK] fixture=" << fixture
                  << " repetitions=" << repetitions
                  << " min-ms=" << timings.minimumMilliseconds
                  << " median-ms=" << timings.medianMilliseconds
                  << " max-ms=" << timings.maximumMilliseconds
                  << " jacobians=" << work.jacobianConstructions
                  << " nominal-integrations="
                  << work.connectorIntegrations - work.trialIntegrations
                  << " connector-integrations=" << work.connectorIntegrations
                  << " sensitivity-traversals="
                  << work.sensitivityTraversals
                  << " trial-integrations=" << work.trialIntegrations
                  << " damping-trials=" << work.dampingTrials << '\n';
    }

    int runBenchmarks()
    {
        constexpr std::size_t successRepetitions = 7;
        constexpr std::size_t failureRepetitions = 5;
        constexpr std::size_t workloadRepetitions = 3;
        CircuitCompletionSettings canonicalSettings;
        canonicalSettings.preferredConnectorLength = 40.0;
        const AuthoredTrack canonical = makeStraightTrack(15.0);
        const SolveObservation canonicalFiniteDifference = solve(
            canonical,
            canonicalSettings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        const SolveObservation canonicalSensitivity = solve(
            canonical,
            canonicalSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        const TimingPair canonicalTimings = benchmarkSolvePair(
            canonical,
            canonicalSettings,
            successRepetitions);
        printBenchmark(
            "15m/40m",
            canonicalTimings,
            canonicalFiniteDifference.diagnostics.work,
            canonicalSensitivity.diagnostics.work,
            successRepetitions);
        printProductionBenchmark(
            "15m/40m",
            benchmarkProductionSolve(
                canonical,
                canonicalSettings,
                successRepetitions),
            canonicalSensitivity.diagnostics.work,
            successRepetitions);

        const AuthoredTrack focusedTwentyForty = makeStraightTrack(20.0);
        const SolveObservation focusedTwentyFortySensitivity = solve(
            focusedTwentyForty,
            canonicalSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        printProductionBenchmark(
            "20m/40m",
            benchmarkProductionSolve(
                focusedTwentyForty,
                canonicalSettings,
                successRepetitions),
            focusedTwentyFortySensitivity.diagnostics.work,
            successRepetitions);

        CircuitCompletionSettings longSettings;
        longSettings.preferredConnectorLength = 60.0;
        const AuthoredTrack focusedThirtySixty = makeStraightTrack(30.0);
        const SolveObservation focusedThirtySixtySensitivity = solve(
            focusedThirtySixty,
            longSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        printProductionBenchmark(
            "30m/60m",
            benchmarkProductionSolve(
                focusedThirtySixty,
                longSettings,
                successRepetitions),
            focusedThirtySixtySensitivity.diagnostics.work,
            successRepetitions);

        const AuthoredTrack nonconvergent = makeStraightTrack(30.0);
        const SolveObservation failureFiniteDifference = solve(
            nonconvergent,
            canonicalSettings,
            CircuitCompletionJacobianStrategy::FiniteDifference);
        const SolveObservation failureSensitivity = solve(
            nonconvergent,
            canonicalSettings,
            CircuitCompletionJacobianStrategy::Sensitivity);
        const TimingPair failureTimings = benchmarkSolvePair(
            nonconvergent,
            canonicalSettings,
            failureRepetitions);
        printBenchmark(
            "30m/40m",
            failureTimings,
            failureFiniteDifference.diagnostics.work,
            failureSensitivity.diagnostics.work,
            failureRepetitions);
        printProductionBenchmark(
            "30m/40m",
            benchmarkProductionSolve(
                nonconvergent,
                canonicalSettings,
                failureRepetitions),
            failureSensitivity.diagnostics.work,
            failureRepetitions);

        const CircuitCompletionLmWorkCounts focusedFiniteDifference =
            focusedWorkCounts(
                CircuitCompletionJacobianStrategy::FiniteDifference);
        const CircuitCompletionLmWorkCounts focusedSensitivity =
            focusedWorkCounts(CircuitCompletionJacobianStrategy::Sensitivity);
        const TimingPair focusedTimings = benchmarkFocusedWorkloadPair(
            workloadRepetitions);
        printBenchmark(
            "focused-five-fixture-workload",
            focusedTimings,
            focusedFiniteDifference,
            focusedSensitivity,
            workloadRepetitions);
        printProductionBenchmark(
            "focused-five-fixture-workload",
            benchmarkProductionFocusedWorkload(workloadRepetitions),
            focusedSensitivity,
            workloadRepetitions);
        return 0;
    }

    int runValidation()
    {
        int passed = 0;
        const auto run = [&passed](
            const std::string_view name,
            const auto& function)
        {
            function();
            ++passed;
            std::cout << "[PASS] " << name << '\n';
        };

        const FixturePair canonical = runFixturePair(
            "15m/40m",
            15.0,
            40.0);
        const FixturePair nonconvergent = runFixturePair(
            "30m/40m",
            30.0,
            40.0);
        const FixturePair additional = runFixturePair(
            "20m/60m",
            20.0,
            60.0);
        const FixturePair focusedTwentyForty = runFixturePair(
            "20m/40m",
            20.0,
            40.0);
        const FixturePair focusedThirtySixty = runFixturePair(
            "30m/60m",
            30.0,
            60.0);

        run("production default uses sensitivities", [&]
        {
            testProductionDefaultUsesSensitivity(canonical);
        });
        run("solver fixture A/B classifications", [&]
        {
            testFixturePairs(
                canonical,
                nonconvergent,
                additional,
                focusedTwentyForty,
                focusedThirtySixty);
        });
        run("Jacobian work accounting", [&]
        {
            testJacobianWorkAccounting({
                &canonical,
                &nonconvergent,
                &additional,
                &focusedTwentyForty,
                &focusedThirtySixty});
        });
        run("seed budget reachability", [&]
        {
            testSeedBudgetReachability(
                canonical,
                nonconvergent,
                additional,
                focusedTwentyForty,
                focusedThirtySixty);
        });

        const OverrideCases overrides = testInitialParamOverride();
        ++passed;
        std::cout << "[PASS] initialParamOverride A/B coverage\n";

        run("determinism", [&]
        {
            CircuitCompletionSettings canonicalSettings;
            canonicalSettings.preferredConnectorLength = 40.0;
            const AuthoredTrack canonicalTrack = makeStraightTrack(15.0);
            const AuthoredTrack nonconvergentTrack = makeStraightTrack(30.0);
            const AuthoredTrack focusedTwentyFortyTrack =
                makeStraightTrack(20.0);
            const AuthoredTrack focusedThirtySixtyTrack =
                makeStraightTrack(30.0);
            CircuitCompletionSettings additionalSettings;
            additionalSettings.preferredConnectorLength = 60.0;
            const AuthoredTrack additionalTrack = makeStraightTrack(20.0);

            for (const CircuitCompletionJacobianStrategy strategy : {
                     CircuitCompletionJacobianStrategy::FiniteDifference,
                     CircuitCompletionJacobianStrategy::Sensitivity})
            {
                const bool finiteDifference = strategy
                    == CircuitCompletionJacobianStrategy::FiniteDifference;
                requireDeterministic(
                    canonicalTrack,
                    canonicalSettings,
                    strategy,
                    finiteDifference
                        ? canonical.finiteDifference
                        : canonical.sensitivity,
                    "15m/40m");
                requireDeterministic(
                    nonconvergentTrack,
                    canonicalSettings,
                    strategy,
                    finiteDifference
                        ? nonconvergent.finiteDifference
                        : nonconvergent.sensitivity,
                    "30m/40m");
                requireDeterministic(
                    additionalTrack,
                    additionalSettings,
                    strategy,
                    finiteDifference
                        ? additional.finiteDifference
                        : additional.sensitivity,
                    "20m/60m");
                requireDeterministic(
                    focusedTwentyFortyTrack,
                    canonicalSettings,
                    strategy,
                    finiteDifference
                        ? focusedTwentyForty.finiteDifference
                        : focusedTwentyForty.sensitivity,
                    "20m/40m");
                requireDeterministic(
                    focusedThirtySixtyTrack,
                    additionalSettings,
                    strategy,
                    finiteDifference
                        ? focusedThirtySixty.finiteDifference
                        : focusedThirtySixty.sensitivity,
                    "30m/60m");

                CircuitCompletionSettings overrideSettings;
                overrideSettings.preferredConnectorLength = 40.0;
                overrideSettings.initialParamOverride =
                    std::array<double, 6>{
                        0.0, 0.0,
                        16.79 / 40.0, -16.79 / 40.0,
                        0.0, 0.0};
                requireDeterministic(
                    canonicalTrack,
                    overrideSettings,
                    strategy,
                    finiteDifference
                        ? overrides.successfulFiniteDifference
                        : overrides.successfulSensitivity,
                    "override-success");
            }
        });

        run("interior geometry", [&]
        {
            printGeometry(
                "15m/40m",
                compareInteriorGeometry(
                    canonical.finiteDifference,
                    canonical.sensitivity,
                    canonical.sourceLength,
                    canonical.connectorLength));
            printGeometry(
                "20m/60m",
                compareInteriorGeometry(
                    additional.finiteDifference,
                    additional.sensitivity,
                    additional.sourceLength,
                    additional.connectorLength));
            const GeometryComparison twentyForty = compareInteriorGeometry(
                focusedTwentyForty.finiteDifference,
                focusedTwentyForty.sensitivity,
                focusedTwentyForty.sourceLength,
                focusedTwentyForty.connectorLength);
            printGeometry("20m/40m", twentyForty);
            require(
                twentyForty.maximumCenterlineDisplacement <= 0.25
                    && twentyForty.maximumTangentAngleDegrees <= 2.0
                    && twentyForty.maximumLateralAngleDegrees <= 2.0
                    && twentyForty.maximumUpAngleDegrees <= 2.0
                    && twentyForty.maximumPitchRateDifference <= 0.01
                    && twentyForty.maximumYawRateDifference <= 0.01
                    && twentyForty.maximumRollRateDifference <= 0.01,
                "20m/40m recovered geometry left the FD-like basin");
            const GeometryComparison thirtySixty = compareInteriorGeometry(
                focusedThirtySixty.finiteDifference,
                focusedThirtySixty.sensitivity,
                focusedThirtySixty.sourceLength,
                focusedThirtySixty.connectorLength);
            printGeometry("30m/60m", thirtySixty);
            require(
                thirtySixty.maximumCenterlineDisplacement <= 0.25
                    && thirtySixty.maximumTangentAngleDegrees <= 2.0
                    && thirtySixty.maximumLateralAngleDegrees <= 2.0
                    && thirtySixty.maximumUpAngleDegrees <= 2.0
                    && thirtySixty.maximumPitchRateDifference <= 0.01
                    && thirtySixty.maximumYawRateDifference <= 0.01
                    && thirtySixty.maximumRollRateDifference <= 0.01,
                "30m/60m recovered geometry left the FD-like basin");
        });

        run("null-space diagnostics", [&]
        {
            printNullSpaceComparison(
                "15m/40m",
                canonical.finiteDifference,
                canonical.sensitivity);
            printNullSpaceComparison(
                "30m/40m",
                nonconvergent.finiteDifference,
                nonconvergent.sensitivity);
        });
        run("failure safety", testFailureSafety);

        std::cout << "[RESULT] " << passed << " test groups passed\n";
        return 0;
    }
}

int main(const int argc, char** argv)
{
    std::cout << std::setprecision(17);
    try
    {
        if (argc == 2 && std::string_view(argv[1]) == "--benchmark")
        {
            return runBenchmarks();
        }
        return runValidation();
    }
    catch (const std::exception& error)
    {
        std::cerr << "[FAIL] " << error.what() << '\n';
        return 1;
    }
}
