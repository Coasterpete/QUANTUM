#include <quantum/coaster/CircuitCompletion.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/detail/CircuitCompletionDetail.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace quantum::coaster
{
    namespace
    {
        constexpr double piRadians = 3.14159265358979323846;
        constexpr double degreesPerRadian = 180.0 / piRadians;
        constexpr double finiteDiffEpsilon = 1e-7;
        constexpr std::uint32_t maxIterations = 120;
        constexpr double convergencePositionGap = 0.005;
        constexpr double convergenceAngleDeg = 0.25;
        constexpr double defaultConnectorLength = 40.0;
        constexpr double integrationSpacing = 0.5;

        // Number of free parameters: start/mid/end rates for
        // pitch, yaw, roll.
        constexpr std::size_t parameterCount =
            detail::circuitCompletionParameterCount;

        // Position difference plus full tangent and up-vector differences.
        constexpr std::size_t residualCount =
            detail::circuitCompletionResidualCount;

        using ParameterVector =
            detail::CircuitCompletionParameterVector;
        using ErrorVector = detail::CircuitCompletionResidual;
        using JacobianMatrix =
            std::array<ErrorVector, parameterCount>;
        using AugmentedRow =
            std::array<double, parameterCount + 1>;

        // Build a single-segment ChannelProfile covering [0, length]
        // with the given constant value.
        [[nodiscard]] ChannelProfile makeConstantProfile(
            const double value,
            const double length)
        {
            ChannelProfile profile;
            profile.nextSegmentId = 2;

            ProfileSegment segment;
            segment.id = 1;
            segment.transition.domainBegin = 0.0;
            segment.transition.domainEnd = length;
            segment.transition.valueBegin = value;
            segment.transition.valueEnd = value;
            segment.transition.transitionType =
                math::TransitionType::Linear;

            profile.segments.push_back(segment);
            return profile;
        }

        // Build the 3-channel rate profile for a connector section
        // from the 9-element parameter vector [p0,p_m,p_1, y0,y_m,y_1, r0,r_m,r_1].
        // Each channel uses a piecewise-linear rate with a breakpoint at s = L/2:
        //
        //   c(s) = c_0 + (c_m - c_0) * (2s/L),  0 <= s <= L/2
        //   c(s) = c_m + (c_1 - c_m) * (2(s-L/2)/L),  L/2 < s <= L
        //
        // The old six-parameter profile is reproduced exactly in function space
        // when p_m=(p_0+p_1)/2, y_m=(y_0+y_1)/2, r_m=(r_0+r_1)/2.
        [[nodiscard]] GeometricSection buildConnectorProfiles(
            const ParameterVector& params,
            const double length)
        {
            GeometricSection profiles;

            // --- Pitch: 2 segments, breakpoint at L/2 ---
            profiles.pitch.nextSegmentId = 3;
            {
                // Segment 1: [0, L/2], from p_0 to p_m
                ProfileSegment seg1;
                seg1.id = 1;
                seg1.transition.domainBegin = 0.0;
                seg1.transition.domainEnd = length * 0.5;
                seg1.transition.valueBegin = params[0];
                seg1.transition.valueEnd = params[1];
                seg1.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.pitch.segments.push_back(seg1);

                // Segment 2: [L/2, L], from p_m to p_1
                ProfileSegment seg2;
                seg2.id = 2;
                seg2.transition.domainBegin = length * 0.5;
                seg2.transition.domainEnd = length;
                seg2.transition.valueBegin = params[1];
                seg2.transition.valueEnd = params[2];
                seg2.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.pitch.segments.push_back(seg2);
            }

            // --- Yaw: 2 segments, breakpoint at L/2 ---
            profiles.yaw.nextSegmentId = 5;
            {
                // Segment 1: [0, L/2], from y_0 to y_m
                ProfileSegment seg1;
                seg1.id = 3;
                seg1.transition.domainBegin = 0.0;
                seg1.transition.domainEnd = length * 0.5;
                seg1.transition.valueBegin = params[3];
                seg1.transition.valueEnd = params[4];
                seg1.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.yaw.segments.push_back(seg1);

                // Segment 2: [L/2, L], from y_m to y_1
                ProfileSegment seg2;
                seg2.id = 4;
                seg2.transition.domainBegin = length * 0.5;
                seg2.transition.domainEnd = length;
                seg2.transition.valueBegin = params[4];
                seg2.transition.valueEnd = params[5];
                seg2.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.yaw.segments.push_back(seg2);
            }

            // --- Roll: 2 segments, breakpoint at L/2 ---
            profiles.roll.nextSegmentId = 7;
            {
                // Segment 1: [0, L/2], from r_0 to r_m
                ProfileSegment seg1;
                seg1.id = 5;
                seg1.transition.domainBegin = 0.0;
                seg1.transition.domainEnd = length * 0.5;
                seg1.transition.valueBegin = params[6];
                seg1.transition.valueEnd = params[7];
                seg1.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.roll.segments.push_back(seg1);

                // Segment 2: [L/2, L], from r_m to r_1
                ProfileSegment seg2;
                seg2.id = 6;
                seg2.transition.domainBegin = length * 0.5;
                seg2.transition.domainEnd = length;
                seg2.transition.valueBegin = params[7];
                seg2.transition.valueEnd = params[8];
                seg2.transition.transitionType =
                    math::TransitionType::Linear;
                profiles.roll.segments.push_back(seg2);
            }

            return profiles;
        }

        // Integrate a connector from a given starting state and
        // return the endpoint state {position, tangent, up}.
        using EndpointState = detail::CircuitCompletionEndpoint;

        [[nodiscard]] EndpointState integrateConnector(
            const EndpointState& startState,
            const ParameterVector& params,
            const double length)
        {
            const GeometricSection profiles =
                buildConnectorProfiles(params, length);

            geometry::CurveFrame startFrame;
            startFrame.tangent = startState.tangent;
            startFrame.lateral = glm::normalize(
                glm::cross(startState.up, startState.tangent));
            startFrame.up = startState.up;

            const std::vector<RiderLocalGeometryState> states =
                integrateLocalRollPitchYawRateProfiles(
                    startState.position,
                    startFrame,
                    profiles.roll,
                    profiles.pitch,
                    profiles.yaw,
                    length,
                    integrationSpacing);

            const RiderLocalGeometryState& endState = states.back();
            return {endState.position,
                    endState.frame.tangent,
                    endState.frame.up};
        }

        // Compute a scale-normalised error vector.  Position
        // components are divided by connector length so that the
        // six components have comparable magnitude.
        //
        // Orientation uses full tangent and up-vector differences.
        // These chordal residuals are smooth, linear near the target,
        // and distinguish the intended frame from a 180-degree reversal.
        [[nodiscard]] ErrorVector computeNormalisedError(
            const EndpointState& connectorEnd,
            const EndpointState& trackStart,
            const double length)
        {
            return detail::computeCircuitCompletionResidual(
                connectorEnd,
                trackStart,
                length
            );
        }

        // RMS norm of the normalised error vector.
        [[nodiscard]] double normalisedRms(
            const ErrorVector& e)
        {
            double sum = 0.0;
            for (const double v : e)
            {
                sum += v * v;
            }
            return std::sqrt(sum / static_cast<double>(residualCount));
        }

        // Compute initial parameter guess based on the gap between
        // the track end and start points.
        [[nodiscard]] ParameterVector computeInitialGuess(
            const EndpointState& trackEnd,
            const EndpointState& trackStart,
            const double length)
        {
            const glm::dvec3 gap =
                trackStart.position - trackEnd.position;

            const double invL = 1.0 / length;
            const double invL2 = invL * invL;

            // Decompose gap into along-tangent and perpendicular
            // components.
            const double alongDot =
                glm::dot(gap, trackStart.tangent);
            const double alongFrac =
                std::abs(alongDot) * invL;

            // Pitch: for a straight track with gap.z != 0,
            // pitch rate ~ -2*gap.z / L^2.
            const double pitchPos =
                -2.0 * gap.z * invL2;

            // Yaw: cross product captures perpendicular gap.
            const double yawPos =
                2.0 * (gap.x * trackStart.tangent.y
                     - gap.y * trackStart.tangent.x) * invL2;

            // Tangent alignment.
            const double tangentErrLength =
                glm::length(trackStart.tangent - trackEnd.tangent);

            double pitchStart = pitchPos * 0.5;
            double pitchEnd = pitchPos * 0.5
                + tangentErrLength * invL * 0.5;
            double yawStart = yawPos * 0.5;
            double yawEnd = yawPos * 0.5
                + tangentErrLength * invL * 0.5;

            // When the gap is primarily along the tangent, the
            // connector must reverse direction.  Seed an
            // anti-symmetric clothoid (yawStart = -yawEnd) whose
            // lateral displacement y(L) is zero.
            //
            // For an anti-symmetric clothoid the heading is
            //   theta(s) = omega * s * (1 - s/L)
            // The condition y(L)=0 is satisfied when
            //   omega * L = C  where C ≈ 16.79.
            //
            // This produces x(L) ≈ -0.429 * L.  The remaining
            // position gap is closed by the LM solver.
            if (alongFrac > 0.1)
            {
                const double sign =
                    alongDot > 0.0 ? 1.0 : -1.0;

                constexpr double clothoidConstant = 16.79;
                const double omega =
                    clothoidConstant * invL;

                // Anti-symmetric: curves one way then back.
                yawStart = -sign * omega;
                yawEnd = sign * omega;

                // Also seed pitch proportionally for the 3-D
                // component of the gap.
                const double perpGapZ =
                    glm::dot(gap, trackStart.up);
                if (std::abs(perpGapZ) > 0.01)
                {
                    pitchStart = -perpGapZ * invL2;
                    pitchEnd = -perpGapZ * invL2;
                }
            }

            // Roll: match the frame mismatch at the start.
            const double rollStart = 0.0;
            const double rollEnd = 0.0;

            return {
                pitchStart, (pitchStart + pitchEnd) * 0.5, pitchEnd,
                yawStart, (yawStart + yawEnd) * 0.5, yawEnd,
                rollStart, (rollStart + rollEnd) * 0.5, rollEnd};
        }

        struct LevenbergMarquardtAttempt
        {
            ParameterVector params{};
            EndpointState connectorEnd{};
            bool converged = false;

            // True when the iteration budget was exhausted without
            // meeting the tolerances (as opposed to an early divergence
            // guard trip).
            bool hitIterationLimit = false;
            std::uint32_t iterationCount = 0;
        };

        // Runs the Levenberg-Marquardt refinement from `initialParams`
        // and returns the final state of that single attempt.
        [[nodiscard]] LevenbergMarquardtAttempt runLevenbergMarquardt(
            const EndpointState& trackEnd,
            const EndpointState& trackStart,
            const double length,
            ParameterVector params)
        {
            LevenbergMarquardtAttempt attempt;
            attempt.params = params;

            EndpointState connectorEnd =
                integrateConnector(trackEnd, params, length);

            ErrorVector normErrors =
                computeNormalisedError(
                    connectorEnd, trackStart, length);

            // Levenberg-Marquardt damping parameter.
            double lambda = 1e-3;
            constexpr double lambdaMin = 1e-12;
            constexpr double lambdaMax = 1e4;
            constexpr double lambdaUp = 10.0;
            constexpr double lambdaDown = 0.1;
            // Maximum LM retries per iteration (re-solve with
            // increased damping if no improvement found).
            constexpr int maxLmRetries = 8;

            // Parameter magnitude guard.
            constexpr double maxParamMagnitude = 10.0;

            constexpr TopologyTolerances tolerances;

            bool converged = false;
            bool diverged = false;

            std::uint32_t iteration = 0;
            for (; iteration < maxIterations; ++iteration)
            {
                // Check convergence against real tolerances.
                const double posGap =
                    glm::length(connectorEnd.position
                        - trackStart.position);
                const double tangDot = glm::clamp(
                    glm::dot(connectorEnd.tangent,
                        trackStart.tangent),
                    -1.0, 1.0);
                const double tangDeg =
                    std::acos(tangDot) * degreesPerRadian;
                const double frameDot = glm::clamp(
                    glm::dot(connectorEnd.up, trackStart.up),
                    -1.0, 1.0);
                const double frameDeg =
                    std::acos(frameDot) * degreesPerRadian;

                if (posGap <= tolerances.closureGapTolerance
                    && tangDeg <= tolerances.angleTolerance
                    && frameDeg <= tolerances.angleTolerance)
                {
                    converged = true;
                    break;
                }

                // Finite-difference Jacobian of the normalised error.
                JacobianMatrix jacobian;

                for (std::size_t col = 0;
                     col < parameterCount; ++col)
                {
                    ParameterVector perturbed = params;
                    perturbed[col] += finiteDiffEpsilon;

                    const EndpointState perturbedEnd =
                        integrateConnector(
                            trackEnd, perturbed, length);
                    const ErrorVector perturbedNorm =
                        computeNormalisedError(
                            perturbedEnd, trackStart, length);

                    for (std::size_t row = 0;
                         row < residualCount; ++row)
                    {
                        jacobian[col][row] =
                            (perturbedNorm[row] - normErrors[row])
                            / finiteDiffEpsilon;
                    }
                }

                // Build normal equations:
                // (J^T J + lambda I) dx = -J^T e.
                using NormalMatrix =
                    std::array<std::array<double, parameterCount>,
                        parameterCount>;
                using RhsVector =
                    std::array<double, parameterCount>;

                NormalMatrix jtj{};
                RhsVector jte{};

                for (std::size_t i = 0;
                     i < parameterCount; ++i)
                {
                    for (std::size_t j = 0;
                         j < parameterCount; ++j)
                    {
                        double sum = 0.0;
                        for (std::size_t k = 0;
                             k < residualCount; ++k)
                        {
                            sum += jacobian[j][k]
                                * jacobian[i][k];
                        }
                        jtj[i][j] = sum;
                    }

                    double sum = 0.0;
                    for (std::size_t k = 0;
                         k < residualCount; ++k)
                    {
                        sum += jacobian[i][k]
                            * normErrors[k];
                    }
                    jte[i] = -sum;
                }

                // LM inner loop: try increasing damping until the
                // step improves the normalised residual.
                const double oldNorm = normalisedRms(normErrors);
                bool stepAccepted = false;

                for (int lmRetry = 0;
                     lmRetry < maxLmRetries; ++lmRetry)
                {
                    // Augmented system:
                    // (J^T J + lambda*I) dx = -J^T e.
                    std::array<AugmentedRow, parameterCount>
                        augmented;
                    for (std::size_t r = 0;
                         r < parameterCount; ++r)
                    {
                        for (std::size_t c = 0;
                             c < parameterCount; ++c)
                        {
                            augmented[r][c] = jtj[r][c];
                        }
                        augmented[r][r] += lambda;
                        augmented[r][parameterCount] = jte[r];
                    }

                    // Gaussian elimination with partial pivoting.
                    for (std::size_t col = 0;
                         col < parameterCount; ++col)
                    {
                        std::size_t pivotRow = col;
                        double pivotVal =
                            std::abs(augmented[col][col]);
                        for (std::size_t row = col + 1;
                             row < parameterCount; ++row)
                        {
                            const double val =
                                std::abs(augmented[row][col]);
                            if (val > pivotVal)
                            {
                                pivotVal = val;
                                pivotRow = row;
                            }
                        }

                        if (pivotVal < 1e-14)
                        {
                            break;
                        }

                        if (pivotRow != col)
                        {
                            std::swap(
                                augmented[col],
                                augmented[pivotRow]);
                        }

                        const double invPivot =
                            1.0 / augmented[col][col];
                        for (std::size_t row = col + 1;
                             row < parameterCount; ++row)
                        {
                            const double factor =
                                augmented[row][col] * invPivot;
                            for (std::size_t k = col;
                                 k <= parameterCount; ++k)
                            {
                                augmented[row][k] -=
                                    factor * augmented[col][k];
                            }
                        }
                    }

                    // Back-substitution.
                    ParameterVector dx{};
                    for (std::size_t row = parameterCount;
                         row > 0; --row)
                    {
                        const std::size_t r = row - 1;
                        double sum =
                            augmented[r][parameterCount];
                        for (std::size_t col = r + 1;
                             col < parameterCount; ++col)
                        {
                            sum -= augmented[r][col] * dx[col];
                        }
                        const double diag = augmented[r][r];
                        dx[r] = std::abs(diag) > 1e-14
                            ? sum / diag
                            : 0.0;
                    }

                    // Trial step.
                    ParameterVector trialParams = params;
                    for (std::size_t i = 0;
                         i < parameterCount; ++i)
                    {
                        trialParams[i] += dx[i];
                    }

                    // Parameter magnitude guard.
                    for (double& p : trialParams)
                    {
                        p = std::clamp(
                            p,
                            -maxParamMagnitude,
                            maxParamMagnitude);
                    }

                    const EndpointState trialEnd =
                        integrateConnector(
                            trackEnd, trialParams, length);
                    const ErrorVector trialNorm =
                        computeNormalisedError(
                            trialEnd, trackStart, length);
                    const double trialRms =
                        normalisedRms(trialNorm);

                    if (trialRms < oldNorm)
                    {
                        params = trialParams;
                        connectorEnd = trialEnd;
                        normErrors = trialNorm;
                        lambda = std::max(
                            lambda * lambdaDown, lambdaMin);
                        stepAccepted = true;
                        break;
                    }

                    // Increase damping and retry.
                    lambda = std::min(
                        lambda * lambdaUp, lambdaMax);
                }

                if (!stepAccepted)
                {
                    // Even maximal damping did not help.
                    // Increase lambda further for the next iteration
                    // and hope the landscape improves.
                    lambda = std::min(
                        lambda * lambdaUp, lambdaMax);
                }

                // Divergence guard.
                if (normalisedRms(normErrors) > 1e4)
                {
                    diverged = true;
                    break;
                }
            }

            attempt.connectorEnd = connectorEnd;
            attempt.params = params;
            attempt.converged = converged;
            attempt.hitIterationLimit =
                !converged && !diverged && iteration >= maxIterations;
            attempt.iterationCount = iteration;
            return attempt;
        }

        // Total LM iteration budget across all seed candidates.  Bounds
        // the worst-case cost of open-ended geometry while keeping the
        // search deterministic.
        constexpr std::uint32_t maxTotalSolverIterations =
            maxIterations * 5u;

        // Deterministic ordered seed candidates for the solver.  The
        // heuristic guess comes first; the remaining candidates are
        // anti-symmetric rate pairs (curve one way then back, returning
        // the heading to its start direction) on successive branches.
        // Parameter sweeps show the first such branch peaks near
        // omega*L ~= 16.8; later branches and coupled pitch+yaw spirals
        // reach connector shapes unavailable to planar seeds.  LM is a
        // local method, so the branch the seed lands in decides the
        // basin it converges towards.
        [[nodiscard]] std::vector<ParameterVector>
        computeSeedCandidates(
            const EndpointState& trackEnd,
            const EndpointState& trackStart,
            const double length)
        {
            std::vector<ParameterVector> seeds;
            seeds.push_back(
                computeInitialGuess(trackEnd, trackStart, length));

            const double invL = 1.0 / length;

            for (const double omegaLength : {8.0, 16.79, 25.0})
            {
                for (const double sign : {1.0, -1.0})
                {
                    ParameterVector yawSeed{};
                    yawSeed[3] = -sign * omegaLength * invL;
                    yawSeed[5] = sign * omegaLength * invL;
                    seeds.push_back(yawSeed);

                    ParameterVector pitchSeed{};
                    pitchSeed[0] = -sign * omegaLength * invL;
                    pitchSeed[2] = sign * omegaLength * invL;
                    seeds.push_back(pitchSeed);
                }
            }

            ParameterVector spiralSeed{};
            spiralSeed[0] = -16.79 * invL;
            spiralSeed[2] = 16.79 * invL;
            spiralSeed[3] = -16.79 * invL;
            spiralSeed[5] = 16.79 * invL;
            seeds.push_back(spiralSeed);

            // Skip seeds identical to an earlier candidate (the
            // heuristic already uses one of the anti-symmetric
            // branches when the gap is mostly along the tangent).
            std::vector<ParameterVector> uniqueSeeds;
            uniqueSeeds.reserve(seeds.size());
            for (const ParameterVector& seed : seeds)
            {
                bool duplicate = false;
                for (const ParameterVector& existing : uniqueSeeds)
                {
                    duplicate = duplicate || (seed == existing);
                }
                if (!duplicate)
                {
                    uniqueSeeds.push_back(seed);
                }
            }
            return uniqueSeeds;
        }
    }

    detail::CircuitCompletionRateBasis
    detail::evaluateCircuitCompletionRateBasis(
        const double normalizedCoordinate
    ) noexcept
    {
        if (normalizedCoordinate <= 0.5)
        {
            return {
                1.0 - 2.0 * normalizedCoordinate,
                2.0 * normalizedCoordinate,
                0.0
            };
        }

        return {
            0.0,
            2.0 - 2.0 * normalizedCoordinate,
            2.0 * normalizedCoordinate - 1.0
        };
    }

    void detail::evaluateCircuitCompletionLocalRateDerivatives(
        const double profileCoordinate,
        const double profileLength,
        CoupledLocalRateDerivatives& derivatives
    ) noexcept
    {
        const CircuitCompletionRateBasis basis =
            evaluateCircuitCompletionRateBasis(
                profileCoordinate / profileLength
            );
        const std::array<double, 3> knotBasis{
            basis.start,
            basis.midpoint,
            basis.end
        };

        for (std::size_t knot = 0; knot < knotBasis.size(); ++knot)
        {
            derivatives[knot] = {0.0, knotBasis[knot], 0.0};
            derivatives[3 + knot] = {0.0, 0.0, knotBasis[knot]};
            derivatives[6 + knot] = {knotBasis[knot], 0.0, 0.0};
        }
    }

    detail::CircuitCompletionIntegrationSchedule
    detail::makeCircuitCompletionIntegrationSchedule(
        const CircuitCompletionParameterVector& parameters,
        const double connectorLength
    )
    {
        const GeometricSection profiles = buildConnectorProfiles(
            parameters,
            connectorLength
        );
        CircuitCompletionIntegrationSchedule schedule{};
        for (std::size_t span = 0; span < schedule.spans.size(); ++span)
        {
            schedule.spans[span] = makeCoupledIntegrationSchedule(
                &profiles.roll.segments[span].transition,
                profiles.pitch.segments[span].transition,
                profiles.yaw.segments[span].transition,
                integrationSpacing
            );
        }
        return schedule;
    }

    detail::CircuitCompletionEndpoint
    detail::evaluateCircuitCompletionFullCoupledEndpoint(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        const double connectorLength,
        const CircuitCompletionIntegrationSchedule& schedule
    )
    {
        const GeometricSection profiles = buildConnectorProfiles(
            parameters,
            connectorLength
        );
        glm::dvec3 position = startingEndpoint.position;
        geometry::CurveFrame frame{
            startingEndpoint.tangent,
            glm::normalize(glm::cross(
                startingEndpoint.up,
                startingEndpoint.tangent
            )),
            startingEndpoint.up
        };

        for (std::size_t span = 0; span < schedule.spans.size(); ++span)
        {
            const RiderLocalGeometryState endpoint =
                integrateCoupledRateProfilesEndpoint(
                    position,
                    frame,
                    &profiles.roll.segments[span].transition,
                    profiles.pitch.segments[span].transition,
                    profiles.yaw.segments[span].transition,
                    schedule.spans[span]
                );
            position = endpoint.position;
            frame = endpoint.frame;
        }

        return {position, frame.tangent, frame.up};
    }

    detail::CircuitCompletionEndpoint
    detail::evaluateCircuitCompletionProductionEndpoint(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        const double connectorLength
    )
    {
        return integrateConnector(
            startingEndpoint,
            parameters,
            connectorLength
        );
    }

    detail::CircuitCompletionSensitivityResult
    detail::evaluateCircuitCompletionEndpointSensitivities(
        const CircuitCompletionEndpoint& startingEndpoint,
        const CircuitCompletionParameterVector& parameters,
        const double connectorLength,
        const CircuitCompletionIntegrationSchedule& schedule
    )
    {
        const GeometricSection profiles = buildConnectorProfiles(
            parameters,
            connectorLength
        );
        CoupledEndpointSensitivityState sensitivity{
            {
                0.0,
                startingEndpoint.position,
                {
                    startingEndpoint.tangent,
                    glm::normalize(glm::cross(
                        startingEndpoint.up,
                        startingEndpoint.tangent
                    )),
                    startingEndpoint.up
                }
            },
            {},
            {}
        };

        for (std::size_t span = 0; span < schedule.spans.size(); ++span)
        {
            sensitivity =
                integrateCoupledRateProfileSensitivitiesEndpoint(
                    sensitivity,
                    &profiles.roll.segments[span].transition,
                    profiles.pitch.segments[span].transition,
                    profiles.yaw.segments[span].transition,
                    schedule.spans[span],
                    connectorLength,
                    evaluateCircuitCompletionLocalRateDerivatives
                );
        }

        CircuitCompletionJacobian jacobian{};
        const geometry::CurveFrame& endpointFrame =
            sensitivity.endpoint.frame;
        const double inverseLength = 1.0 / connectorLength;
        for (std::size_t parameter = 0;
             parameter < circuitCompletionParameterCount;
             ++parameter)
        {
            const glm::dvec3 dTangent = glm::cross(
                sensitivity.rotationSensitivity[parameter],
                endpointFrame.tangent
            );
            const glm::dvec3 dUp = glm::cross(
                sensitivity.rotationSensitivity[parameter],
                endpointFrame.up
            );
            const glm::dvec3 dPosition =
                inverseLength * sensitivity.dPosition[parameter];
            jacobian[parameter] = {
                dPosition.x,
                dPosition.y,
                dPosition.z,
                dTangent.x,
                dTangent.y,
                dTangent.z,
                dUp.x,
                dUp.y,
                dUp.z
            };
        }

        return {sensitivity, jacobian};
    }

    detail::CircuitCompletionResidual
    detail::computeCircuitCompletionResidual(
        const CircuitCompletionEndpoint& actualEndpoint,
        const CircuitCompletionEndpoint& desiredEndpoint,
        const double connectorLength
    )
    {
        const glm::dvec3 positionDifference =
            actualEndpoint.position - desiredEndpoint.position;
        const CircuitCompletionOrientationResidual orientationResidual =
            computeCircuitCompletionOrientationResidual(
                actualEndpoint.tangent,
                actualEndpoint.up,
                desiredEndpoint.tangent,
                desiredEndpoint.up
            );
        const double inverseLength = 1.0 / connectorLength;

        return {
            positionDifference.x * inverseLength,
            positionDifference.y * inverseLength,
            positionDifference.z * inverseLength,
            orientationResidual[0],
            orientationResidual[1],
            orientationResidual[2],
            orientationResidual[3],
            orientationResidual[4],
            orientationResidual[5]
        };
    }

    // ----------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------

    const char* circuitCompletionFailureLabel(
        const CircuitCompletionFailure failure) noexcept
    {
        switch (failure)
        {
        case CircuitCompletionFailure::None:
            return "No failure";
        case CircuitCompletionFailure::AlreadyClosed:
            return "Track is already a closed circuit";
        case CircuitCompletionFailure::ShuttleLayout:
            return "Circuit completion is not available for "
                   "Shuttle layouts";
        case CircuitCompletionFailure::InvalidInput:
            return "Invalid input: track has no sections or "
                   "contains unsupported region types";
        case CircuitCompletionFailure::UnsupportedGeometry:
            return "The solver could not find a valid connector "
                   "for this geometry";
        case CircuitCompletionFailure::DidNotConverge:
            return "The solver did not converge within the "
                   "iteration limit";
        case CircuitCompletionFailure::ValidationFailed:
            return "Generated connector failed validation";
        }
        return "Unknown failure";
    }

    CircuitCompletionResult
    completeCircuitCandidate(
        const AuthoredTrack& source,
        const CircuitCompletionSettings& settings)
    {
        auto fail = [](CircuitCompletionFailure reason,
                       std::string message,
                       double gap = 0.0,
                       double tang = 0.0,
                       double frame = 0.0,
                       std::uint32_t iter = 0) mutable
            -> CircuitCompletionResult
        {
            CircuitCompletionResult r;
            r.failureReason = reason;
            r.failureMessage = std::move(message);
            r.finalPositionalGap = gap;
            r.finalTangentErrorDegrees = tang;
            r.finalFrameErrorDegrees = frame;
            r.iterationCount = iter;
            return r;
        };

        // --- Pre-condition checks ---

        if (source.layoutMode() == LayoutMode::Shuttle)
        {
            return fail(
                CircuitCompletionFailure::ShuttleLayout,
                circuitCompletionFailureLabel(
                    CircuitCompletionFailure::ShuttleLayout));
        }

        if (source.sectionCount() == 0)
        {
            return fail(
                CircuitCompletionFailure::InvalidInput,
                circuitCompletionFailureLabel(
                    CircuitCompletionFailure::InvalidInput));
        }

        // Check that the source contains only RateProfile regions
        for (std::size_t i = 0; i < source.sectionCount(); ++i)
        {
            if (source.section(i).kind != RegionKind::RateProfiles)
            {
                return fail(
                    CircuitCompletionFailure::InvalidInput,
                    "Section "
                    + std::to_string(i)
                    + " uses Geometry authoring which is not "
                      "supported by the completion solver");
            }
        }

        // Check if already closed.
        const TrackTopology topology =
            computeTrackTopology(source);

        if (topology.kind == TopologyKind::ClosedCircuit)
        {
            return fail(
                CircuitCompletionFailure::AlreadyClosed,
                circuitCompletionFailureLabel(
                    CircuitCompletionFailure::AlreadyClosed),
                topology.diagnostics.positionalGap,
                topology.diagnostics.tangentMismatchDegrees,
                topology.diagnostics.frameMismatchDegrees);
        }

        // --- Extract endpoint states ---

        const std::vector<RiderLocalGeometryState> states =
            integrateAuthoredTrack(source, integrationSpacing);

        const RiderLocalGeometryState& startState = states.front();
        const RiderLocalGeometryState& endState = states.back();

        const EndpointState trackStart{
            startState.position,
            startState.frame.tangent,
            startState.frame.up};

        const EndpointState trackEnd{
            endState.position,
            endState.frame.tangent,
            endState.frame.up};

        const double length =
            settings.preferredConnectorLength;

        if (!std::isfinite(length) || length <= 0.0)
        {
            return fail(
                CircuitCompletionFailure::InvalidInput,
                "Connector length must be positive and finite");
        }

        // --- Levenberg-Marquardt solver ---

        // Deterministic seed candidates are tried in order until one
        // converges; otherwise the attempt with the smallest residual
        // is kept for reporting.
        std::vector<ParameterVector> seeds;

        if (settings.initialParamOverride.has_value())
        {
            seeds.push_back(
                detail::expandLegacyCircuitCompletionParameters(
                    *settings.initialParamOverride));
        }
        else
        {
            seeds = computeSeedCandidates(
                trackEnd, trackStart, length);
        }

        bool haveBestAttempt = false;
        LevenbergMarquardtAttempt bestAttempt;
        double bestResidualRms = 0.0;
        std::uint32_t totalIterationsSpent = 0;

        for (const ParameterVector& seed : seeds)
        {
            LevenbergMarquardtAttempt attempt =
                runLevenbergMarquardt(
                    trackEnd, trackStart, length, seed);

            totalIterationsSpent += attempt.iterationCount;

            const double residualRms = normalisedRms(
                computeNormalisedError(
                    attempt.connectorEnd, trackStart, length));

            if (detail::shouldReplaceCircuitCompletionAttempt(
                    haveBestAttempt,
                    haveBestAttempt && bestAttempt.converged,
                    bestResidualRms,
                    attempt.converged,
                    residualRms))
            {
                haveBestAttempt = true;
                bestAttempt = attempt;
                bestResidualRms = residualRms;
            }

            if (attempt.converged
                || totalIterationsSpent >= maxTotalSolverIterations)
            {
                break;
            }
        }

        const EndpointState& connectorEnd =
            bestAttempt.connectorEnd;

        // --- Evaluate convergence ---

        constexpr TopologyTolerances tolerances;

        const double positionalGap =
            glm::length(connectorEnd.position - trackStart.position);
        const double tangentDot = glm::clamp(
            glm::dot(connectorEnd.tangent, trackStart.tangent),
            -1.0, 1.0);
        const double tangentErrorDeg =
            std::acos(tangentDot) * degreesPerRadian;
        const double frameDot = glm::clamp(
            glm::dot(connectorEnd.up, trackStart.up),
            -1.0, 1.0);
        const double frameErrorDeg =
            std::acos(frameDot) * degreesPerRadian;

        if (positionalGap > tolerances.closureGapTolerance
            || tangentErrorDeg > tolerances.angleTolerance
            || frameErrorDeg > tolerances.angleTolerance)
        {
            const bool exhaustedIterations =
                bestAttempt.hitIterationLimit;

            return fail(
                exhaustedIterations
                    ? CircuitCompletionFailure::DidNotConverge
                    : CircuitCompletionFailure::UnsupportedGeometry,
                exhaustedIterations
                    ? circuitCompletionFailureLabel(
                        CircuitCompletionFailure::DidNotConverge)
                    : circuitCompletionFailureLabel(
                        CircuitCompletionFailure::UnsupportedGeometry),
                positionalGap, tangentErrorDeg, frameErrorDeg,
                bestAttempt.iterationCount);
        }

        // --- Build the candidate ---

        AuthoredTrack candidate = source;

        AuthoredTrackSection connectorSection;
        connectorSection.kind = RegionKind::RateProfiles;
        connectorSection.length = length;

        const GeometricSection profiles =
            buildConnectorProfiles(bestAttempt.params, length);
        connectorSection.region = RateProfileRegion{profiles};

        try
        {
            candidate.insertSectionAfter(
                candidate.sectionCount() - 1,
                connectorSection);
        }
        catch (const std::exception& e)
        {
            return fail(
                CircuitCompletionFailure::ValidationFailed,
                std::string("Generated connector failed "
                    "validation: ") + e.what(),
                positionalGap, tangentErrorDeg, frameErrorDeg,
                bestAttempt.iterationCount);
        }

        // --- Verify closure on the full candidate ---

        const TrackTopology verifyTopology =
            computeTrackTopology(candidate);

        if (verifyTopology.kind != TopologyKind::ClosedCircuit)
        {
            return fail(
                CircuitCompletionFailure::ValidationFailed,
                "Post-completion topology verification failed",
                verifyTopology.diagnostics.positionalGap,
                verifyTopology.diagnostics.tangentMismatchDegrees,
                verifyTopology.diagnostics.frameMismatchDegrees,
                bestAttempt.iterationCount);
        }

        CircuitCompletionResult result;
        result.success = true;
        result.completedTrack = std::move(candidate);
        result.connectorRegionCount = 1;
        result.finalPositionalGap =
            verifyTopology.diagnostics.positionalGap;
        result.finalTangentErrorDegrees =
            verifyTopology.diagnostics.tangentMismatchDegrees;
        result.finalFrameErrorDegrees =
            verifyTopology.diagnostics.frameMismatchDegrees;
        result.finalMaxAbsParam = 0.0;
        for (std::size_t i = 0; i < parameterCount; ++i)
        {
            result.finalMaxAbsParam =
                std::max(result.finalMaxAbsParam,
                    std::abs(bestAttempt.params[i]));
        }
        result.iterationCount = bestAttempt.iterationCount;
        return result;
    }
}
