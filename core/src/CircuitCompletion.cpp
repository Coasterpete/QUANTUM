#include <quantum/coaster/CircuitCompletion.hpp>

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
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

        // Number of free parameters: start/end rates for
        // pitch, yaw, roll.
        constexpr std::size_t parameterCount = 6;

        using ParameterVector =
            std::array<double, parameterCount>;
        using ErrorVector =
            std::array<double, parameterCount>;
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

        // Build a single-segment ChannelProfile with linear
        // interpolation from valueBegin to valueEnd over [0, length].
        [[nodiscard]] ChannelProfile makeLinearProfile(
            const double valueBegin,
            const double valueEnd,
            const double length)
        {
            ChannelProfile profile;
            profile.nextSegmentId = 2;

            ProfileSegment segment;
            segment.id = 1;
            segment.transition.domainBegin = 0.0;
            segment.transition.domainEnd = length;
            segment.transition.valueBegin = valueBegin;
            segment.transition.valueEnd = valueEnd;
            segment.transition.transitionType =
                math::TransitionType::Linear;

            profile.segments.push_back(segment);
            return profile;
        }

        // Build the 3-channel rate profile for a connector section
        // from the 6-element parameter vector.
        [[nodiscard]] GeometricSection buildConnectorProfiles(
            const ParameterVector& params,
            const double length)
        {
            GeometricSection profiles;
            profiles.pitch = makeLinearProfile(
                params[0], params[1], length);
            profiles.yaw = makeLinearProfile(
                params[2], params[3], length);
            profiles.roll = makeLinearProfile(
                params[4], params[5], length);
            return profiles;
        }

        // Integrate a connector from a given starting state and
        // return the endpoint state {position, tangent, up}.
        struct EndpointState
        {
            glm::dvec3 position;
            glm::dvec3 tangent;
            glm::dvec3 up;
        };

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

        // Compute the raw error vector for the closure.
        //
        // Error layout:
        //   [0] = position.x gap  (metres)
        //   [1] = position.y gap  (metres)
        //   [2] = position.z gap  (metres)
        //   [3] = tangent error x (dimensionless)
        //   [4] = tangent error y (dimensionless)
        //   [5] = frame error z   (dimensionless)
        [[nodiscard]] ErrorVector computeClosureError(
            const EndpointState& connectorEnd,
            const EndpointState& trackStart)
        {
            const glm::dvec3 posGap =
                connectorEnd.position - trackStart.position;
            const glm::dvec3 tangentError =
                connectorEnd.tangent - trackStart.tangent;
            const glm::dvec3 frameError =
                connectorEnd.up - trackStart.up;

            return {posGap.x, posGap.y, posGap.z,
                    tangentError.x, tangentError.y,
                    frameError.z};
        }

        // Compute a scale-normalised error vector.  Position
        // components are divided by connector length so that the
        // six components have comparable magnitude.
        //
        // Orientation errors use two locally linear projections
        // of the tangent error onto the start-frame right and up
        // directions, plus the frame roll projection:
        //   [3] = tangent-up error    (linear in pitch heading)
        //   [4] = tangent-right error (linear in yaw heading)
        //   [5] = frame roll error    (linear in roll)
        //
        // All three orientation components are linear in their
        // respective angles, providing gradient everywhere.
        [[nodiscard]] ErrorVector computeNormalisedError(
            const EndpointState& connectorEnd,
            const EndpointState& trackStart,
            const double length)
        {
            const glm::dvec3 posGap =
                connectorEnd.position - trackStart.position;
            const glm::dvec3 tangentError =
                connectorEnd.tangent - trackStart.tangent;

            // Local perpendicular frame at the track start.
            const glm::dvec3 right = glm::normalize(
                glm::cross(trackStart.tangent, trackStart.up));

            // Tangent-up component: projects the tangent error onto
            // the start-frame up direction.  This is linear in the
            // pitch component of the heading error and provides
            // gradient everywhere the heading deviates from nominal.
            const double tangentUp =
                glm::dot(tangentError, trackStart.up);

            // Tangent-right component: projects the tangent error
            // onto the start-frame right direction.  This is linear
            // in the yaw component of the heading error.
            const double tangentRight =
                glm::dot(tangentError, right);

            // Frame roll error: projects the frame error onto the
            // start-frame right direction.  Linear for all angles.
            const double frameRoll =
                glm::dot(connectorEnd.up - trackStart.up, right);

            const double invL = 1.0 / length;
            return {
                posGap.x * invL, posGap.y * invL, posGap.z * invL,
                tangentUp, tangentRight, frameRoll};
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
            return std::sqrt(sum / static_cast<double>(parameterCount));
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

            return {pitchStart, pitchEnd,
                    yawStart, yawEnd,
                    rollStart, rollEnd};
        }
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

        ParameterVector params{};
        if (settings.initialParamOverride.has_value())
        {
            for (std::size_t i = 0; i < parameterCount; ++i)
            {
                params[i] = (*settings.initialParamOverride)[i];
            }
        }
        else
        {
            params =
                computeInitialGuess(trackEnd, trackStart, length);
        }

        EndpointState connectorEnd =
            integrateConnector(trackEnd, params, length);

        // Compute raw errors for the convergence check
        // (position in metres, angles in radians).
        ErrorVector rawErrors =
            computeClosureError(connectorEnd, trackStart);

        // Compute normalised errors for the Jacobian and LM solve.
        ErrorVector normErrors =
            computeNormalisedError(
                connectorEnd, trackStart, length);

        using Clock = std::chrono::steady_clock;

        std::uint32_t iteration = 0;
        double totalJacobianMs = 0.0;
        double totalSolveMs = 0.0;
        double totalStepMs = 0.0;

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

        for (iteration = 0; iteration < maxIterations; ++iteration)
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
                break;
            }

            if (iteration < 8 || iteration % 20 == 0)
            {
                std::fprintf(stderr,
                    "[solver] iter=%u posGap=%.4f "
                    "tang=%.4f frame=%.4f "
                    "p=(%.4f,%.4f) y=(%.4f,%.4f) "
                    "lam=%.2e\n",
                    iteration, posGap, tangDeg, frameDeg,
                    params[0], params[1], params[2],
                    params[3], lambda);
                fflush(stderr);
            }

            // Finite-difference Jacobian of the normalised error.
            JacobianMatrix jacobian;

            auto tJac0 = Clock::now();

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
                     row < parameterCount; ++row)
                {
                    jacobian[col][row] =
                        (perturbedNorm[row] - normErrors[row])
                        / finiteDiffEpsilon;
                }
            }

            auto tJac1 = Clock::now();
            totalJacobianMs += std::chrono::duration<double,
                std::milli>(tJac1 - tJac0).count();

            // Report Jacobian conditioning on first iteration.
            if (iteration == 0)
            {
                double maxCol = 0.0;
                double minCol =
                    std::numeric_limits<double>::max();
                for (std::size_t c = 0;
                     c < parameterCount; ++c)
                {
                    double cn = 0.0;
                    for (std::size_t r = 0;
                         r < parameterCount; ++r)
                    {
                        cn += jacobian[c][r]
                            * jacobian[c][r];
                    }
                    cn = std::sqrt(cn);
                    maxCol = std::max(maxCol, cn);
                    minCol = std::min(minCol, cn);
                }
                std::fprintf(stderr,
                    "[solver] jacobian cond: %.2f "
                    "(maxCol=%.4f minCol=%.6f)\n",
                    minCol > 0.0
                        ? maxCol / minCol
                        : std::numeric_limits<double>::infinity(),
                    maxCol, minCol);
                fflush(stderr);
            }

            // Build normal equations:  (J^T J + lambda I) dx = -J^T e.
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
                         k < parameterCount; ++k)
                    {
                        sum += jacobian[j][k]
                            * jacobian[i][k];
                    }
                    jtj[i][j] = sum;
                }

                double sum = 0.0;
                for (std::size_t k = 0;
                     k < parameterCount; ++k)
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

            auto tStep0 = Clock::now();

            for (int lmRetry = 0;
                 lmRetry < maxLmRetries; ++lmRetry)
            {
                // Augmented system: (J^T J + lambda*I) dx = -J^T e.
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

                // Clamp step magnitude.
                double maxDx = 0.0;
                for (std::size_t i = 0;
                     i < parameterCount; ++i)
                {
                    maxDx = std::max(maxDx, std::abs(dx[i]));
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
                    rawErrors = computeClosureError(
                        connectorEnd, trackStart);
                    lambda = std::max(
                        lambda * lambdaDown, lambdaMin);
                    stepAccepted = true;
                    break;
                }

                // Increase damping and retry.
                lambda = std::min(
                    lambda * lambdaUp, lambdaMax);
            }

            auto tStep1 = Clock::now();
            totalStepMs += std::chrono::duration<double,
                std::milli>(tStep1 - tStep0).count();

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
                break;
            }
        }

        // --- Evaluate convergence ---

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
            return fail(
                iteration >= maxIterations
                    ? CircuitCompletionFailure::DidNotConverge
                    : CircuitCompletionFailure::UnsupportedGeometry,
                iteration >= maxIterations
                    ? circuitCompletionFailureLabel(
                        CircuitCompletionFailure::DidNotConverge)
                    : circuitCompletionFailureLabel(
                        CircuitCompletionFailure::UnsupportedGeometry),
                positionalGap, tangentErrorDeg, frameErrorDeg,
                iteration);
        }

        // --- Build the candidate ---

        AuthoredTrack candidate = source;

        AuthoredTrackSection connectorSection;
        connectorSection.kind = RegionKind::RateProfiles;
        connectorSection.length = length;

        const GeometricSection profiles =
            buildConnectorProfiles(params, length);
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
                iteration);
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
                iteration);
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
                    std::abs(params[i]));
        }
        result.iterationCount = iteration;
        return result;
    }
}
