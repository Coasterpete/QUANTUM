#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::LayoutMode;
    using quantum::coaster::RateProfileRegion;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::geometry::CurveFrame;

    constexpr double piRadians = 3.14159265358979323846;
    constexpr double finiteDiffEpsilon = 1e-7;
    constexpr double integrationSpacing = 0.5;
    constexpr std::size_t parameterCount = 6;

    using ParameterVector = std::array<double, parameterCount>;
    using ErrorVector     = std::array<double, parameterCount>;
    using JacobianMatrix  = std::array<ErrorVector, parameterCount>;

    AuthoredTrack makeStraightTrack(const double length)
    {
        AuthoredTrack track;
        track.setLayoutMode(LayoutMode::Circuit);
        track.prependSection();

        AuthoredTrackSection section;
        section.kind = RegionKind::RateProfiles;
        section.length = length;

        auto makeZero = [&](double len)
        {
            ChannelProfile p;
            p.nextSegmentId = 2;
            ProfileSegment seg;
            seg.id = 1;
            seg.transition.domainBegin = 0.0;
            seg.transition.domainEnd = len;
            seg.transition.valueBegin = 0.0;
            seg.transition.valueEnd = 0.0;
            seg.transition.transitionType =
                quantum::math::TransitionType::Linear;
            p.segments.push_back(seg);
            return p;
        };

        GeometricSection profiles;
        profiles.pitch = makeZero(length);
        profiles.yaw = makeZero(length);
        profiles.roll = makeZero(length);
        section.region = RateProfileRegion{profiles};
        track.section(0) = std::move(section);
        return track;
    }

    // --- Helpers copied from CircuitCompletion.cpp anonymous namespace ---

    struct EndpointState
    {
        glm::dvec3 position;
        glm::dvec3 tangent;
        glm::dvec3 up;
    };

    ChannelProfile makeLinearProfile(
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
            quantum::math::TransitionType::Linear;

        profile.segments.push_back(segment);
        return profile;
    }

    GeometricSection buildConnectorProfiles(
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

    EndpointState integrateConnector(
        const EndpointState& startState,
        const ParameterVector& params,
        const double length)
    {
        const GeometricSection profiles =
            buildConnectorProfiles(params, length);

        CurveFrame startFrame;
        startFrame.tangent = startState.tangent;
        startFrame.lateral = glm::normalize(
            glm::cross(startState.up, startState.tangent));
        startFrame.up = startState.up;

        const std::vector<RiderLocalGeometryState> states =
            quantum::coaster::integrateLocalRollPitchYawRateProfiles(
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

    ErrorVector computeNormalisedError(
        const EndpointState& connectorEnd,
        const EndpointState& trackStart,
        const double length)
    {
        const glm::dvec3 posGap =
            connectorEnd.position - trackStart.position;
        const glm::dvec3 tangentError =
            connectorEnd.tangent - trackStart.tangent;

        const glm::dvec3 right = glm::normalize(
            glm::cross(trackStart.tangent, trackStart.up));

        const double alongTangent =
            glm::dot(tangentError, trackStart.tangent);
        const double perpTangent =
            glm::dot(tangentError, right);
        const double frameRoll =
            glm::dot(connectorEnd.up - trackStart.up, right);

        const double invL = 1.0 / length;
        return {
            posGap.x * invL, posGap.y * invL, posGap.z * invL,
            alongTangent, perpTangent, frameRoll};
    }

    double normalisedRms(const ErrorVector& e)
    {
        double sum = 0.0;
        for (const double v : e)
            sum += v * v;
        return std::sqrt(sum / static_cast<double>(parameterCount));
    }

    ParameterVector computeInitialGuess(
        const EndpointState& trackEnd,
        const EndpointState& trackStart,
        const double length)
    {
        const glm::dvec3 gap =
            trackStart.position - trackEnd.position;

        const double invL = 1.0 / length;
        const double invL2 = invL * invL;

        const double alongDot =
            glm::dot(gap, trackStart.tangent);
        const double alongFrac =
            std::abs(alongDot) * invL;

        const double pitchPos =
            -2.0 * gap.z * invL2;

        const double yawPos =
            2.0 * (gap.x * trackStart.tangent.y
                 - gap.y * trackStart.tangent.x) * invL2;

        const double tangentErrLength =
            glm::length(trackStart.tangent - trackEnd.tangent);

        double pitchStart = pitchPos * 0.5;
        double pitchEnd = pitchPos * 0.5
            + tangentErrLength * invL * 0.5;
        double yawStart = yawPos * 0.5;
        double yawEnd = yawPos * 0.5
            + tangentErrLength * invL * 0.5;

        if (alongFrac > 0.1)
        {
            const double sign =
                alongDot > 0.0 ? 1.0 : -1.0;

            constexpr double clothoidConstant = 16.79;
            const double omega =
                clothoidConstant * invL;

            yawStart = -sign * omega;
            yawEnd = sign * omega;

            const double perpGapZ =
                glm::dot(gap, trackStart.up);
            if (std::abs(perpGapZ) > 0.01)
            {
                pitchStart = -perpGapZ * invL2;
                pitchEnd = -perpGapZ * invL2;
            }
        }

        const double rollStart = 0.0;
        const double rollEnd = 0.0;

        return {pitchStart, pitchEnd,
                yawStart, yawEnd,
                rollStart, rollEnd};
    }

    // --- Printing helpers ---

    void printVec6(const char* label, const std::array<double, 6>& v)
    {
        std::fprintf(stdout, "%s: [", label);
        for (std::size_t i = 0; i < 6; ++i)
        {
            std::fprintf(stdout, "%12.6f", v[i]);
            if (i < 5) std::fprintf(stdout, ", ");
        }
        std::fprintf(stdout, "]\n");
    }

    void printMatrix6x6(const char* label, const JacobianMatrix& m)
    {
        std::fprintf(stdout, "%s:\n", label);
        for (std::size_t r = 0; r < 6; ++r)
        {
            std::fprintf(stdout, "  [");
            for (std::size_t c = 0; c < 6; ++c)
            {
                std::fprintf(stdout, "%12.6f", m[r][c]);
                if (c < 5) std::fprintf(stdout, " ");
            }
            std::fprintf(stdout, "]\n");
        }
    }

    void analyzePoint(
        const char* label,
        const EndpointState& trackEnd,
        const EndpointState& trackStart,
        const double length,
        const ParameterVector& params)
    {
        std::fprintf(stdout, "\n========================================\n");
        std::fprintf(stdout, "  %s\n", label);
        std::fprintf(stdout, "========================================\n");

        printVec6("params", params);

        // Integrate connector
        EndpointState connectorEnd =
            integrateConnector(trackEnd, params, length);

        // Raw diagnostics
        const double posGap =
            glm::length(connectorEnd.position - trackStart.position);
        const double tangDot = glm::clamp(
            glm::dot(connectorEnd.tangent, trackStart.tangent),
            -1.0, 1.0);
        const double tangDeg = std::acos(tangDot) * (180.0 / piRadians);
        const double frameDot = glm::clamp(
            glm::dot(connectorEnd.up, trackStart.up),
            -1.0, 1.0);
        const double frameDeg = std::acos(frameDot) * (180.0 / piRadians);

        std::fprintf(stdout, "\n--- Closure diagnostics ---\n");
        std::fprintf(stdout, "  Position gap:     %.6f m\n", posGap);
        std::fprintf(stdout, "  Tangent error:    %.6f deg\n", tangDeg);
        std::fprintf(stdout, "  Frame error:      %.6f deg\n", frameDeg);

        // Normalised error
        ErrorVector normErrors =
            computeNormalisedError(connectorEnd, trackStart, length);
        printVec6("normalised error", normErrors);
        std::fprintf(stdout, "  RMS:              %.8f\n",
            normalisedRms(normErrors));

        // Finite-difference Jacobian
        JacobianMatrix jacobian{};
        for (std::size_t col = 0; col < parameterCount; ++col)
        {
            ParameterVector perturbed = params;
            perturbed[col] += finiteDiffEpsilon;

            const EndpointState perturbedEnd =
                integrateConnector(trackEnd, perturbed, length);
            const ErrorVector perturbedNorm =
                computeNormalisedError(
                    perturbedEnd, trackStart, length);

            for (std::size_t row = 0; row < parameterCount; ++row)
            {
                jacobian[col][row] =
                    (perturbedNorm[row] - normErrors[row])
                    / finiteDiffEpsilon;
            }
        }

        printMatrix6x6("Jacobian J (cols=params, rows=errors)", jacobian);

        // J^T J
        JacobianMatrix jtj{};
        for (std::size_t i = 0; i < 6; ++i)
        {
            for (std::size_t j = 0; j < 6; ++j)
            {
                double sum = 0.0;
                for (std::size_t k = 0; k < 6; ++k)
                {
                    sum += jacobian[i][k] * jacobian[j][k];
                }
                jtj[i][j] = sum;
            }
        }

        printMatrix6x6("J^T J", jtj);

        // Column norms of J (sqrt of diagonal of J^T J)
        std::fprintf(stdout, "\n--- Column norms of J (sqrt of diag(J^T J)) ---\n");
        double maxCol = 0.0;
        double minCol = std::numeric_limits<double>::max();
        for (std::size_t c = 0; c < 6; ++c)
        {
            double cn = std::sqrt(jtj[c][c]);
            std::fprintf(stdout, "  col %u: %.8f\n",
                static_cast<unsigned>(c), cn);
            maxCol = std::max(maxCol, cn);
            if (cn > 1e-14) minCol = std::min(minCol, cn);
        }

        double condNum = minCol > 1e-14
            ? maxCol / minCol
            : std::numeric_limits<double>::infinity();
        std::fprintf(stdout, "  max col norm:  %.8f\n", maxCol);
        std::fprintf(stdout, "  min col norm:  %.8f\n", minCol);
        std::fprintf(stdout, "  condition #:   %.4f\n", condNum);

        // Trace and determinant of J^T J (rough diagnostics)
        double trace = 0.0;
        for (std::size_t i = 0; i < 6; ++i)
            trace += jtj[i][i];
        std::fprintf(stdout, "  trace(J^T J):  %.8f\n", trace);
    }
}

int main()
{
    const double connectorLength = 60.0;
    AuthoredTrack track30 = makeStraightTrack(30.0);

    // Extract track endpoints
    const std::vector<RiderLocalGeometryState> states =
        quantum::coaster::integrateAuthoredTrack(
            track30, integrationSpacing);

    const EndpointState trackStart{
        states.front().position,
        states.front().frame.tangent,
        states.front().frame.up};

    const EndpointState trackEnd{
        states.back().position,
        states.back().frame.tangent,
        states.back().frame.up};

    std::fprintf(stdout,
        "Track: straight %.0fm, connector: %.0fm\n",
        30.0, connectorLength);
    std::fprintf(stdout,
        "Track end position:  (%.4f, %.4f, %.4f)\n",
        trackEnd.position.x, trackEnd.position.y, trackEnd.position.z);
    std::fprintf(stdout,
        "Track start position: (%.4f, %.4f, %.4f)\n",
        trackStart.position.x, trackStart.position.y, trackStart.position.z);
    std::fprintf(stdout,
        "Gap vector:          (%.4f, %.4f, %.4f)\n",
        trackStart.position.x - trackEnd.position.x,
        trackStart.position.y - trackEnd.position.y,
        trackStart.position.z - trackEnd.position.z);
    fflush(stdout);

    // Point A: all zeros
    ParameterVector zeros{0, 0, 0, 0, 0, 0};
    analyzePoint(
        "Point A: all-zero params",
        trackEnd, trackStart, connectorLength, zeros);

    // Point B: clothoid initial guess
    ParameterVector clothoidGuess =
        computeInitialGuess(trackEnd, trackStart, connectorLength);
    analyzePoint(
        "Point B: clothoid initial guess",
        trackEnd, trackStart, connectorLength, clothoidGuess);

    // Point C: pitch-only local minimum
    ParameterVector pitchLocalMin{0.2736, -0.2667, 0, 0, 0, 0};
    analyzePoint(
        "Point C: pitch-only local minimum",
        trackEnd, trackStart, connectorLength, pitchLocalMin);

    std::fprintf(stdout, "\n========================================\n");
    std::fprintf(stdout, "  Summary\n");
    std::fprintf(stdout, "========================================\n\n");
    fflush(stdout);

    return 0;
}
