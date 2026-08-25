#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <glm/glm.hpp>
#include <cmath>
#include <cstdio>
#include <array>
#include <vector>

using namespace quantum::coaster;

static AuthoredTrack makeStraightTrack(double length)
{
    AuthoredTrack track;
    track.setLayoutMode(LayoutMode::Circuit);
    track.prependSection();
    AuthoredTrackSection section;
    section.kind = RegionKind::RateProfiles;
    section.length = length;
    auto makeZero = [](double len) {
        ChannelProfile p;
        p.nextSegmentId = 2;
        ProfileSegment seg;
        seg.id = 1;
        seg.transition.domainBegin = 0.0;
        seg.transition.domainEnd = len;
        seg.transition.valueBegin = 0.0;
        seg.transition.valueEnd = 0.0;
        seg.transition.transitionType = quantum::math::TransitionType::Linear;
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

struct EndpointState {
    glm::dvec3 position;
    glm::dvec3 tangent;
    glm::dvec3 up;
};

static ChannelProfile makeLinearProfile(double v0, double v1, double len)
{
    ChannelProfile p;
    p.nextSegmentId = 2;
    ProfileSegment seg;
    seg.id = 1;
    seg.transition.domainBegin = 0.0;
    seg.transition.domainEnd = len;
    seg.transition.valueBegin = v0;
    seg.transition.valueEnd = v1;
    seg.transition.transitionType = quantum::math::TransitionType::Linear;
    p.segments.push_back(seg);
    return p;
}

static GeometricSection buildProfiles(const std::array<double,6>& params, double len)
{
    GeometricSection g;
    g.pitch = makeLinearProfile(params[0], params[1], len);
    g.yaw   = makeLinearProfile(params[2], params[3], len);
    g.roll  = makeLinearProfile(params[4], params[5], len);
    return g;
}

static EndpointState integrate(
    const EndpointState& start,
    const std::array<double,6>& params,
    double length)
{
    auto profiles = buildProfiles(params, length);
    quantum::geometry::CurveFrame sf;
    sf.tangent = start.tangent;
    sf.lateral = glm::normalize(glm::cross(start.up, start.tangent));
    sf.up = start.up;
    auto states = integrateLocalRollPitchYawRateProfiles(
        start.position, sf, profiles.roll, profiles.pitch, profiles.yaw,
        length, 0.5);
    const auto& endState = states.back();
    return {endState.position, endState.frame.tangent, endState.frame.up};
}

static std::array<double,6> computeError(
    const EndpointState& connEnd,
    const EndpointState& target)
{
    auto posGap = connEnd.position - target.position;
    auto tanErr = connEnd.tangent - target.tangent;
    auto frmErr = connEnd.up - target.up;
    return {posGap.x, posGap.y, posGap.z,
            tanErr.x, tanErr.y, frmErr.z};
}

static double rms(const std::array<double,6>& e)
{
    double s = 0;
    for (double v : e) s += v*v;
    return std::sqrt(s / 6.0);
}

int main()
{
    std::fprintf(stderr, "=== NUMERICAL FEASIBILITY ANALYSIS ===\n\n");

    auto track = makeStraightTrack(30.0);
    auto states = integrateAuthoredTrack(track, 0.5);
    EndpointState trackStart = {states.front().position,
        states.front().frame.tangent, states.front().frame.up};
    EndpointState trackEnd = {states.back().position,
        states.back().frame.tangent, states.back().frame.up};

    double length = 40.0;

    std::fprintf(stderr, "trackStart pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
        trackStart.position.x, trackStart.position.y, trackStart.position.z,
        trackStart.tangent.x, trackStart.tangent.y, trackStart.tangent.z,
        trackStart.up.x, trackStart.up.y, trackStart.up.z);
    std::fprintf(stderr, "trackEnd   pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
        trackEnd.position.x, trackEnd.position.y, trackEnd.position.z,
        trackEnd.tangent.x, trackEnd.tangent.y, trackEnd.tangent.z,
        trackEnd.up.x, trackEnd.up.y, trackEnd.up.z);

    // Gap vector
    auto gap = trackStart.position - trackEnd.position;
    double gapLen = glm::length(gap);
    std::fprintf(stderr, "gap=(%.4f,%.4f,%.4f) |gap|=%.4f\n\n",
        gap.x, gap.y, gap.z, gapLen);

    // --- Test 1: Zero parameters (straight connector) ---
    std::fprintf(stderr, "--- Test 1: Zero parameters (straight connector) ---\n");
    {
        std::array<double,6> p = {0,0,0,0,0,0};
        auto e = integrate(trackEnd, p, length);
        auto err = computeError(e, trackStart);
        std::fprintf(stderr, "  endpoint: pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
            e.position.x, e.position.y, e.position.z,
            e.tangent.x, e.tangent.y, e.tangent.z,
            e.up.x, e.up.y, e.up.z);
        std::fprintf(stderr, "  error: [%.4f, %.4f, %.4f, %.6f, %.6f, %.6f] rms=%.6f\n",
            err[0], err[1], err[2], err[3], err[4], err[5], rms(err));
    }

    // --- Test 2: Sweep each parameter independently ---
    std::fprintf(stderr, "\n--- Test 2: Parameter sensitivity (sweep each from -2 to +2) ---\n");
    std::array<double,6> base = {0,0,0,0,0,0};
    const char* names[] = {"pitch_s", "pitch_e", "yaw_s", "yaw_e", "roll_s", "roll_e"};
    for (int i = 0; i < 6; i++) {
        double vals[] = {-2.0, -1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 1.0, 2.0};
        std::fprintf(stderr, "\n  param[%d] %s sweep:\n", i, names[i]);
        for (double v : vals) {
            auto p = base;
            p[i] = v;
            auto e = integrate(trackEnd, p, length);
            auto err = computeError(e, trackStart);
            std::fprintf(stderr, "    %+.2f: pos=(%+.3f,%+.3f,%+.3f) tan_err=(%+.4f,%+.4f) frame_err=%+.4f rms=%.4f\n",
                v, e.position.x, e.position.y, e.position.z,
                err[3], err[4], err[5], rms(err));
        }
    }

    // --- Test 3: Compute Jacobian at multiple parameter states ---
    std::fprintf(stderr, "\n--- Test 3: Jacobian analysis ---\n");
    std::array<std::array<double,6>,6> testStates = {{
        {0,0,0,0,0,0},
        {0.1,0.1,0.1,0.1,0,0},
        {-0.1,-0.1,-0.1,-0.1,0,0},
        {0,0,0.15,0.15,0,0},
        {0,0,-0.15,-0.15,0,0},
        {0.05,0.05,0.05,0.05,0.05,0.05}
    }};

    for (auto& params : testStates) {
        auto e0 = integrate(trackEnd, params, length);
        auto err0 = computeError(e0, trackStart);

        std::fprintf(stderr, "\n  state pitch=(%.3f,%.3f) yaw=(%.3f,%.3f) roll=(%.3f,%.3f):\n",
            params[0], params[1], params[2], params[3], params[4], params[5]);
        std::fprintf(stderr, "    endpoint: pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
            e0.position.x, e0.position.y, e0.position.z,
            e0.tangent.x, e0.tangent.y, e0.tangent.z,
            e0.up.x, e0.up.y, e0.up.z);
        std::fprintf(stderr, "    error: [%.4f, %.4f, %.4f, %.6f, %.6f, %.6f] rms=%.6f\n",
            err0[0], err0[1], err0[2], err0[3], err0[4], err0[5], rms(err0));

        // Compute Jacobian numerically
        double eps = 1e-6;
        std::array<std::array<double,6>,6> J; // J[row][col]
        for (int col = 0; col < 6; col++) {
            auto p2 = params;
            p2[col] += eps;
            auto e2 = integrate(trackEnd, p2, length);
            auto err2 = computeError(e2, trackStart);
            for (int row = 0; row < 6; row++) {
                J[row][col] = (err2[row] - err0[row]) / eps;
            }
        }

        std::fprintf(stderr, "    Jacobian:\n");
        const char* labels[] = {"dx ", "dy ", "dz ", "dtx", "dty", "dfz"};
        for (int row = 0; row < 6; row++) {
            std::fprintf(stderr, "      %s: [%+.4e %+.4e %+.4e %+.4e %+.4e %+.4e]\n",
                labels[row],
                J[row][0], J[row][1], J[row][2],
                J[row][3], J[row][4], J[row][5]);
        }

        // Compute singular values via simple means
        // Check rank: compute column norms
        double colNorms[6] = {};
        for (int c = 0; c < 6; c++)
            for (int r = 0; r < 6; r++)
                colNorms[c] += J[r][c] * J[r][c];

        double maxNorm = 0, minNorm = 1e30;
        for (int c = 0; c < 6; c++) {
            colNorms[c] = std::sqrt(colNorms[c]);
            if (colNorms[c] > maxNorm) maxNorm = colNorms[c];
            if (colNorms[c] > 1e-14 && colNorms[c] < minNorm) minNorm = colNorms[c];
        }
        std::fprintf(stderr, "    col norms: [%.4e %.4e %.4e %.4e %.4e %.4e]\n",
            colNorms[0], colNorms[1], colNorms[2],
            colNorms[3], colNorms[4], colNorms[5]);
        std::fprintf(stderr, "    max/min col norm ratio: %.4e\n",
            maxNorm / (minNorm > 1e-14 ? minNorm : 1e-14));

        // Check if any row of the Jacobian is near-zero
        for (int r = 0; r < 6; r++) {
            double rowNorm = 0;
            for (int c = 0; c < 6; c++) rowNorm += J[r][c] * J[r][c];
            rowNorm = std::sqrt(rowNorm);
            if (rowNorm < 1e-10) {
                std::fprintf(stderr, "    WARNING: row %d (%s) has near-zero norm %.4e\n",
                    r, labels[r], rowNorm);
            }
        }
    }

    // --- Test 4: Can any parameter combination reach the target position? ---
    std::fprintf(stderr, "\n--- Test 4: Brute-force search for position match ---\n");
    {
        double bestRms = 1e30;
        std::array<double,6> bestParams = {};
        // Coarse grid search in yaw space (position is mainly controlled by yaw for straight track)
        for (double ys = -1.0; ys <= 1.0; ys += 0.1) {
            for (double ye = -1.0; ye <= 1.0; ye += 0.1) {
                for (double ps = -0.5; ps <= 0.5; ps += 0.1) {
                    for (double pe = -0.5; pe <= 0.5; pe += 0.1) {
                        std::array<double,6> p = {ps, pe, ys, ye, 0, 0};
                        auto e = integrate(trackEnd, p, length);
                        auto err = computeError(e, trackStart);
                        double r = rms(err);
                        if (r < bestRms) {
                            bestRms = r;
                            bestParams = p;
                        }
                    }
                }
            }
        }
        std::fprintf(stderr, "  best coarse: pitch=(%.2f,%.2f) yaw=(%.2f,%.2f) rms=%.6f\n",
            bestParams[0], bestParams[1], bestParams[2], bestParams[3], bestRms);
        auto e = integrate(trackEnd, bestParams, length);
        auto err = computeError(e, trackStart);
        std::fprintf(stderr, "  best endpoint: pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
            e.position.x, e.position.y, e.position.z,
            e.tangent.x, e.tangent.y, e.tangent.z,
            e.up.x, e.up.y, e.up.z);
        std::fprintf(stderr, "  best error: [%.4f, %.4f, %.4f, %.6f, %.6f, %.6f]\n",
            err[0], err[1], err[2], err[3], err[4], err[5]);

        // Refine around best
        for (int iter = 0; iter < 3; iter++) {
            double step = 0.05 / (iter + 1);
            auto prevBest = bestParams;
            for (int i = 0; i < 6; i++) {
                for (double dv : {-step, step}) {
                    auto p = bestParams;
                    p[i] += dv;
                    auto e2 = integrate(trackEnd, p, length);
                    auto err2 = computeError(e2, trackStart);
                    double r2 = rms(err2);
                    if (r2 < bestRms) {
                        bestRms = r2;
                        bestParams = p;
                    }
                }
            }
        }
        std::fprintf(stderr, "  refined: pitch=(%.4f,%.4f) yaw=(%.4f,%.4f) roll=(%.4f,%.4f) rms=%.6f\n",
            bestParams[0], bestParams[1], bestParams[2], bestParams[3], bestParams[4], bestParams[5], bestRms);
        auto e2 = integrate(trackEnd, bestParams, length);
        auto err2 = computeError(e2, trackStart);
        std::fprintf(stderr, "  refined endpoint: pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f) up=(%.4f,%.4f,%.4f)\n",
            e2.position.x, e2.position.y, e2.position.z,
            e2.tangent.x, e2.tangent.y, e2.tangent.z,
            e2.up.x, e2.up.y, e2.up.z);
        std::fprintf(stderr, "  refined error: [%.4f, %.4f, %.4f, %.6f, %.6f, %.6f]\n",
            err2[0], err2[1], err2[2], err2[3], err2[4], err2[5]);
    }

    // --- Test 5: Target tangent is (1,0,0). Can ANY combination of rates produce endpoint at (0,0,0) with tangent (1,0,0)? ---
    std::fprintf(stderr, "\n--- Test 5: Can connector reach target position at all? (ignore tangent/frame) ---\n");
    {
        double bestPos = 1e30;
        std::array<double,6> bestP = {};
        for (double ys = -2.0; ys <= 2.0; ys += 0.05) {
            for (double ye = -2.0; ye <= 2.0; ye += 0.05) {
                for (double ps = -1.0; ps <= 1.0; ps += 0.1) {
                    for (double pe = -1.0; pe <= 1.0; pe += 0.1) {
                        std::array<double,6> p = {ps, pe, ys, ye, 0, 0};
                        auto e = integrate(trackEnd, p, length);
                        double d = glm::length(e.position - trackStart.position);
                        if (d < bestPos) {
                            bestPos = d;
                            bestP = p;
                        }
                    }
                }
            }
        }
        std::fprintf(stderr, "  best position-only: pitch=(%.2f,%.2f) yaw=(%.2f,%.2f) dist=%.6f\n",
            bestP[0], bestP[1], bestP[2], bestP[3], bestPos);
        auto e = integrate(trackEnd, bestP, length);
        std::fprintf(stderr, "  endpoint: pos=(%.4f,%.4f,%.4f) tan=(%.4f,%.4f,%.4f)\n",
            e.position.x, e.position.y, e.position.z,
            e.tangent.x, e.tangent.y, e.tangent.z);
    }

    return 0;
}
