#pragma once

#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <glm/vec3.hpp>

#include <vector>

namespace quantum::coaster
{
    // One centerline state produced by the rider-local geometry probe.
    // Distance and position use the project's coordinate units.
    struct RiderLocalGeometryState
    {
        double distance;
        glm::dvec3 position;
        geometry::CurveFrame frame;
    };

    // Advances a position and frame through a distance-domain section with a
    // constant rider-local pitch rate and zero yaw. localPitchRate is measured
    // in radians per coordinate unit; it is deliberately separate from the
    // currently unspecified GeometricSection pitch-channel semantics.
    //
    // Positive pitch follows applyLocalPitch(): the tangent turns toward the
    // frame's current -up direction about its current lateral axis. A rolled
    // starting frame therefore rotates the resulting curvature plane with the
    // rider. integrationSpacing is the maximum distance between returned
    // states and must be positive and finite.
    //
    // Constant-rate frame rotation and the rotating tangent are integrated
    // analytically over every spacing interval. The returned vector always
    // contains the starting state; zero section length returns only that state.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateConstantLocalPitchRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        double sectionLength,
        double localPitchRate,
        double integrationSpacing
    );

    // Advances a position and frame through a distance-domain section with a
    // constant rider-local yaw rate and zero pitch. localYawRate is measured
    // in radians per coordinate unit and rotates about the frame's current up
    // axis according to applyLocalYaw().
    //
    // Constant-rate frame rotation and the rotating tangent are integrated
    // analytically over every spacing interval. integrationSpacing controls
    // the returned centerline resolution and must be positive and finite.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateConstantLocalYawRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        double sectionLength,
        double localYawRate,
        double integrationSpacing
    );

    // Advances a position and frame through a distance-domain section with a
    // constant rider-local roll rate and zero pitch/yaw. localRollRate is
    // measured in radians per coordinate unit and follows applyRoll().
    //
    // Roll rotates the lateral/up axes about the supplied tangent without
    // changing that tangent. The centerline is therefore evaluated exactly as
    // startingPosition + distance * startingFrame.tangent; spacing controls
    // only the returned sample density.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateConstantLocalRollRate(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        double sectionLength,
        double localRollRate,
        double integrationSpacing
    );

    // Integrates an authored rider-local pitch-rate profile over the
    // ScalarTransition distance domain. The transition values are pitch rate
    // in radians per coordinate unit; they are not absolute pitch angles.
    // The transition domain is an authored absolute coordinate, while every
    // returned state's distance is distance traveled from domainBegin and
    // therefore ranges from zero to domainEnd - domainBegin.
    //
    // Frame orientation uses the analytic accumulated transition area.
    // Position uses deterministic high-order quadrature of the continuously
    // rotating tangent, independent of the requested output sample count.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateLocalPitchRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& pitchRateTransition,
        double integrationSpacing
    );

    // Integrates an authored rider-local yaw-rate profile over the
    // ScalarTransition distance domain. Transition values are yaw rate in
    // radians per coordinate unit, not absolute yaw angles. Returned distance
    // is traveled distance from domainBegin and therefore starts at zero.
    //
    // Frame orientation uses the analytic accumulated transition area.
    // Position uses the same deterministic high-order tangent quadrature as
    // variable local pitch while the rider-local up axis remains unchanged.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateLocalYawRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& yawRateTransition,
        double integrationSpacing
    );

    // Integrates an authored rider-local roll-rate profile over the
    // ScalarTransition distance domain. Transition values are roll rate in
    // radians per coordinate unit, not absolute roll angles. Returned
    // distance is traveled distance from domainBegin and starts at zero.
    //
    // The accumulated roll is the analytic ScalarTransition integral. At
    // every state the tangent is the supplied starting tangent and position
    // uses the exact straight-line solution; no frame or position quadrature
    // is required.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateLocalRollRateProfile(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& rollRateTransition,
        double integrationSpacing
    );

    // Simultaneously integrates authored rider-local pitch- and yaw-rate
    // profiles over one shared ScalarTransition distance domain. Both
    // transition values are angular rates in radians per coordinate unit.
    // Returned distance is traveled distance from the shared domain beginning.
    //
    // Pitch and yaw evolve one frame continuously; they are not accumulated as
    // independent Euler angles. Frame updates use a fourth-order Lie-group
    // method, and position uses matching high-order tangent quadrature with
    // private refinement independent of the requested output sample density.
    // Roll rate is deliberately excluded. For the future full system, the
    // established applyLocalPitch/applyLocalYaw/applyRoll signs imply
    //
    //     T' =  y L - p U
    //     L' = -y T + r U
    //     U' =  p T - r L
    //
    // and local angular-rate vector omega = (r, p, y) for F = [T L U]
    // under the current right-acting matrix/skew convention.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateLocalPitchYawRateProfiles(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        double integrationSpacing
    );

    // Simultaneously integrates authored rider-local roll-, pitch-, and
    // yaw-rate profiles over one exactly shared ScalarTransition distance
    // domain. All transition values are angular rates in radians per
    // coordinate unit, with local angular-rate vector omega = (r, p, y).
    // Returned distance is traveled distance from the common domain beginning.
    //
    // The frame evolves on SO(3) according to
    //
    //     T' =  y L - p U
    //     L' = -y T + r U
    //     U' =  p T - r L
    //
    // using simultaneous local rates rather than a fixed Euler-angle order.
    // Position satisfies P' = T. Private solver refinement is independent of
    // integrationSpacing, which controls only returned sample density.
    [[nodiscard]] std::vector<RiderLocalGeometryState>
    integrateLocalRollPitchYawRateProfiles(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const math::ScalarTransition& rollRateTransition,
        const math::ScalarTransition& pitchRateTransition,
        const math::ScalarTransition& yawRateTransition,
        double integrationSpacing
    );
}
