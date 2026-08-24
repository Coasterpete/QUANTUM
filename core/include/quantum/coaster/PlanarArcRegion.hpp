#pragma once

#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <vector>

namespace quantum::coaster
{
    // Geometry-driven planar circular arc authored in track-design terms and
    // measured relative to the section's entry frame (entry tangent T0,
    // lateral L0, up U0).
    //
    // The authored plane contains the entry tangent. Its unit normal, in
    // entry-frame components, is
    //
    //     n = cos(planeTilt) * U0 - sin(planeTilt) * L0,
    //
    // i.e. planeTilt rotates that normal about the entry tangent. 0 authors
    // a horizontal turn whose plane normal is the entry up direction, +pi/2
    // a vertical arch rising above the entry height for positive sweep
    // (negative pitch turns the tangent toward +up), -pi/2 the mirrored dip
    // below it.
    //
    // Centerline compilation: with kappa = sign(sweptAngle) / radius,
    //
    //     pitchRate = -kappa * sin(planeTilt)
    //     yawRate   =  kappa * cos(planeTilt)
    //
    // drive the shared RiderLocalGeometry solver with zero roll rate.
    // Constant rates rotate the frame about one world-fixed axis (kappa * n)
    // that is perpendicular to the entry tangent, so the tangent sweeps
    // exactly sweptAngle along a circle of the authored radius inside the
    // authored plane.
    //
    // Banking is deliberately NOT compiled into a constant roll rate: in
    // the simultaneous local-rate solver a nonzero roll rate tilts the
    // rotation axis toward the tangent, which bends the centerline out of
    // the authored plane. Instead bankChange is superposed analytically as
    // a rotation of lateral/up about each solved state's own tangent by
    // psi(d) = bankChange * d / length, which rolls the rider frame without
    // ever moving the centerline.
    //
    // This separation is a deliberate GeometryRegion authoring rule:
    // geometry parameters own the centerline and bank owns rider
    // orientation about it. It is not a claim that arbitrary roll can
    // always be separated from centerline integration in every QUANTUM
    // authoring model. Rate-profile authoring integrates roll
    // simultaneously with pitch/yaw, where rolled frames legitimately bend
    // the resulting path, and a rolled entry frame rotates any
    // construction's curvature plane with the rider.
    struct PlanarArcRegion
    {
        double radius = 25.0;
        double sweptAngle = 3.141592653589793;
        double planeTilt = 0.0;
        double bankChange = 0.0;
    };

    // Arc length implied by the construction: |sweptAngle| * radius.
    [[nodiscard]] double planarArcLength(const PlanarArcRegion& region);

    // Validates finite parameters, a positive radius, a nonzero sweep, and
    // agreement between the stored section length and the constructed arc
    // length within a small relative tolerance. Throws std::invalid_argument.
    void validatePlanarArcRegion(
        const PlanarArcRegion& region,
        double storedLength);

    // The compiled centerline-driving rate representation of the
    // construction. This is the defined compilation rule of the geometry
    // authoring layer, exposed so tests can verify equivalence against the
    // rate-profile solver directly; it is deliberately not part of the
    // GeometryRegion authoring API. Banking is not part of this structure;
    // it is superposed during integration.
    struct PlanarArcCompiledRates
    {
        math::ScalarTransition pitchRate;
        math::ScalarTransition yawRate;
    };

    [[nodiscard]] PlanarArcCompiledRates compilePlanarArcRates(
        const PlanarArcRegion& region,
        double length);

    // Solves the construction by compiling its centerline rates, handing
    // them to the shared RiderLocalGeometry integration with zero roll, and
    // superposing bankChange analytically about the solved tangents.
    [[nodiscard]] std::vector<RiderLocalGeometryState> integratePlanarArcRegion(
        const glm::dvec3& startingPosition,
        const geometry::CurveFrame& startingFrame,
        const PlanarArcRegion& region,
        double integrationSpacing);
}
