#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace quantum::editor
{
    // One editable scalar profile row of the geometry editor graph. The
    // three rows are named after the rider-local axis they control and are
    // also the stable tie-break order used when two graph curves are equally
    // close. Which authored ChannelProfile a row addresses depends on the
    // selected region's authoring model, so the row alone never implies a
    // unit: see ProfileRowStyle.
    //
    //  - A rate-profile region authors an angular rate on that axis.
    //  - A force-driven region authors the rider-force target on the same
    //    axis (normal G, lateral G) or the authored roll rate.
    enum class ProfileChannel
    {
        Roll,
        Pitch,
        Yaw
    };

    inline constexpr std::size_t profileChannelCount = 3;

    enum class ScalarProfileEndpoint
    {
        None,
        Begin,
        End
    };

    // Resolves one graph row to the authored ChannelProfile of the section.
    // Throws std::logic_error when the section's authoring model does not
    // define that row, which keeps every edit routed through one place.
    [[nodiscard]] coaster::ChannelProfile& sectionProfileChannel(
        coaster::AuthoredTrackSection& section,
        ProfileChannel channel
    );

    [[nodiscard]] const coaster::ChannelProfile& sectionProfileChannel(
        const coaster::AuthoredTrackSection& section,
        ProfileChannel channel
    );

    // Which derived read-out a row's value supports. Pitch/Yaw rate rows
    // relate to centerline curvature and radius, a roll row integrates to a
    // net rotation, and a force target is a rider load that is only related
    // to the resulting geometry through the document's physics.
    enum class ProfileRowDiagnostic
    {
        Curvature,
        IntegratedRotation,
        ForceTarget
    };

    // Presentation and unit policy for one editable profile row. Authored
    // Core data is never rescaled by a style: the two factors only convert an
    // authored value to the displayed unit and back. minimumGraphMagnitude
    // is in authored units and only picks a useful vertical view for a flat
    // profile; it never clamps authored data.
    struct ProfileRowStyle
    {
        const char* label = "";
        const char* valueUnitLabel = "";
        const char* beginValueLabel = "";
        const char* endValueLabel = "";
        double authoredToDisplay = 1.0;
        double displayToAuthored = 1.0;
        double minimumGraphMagnitude = 1.0;
        ProfileRowDiagnostic diagnostic = ProfileRowDiagnostic::Curvature;
    };

    // Angular rates are authored in radians per Core coordinate unit, so a
    // degrees-per-meter presentation divides by the document's
    // metersPerCoordinateUnit. Both factories therefore take that scale and
    // return a matching displayToAuthored reciprocal.
    [[nodiscard]] std::array<ProfileRowStyle, profileChannelCount>
        rateProfileRowStyles(double metersPerCoordinateUnit);

    [[nodiscard]] std::array<ProfileRowStyle, profileChannelCount>
        forceDrivenRowStyles(double metersPerCoordinateUnit);

    inline constexpr double radiansPerDegree =
        0.017453292519943295769236907684886;
    inline constexpr double degreesPerRadian =
        57.295779513082320876798154814105;

    // Unit conversions shared by all three angular-rate channels. These do
    // not imply that Roll Rate is centerline curvature.
    [[nodiscard]] double angularRateDegreesToRadians(
        double rateDegreesPerMeter
    );

    [[nodiscard]] double angularRateRadiansToDegrees(
        double rateRadiansPerMeter
    );

    // Pitch/Yaw centerline-curvature terminology retained for diagnostics.
    [[nodiscard]] double angularRateDegreesToCurvature(
        double rateDegreesPerMeter
    );

    [[nodiscard]] double curvatureToAngularRateDegrees(
        double curvaturePerMeter
    );

    // Empty radius represents a straight/effectively straight channel.
    // No infinity is introduced into authored data.
    [[nodiscard]] std::optional<double> curvatureRadiusMeters(
        double curvaturePerMeter
    );

    [[nodiscard]] std::optional<double> angularRateRadiusMeters(
        double rateDegreesPerMeter
    );

    // Radius is a magnitude. directionSign supplies the signed curvature
    // convention and must be finite and nonzero.
    [[nodiscard]] double radiusToAngularRateDegrees(
        double radiusMeters,
        double directionSign
    );

    struct CurvatureDiagnostic
    {
        double rateDegreesPerMeter = 0.0;
        double curvaturePerMeter = 0.0;
        std::optional<double> radiusMeters;
    };

    [[nodiscard]] CurvatureDiagnostic curvatureDiagnosticFromRateRadians(
        double rateRadiansPerMeter
    );

    [[nodiscard]] CurvatureDiagnostic resultantCurvatureDiagnostic(
        double pitchRateRadiansPerMeter,
        double yawRateRadiansPerMeter
    );

    struct GraphValueRange
    {
        double minimum = 0.0;
        double maximum = 0.0;

        [[nodiscard]] bool valid() const noexcept;
        [[nodiscard]] double magnitude() const noexcept;
    };

    // Useful flat-profile presentation ranges for each independently
    // transformed rate-profile row. They tune drag feel but never clamp
    // authored data. Force-driven rows carry their own magnitudes in
    // ProfileRowStyle because they are not angular rates.
    [[nodiscard]] double defaultGraphMagnitude(
        ProfileChannel channel
    ) noexcept;

    // The fallback only chooses a useful view for a flat profile. It is not
    // an authoring limit; graph drags and numeric input remain unbounded
    // apart from finite-value validation.
    [[nodiscard]] GraphValueRange fitSymmetricGraphRange(
        std::span<const double> values,
        double minimumMagnitude,
        double paddingFraction = 0.15
    );

    [[nodiscard]] GraphValueRange scaleGraphRange(
        GraphValueRange range,
        double scale
    );

    // Leaves the view unchanged while value remains visible; otherwise
    // expands a symmetric range with padding so direct edits stay on-screen.
    [[nodiscard]] GraphValueRange expandGraphRangeToInclude(
        GraphValueRange range,
        double value,
        double paddingFraction = 0.15
    );

    [[nodiscard]] double graphValueToNormalized(
        double value,
        GraphValueRange range
    );

    [[nodiscard]] double normalizedToGraphValue(
        double normalizedValue,
        GraphValueRange range
    );

    [[nodiscard]] double graphValueUnitsPerPixel(
        GraphValueRange range,
        double pixelHeight
    );

    [[nodiscard]] double graphDistanceToNormalized(
        double distance,
        double domainBegin,
        double domainEnd
    );

    // Exact zero/one positions preserve the corresponding authored domain
    // endpoint so graph sampling cannot drift outside an inclusive domain.
    [[nodiscard]] double normalizedToGraphDistance(
        double normalizedDistance,
        double domainBegin,
        double domainEnd
    );

    // One real authored boundary value in an analytic channel profile. A
    // valid N-segment profile produces N+1 markers: the first segment Begin
    // plus every segment End. An interior marker is deliberately owned by
    // the left segment's End endpoint, matching Core's shared-boundary edit
    // semantics; sampled curve vertices never appear here.
    struct SemanticProfileMarker
    {
        double distance = 0.0;
        double value = 0.0;
        coaster::SegmentId segmentId = coaster::invalidSegmentId;
        ScalarProfileEndpoint endpoint = ScalarProfileEndpoint::None;
        bool regionBoundary = false;
    };

    [[nodiscard]] std::vector<SemanticProfileMarker>
    extractSemanticProfileMarkers(const coaster::ChannelProfile& profile);

    struct ProfileBoundaryMoveBounds
    {
        double minimum = 0.0;
        double maximum = 0.0;
    };

    // Returns the neighbouring outer bounds for a movable interior marker.
    // Region start/end markers are pinned and therefore return no bounds.
    [[nodiscard]] std::optional<ProfileBoundaryMoveBounds>
    profileBoundaryMoveBounds(
        const coaster::ChannelProfile& profile,
        coaster::SegmentId segmentId,
        ScalarProfileEndpoint endpoint
    ) noexcept;

    // Deterministic engineering-space proposals shared by marker drags and
    // their tests. Screen Y grows down while authored values grow up.
    [[nodiscard]] double proposeMarkerValueDrag(
        double currentValue,
        double pixelDeltaY,
        double valueUnitsPerPixel,
        double gain,
        std::optional<double> snapIncrement = std::nullopt
    );

    [[nodiscard]] double proposeBoundaryDistanceDrag(
        double currentDistance,
        double pixelDeltaX,
        double distanceUnitsPerPixel,
        double gain,
        ProfileBoundaryMoveBounds bounds,
        std::optional<double> snapIncrement = std::nullopt
    );

    struct CurveHitCandidate
    {
        ProfileChannel channel = ProfileChannel::Roll;
        double distanceSquared = 0.0;
    };

    // Chooses within hitRadius using active channel, previous hover, nearest
    // distance, then ProfileChannel order. Render order never decides a hit.
    [[nodiscard]] std::optional<ProfileChannel> chooseCurveHit(
        std::span<const CurveHitCandidate> candidates,
        double hitRadius,
        ProfileChannel activeChannel,
        std::optional<ProfileChannel> previouslyHovered
    );

    struct GraphMarkerId
    {
        ProfileChannel channel = ProfileChannel::Roll;
        coaster::SegmentId segmentId = coaster::invalidSegmentId;
        ScalarProfileEndpoint endpoint = ScalarProfileEndpoint::None;

        [[nodiscard]] bool operator==(const GraphMarkerId&) const = default;
    };

    struct MarkerHitCandidate
    {
        GraphMarkerId marker;
        double distanceSquared = 0.0;
    };

    // Marker acquisition precedes curve acquisition. Eligible markers use
    // active channel, the exact previously hovered marker, nearest distance,
    // then stable semantic identity; draw order never affects the result.
    [[nodiscard]] std::optional<GraphMarkerId> chooseMarkerHit(
        std::span<const MarkerHitCandidate> candidates,
        double hitRadius,
        ProfileChannel activeChannel,
        std::optional<GraphMarkerId> previouslyHovered
    );

    [[nodiscard]] double squaredDistanceToLineSegment(
        double pointX,
        double pointY,
        double beginX,
        double beginY,
        double endX,
        double endY
    ) noexcept;
}
