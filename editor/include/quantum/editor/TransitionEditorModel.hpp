#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace quantum::editor
{
    // One authored rate-profile channel. The order is also the stable
    // tie-break order used when two graph curves are equally close.
    enum class RateChannel
    {
        Roll,
        Pitch,
        Yaw
    };

    inline constexpr std::size_t rateChannelCount = 3;

    enum class ScalarProfileEndpoint
    {
        None,
        Begin,
        End
    };

    [[nodiscard]] coaster::ChannelProfile& sectionRateChannel(
        coaster::AuthoredTrackSection& section,
        RateChannel channel
    );

    [[nodiscard]] const coaster::ChannelProfile& sectionRateChannel(
        const coaster::AuthoredTrackSection& section,
        RateChannel channel
    );

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
    // transformed channel. They tune drag feel but never clamp authored data.
    [[nodiscard]] double defaultGraphMagnitude(
        RateChannel channel
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
        RateChannel channel = RateChannel::Roll;
        double distanceSquared = 0.0;
    };

    // Chooses within hitRadius using active channel, previous hover, nearest
    // distance, then RateChannel order. Render order never decides a hit.
    [[nodiscard]] std::optional<RateChannel> chooseCurveHit(
        std::span<const CurveHitCandidate> candidates,
        double hitRadius,
        RateChannel activeChannel,
        std::optional<RateChannel> previouslyHovered
    );

    struct GraphMarkerId
    {
        RateChannel channel = RateChannel::Roll;
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
        RateChannel activeChannel,
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
