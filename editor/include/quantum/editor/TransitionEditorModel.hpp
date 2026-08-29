#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <cstddef>
#include <optional>
#include <span>

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

    [[nodiscard]] double squaredDistanceToLineSegment(
        double pointX,
        double pointY,
        double beginX,
        double beginY,
        double endX,
        double endY
    ) noexcept;
}
