#include <quantum/editor/TransitionEditorModel.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace quantum::editor
{
    namespace
    {
        constexpr double straightCurvatureThreshold = 1e-12;

        void requireFinite(const double value, const char* const message)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(message);
            }
        }

        [[nodiscard]] std::optional<double> candidateDistance(
            const std::span<const CurveHitCandidate> candidates,
            const RateChannel channel,
            const double maximumDistanceSquared)
        {
            std::optional<double> best;
            for (const CurveHitCandidate& candidate : candidates)
            {
                if (candidate.channel != channel
                    || !std::isfinite(candidate.distanceSquared)
                    || candidate.distanceSquared > maximumDistanceSquared)
                {
                    continue;
                }

                if (!best.has_value() || candidate.distanceSquared < *best)
                {
                    best = candidate.distanceSquared;
                }
            }
            return best;
        }
    }

    coaster::ChannelProfile& sectionRateChannel(
        coaster::AuthoredTrackSection& section,
        const RateChannel channel)
    {
        switch (channel)
        {
        case RateChannel::Roll:
            return section.rateProfileRegion().rateProfiles.roll;
        case RateChannel::Pitch:
            return section.rateProfileRegion().rateProfiles.pitch;
        case RateChannel::Yaw:
        default:
            return section.rateProfileRegion().rateProfiles.yaw;
        }
    }

    const coaster::ChannelProfile& sectionRateChannel(
        const coaster::AuthoredTrackSection& section,
        const RateChannel channel)
    {
        switch (channel)
        {
        case RateChannel::Roll:
            return section.rateProfileRegion().rateProfiles.roll;
        case RateChannel::Pitch:
            return section.rateProfileRegion().rateProfiles.pitch;
        case RateChannel::Yaw:
        default:
            return section.rateProfileRegion().rateProfiles.yaw;
        }
    }

    double angularRateDegreesToRadians(
        const double rateDegreesPerMeter)
    {
        requireFinite(
            rateDegreesPerMeter,
            "Angular rate must be finite."
        );
        return rateDegreesPerMeter * radiansPerDegree;
    }

    double angularRateRadiansToDegrees(const double rateRadiansPerMeter)
    {
        requireFinite(
            rateRadiansPerMeter,
            "Angular rate must be finite."
        );
        const double rate = rateRadiansPerMeter * degreesPerRadian;
        requireFinite(rate, "Converted angular rate must be finite.");
        return rate;
    }

    double angularRateDegreesToCurvature(
        const double rateDegreesPerMeter)
    {
        return angularRateDegreesToRadians(rateDegreesPerMeter);
    }

    double curvatureToAngularRateDegrees(const double curvaturePerMeter)
    {
        requireFinite(curvaturePerMeter, "Curvature must be finite.");
        return angularRateRadiansToDegrees(curvaturePerMeter);
    }

    std::optional<double> curvatureRadiusMeters(
        const double curvaturePerMeter)
    {
        requireFinite(curvaturePerMeter, "Curvature must be finite.");
        if (std::abs(curvaturePerMeter) <= straightCurvatureThreshold)
        {
            return std::nullopt;
        }
        return 1.0 / std::abs(curvaturePerMeter);
    }

    std::optional<double> angularRateRadiusMeters(
        const double rateDegreesPerMeter)
    {
        return curvatureRadiusMeters(
            angularRateDegreesToCurvature(rateDegreesPerMeter)
        );
    }

    double radiusToAngularRateDegrees(
        const double radiusMeters,
        const double directionSign)
    {
        if (!std::isfinite(radiusMeters) || radiusMeters <= 0.0)
        {
            throw std::invalid_argument(
                "Curvature radius must be positive and finite."
            );
        }
        if (!std::isfinite(directionSign) || directionSign == 0.0)
        {
            throw std::invalid_argument(
                "Curvature direction sign must be finite and nonzero."
            );
        }

        const double rate = std::copysign(
            degreesPerRadian / radiusMeters,
            directionSign
        );
        requireFinite(rate, "Converted angular rate must be finite.");
        return rate;
    }

    CurvatureDiagnostic curvatureDiagnosticFromRateRadians(
        const double rateRadiansPerMeter)
    {
        requireFinite(
            rateRadiansPerMeter,
            "Angular rate must be finite."
        );
        const double rateDegrees = curvatureToAngularRateDegrees(
            rateRadiansPerMeter
        );
        return {
            rateDegrees,
            rateRadiansPerMeter,
            curvatureRadiusMeters(rateRadiansPerMeter)
        };
    }

    CurvatureDiagnostic resultantCurvatureDiagnostic(
        const double pitchRateRadiansPerMeter,
        const double yawRateRadiansPerMeter)
    {
        requireFinite(
            pitchRateRadiansPerMeter,
            "Pitch angular rate must be finite."
        );
        requireFinite(
            yawRateRadiansPerMeter,
            "Yaw angular rate must be finite."
        );
        const double magnitude = std::hypot(
            pitchRateRadiansPerMeter,
            yawRateRadiansPerMeter
        );
        if (!std::isfinite(magnitude))
        {
            throw std::invalid_argument(
                "Resultant curvature must be finite."
            );
        }
        return {
            curvatureToAngularRateDegrees(magnitude),
            magnitude,
            curvatureRadiusMeters(magnitude)
        };
    }

    bool GraphValueRange::valid() const noexcept
    {
        return std::isfinite(minimum)
            && std::isfinite(maximum)
            && minimum < maximum;
    }

    double GraphValueRange::magnitude() const noexcept
    {
        return std::max(std::abs(minimum), std::abs(maximum));
    }

    double defaultGraphMagnitude(const RateChannel channel) noexcept
    {
        switch (channel)
        {
        case RateChannel::Roll:
            return 30.0 * radiansPerDegree;
        case RateChannel::Pitch:
            return 2.0 * radiansPerDegree;
        case RateChannel::Yaw:
        default:
            return 0.5 * radiansPerDegree;
        }
    }

    GraphValueRange fitSymmetricGraphRange(
        const std::span<const double> values,
        const double minimumMagnitude,
        const double paddingFraction)
    {
        if (!std::isfinite(minimumMagnitude) || minimumMagnitude <= 0.0)
        {
            throw std::invalid_argument(
                "Minimum graph magnitude must be positive and finite."
            );
        }
        if (!std::isfinite(paddingFraction) || paddingFraction < 0.0)
        {
            throw std::invalid_argument(
                "Graph padding must be finite and nonnegative."
            );
        }

        double maximumMagnitude = 0.0;
        for (const double value : values)
        {
            requireFinite(value, "Graph values must be finite.");
            maximumMagnitude = std::max(maximumMagnitude, std::abs(value));
        }

        const double paddedMagnitude = maximumMagnitude
            * (1.0 + paddingFraction);
        const double magnitude = std::max(
            minimumMagnitude,
            paddedMagnitude
        );
        if (!std::isfinite(magnitude))
        {
            throw std::invalid_argument("Graph range is not finite.");
        }
        return {-magnitude, magnitude};
    }

    GraphValueRange scaleGraphRange(
        const GraphValueRange range,
        const double scale)
    {
        if (!range.valid() || !std::isfinite(scale) || scale <= 0.0)
        {
            throw std::invalid_argument(
                "Graph range and scale must be valid and finite."
            );
        }
        const double center = (range.minimum + range.maximum) * 0.5;
        const double halfSpan = (range.maximum - range.minimum) * 0.5
            * scale;
        if (!std::isfinite(center) || !std::isfinite(halfSpan)
            || halfSpan <= 0.0)
        {
            throw std::invalid_argument("Scaled graph range is invalid.");
        }
        return {center - halfSpan, center + halfSpan};
    }

    GraphValueRange expandGraphRangeToInclude(
        const GraphValueRange range,
        const double value,
        const double paddingFraction)
    {
        requireFinite(value, "Graph value must be finite.");
        if (!range.valid() || !std::isfinite(paddingFraction)
            || paddingFraction < 0.0)
        {
            throw std::invalid_argument(
                "Graph range and padding must be valid and finite."
            );
        }
        if (value >= range.minimum && value <= range.maximum)
        {
            return range;
        }

        const double expandedMagnitude = std::max(
            range.magnitude(),
            std::abs(value) * (1.0 + paddingFraction)
        );
        if (!std::isfinite(expandedMagnitude))
        {
            throw std::invalid_argument(
                "Expanded graph range is not finite."
            );
        }
        return {-expandedMagnitude, expandedMagnitude};
    }

    double graphValueToNormalized(
        const double value,
        const GraphValueRange range)
    {
        requireFinite(value, "Graph value must be finite.");
        if (!range.valid())
        {
            throw std::invalid_argument("Graph range must be valid.");
        }
        return (value - range.minimum) / (range.maximum - range.minimum);
    }

    double normalizedToGraphValue(
        const double normalizedValue,
        const GraphValueRange range)
    {
        requireFinite(
            normalizedValue,
            "Normalized graph value must be finite."
        );
        if (!range.valid())
        {
            throw std::invalid_argument("Graph range must be valid.");
        }
        return range.minimum
            + normalizedValue * (range.maximum - range.minimum);
    }

    double graphValueUnitsPerPixel(
        const GraphValueRange range,
        const double pixelHeight)
    {
        if (!range.valid() || !std::isfinite(pixelHeight)
            || pixelHeight <= 0.0)
        {
            throw std::invalid_argument(
                "Graph range and pixel height must be valid."
            );
        }
        return (range.maximum - range.minimum) / pixelHeight;
    }

    std::optional<RateChannel> chooseCurveHit(
        const std::span<const CurveHitCandidate> candidates,
        const double hitRadius,
        const RateChannel activeChannel,
        const std::optional<RateChannel> previouslyHovered)
    {
        if (!std::isfinite(hitRadius) || hitRadius < 0.0)
        {
            throw std::invalid_argument(
                "Curve hit radius must be finite and nonnegative."
            );
        }
        const double maximumDistanceSquared = hitRadius * hitRadius;

        if (candidateDistance(
                candidates,
                activeChannel,
                maximumDistanceSquared).has_value())
        {
            return activeChannel;
        }
        if (previouslyHovered.has_value()
            && candidateDistance(
                candidates,
                *previouslyHovered,
                maximumDistanceSquared).has_value())
        {
            return previouslyHovered;
        }

        std::optional<CurveHitCandidate> best;
        for (const CurveHitCandidate& candidate : candidates)
        {
            if (!std::isfinite(candidate.distanceSquared)
                || candidate.distanceSquared > maximumDistanceSquared)
            {
                continue;
            }
            if (!best.has_value()
                || candidate.distanceSquared < best->distanceSquared
                || (candidate.distanceSquared == best->distanceSquared
                    && static_cast<std::size_t>(candidate.channel)
                        < static_cast<std::size_t>(best->channel)))
            {
                best = candidate;
            }
        }
        return best.has_value()
            ? std::optional<RateChannel>{best->channel}
            : std::nullopt;
    }

    double squaredDistanceToLineSegment(
        const double pointX,
        const double pointY,
        const double beginX,
        const double beginY,
        const double endX,
        const double endY) noexcept
    {
        const double spanX = endX - beginX;
        const double spanY = endY - beginY;
        const double spanLengthSquared = spanX * spanX + spanY * spanY;
        if (spanLengthSquared == 0.0)
        {
            const double deltaX = pointX - beginX;
            const double deltaY = pointY - beginY;
            return deltaX * deltaX + deltaY * deltaY;
        }

        const double projection = std::clamp(
            ((pointX - beginX) * spanX + (pointY - beginY) * spanY)
                / spanLengthSquared,
            0.0,
            1.0
        );
        const double nearestX = beginX + projection * spanX;
        const double nearestY = beginY + projection * spanY;
        const double deltaX = pointX - nearestX;
        const double deltaY = pointY - nearestY;
        return deltaX * deltaX + deltaY * deltaY;
    }
}
