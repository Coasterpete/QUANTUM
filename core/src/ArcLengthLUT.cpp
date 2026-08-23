#include <quantum/geometry/ArcLengthLUT.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace quantum::geometry
{
    ArcLengthLUT::ArcLengthLUT(std::vector<ArcLengthSample> samples)
        : samples_(std::move(samples))
    {
    }

    double ArcLengthLUT::totalLength() const noexcept
    {
        return samples_.back().distance;
    }

    const std::vector<ArcLengthSample>& ArcLengthLUT::samples() const noexcept
    {
        return samples_;
    }

    double ArcLengthLUT::distanceAtParameter(const double parameter) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "Arc-length LUT query parameter must be finite."
            );
        }

        if (parameter < samples_.front().parameter
            || parameter > samples_.back().parameter)
        {
            throw std::out_of_range(
                "Arc-length LUT query parameter is outside the table interval."
            );
        }

        if (parameter == samples_.front().parameter)
        {
            return 0.0;
        }

        if (parameter == samples_.back().parameter)
        {
            return totalLength();
        }

        const auto upper = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            parameter,
            [](const ArcLengthSample& sample, const double value)
            {
                return sample.parameter < value;
            }
        );

        if (upper->parameter == parameter)
        {
            return upper->distance;
        }

        const ArcLengthSample& lower = *(upper - 1);
        const double fraction =
            (parameter - lower.parameter)
            / (upper->parameter - lower.parameter);

        return std::lerp(lower.distance, upper->distance, fraction);
    }

    double ArcLengthLUT::parameterAtDistance(const double distance) const
    {
        if (!std::isfinite(distance))
        {
            throw std::invalid_argument(
                "Arc-length LUT query distance must be finite."
            );
        }

        if (distance < 0.0 || distance > totalLength())
        {
            throw std::out_of_range(
                "Arc-length LUT query distance is outside the table interval."
            );
        }

        if (distance == 0.0)
        {
            // This also defines the non-unique inverse of a geometrically
            // zero-length interval as its beginning parameter.
            return samples_.front().parameter;
        }

        if (distance == totalLength())
        {
            return samples_.back().parameter;
        }

        const auto upper = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            distance,
            [](const ArcLengthSample& sample, const double value)
            {
                return sample.distance < value;
            }
        );

        if (upper->distance == distance)
        {
            // lower_bound selects the earliest parameter when a zero-length
            // plateau makes the inverse non-unique.
            return upper->parameter;
        }

        const ArcLengthSample& lower = *(upper - 1);
        const double fraction =
            (distance - lower.distance)
            / (upper->distance - lower.distance);

        return std::lerp(lower.parameter, upper->parameter, fraction);
    }
}
