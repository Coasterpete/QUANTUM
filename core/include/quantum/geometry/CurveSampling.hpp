#pragma once

#include <quantum/geometry/ArcLengthLUT.hpp>

#include <glm/vec3.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace quantum::geometry
{
    struct CurveSample
    {
        double distance = 0.0;
        double parameter = 0.0;
        glm::dvec3 position{0.0};

        [[nodiscard]] friend bool operator==(
            const CurveSample&,
            const CurveSample&
        ) = default;
    };

    // Canonical sampling is intentionally bounded. Requests that would need
    // more samples fail before the output vector is allocated.
    inline constexpr std::size_t maximumCurveSampleCount = 1'000'000;

    // Samples an inclusive curve interval at independently computed targets
    // i * spacing in geometric distance space. A target is an interior sample
    // only when its representable double value is strictly less than the LUT's
    // total length; equality belongs to the exact endpoint. This avoids both
    // accumulated addition drift and a coordinate-unit-specific epsilon.
    //
    // One ArcLengthLUT is built for the operation. Interior parameters use its
    // fast inverse, while the beginning and endpoint parameters are supplied
    // directly. Zero-width and geometrically zero-length intervals return only
    // the beginning sample.
    template<typename Curve>
    [[nodiscard]] std::vector<CurveSample> sampleCurveByArcLength(
        const Curve& curve,
        const double parameterBegin,
        const double parameterEnd,
        const double spacing,
        const ArcLengthLUTOptions& lutOptions = {}
    )
    {
        if (!std::isfinite(spacing) || spacing <= 0.0)
        {
            throw std::invalid_argument(
                "Curve sample spacing must be finite and greater than zero."
            );
        }

        const ArcLengthLUT lut = ArcLengthLUT::build(
            curve,
            parameterBegin,
            parameterEnd,
            lutOptions
        );
        const double totalLength = lut.totalLength();

        if (!std::isfinite(totalLength) || totalLength < 0.0)
        {
            throw std::domain_error(
                "Curve sampling received an invalid total length from its arc-length LUT."
            );
        }

        const glm::dvec3 beginningPosition = curve.evaluate(parameterBegin);

        if (!std::isfinite(beginningPosition.x)
            || !std::isfinite(beginningPosition.y)
            || !std::isfinite(beginningPosition.z))
        {
            throw std::domain_error(
                "Curve sampling produced a non-finite beginning position."
            );
        }

        if (totalLength == 0.0)
        {
            return {{0.0, parameterBegin, beginningPosition}};
        }

        // If this target is still interior, the beginning, all targets through
        // this index, and the endpoint would exceed the supported count.
        constexpr std::size_t maximumInteriorIndex =
            maximumCurveSampleCount - 1;
        if (static_cast<double>(maximumInteriorIndex) * spacing < totalLength)
        {
            throw std::length_error(
                "Curve sampling request exceeds the maximum supported sample count."
            );
        }

        std::size_t interiorSampleCount = 0;

        for (std::size_t index = 1;
             index < maximumCurveSampleCount;
             ++index)
        {
            const double targetDistance =
                static_cast<double>(index) * spacing;

            if (!(targetDistance < totalLength))
            {
                break;
            }

            ++interiorSampleCount;
        }

        if (interiorSampleCount > maximumCurveSampleCount - 2)
        {
            throw std::length_error(
                "Curve sampling request exceeds the maximum supported sample count."
            );
        }

        std::vector<CurveSample> samples;
        samples.reserve(interiorSampleCount + 2);
        samples.push_back({0.0, parameterBegin, beginningPosition});

        for (std::size_t index = 1;
             index <= interiorSampleCount;
             ++index)
        {
            const double targetDistance =
                static_cast<double>(index) * spacing;

            if (!std::isfinite(targetDistance)
                || !(targetDistance > samples.back().distance)
                || !(targetDistance < totalLength))
            {
                throw std::domain_error(
                    "Curve sampling could not produce a strictly increasing interior distance sequence."
                );
            }

            const double parameter = lut.parameterAtDistance(targetDistance);

            if (!(parameter > samples.back().parameter)
                || !(parameter < parameterEnd))
            {
                throw std::domain_error(
                    "Curve sampling could not produce a strictly increasing interior parameter sequence."
                );
            }

            const glm::dvec3 position = curve.evaluate(parameter);

            if (!std::isfinite(position.x)
                || !std::isfinite(position.y)
                || !std::isfinite(position.z))
            {
                throw std::domain_error(
                    "Curve sampling produced a non-finite interior position."
                );
            }

            samples.push_back({targetDistance, parameter, position});
        }

        const glm::dvec3 endpointPosition = curve.evaluate(parameterEnd);

        if (!std::isfinite(endpointPosition.x)
            || !std::isfinite(endpointPosition.y)
            || !std::isfinite(endpointPosition.z))
        {
            throw std::domain_error(
                "Curve sampling produced a non-finite endpoint position."
            );
        }

        samples.push_back({totalLength, parameterEnd, endpointPosition});
        return samples;
    }
}
