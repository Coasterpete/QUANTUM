#pragma once

#include <quantum/geometry/CurveGeometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace quantum::geometry
{
    struct ArcLengthSample
    {
        double parameter = 0.0;
        double distance = 0.0;

        [[nodiscard]] friend bool operator==(
            const ArcLengthSample&,
            const ArcLengthSample&
        ) = default;
    };

    struct ArcLengthLUTOptions
    {
        // Each accepted piecewise-linear segment is checked at its quarter,
        // midpoint, and three-quarter parameters. At those checkpoints, the
        // difference between the interpolated distance and authoritative
        // arc length must not exceed this absolute-plus-relative tolerance.
        // Distances use the same coordinate units as the curve.
        double absoluteTolerance = 1.0e-6;
        double relativeTolerance = 1.0e-6;

        // This depth applies independently to each distinct knot span. If a
        // segment still fails its checkpoint tests at this depth, construction
        // throws rather than returning a table outside the stated contract.
        std::size_t maximumSubdivisionDepth = 20;

        // The direct arc-length evaluator remains authoritative during LUT
        // construction. Its accuracy and work limit are controlled here and
        // are intentionally independent of the LUT interpolation tolerance.
        ArcLengthOptions arcLength{};
    };

    namespace detail
    {
        inline void validateArcLengthLUTOptions(
            const ArcLengthLUTOptions& options
        )
        {
            if (!std::isfinite(options.absoluteTolerance)
                || options.absoluteTolerance < 0.0
                || !std::isfinite(options.relativeTolerance)
                || options.relativeTolerance < 0.0
                || options.relativeTolerance >= 1.0
                || (options.absoluteTolerance == 0.0
                    && options.relativeTolerance == 0.0))
            {
                throw std::invalid_argument(
                    "Arc-length LUT tolerances must be finite and non-negative, relative tolerance must be less than one, and at least one tolerance must be positive."
                );
            }
        }

        template<typename Curve>
        class ArcLengthLUTBuilder
        {
        public:
            ArcLengthLUTBuilder(
                const Curve& curve,
                const double parameterBegin,
                const double parameterEnd,
                const ArcLengthLUTOptions& options
            )
                : curve_(curve),
                  parameterBegin_(parameterBegin),
                  parameterEnd_(parameterEnd),
                  options_(options)
            {
            }

            [[nodiscard]] std::vector<ArcLengthSample> build()
            {
                validateArcLengthLUTOptions(options_);

                totalLength_ = evaluateArcLength(
                    curve_,
                    parameterBegin_,
                    parameterEnd_,
                    options_.arcLength
                );

                distanceTolerance_ =
                    options_.absoluteTolerance
                    + options_.relativeTolerance * totalLength_;

                if (!std::isfinite(distanceTolerance_))
                {
                    distanceTolerance_ =
                        std::numeric_limits<double>::max();
                }

                samples_.push_back({parameterBegin_, 0.0});

                if (parameterBegin_ == parameterEnd_)
                {
                    return std::move(samples_);
                }

                std::vector<double> boundaries{parameterBegin_};

                for (const double knot : curve_.knots())
                {
                    if (knot > parameterBegin_
                        && knot < parameterEnd_
                        && knot != boundaries.back())
                    {
                        boundaries.push_back(knot);
                    }
                }

                boundaries.push_back(parameterEnd_);

                if (totalLength_ == 0.0)
                {
                    for (std::size_t index = 1;
                         index < boundaries.size();
                         ++index)
                    {
                        samples_.push_back({boundaries[index], 0.0});
                    }

                    return std::move(samples_);
                }

                for (std::size_t index = 1;
                     index < boundaries.size();
                     ++index)
                {
                    const double parameter = boundaries[index];
                    const double distance =
                        parameter == parameterEnd_
                        ? totalLength_
                        : intervalLength(parameterBegin_, parameter);
                    const double normalizedDistance = normalizeDistance(
                        distance,
                        samples_.back().distance,
                        totalLength_
                    );

                    refine(
                        samples_.back().parameter,
                        samples_.back().distance,
                        parameter,
                        normalizedDistance,
                        0
                    );
                }

                return std::move(samples_);
            }

        private:
            [[nodiscard]] double intervalLength(
                const double parameterBegin,
                const double parameterEnd
            ) const
            {
                return evaluateArcLength(
                    curve_,
                    parameterBegin,
                    parameterEnd,
                    options_.arcLength
                );
            }

            [[nodiscard]] double normalizeDistance(
                const double distance,
                const double lowerDistance,
                const double upperDistance
            ) const
            {
                if (!std::isfinite(distance))
                {
                    throw std::domain_error(
                        "Arc-length LUT construction produced a non-finite cumulative distance."
                    );
                }

                const double roundoffAllowance =
                    64.0
                    * std::numeric_limits<double>::epsilon()
                    * std::max(1.0, totalLength_);

                if (distance < lowerDistance)
                {
                    if (lowerDistance - distance <= roundoffAllowance)
                    {
                        return lowerDistance;
                    }

                    throw std::domain_error(
                        "Arc-length LUT construction produced a materially decreasing cumulative distance."
                    );
                }

                if (distance > upperDistance)
                {
                    if (distance - upperDistance <= roundoffAllowance)
                    {
                        return upperDistance;
                    }

                    throw std::domain_error(
                        "Arc-length LUT construction produced a cumulative distance outside its enclosing interval."
                    );
                }

                return distance;
            }

            void refine(
                const double parameterBegin,
                const double distanceBegin,
                const double parameterEnd,
                const double distanceEnd,
                const std::size_t subdivisionDepth
            )
            {
                const double parameterMiddle =
                    std::midpoint(parameterBegin, parameterEnd);
                const std::array<double, 3> checkpointParameters{
                    std::midpoint(parameterBegin, parameterMiddle),
                    parameterMiddle,
                    std::midpoint(parameterMiddle, parameterEnd)
                };
                constexpr std::array<double, 3> checkpointFractions{
                    0.25,
                    0.5,
                    0.75
                };

                std::array<double, 3> checkpointDistances{
                    normalizeDistance(
                        distanceBegin
                            + intervalLength(
                                parameterBegin,
                                checkpointParameters[0]
                            ),
                        distanceBegin,
                        distanceEnd
                    ),
                    normalizeDistance(
                        distanceBegin
                            + intervalLength(
                                parameterBegin,
                                checkpointParameters[1]
                            ),
                        distanceBegin,
                        distanceEnd
                    ),
                    normalizeDistance(
                        distanceEnd
                            - intervalLength(
                                checkpointParameters[2],
                                parameterEnd
                            ),
                        distanceBegin,
                        distanceEnd
                    )
                };
                double maximumError = 0.0;

                for (std::size_t index = 0;
                     index < checkpointParameters.size();
                     ++index)
                {
                    const double interpolatedDistance = std::lerp(
                        distanceBegin,
                        distanceEnd,
                        checkpointFractions[index]
                    );
                    maximumError = std::max(
                        maximumError,
                        std::abs(
                            checkpointDistances[index]
                            - interpolatedDistance
                        )
                    );
                }

                if (maximumError <= distanceTolerance_)
                {
                    samples_.push_back({parameterEnd, distanceEnd});
                    return;
                }

                if (subdivisionDepth
                    >= options_.maximumSubdivisionDepth)
                {
                    throw std::runtime_error(
                        "Arc-length LUT construction could not meet the requested distance tolerance within the maximum subdivision depth."
                    );
                }

                if (parameterMiddle == parameterBegin
                    || parameterMiddle == parameterEnd)
                {
                    throw std::runtime_error(
                        "Arc-length LUT construction exhausted the representable parameter interval before meeting the requested distance tolerance."
                    );
                }

                const double distanceMiddle = checkpointDistances[1];

                refine(
                    parameterBegin,
                    distanceBegin,
                    parameterMiddle,
                    distanceMiddle,
                    subdivisionDepth + 1
                );
                refine(
                    parameterMiddle,
                    distanceMiddle,
                    parameterEnd,
                    distanceEnd,
                    subdivisionDepth + 1
                );
            }

            const Curve& curve_;
            double parameterBegin_ = 0.0;
            double parameterEnd_ = 0.0;
            const ArcLengthLUTOptions& options_;
            double totalLength_ = 0.0;
            double distanceTolerance_ = 0.0;
            std::vector<ArcLengthSample> samples_;
        };
    }

    // ArcLengthLUT is an approximation/acceleration structure. It does not
    // replace evaluateArcLength() or evaluateParameterAtArcLength(), which
    // remain the authoritative mathematical path. Construction adaptively
    // samples the direct arc-length mapping, while ordinary queries use only
    // binary search and piecewise-linear interpolation over stored samples.
    class ArcLengthLUT
    {
    public:
        // Curve must provide parameterDomain(), knots(), and the analytic
        // evaluateFirstDerivative() required by evaluateArcLength(). Every
        // distinct interior knot in the requested interval is an explicit
        // initial subdivision boundary and therefore a stored LUT sample.
        template<typename Curve>
        [[nodiscard]] static ArcLengthLUT build(
            const Curve& curve,
            const double parameterBegin,
            const double parameterEnd,
            const ArcLengthLUTOptions& options = {}
        )
        {
            return ArcLengthLUT(
                detail::ArcLengthLUTBuilder<Curve>(
                    curve,
                    parameterBegin,
                    parameterEnd,
                    options
                ).build()
            );
        }

        [[nodiscard]] double totalLength() const noexcept;
        [[nodiscard]] const std::vector<ArcLengthSample>& samples()
            const noexcept;

        // Queries accept the inclusive LUT interval. Non-finite and
        // out-of-range inputs are rejected; exact endpoints return the exact
        // stored endpoint values.
        [[nodiscard]] double distanceAtParameter(double parameter) const;
        [[nodiscard]] double parameterAtDistance(double distance) const;

    private:
        explicit ArcLengthLUT(std::vector<ArcLengthSample> samples);

        std::vector<ArcLengthSample> samples_;
    };
}
