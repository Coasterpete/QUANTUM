#pragma once

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace quantum::geometry
{
    struct ArcLengthOptions
    {
        // The adaptive error estimate for the complete requested interval must
        // not exceed absoluteTolerance + relativeTolerance times the initial
        // quadrature estimate of |L|. The defaults target nanometre-scale
        // absolute error when coordinates are metres while retaining a
        // scale-relative policy for other units.
        double absoluteTolerance = 1.0e-9;
        double relativeTolerance = 1.0e-9;

        // This depth applies independently to each distinct knot span in the
        // requested interval. Work is bounded because every subdivision
        // bisects an interval and depth never exceeds this value.
        std::size_t maximumSubdivisionDepth = 20;
    };

    struct ArcLengthInversionOptions
    {
        ArcLengthOptions arcLength{};

        // Bisection evaluates arc length at most once per iteration. Failure
        // to meet the distance residual before this limit throws rather than
        // returning an unqualified parameter.
        std::size_t maximumIterations = 64;
    };

    namespace detail
    {
        struct ArcLengthQuadratureEstimate
        {
            double integral = 0.0;
            double error = 0.0;
        };

        template<typename Curve>
        [[nodiscard]] double evaluateCurveSpeed(
            const Curve& curve,
            const double parameter
        )
        {
            const auto firstDerivative =
                curve.evaluateFirstDerivative(parameter);
            const double speed = std::hypot(
                firstDerivative.x,
                firstDerivative.y,
                firstDerivative.z
            );

            if (!std::isfinite(speed))
            {
                throw std::domain_error(
                    "Curve arc-length evaluation produced a non-finite first-derivative magnitude."
                );
            }

            return speed;
        }

        // Embedded seven-point Gauss and fifteen-point Kronrod quadrature.
        // All abscissae are strictly inside the interval, so a knot used as an
        // interval boundary is never sampled and no arbitrary offset is
        // needed to choose a derivative side.
        template<typename Curve>
        [[nodiscard]] ArcLengthQuadratureEstimate estimateArcLength(
            const Curve& curve,
            const double parameterBegin,
            const double parameterEnd
        )
        {
            constexpr std::array<double, 8> kronrodAbscissae{
                0.991455371120812639206854697526329,
                0.949107912342758524526189684047851,
                0.864864423359769072789712788640926,
                0.741531185599394439863864773280788,
                0.586087235467691130294144838258730,
                0.405845151377397166906606412076961,
                0.207784955007898467600689403773245,
                0.0
            };
            constexpr std::array<double, 8> kronrodWeights{
                0.022935322010529224963732008058970,
                0.063092092629978553290700663189204,
                0.104790010322250183839876322541518,
                0.140653259715525918745189590510238,
                0.169004726639267902826583426598550,
                0.190350578064785409913256402421014,
                0.204432940075298892414161999234649,
                0.209482141084727828012999174891714
            };
            constexpr std::array<double, 4> gaussWeights{
                0.129484966168869693270611432679082,
                0.279705391489276667901467771423780,
                0.381830050505118944950369775488975,
                0.417959183673469387755102040816327
            };

            const double center =
                std::midpoint(parameterBegin, parameterEnd);
            const double halfLength =
                0.5 * parameterEnd - 0.5 * parameterBegin;

            if (!std::isfinite(halfLength) || halfLength <= 0.0)
            {
                throw std::domain_error(
                    "Curve arc-length integration interval cannot be represented safely."
                );
            }

            const double centerSpeed = evaluateCurveSpeed(curve, center);
            double kronrodSum = kronrodWeights.back() * centerSpeed;
            double gaussSum = gaussWeights.back() * centerSpeed;

            for (std::size_t index = 0; index + 1 < kronrodAbscissae.size(); ++index)
            {
                const double offset =
                    halfLength * kronrodAbscissae[index];
                const double pairedSpeed =
                    evaluateCurveSpeed(curve, center - offset)
                    + evaluateCurveSpeed(curve, center + offset);

                kronrodSum += kronrodWeights[index] * pairedSpeed;

                if (index % 2 == 1)
                {
                    gaussSum += gaussWeights[index / 2] * pairedSpeed;
                }
            }

            const double kronrodIntegral = halfLength * kronrodSum;
            const double gaussIntegral = halfLength * gaussSum;
            const double embeddedError =
                std::abs(kronrodIntegral - gaussIntegral);

            // Do not characterize accuracy below the practical roundoff floor
            // used by established adaptive Gauss-Kronrod implementations.
            const double roundoffFloor =
                50.0
                * std::numeric_limits<double>::epsilon()
                * std::abs(kronrodIntegral);
            const double error = std::max(embeddedError, roundoffFloor);

            if (!std::isfinite(kronrodIntegral)
                || !std::isfinite(gaussIntegral)
                || !std::isfinite(error)
                || kronrodIntegral < 0.0)
            {
                throw std::domain_error(
                    "Curve arc-length quadrature produced a non-finite or invalid result."
                );
            }

            return {kronrodIntegral, error};
        }

        template<typename Curve>
        [[nodiscard]] double integrateArcLengthInterval(
            const Curve& curve,
            const double parameterBegin,
            const double parameterEnd,
            const ArcLengthQuadratureEstimate estimate,
            const double errorTolerance,
            const std::size_t subdivisionDepth,
            const std::size_t maximumSubdivisionDepth
        )
        {
            if (estimate.error <= errorTolerance)
            {
                return estimate.integral;
            }

            if (subdivisionDepth >= maximumSubdivisionDepth)
            {
                throw std::runtime_error(
                    "Curve arc-length integration could not meet the requested tolerance within the maximum subdivision depth."
                );
            }

            const double middle =
                std::midpoint(parameterBegin, parameterEnd);
            const ArcLengthQuadratureEstimate leftEstimate =
                estimateArcLength(curve, parameterBegin, middle);
            const ArcLengthQuadratureEstimate rightEstimate =
                estimateArcLength(curve, middle, parameterEnd);
            const double childTolerance = 0.5 * errorTolerance;

            const double leftLength = integrateArcLengthInterval(
                curve,
                parameterBegin,
                middle,
                leftEstimate,
                childTolerance,
                subdivisionDepth + 1,
                maximumSubdivisionDepth
            );
            const double rightLength = integrateArcLengthInterval(
                curve,
                middle,
                parameterEnd,
                rightEstimate,
                childTolerance,
                subdivisionDepth + 1,
                maximumSubdivisionDepth
            );
            const double length = leftLength + rightLength;

            if (!std::isfinite(length))
            {
                throw std::domain_error(
                    "Curve arc-length integration produced a non-finite accumulated length."
                );
            }

            return length;
        }
    }

    // Numerically integrates |C'(u)| over a valid inclusive parameter
    // interval. The curve must provide parameterDomain(), knots(), and an
    // analytic evaluateFirstDerivative(). Distinct interior knots split the
    // request into separate quadrature intervals so reduced continuity does
    // not cross an adaptive integration interval.
    //
    // The embedded Gauss-Kronrod difference supplies the local error estimate.
    // Its global budget is absoluteTolerance + relativeTolerance times an
    // initial length estimate and is divided across knot spans and recursive
    // subdivisions. Failure to satisfy that budget within the configured
    // depth throws std::runtime_error rather than returning an unqualified
    // result.
    template<typename Curve>
    [[nodiscard]] double evaluateArcLength(
        const Curve& curve,
        const double parameterBegin,
        const double parameterEnd,
        const ArcLengthOptions& options = {}
    )
    {
        if (!std::isfinite(parameterBegin)
            || !std::isfinite(parameterEnd))
        {
            throw std::invalid_argument(
                "Curve arc-length interval parameters must be finite."
            );
        }

        if (parameterBegin > parameterEnd)
        {
            throw std::invalid_argument(
                "Curve arc-length interval must not be reversed."
            );
        }

        const auto [domainBegin, domainEnd] = curve.parameterDomain();

        if (parameterBegin < domainBegin || parameterEnd > domainEnd)
        {
            throw std::out_of_range(
                "Curve arc-length interval is outside the parameter domain."
            );
        }

        if (!std::isfinite(options.absoluteTolerance)
            || options.absoluteTolerance < 0.0
            || !std::isfinite(options.relativeTolerance)
            || options.relativeTolerance < 0.0
            || options.relativeTolerance >= 1.0
            || (options.absoluteTolerance == 0.0
                && options.relativeTolerance == 0.0))
        {
            throw std::invalid_argument(
                "Arc-length tolerances must be finite and non-negative, relative tolerance must be less than one, and at least one tolerance must be positive."
            );
        }

        if (parameterBegin == parameterEnd)
        {
            return 0.0;
        }

        std::vector<double> boundaries{parameterBegin};

        for (const double knot : curve.knots())
        {
            if (knot > parameterBegin
                && knot < parameterEnd
                && knot != boundaries.back())
            {
                boundaries.push_back(knot);
            }
        }

        boundaries.push_back(parameterEnd);

        std::vector<detail::ArcLengthQuadratureEstimate> estimates;
        estimates.reserve(boundaries.size() - 1);
        double initialLength = 0.0;

        for (std::size_t index = 1; index < boundaries.size(); ++index)
        {
            const detail::ArcLengthQuadratureEstimate estimate =
                detail::estimateArcLength(
                    curve,
                    boundaries[index - 1],
                    boundaries[index]
                );
            estimates.push_back(estimate);
            initialLength += estimate.integral;

            if (!std::isfinite(initialLength))
            {
                throw std::domain_error(
                    "Curve arc-length initial estimate is not representable as a finite double."
                );
            }
        }

        double totalErrorTolerance =
            options.absoluteTolerance
            + options.relativeTolerance * initialLength;

        if (!std::isfinite(totalErrorTolerance))
        {
            totalErrorTolerance = std::numeric_limits<double>::max();
        }

        const double intervalErrorTolerance =
            totalErrorTolerance / static_cast<double>(estimates.size());
        double length = 0.0;

        for (std::size_t index = 0; index < estimates.size(); ++index)
        {
            length += detail::integrateArcLengthInterval(
                curve,
                boundaries[index],
                boundaries[index + 1],
                estimates[index],
                intervalErrorTolerance,
                0,
                options.maximumSubdivisionDepth
            );

            if (!std::isfinite(length))
            {
                throw std::domain_error(
                    "Curve arc-length result is not representable as a finite double."
                );
            }
        }

        return length;
    }

    // Inverts cumulative geometric distance S(u) = L(parameterBegin, u) on a
    // valid inclusive interval. Bisection is used because it preserves the
    // natural parameter bracket and remains valid at isolated stationary
    // points where |C'(u)| is zero. Each residual evaluation reuses
    // evaluateArcLength(), including its knot splitting and accuracy policy.
    //
    // The authoritative stopping test is
    //
    //   |L(parameterBegin, u) - targetArcLength|
    //       <= absoluteTolerance + relativeTolerance * L(parameterBegin,
    //                                                        parameterEnd).
    //
    // If no distinct floating-point midpoint remains, the nearer bracket
    // endpoint is accepted only when it meets the same distance test. Work is
    // bounded by maximumIterations; otherwise std::runtime_error is thrown.
    // A target slightly above the numerically evaluated total is treated as
    // the end only when the excess is within this same distance tolerance.
    // Larger out-of-range targets are rejected rather than clamped.
    //
    // A zero-width interval returns parameterBegin only for a zero target. A
    // geometrically zero-length interval uses the same deterministic policy,
    // because its inverse is not unique. Any positive target is rejected.
    template<typename Curve>
    [[nodiscard]] double evaluateParameterAtArcLength(
        const Curve& curve,
        const double parameterBegin,
        const double parameterEnd,
        const double targetArcLength,
        const ArcLengthInversionOptions& options = {}
    )
    {
        if (!std::isfinite(targetArcLength))
        {
            throw std::invalid_argument(
                "Target arc length must be finite."
            );
        }

        if (targetArcLength < 0.0)
        {
            throw std::out_of_range(
                "Target arc length must not be negative."
            );
        }

        const double totalLength = evaluateArcLength(
            curve,
            parameterBegin,
            parameterEnd,
            options.arcLength
        );

        if (totalLength == 0.0)
        {
            if (targetArcLength == 0.0)
            {
                return parameterBegin;
            }

            throw std::out_of_range(
                "A zero-length curve interval has no positive target arc length."
            );
        }

        if (targetArcLength == 0.0)
        {
            return parameterBegin;
        }

        double distanceTolerance =
            options.arcLength.absoluteTolerance
            + options.arcLength.relativeTolerance * totalLength;

        if (!std::isfinite(distanceTolerance))
        {
            distanceTolerance = std::numeric_limits<double>::max();
        }

        if (targetArcLength == totalLength)
        {
            return parameterEnd;
        }

        if (targetArcLength > totalLength)
        {
            if (targetArcLength - totalLength <= distanceTolerance)
            {
                return parameterEnd;
            }

            throw std::out_of_range(
                "Target arc length exceeds the curve interval length."
            );
        }

        double lowerParameter = parameterBegin;
        double upperParameter = parameterEnd;
        double lowerLength = 0.0;
        double upperLength = totalLength;

        for (std::size_t iteration = 0;
             iteration < options.maximumIterations;
             ++iteration)
        {
            const double middleParameter =
                std::midpoint(lowerParameter, upperParameter);

            if (middleParameter == lowerParameter
                || middleParameter == upperParameter)
            {
                const double lowerResidual =
                    std::abs(lowerLength - targetArcLength);
                const double upperResidual =
                    std::abs(upperLength - targetArcLength);

                if (lowerResidual <= upperResidual
                    && lowerResidual <= distanceTolerance)
                {
                    return lowerParameter;
                }

                if (upperResidual <= distanceTolerance)
                {
                    return upperParameter;
                }

                throw std::runtime_error(
                    "Arc-length inversion exhausted the representable parameter bracket without meeting the distance tolerance."
                );
            }

            const double middleLength = evaluateArcLength(
                curve,
                parameterBegin,
                middleParameter,
                options.arcLength
            );
            const double distanceResidual =
                middleLength - targetArcLength;

            if (std::abs(distanceResidual) <= distanceTolerance)
            {
                return middleParameter;
            }

            if (distanceResidual < 0.0)
            {
                lowerParameter = middleParameter;
                lowerLength = middleLength;
            }
            else
            {
                upperParameter = middleParameter;
                upperLength = middleLength;
            }
        }

        throw std::runtime_error(
            "Arc-length inversion could not meet the distance tolerance within the maximum iteration count."
        );
    }

    // Normalizes a curve's analytic first derivative to produce its unit
    // forward tangent. Parameter validation and exact-knot side selection are
    // inherited directly from evaluateFirstDerivative().
    template<typename Curve>
    [[nodiscard]] glm::dvec3 evaluateUnitTangent(
        const Curve& curve,
        const double parameter
    )
    {
        const auto firstDerivative =
            curve.evaluateFirstDerivative(parameter);
        const double speed = std::hypot(
            firstDerivative.x,
            firstDerivative.y,
            firstDerivative.z
        );

        // The ordinary unit tangent is defined only at regular points. No
        // epsilon is used: any nonzero representable derivative remains valid.
        if (speed == 0.0)
        {
            throw std::domain_error(
                "Curve unit tangent is undefined where the first derivative is zero."
            );
        }

        if (!std::isfinite(speed))
        {
            throw std::domain_error(
                "Curve unit-tangent evaluation produced a non-finite first-derivative magnitude."
            );
        }

        const glm::dvec3 tangent = firstDerivative / speed;

        if (!std::isfinite(tangent.x)
            || !std::isfinite(tangent.y)
            || !std::isfinite(tangent.z))
        {
            throw std::domain_error(
                "Curve unit-tangent evaluation produced a non-finite result."
            );
        }

        return tangent;
    }

    // Computes geometric curvature from a curve's analytic parameter
    // derivatives. The curve must provide evaluateFirstDerivative() and
    // evaluateSecondDerivative() returning double-precision 3D vectors.
    // Parameter validation and exact-knot side selection are therefore
    // inherited from the curve's derivative evaluators.
    //
    // Curvature has inverse-coordinate units. Radius of curvature has the same
    // units as the curve coordinates; no particular physical unit is assumed.
    template<typename Curve>
    [[nodiscard]] double evaluateCurvature(
        const Curve& curve,
        const double parameter
    )
    {
        const auto firstDerivative =
            curve.evaluateFirstDerivative(parameter);
        const auto secondDerivative =
            curve.evaluateSecondDerivative(parameter);

        const double speed = std::hypot(
            firstDerivative.x,
            firstDerivative.y,
            firstDerivative.z
        );

        // The ordinary parametric-curve formula is defined only for regular
        // points. No epsilon is used: a small representable derivative remains
        // valid, while an exactly zero derivative is explicitly rejected.
        if (speed == 0.0)
        {
            throw std::domain_error(
                "Curve curvature is undefined where the first derivative is zero."
            );
        }

        if (!std::isfinite(speed))
        {
            throw std::domain_error(
                "Curve curvature evaluation produced a non-finite first-derivative magnitude."
            );
        }

        const auto derivativeCrossProduct =
            glm::cross(firstDerivative, secondDerivative);
        const double crossProductMagnitude = std::hypot(
            derivativeCrossProduct.x,
            derivativeCrossProduct.y,
            derivativeCrossProduct.z
        );
        const double secondDerivativeMagnitude = std::hypot(
            secondDerivative.x,
            secondDerivative.y,
            secondDerivative.z
        );
        const double derivativeProductMagnitude =
            speed * secondDerivativeMagnitude;
        const double speedCubed = speed * speed * speed;

        if (!std::isfinite(crossProductMagnitude)
            || !std::isfinite(secondDerivativeMagnitude)
            || !std::isfinite(derivativeProductMagnitude)
            || !std::isfinite(speedCubed)
            || speedCubed == 0.0)
        {
            throw std::domain_error(
                "Curve curvature evaluation produced a non-finite or unrepresentable intermediate value."
            );
        }

        // Independently evaluated floating-point derivatives of a straight
        // rational curve can differ from exact collinearity by one rounding
        // unit. Since |C' x C''| is scaled by |C'||C''|, a residual no larger
        // than epsilon times that scale is indistinguishable from cross-product
        // roundoff. This narrowly scoped, scale-relative test prevents such a
        // residual from turning a mathematically infinite radius into a large
        // finite value without imposing a coordinate-unit epsilon.
        const double crossProductRoundoffBound =
            std::numeric_limits<double>::epsilon()
            * derivativeProductMagnitude;
        const double geometricCrossProductMagnitude =
            crossProductMagnitude <= crossProductRoundoffBound
            ? 0.0
            : crossProductMagnitude;
        const double curvature =
            geometricCrossProductMagnitude / speedCubed;

        if (!std::isfinite(curvature)
            || curvature < 0.0
            || (geometricCrossProductMagnitude > 0.0
                && curvature == 0.0))
        {
            throw std::domain_error(
                "Curve curvature evaluation produced an invalid result."
            );
        }

        return curvature;
    }

    template<typename Curve>
    [[nodiscard]] double evaluateRadiusOfCurvature(
        const Curve& curve,
        const double parameter
    )
    {
        const double curvature = evaluateCurvature(curve, parameter);

        if (curvature == 0.0)
        {
            return std::numeric_limits<double>::infinity();
        }

        const double radius = 1.0 / curvature;

        if (!std::isfinite(radius))
        {
            throw std::domain_error(
                "Curve radius-of-curvature evaluation produced a non-finite result for nonzero curvature."
            );
        }

        return radius;
    }
}
