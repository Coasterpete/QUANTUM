#include <quantum/geometry/BSplineCurve.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace quantum::geometry
{
    BSplineCurve::BSplineCurve(
        std::vector<Point> controlPoints,
        const int degree,
        std::vector<double> knots
    )
        : controlPoints_(std::move(controlPoints)),
          knots_(std::move(knots))
    {
        if (degree < 0)
        {
            throw std::invalid_argument(
                "B-spline degree must be non-negative."
            );
        }

        degree_ = static_cast<std::size_t>(degree);

        if (controlPoints_.size() < degree_ + 1)
        {
            throw std::invalid_argument(
                "A B-spline requires at least degree + 1 control points."
            );
        }

        const std::size_t expectedKnotCount =
            controlPoints_.size() + degree_ + 1;

        if (knots_.size() != expectedKnotCount)
        {
            throw std::invalid_argument(
                "B-spline knot count must equal control point count + degree + 1."
            );
        }

        for (const Point& point : controlPoints_)
        {
            if (!std::isfinite(point.x)
                || !std::isfinite(point.y)
                || !std::isfinite(point.z))
            {
                throw std::invalid_argument(
                    "B-spline control points must contain only finite values."
                );
            }
        }

        std::size_t knotMultiplicity = 0;

        for (std::size_t index = 0; index < knots_.size(); ++index)
        {
            if (!std::isfinite(knots_[index]))
            {
                throw std::invalid_argument(
                    "B-spline knots must contain only finite values."
                );
            }

            if (index > 0 && knots_[index] < knots_[index - 1])
            {
                throw std::invalid_argument(
                    "B-spline knots must be non-decreasing."
                );
            }

            if (index == 0 || knots_[index] != knots_[index - 1])
            {
                knotMultiplicity = 1;
            }
            else
            {
                ++knotMultiplicity;
            }

            if (knotMultiplicity > degree_ + 1)
            {
                throw std::invalid_argument(
                    "B-spline knot multiplicity cannot exceed degree + 1."
                );
            }
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (domainStart >= domainEnd)
        {
            throw std::invalid_argument(
                "B-spline parameter domain must have positive length."
            );
        }
    }

    BSplineCurve::Point BSplineCurve::evaluate(const double parameter) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "B-spline evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "B-spline evaluation parameter is outside the parameter domain."
            );
        }

        const std::size_t span = findSpan(parameter);
        std::vector<Point> work(degree_ + 1);

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            work[index] = controlPoints_[span - degree_ + index];
        }

        // Standard de Boor recursion. Each pass linearly combines the active
        // control points until only the curve point for this parameter remains.
        for (std::size_t level = 1; level <= degree_; ++level)
        {
            for (std::size_t index = degree_; index >= level; --index)
            {
                const std::size_t knotIndex = span - degree_ + index;
                const double denominator =
                    knots_[knotIndex + degree_ - level + 1]
                    - knots_[knotIndex];

                if (denominator <= 0.0)
                {
                    throw std::logic_error(
                        "Valid B-spline definition produced a zero de Boor denominator."
                    );
                }

                const double blend =
                    (parameter - knots_[knotIndex]) / denominator;

                work[index] =
                    (1.0 - blend) * work[index - 1]
                    + blend * work[index];
            }
        }

        return work[degree_];
    }

    BSplineCurve::Point BSplineCurve::evaluateFirstDerivative(
        const double parameter
    ) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "B-spline evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "B-spline evaluation parameter is outside the parameter domain."
            );
        }

        if (degree_ == 0)
        {
            // Each span is constant. At knots this is the derivative of the
            // span selected by the same convention as evaluate(), rather than
            // a distributional derivative of a possible jump.
            return Point{0.0, 0.0, 0.0};
        }

        const std::size_t span = findSpan(parameter);
        const std::size_t derivativeDegree = degree_ - 1;
        std::vector<Point> work(degree_);

        // The derivative of a degree-p B-spline is the degree-(p - 1)
        // B-spline with control points
        //
        // D_i = p (P_{i + 1} - P_i) / (U_{i + p + 1} - U_{i + 1}).
        //
        // Only the derivative control points active in the selected span are
        // needed. Their denominators include that non-empty span and are
        // therefore positive even when other knots are repeated.
        for (std::size_t index = 0; index <= derivativeDegree; ++index)
        {
            const std::size_t controlPointIndex =
                span - degree_ + index;
            const double denominator =
                knots_[controlPointIndex + degree_ + 1]
                - knots_[controlPointIndex + 1];

            if (denominator <= 0.0)
            {
                throw std::logic_error(
                    "Valid B-spline definition produced a zero derivative denominator."
                );
            }

            work[index] =
                static_cast<double>(degree_)
                * (controlPoints_[controlPointIndex + 1]
                    - controlPoints_[controlPointIndex])
                / denominator;
        }

        // Evaluate the active derivative control points with de Boor recursion
        // over the original knot vector with its first and last knots omitted.
        for (std::size_t level = 1; level <= derivativeDegree; ++level)
        {
            for (
                std::size_t index = derivativeDegree;
                index >= level;
                --index
            )
            {
                const std::size_t derivativeControlPointIndex =
                    span - degree_ + index;
                const double denominator =
                    knots_[
                        derivativeControlPointIndex
                        + degree_ - level + 1
                    ]
                    - knots_[derivativeControlPointIndex + 1];

                if (denominator <= 0.0)
                {
                    throw std::logic_error(
                        "Valid B-spline definition produced a zero derivative de Boor denominator."
                    );
                }

                const double blend =
                    (parameter
                        - knots_[derivativeControlPointIndex + 1])
                    / denominator;

                work[index] =
                    (1.0 - blend) * work[index - 1]
                    + blend * work[index];
            }
        }

        return work[derivativeDegree];
    }

    BSplineCurve::Point BSplineCurve::evaluateSecondDerivative(
        const double parameter
    ) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "B-spline evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "B-spline evaluation parameter is outside the parameter domain."
            );
        }

        if (degree_ < 2)
        {
            // Constant and linear polynomial spans have zero second
            // derivative. At a knot this is the value in the span selected by
            // the same convention as evaluate().
            return Point{0.0, 0.0, 0.0};
        }

        const std::size_t span = findSpan(parameter);
        const std::size_t firstControlPoint = span - degree_;
        std::vector<Point> firstDerivativeControlPoints(degree_);

        // Differentiate the degree-p control points once:
        //
        // D_i = p (P_{i + 1} - P_i)
        //       / (U_{i + p + 1} - U_{i + 1}).
        for (std::size_t index = 0; index < degree_; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double denominator =
                knots_[controlPointIndex + degree_ + 1]
                - knots_[controlPointIndex + 1];

            if (denominator <= 0.0)
            {
                throw std::logic_error(
                    "Valid B-spline definition produced a zero first-derivative denominator."
                );
            }

            firstDerivativeControlPoints[index] =
                static_cast<double>(degree_)
                * (controlPoints_[controlPointIndex + 1]
                    - controlPoints_[controlPointIndex])
                / denominator;
        }

        const std::size_t secondDerivativeDegree = degree_ - 2;
        std::vector<Point> work(degree_ - 1);

        // Differentiate the degree-(p - 1) derivative curve again:
        //
        // E_i = (p - 1) (D_{i + 1} - D_i)
        //       / (U_{i + p + 1} - U_{i + 2}).
        //
        // The resulting E_i form a degree-(p - 2) B-spline over the original
        // knot vector with its first two and last two knots omitted.
        for (std::size_t index = 0; index <= secondDerivativeDegree; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double denominator =
                knots_[controlPointIndex + degree_ + 1]
                - knots_[controlPointIndex + 2];

            if (denominator <= 0.0)
            {
                throw std::logic_error(
                    "Valid B-spline definition produced a zero second-derivative denominator."
                );
            }

            work[index] =
                static_cast<double>(degree_ - 1)
                * (firstDerivativeControlPoints[index + 1]
                    - firstDerivativeControlPoints[index])
                / denominator;
        }

        for (std::size_t level = 1; level <= secondDerivativeDegree; ++level)
        {
            for (
                std::size_t index = secondDerivativeDegree;
                index >= level;
                --index
            )
            {
                const std::size_t secondDerivativeControlPointIndex =
                    firstControlPoint + index;
                const double denominator =
                    knots_[
                        secondDerivativeControlPointIndex
                        + degree_ - level + 1
                    ]
                    - knots_[secondDerivativeControlPointIndex + 2];

                if (denominator <= 0.0)
                {
                    throw std::logic_error(
                        "Valid B-spline definition produced a zero second-derivative de Boor denominator."
                    );
                }

                const double blend =
                    (parameter
                        - knots_[secondDerivativeControlPointIndex + 2])
                    / denominator;

                work[index] =
                    (1.0 - blend) * work[index - 1]
                    + blend * work[index];
            }
        }

        const Point secondDerivative = work[secondDerivativeDegree];

        if (!std::isfinite(secondDerivative.x)
            || !std::isfinite(secondDerivative.y)
            || !std::isfinite(secondDerivative.z))
        {
            throw std::domain_error(
                "B-spline second-derivative evaluation produced a non-finite second derivative."
            );
        }

        return secondDerivative;
    }

    std::size_t BSplineCurve::degree() const noexcept
    {
        return degree_;
    }

    const std::vector<BSplineCurve::Point>&
    BSplineCurve::controlPoints() const noexcept
    {
        return controlPoints_;
    }

    const std::vector<double>& BSplineCurve::knots() const noexcept
    {
        return knots_;
    }

    std::pair<double, double> BSplineCurve::parameterDomain() const noexcept
    {
        return {
            knots_[degree_],
            knots_[controlPoints_.size()]
        };
    }

    std::size_t BSplineCurve::findSpan(const double parameter) const noexcept
    {
        const double domainEnd = knots_[controlPoints_.size()];

        if (parameter == domainEnd)
        {
            const auto firstEndKnot = std::lower_bound(
                knots_.begin(),
                knots_.end(),
                domainEnd
            );

            return static_cast<std::size_t>(
                std::distance(knots_.begin(), firstEndKnot) - 1
            );
        }

        const auto nextKnot = std::upper_bound(
            knots_.begin(),
            knots_.begin()
                + static_cast<std::ptrdiff_t>(controlPoints_.size() + 1),
            parameter
        );

        return static_cast<std::size_t>(
            std::distance(knots_.begin(), nextKnot) - 1
        );
    }
}
