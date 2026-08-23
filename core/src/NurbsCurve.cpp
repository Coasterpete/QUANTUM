#include <quantum/geometry/NurbsCurve.hpp>

#include <glm/vec4.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace quantum::geometry
{
    NurbsCurve::NurbsCurve(
        std::vector<Point> controlPoints,
        std::vector<double> weights,
        const int degree,
        std::vector<double> knots
    )
        : controlPoints_(std::move(controlPoints)),
          weights_(std::move(weights)),
          knots_(std::move(knots))
    {
        if (degree < 0)
        {
            throw std::invalid_argument(
                "NURBS degree must be non-negative."
            );
        }

        degree_ = static_cast<std::size_t>(degree);

        if (controlPoints_.size() < degree_ + 1)
        {
            throw std::invalid_argument(
                "A NURBS curve requires at least degree + 1 control points."
            );
        }

        if (weights_.size() != controlPoints_.size())
        {
            throw std::invalid_argument(
                "NURBS weight count must equal control point count."
            );
        }

        const std::size_t expectedKnotCount =
            controlPoints_.size() + degree_ + 1;

        if (knots_.size() != expectedKnotCount)
        {
            throw std::invalid_argument(
                "NURBS knot count must equal control point count + degree + 1."
            );
        }

        for (const Point& point : controlPoints_)
        {
            if (!std::isfinite(point.x)
                || !std::isfinite(point.y)
                || !std::isfinite(point.z))
            {
                throw std::invalid_argument(
                    "NURBS control points must contain only finite values."
                );
            }
        }

        for (const double weight : weights_)
        {
            if (!std::isfinite(weight))
            {
                throw std::invalid_argument(
                    "NURBS weights must contain only finite values."
                );
            }

            if (weight <= 0.0)
            {
                throw std::invalid_argument(
                    "NURBS weights must be strictly positive."
                );
            }
        }

        std::size_t knotMultiplicity = 0;

        for (std::size_t index = 0; index < knots_.size(); ++index)
        {
            if (!std::isfinite(knots_[index]))
            {
                throw std::invalid_argument(
                    "NURBS knots must contain only finite values."
                );
            }

            if (index > 0 && knots_[index] < knots_[index - 1])
            {
                throw std::invalid_argument(
                    "NURBS knots must be non-decreasing."
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
                    "NURBS knot multiplicity cannot exceed degree + 1."
                );
            }
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (domainStart >= domainEnd)
        {
            throw std::invalid_argument(
                "NURBS parameter domain must have positive length."
            );
        }
    }

    NurbsCurve::Point NurbsCurve::evaluate(const double parameter) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "NURBS evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "NURBS evaluation parameter is outside the parameter domain."
            );
        }

        const std::size_t span = findSpan(parameter);
        const std::size_t firstControlPoint = span - degree_;
        double weightScale = 0.0;

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            weightScale = std::max(
                weightScale,
                weights_[firstControlPoint + index]
            );
        }

        std::vector<glm::dvec4> work(degree_ + 1);

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double normalizedWeight =
                weights_[controlPointIndex] / weightScale;

            work[index] = glm::dvec4(
                controlPoints_[controlPointIndex] * normalizedWeight,
                normalizedWeight
            );
        }

        // Apply standard de Boor recursion to homogeneous control points
        // (wP, w). Dividing the resulting xyz coordinates by w evaluates the
        // rational curve. Scaling all active weights by their maximum leaves
        // the rational point unchanged while avoiding avoidable overflow from
        // very large finite weights.
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
                        "Valid NURBS definition produced a zero de Boor denominator."
                    );
                }

                const double blend =
                    (parameter - knots_[knotIndex]) / denominator;

                work[index] =
                    (1.0 - blend) * work[index - 1]
                    + blend * work[index];
            }
        }

        const glm::dvec4 homogeneousPoint = work[degree_];

        if (!std::isfinite(homogeneousPoint.w)
            || homogeneousPoint.w <= 0.0)
        {
            throw std::domain_error(
                "NURBS evaluation produced an invalid rational denominator."
            );
        }

        const Point point =
            Point{homogeneousPoint} / homogeneousPoint.w;

        if (!std::isfinite(point.x)
            || !std::isfinite(point.y)
            || !std::isfinite(point.z))
        {
            throw std::domain_error(
                "NURBS evaluation produced a non-finite point."
            );
        }

        return point;
    }

    NurbsCurve::Point NurbsCurve::evaluateFirstDerivative(
        const double parameter
    ) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "NURBS evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "NURBS evaluation parameter is outside the parameter domain."
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
        const std::size_t firstControlPoint = span - degree_;
        double weightScale = 0.0;

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            weightScale = std::max(
                weightScale,
                weights_[firstControlPoint + index]
            );
        }

        std::vector<glm::dvec4> homogeneousControlPoints(degree_ + 1);

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double normalizedWeight =
                weights_[controlPointIndex] / weightScale;

            homogeneousControlPoints[index] = glm::dvec4(
                controlPoints_[controlPointIndex] * normalizedWeight,
                normalizedWeight
            );
        }

        std::vector<glm::dvec4> pointWork = homogeneousControlPoints;

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
                        "Valid NURBS definition produced a zero de Boor denominator."
                    );
                }

                const double blend =
                    (parameter - knots_[knotIndex]) / denominator;

                pointWork[index] =
                    (1.0 - blend) * pointWork[index - 1]
                    + blend * pointWork[index];
            }
        }

        const std::size_t derivativeDegree = degree_ - 1;
        std::vector<glm::dvec4> derivativeWork(degree_);

        // Differentiate the homogeneous B-spline Q(u) = (A(u), W(u)).
        // Its degree-(p - 1) derivative control points are
        //
        // p (Q_{i + 1} - Q_i) / (U_{i + p + 1} - U_{i + 1}).
        for (std::size_t index = 0; index <= derivativeDegree; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double denominator =
                knots_[controlPointIndex + degree_ + 1]
                - knots_[controlPointIndex + 1];

            if (denominator <= 0.0)
            {
                throw std::logic_error(
                    "Valid NURBS definition produced a zero derivative denominator."
                );
            }

            derivativeWork[index] =
                static_cast<double>(degree_)
                * (homogeneousControlPoints[index + 1]
                    - homogeneousControlPoints[index])
                / denominator;
        }

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
                        "Valid NURBS definition produced a zero derivative de Boor denominator."
                    );
                }

                const double blend =
                    (parameter
                        - knots_[derivativeControlPointIndex + 1])
                    / denominator;

                derivativeWork[index] =
                    (1.0 - blend) * derivativeWork[index - 1]
                    + blend * derivativeWork[index];
            }
        }

        const glm::dvec4 homogeneousPoint = pointWork[degree_];
        const glm::dvec4 homogeneousDerivative =
            derivativeWork[derivativeDegree];

        if (!std::isfinite(homogeneousPoint.w)
            || homogeneousPoint.w <= 0.0)
        {
            throw std::domain_error(
                "NURBS derivative evaluation produced an invalid rational denominator."
            );
        }

        if (!std::isfinite(homogeneousDerivative.x)
            || !std::isfinite(homogeneousDerivative.y)
            || !std::isfinite(homogeneousDerivative.z)
            || !std::isfinite(homogeneousDerivative.w))
        {
            throw std::domain_error(
                "NURBS derivative evaluation produced a non-finite homogeneous derivative."
            );
        }

        const Point point =
            Point{homogeneousPoint} / homogeneousPoint.w;

        if (!std::isfinite(point.x)
            || !std::isfinite(point.y)
            || !std::isfinite(point.z))
        {
            throw std::domain_error(
                "NURBS derivative evaluation produced a non-finite point."
            );
        }

        // For C(u) = A(u) / W(u), this is the quotient-rule derivative
        // (A'W - AW') / W^2, arranged as (A' - C W') / W to avoid
        // unnecessarily squaring a very large or very small denominator.
        const Point derivative =
            (Point{homogeneousDerivative}
                - point * homogeneousDerivative.w)
            / homogeneousPoint.w;

        if (!std::isfinite(derivative.x)
            || !std::isfinite(derivative.y)
            || !std::isfinite(derivative.z))
        {
            throw std::domain_error(
                "NURBS derivative evaluation produced a non-finite derivative."
            );
        }

        return derivative;
    }

    NurbsCurve::Point NurbsCurve::evaluateSecondDerivative(
        const double parameter
    ) const
    {
        if (!std::isfinite(parameter))
        {
            throw std::invalid_argument(
                "NURBS evaluation parameter must be finite."
            );
        }

        const auto [domainStart, domainEnd] = parameterDomain();

        if (parameter < domainStart || parameter > domainEnd)
        {
            throw std::out_of_range(
                "NURBS evaluation parameter is outside the parameter domain."
            );
        }

        if (degree_ == 0)
        {
            // Each selected span is a constant rational point.
            return Point{0.0, 0.0, 0.0};
        }

        const std::size_t span = findSpan(parameter);
        const std::size_t firstControlPoint = span - degree_;
        double weightScale = 0.0;

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            weightScale = std::max(
                weightScale,
                weights_[firstControlPoint + index]
            );
        }

        std::vector<glm::dvec4> homogeneousControlPoints(degree_ + 1);

        for (std::size_t index = 0; index <= degree_; ++index)
        {
            const std::size_t controlPointIndex =
                firstControlPoint + index;
            const double normalizedWeight =
                weights_[controlPointIndex] / weightScale;

            homogeneousControlPoints[index] = glm::dvec4(
                controlPoints_[controlPointIndex] * normalizedWeight,
                normalizedWeight
            );
        }

        std::vector<glm::dvec4> pointWork = homogeneousControlPoints;

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
                        "Valid NURBS definition produced a zero de Boor denominator."
                    );
                }

                const double blend =
                    (parameter - knots_[knotIndex]) / denominator;

                pointWork[index] =
                    (1.0 - blend) * pointWork[index - 1]
                    + blend * pointWork[index];
            }
        }

        std::vector<glm::dvec4> firstDerivativeControlPoints(degree_);

        // For the homogeneous B-spline Q(u) = (A(u), W(u)), first form
        //
        // D_i = p (Q_{i + 1} - Q_i)
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
                    "Valid NURBS definition produced a zero first-derivative denominator."
                );
            }

            firstDerivativeControlPoints[index] =
                static_cast<double>(degree_)
                * (homogeneousControlPoints[index + 1]
                    - homogeneousControlPoints[index])
                / denominator;
        }

        const std::size_t firstDerivativeDegree = degree_ - 1;
        std::vector<glm::dvec4> firstDerivativeWork =
            firstDerivativeControlPoints;

        for (std::size_t level = 1; level <= firstDerivativeDegree; ++level)
        {
            for (
                std::size_t index = firstDerivativeDegree;
                index >= level;
                --index
            )
            {
                const std::size_t derivativeControlPointIndex =
                    firstControlPoint + index;
                const double denominator =
                    knots_[
                        derivativeControlPointIndex
                        + degree_ - level + 1
                    ]
                    - knots_[derivativeControlPointIndex + 1];

                if (denominator <= 0.0)
                {
                    throw std::logic_error(
                        "Valid NURBS definition produced a zero first-derivative de Boor denominator."
                    );
                }

                const double blend =
                    (parameter
                        - knots_[derivativeControlPointIndex + 1])
                    / denominator;

                firstDerivativeWork[index] =
                    (1.0 - blend) * firstDerivativeWork[index - 1]
                    + blend * firstDerivativeWork[index];
            }
        }

        glm::dvec4 homogeneousSecondDerivative{0.0};

        if (degree_ >= 2)
        {
            const std::size_t secondDerivativeDegree = degree_ - 2;
            std::vector<glm::dvec4> secondDerivativeWork(degree_ - 1);

            // Differentiate the homogeneous derivative control points:
            //
            // E_i = (p - 1) (D_{i + 1} - D_i)
            //       / (U_{i + p + 1} - U_{i + 2}).
            for (
                std::size_t index = 0;
                index <= secondDerivativeDegree;
                ++index
            )
            {
                const std::size_t controlPointIndex =
                    firstControlPoint + index;
                const double denominator =
                    knots_[controlPointIndex + degree_ + 1]
                    - knots_[controlPointIndex + 2];

                if (denominator <= 0.0)
                {
                    throw std::logic_error(
                        "Valid NURBS definition produced a zero second-derivative denominator."
                    );
                }

                secondDerivativeWork[index] =
                    static_cast<double>(degree_ - 1)
                    * (firstDerivativeControlPoints[index + 1]
                        - firstDerivativeControlPoints[index])
                    / denominator;

                const glm::dvec4& value = secondDerivativeWork[index];

                if (!std::isfinite(value.x)
                    || !std::isfinite(value.y)
                    || !std::isfinite(value.z)
                    || !std::isfinite(value.w))
                {
                    throw std::domain_error(
                        "NURBS second-derivative evaluation produced a non-finite homogeneous second-derivative intermediate."
                    );
                }
            }

            for (
                std::size_t level = 1;
                level <= secondDerivativeDegree;
                ++level
            )
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
                            "Valid NURBS definition produced a zero second-derivative de Boor denominator."
                        );
                    }

                    const double blend =
                        (parameter
                            - knots_[
                                secondDerivativeControlPointIndex + 2
                            ])
                        / denominator;

                    secondDerivativeWork[index] =
                        (1.0 - blend) * secondDerivativeWork[index - 1]
                        + blend * secondDerivativeWork[index];

                    const glm::dvec4& value =
                        secondDerivativeWork[index];

                    if (!std::isfinite(value.x)
                        || !std::isfinite(value.y)
                        || !std::isfinite(value.z)
                        || !std::isfinite(value.w))
                    {
                        throw std::domain_error(
                            "NURBS second-derivative evaluation produced a non-finite homogeneous second-derivative intermediate."
                        );
                    }
                }
            }

            homogeneousSecondDerivative =
                secondDerivativeWork[secondDerivativeDegree];
        }

        const glm::dvec4 homogeneousPoint = pointWork[degree_];
        const glm::dvec4 homogeneousFirstDerivative =
            firstDerivativeWork[firstDerivativeDegree];

        if (!std::isfinite(homogeneousPoint.w)
            || homogeneousPoint.w <= 0.0)
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced an invalid rational denominator."
            );
        }

        if (!std::isfinite(homogeneousFirstDerivative.x)
            || !std::isfinite(homogeneousFirstDerivative.y)
            || !std::isfinite(homogeneousFirstDerivative.z)
            || !std::isfinite(homogeneousFirstDerivative.w))
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced a non-finite homogeneous first derivative."
            );
        }

        if (!std::isfinite(homogeneousSecondDerivative.x)
            || !std::isfinite(homogeneousSecondDerivative.y)
            || !std::isfinite(homogeneousSecondDerivative.z)
            || !std::isfinite(homogeneousSecondDerivative.w))
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced a non-finite homogeneous second derivative."
            );
        }

        const Point point =
            Point{homogeneousPoint} / homogeneousPoint.w;

        if (!std::isfinite(point.x)
            || !std::isfinite(point.y)
            || !std::isfinite(point.z))
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced a non-finite point."
            );
        }

        const Point firstDerivative =
            (Point{homogeneousFirstDerivative}
                - point * homogeneousFirstDerivative.w)
            / homogeneousPoint.w;

        if (!std::isfinite(firstDerivative.x)
            || !std::isfinite(firstDerivative.y)
            || !std::isfinite(firstDerivative.z))
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced a non-finite first derivative."
            );
        }

        // From A = WC, differentiating twice gives
        // A'' = W''C + 2W'C' + WC''. The common active-weight scale applied
        // above multiplies A, W, and both of their derivatives equally, so it
        // cancels from this rational result.
        const Point secondDerivative =
            (Point{homogeneousSecondDerivative}
                - point * homogeneousSecondDerivative.w
                - 2.0 * firstDerivative * homogeneousFirstDerivative.w)
            / homogeneousPoint.w;

        if (!std::isfinite(secondDerivative.x)
            || !std::isfinite(secondDerivative.y)
            || !std::isfinite(secondDerivative.z))
        {
            throw std::domain_error(
                "NURBS second-derivative evaluation produced a non-finite second derivative."
            );
        }

        return secondDerivative;
    }

    std::size_t NurbsCurve::degree() const noexcept
    {
        return degree_;
    }

    const std::vector<NurbsCurve::Point>&
    NurbsCurve::controlPoints() const noexcept
    {
        return controlPoints_;
    }

    const std::vector<double>& NurbsCurve::weights() const noexcept
    {
        return weights_;
    }

    const std::vector<double>& NurbsCurve::knots() const noexcept
    {
        return knots_;
    }

    std::pair<double, double> NurbsCurve::parameterDomain() const noexcept
    {
        return {
            knots_[degree_],
            knots_[controlPoints_.size()]
        };
    }

    std::size_t NurbsCurve::findSpan(const double parameter) const noexcept
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
