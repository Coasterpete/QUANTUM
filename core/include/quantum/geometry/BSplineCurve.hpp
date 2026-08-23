#pragma once

#include <glm/vec3.hpp>

#include <cstddef>
#include <utility>
#include <vector>

namespace quantum::geometry
{
    // A non-rational B-spline curve in three-dimensional double precision.
    //
    // For N control points and non-negative degree p, the knot vector contains
    // N + p + 1 non-decreasing values. The inclusive parameter domain is
    // [U[p], U[N]]. Evaluation uses the de Boor algorithm. The exact beginning
    // uses the first active span, and the exact end uses the limiting value from
    // the final non-empty span. Clamped definitions therefore evaluate to their
    // first and last control points at those boundaries.
    class BSplineCurve
    {
    public:
        using Point = glm::dvec3;

        // Throws std::invalid_argument when the curve definition is malformed.
        BSplineCurve(
            std::vector<Point> controlPoints,
            int degree,
            std::vector<double> knots
        );

        // Throws std::invalid_argument if parameter is not finite and
        // std::out_of_range if it lies outside the inclusive domain.
        [[nodiscard]] Point evaluate(double parameter) const;

        // Evaluates the analytic first derivative with respect to the spline
        // parameter without normalizing it. Parameter validation and span
        // selection match evaluate(): exact interior knots use the span to the
        // right, while the domain end uses the final span from the left. This
        // selects the corresponding one-sided value where a derivative is
        // discontinuous. Degree-zero curves return the zero vector.
        [[nodiscard]] Point evaluateFirstDerivative(double parameter) const;

        // Evaluates the analytic second derivative with respect to the spline
        // parameter without normalizing it. Parameter validation and span
        // selection match evaluate(), including its selected one-sided value
        // at exact knots. Degree-zero and degree-one curves return the zero
        // vector within the selected span.
        [[nodiscard]] Point evaluateSecondDerivative(double parameter) const;

        [[nodiscard]] std::size_t degree() const noexcept;
        [[nodiscard]] const std::vector<Point>& controlPoints() const noexcept;
        [[nodiscard]] const std::vector<double>& knots() const noexcept;
        [[nodiscard]] std::pair<double, double> parameterDomain() const noexcept;

    private:
        [[nodiscard]] std::size_t findSpan(double parameter) const noexcept;

        std::vector<Point> controlPoints_;
        std::size_t degree_ = 0;
        std::vector<double> knots_;
    };
}
