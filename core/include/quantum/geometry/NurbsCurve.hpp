#pragma once

#include <glm/vec3.hpp>

#include <cstddef>
#include <utility>
#include <vector>

namespace quantum::geometry
{
    // A non-uniform rational B-spline curve in three-dimensional double
    // precision.
    //
    // For N control points and non-negative degree p, the knot vector contains
    // N + p + 1 non-decreasing values. The inclusive parameter domain is
    // [U[p], U[N]], with the same boundary and span-selection conventions as
    // BSplineCurve. Weights must be finite and strictly positive; zero and
    // negative weights are outside the conventional safe subset supported by
    // this milestone.
    class NurbsCurve
    {
    public:
        using Point = glm::dvec3;

        // Throws std::invalid_argument when the curve definition is malformed.
        NurbsCurve(
            std::vector<Point> controlPoints,
            std::vector<double> weights,
            int degree,
            std::vector<double> knots
        );

        // Throws std::invalid_argument if parameter is not finite,
        // std::out_of_range if it lies outside the inclusive domain, and
        // std::domain_error if floating-point evaluation cannot produce a
        // finite point with a positive rational denominator.
        [[nodiscard]] Point evaluate(double parameter) const;

        // Evaluates the analytic first derivative with respect to the spline
        // parameter without normalizing it. Parameter validation and span
        // selection match evaluate(): exact interior knots use the span to the
        // right, while the domain end uses the final span from the left. This
        // selects the corresponding one-sided value where a derivative is
        // discontinuous. Degree-zero curves return the zero vector.
        [[nodiscard]] Point evaluateFirstDerivative(double parameter) const;

        // Evaluates the analytic rational second derivative with respect to
        // the spline parameter without normalizing it. Parameter validation
        // and span selection match evaluate(), including its selected
        // one-sided value at exact knots. Degree-zero curves return the zero
        // vector; degree-one curves may have a nonzero second derivative when
        // their rational parameterization is nonlinear.
        [[nodiscard]] Point evaluateSecondDerivative(double parameter) const;

        [[nodiscard]] std::size_t degree() const noexcept;
        [[nodiscard]] const std::vector<Point>& controlPoints() const noexcept;
        [[nodiscard]] const std::vector<double>& weights() const noexcept;
        [[nodiscard]] const std::vector<double>& knots() const noexcept;
        [[nodiscard]] std::pair<double, double> parameterDomain() const noexcept;

    private:
        [[nodiscard]] std::size_t findSpan(double parameter) const noexcept;

        std::vector<Point> controlPoints_;
        std::vector<double> weights_;
        std::size_t degree_ = 0;
        std::vector<double> knots_;
    };
}
