#pragma once

namespace quantum::math
{
    enum class TransitionType
    {
        Linear,
        Smoothstep,
        Smootherstep,

        // Degree-seven generalized smoothstep:
        // f(x) = 35x^4 - 84x^5 + 70x^6 - 20x^7.
        // Its first three derivatives are zero at both endpoints.
        SeventhOrderSmoothstep,

        // f(x) = (1 - cos(pi*x)) / 2. This symmetric transition has zero
        // first derivative at both endpoints and full-domain area 1/2.
        CosineEaseInOut,

        // f(x) = 1 - cos(pi*x/2). Its beginning slope is zero and its
        // full-domain area is 1 - 2/pi.
        SineEaseIn,

        // f(x) = sin(pi*x/2). Its ending slope is zero and its full-domain
        // area is 2/pi.
        SineEaseOut,

        // Power transitions use f(x) = x^n for EaseIn and
        // f(x) = 1 - (1-x)^n for EaseOut. EaseInOut applies the matching
        // scaled power on each half-domain and is symmetric with area 1/2.
        QuadraticEaseIn,
        QuadraticEaseOut,
        QuadraticEaseInOut,
        CubicEaseIn,
        CubicEaseOut,
        CubicEaseInOut,
        QuarticEaseIn,
        QuarticEaseOut,
        QuarticEaseInOut,
        QuinticEaseIn,
        QuinticEaseOut,
        QuinticEaseInOut
    };

    // Evaluates a normalized scalar transition over the inclusive domain
    // [0, 1]. Non-finite progress throws std::invalid_argument, while finite
    // progress outside the domain throws std::out_of_range.
    [[nodiscard]] double evaluateTransition(
        TransitionType type,
        double normalizedProgress
    );

    // Evaluates F(x), the analytic integral of the normalized transition from
    // zero through normalizedProgress. The built-in integrals are:
    //
    // Linear:       x^2/2
    // Smoothstep:   x^3 - x^4/2
    // Smootherstep: x^6 - 3x^5 + 5x^4/2
    // SeventhOrder: 7x^5 - 14x^6 + 10x^7 - 5x^8/2
    // Cosine:       x/2 - sin(pi*x)/(2*pi)
    // SineEaseIn:   x - 2sin(pi*x/2)/pi
    // SineEaseOut:  2(1 - cos(pi*x/2))/pi
    //
    // For power order n, EaseIn uses x^(n+1)/(n+1), while EaseOut uses
    // x - (1 - (1-x)^(n+1))/(n+1). EaseInOut uses
    // (2x)^(n+1)/(4(n+1)) on the lower half and
    // x - 1/2 + (2(1-x))^(n+1)/(4(n+1)) on the upper half.
    //
    // The input has the same inclusive [0, 1] domain and validation policy as
    // evaluateTransition().
    [[nodiscard]] double evaluateTransitionIntegral(
        TransitionType type,
        double normalizedProgress
    );
}
