#pragma once

#include <quantum/math/TransitionFunctions.hpp>

namespace quantum::math
{
    struct ScalarTransition
    {
        double domainBegin;
        double domainEnd;
        double valueBegin;
        double valueEnd;
        TransitionType transitionType;
    };

    // Evaluates an authored scalar transition over its inclusive domain.
    // Non-finite inputs and unordered domains throw std::invalid_argument.
    // Finite queries outside the domain throw std::out_of_range.
    [[nodiscard]] double evaluateScalarTransition(
        const ScalarTransition& transition,
        double independentValue
    );

    // Analytically integrates an authored scalar transition over an ordered
    // subinterval of its inclusive domain. The result has scalar-value times
    // independent-domain units. Equal integration bounds return exactly zero.
    [[nodiscard]] double integrateScalarTransition(
        const ScalarTransition& transition,
        double independentBegin,
        double independentEnd
    );
}
