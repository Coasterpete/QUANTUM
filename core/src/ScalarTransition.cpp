#include <quantum/math/ScalarTransition.hpp>

#include <cmath>
#include <stdexcept>

namespace quantum::math
{
    namespace
    {
        [[nodiscard]] double validatedDomainLength(
            const ScalarTransition& transition
        )
        {
            if (!std::isfinite(transition.domainBegin)
                || !std::isfinite(transition.domainEnd)
                || !std::isfinite(transition.valueBegin)
                || !std::isfinite(transition.valueEnd))
            {
                throw std::invalid_argument(
                    "Scalar transition values must be finite."
                );
            }

            if (transition.domainBegin >= transition.domainEnd)
            {
                throw std::invalid_argument(
                    "Scalar transition domain beginning must be less than its end."
                );
            }

            const double domainLength =
                transition.domainEnd - transition.domainBegin;
            if (!std::isfinite(domainLength))
            {
                throw std::invalid_argument(
                    "Scalar transition domain length must be finite."
                );
            }

            // Validate the enum independently from any exact-boundary path.
            static_cast<void>(evaluateTransitionIntegral(
                transition.transitionType,
                0.0
            ));
            return domainLength;
        }
    }

    double evaluateScalarTransition(
        const ScalarTransition& transition,
        const double independentValue
    )
    {
        if (!std::isfinite(transition.domainBegin)
            || !std::isfinite(transition.domainEnd)
            || !std::isfinite(transition.valueBegin)
            || !std::isfinite(transition.valueEnd)
            || !std::isfinite(independentValue))
        {
            throw std::invalid_argument(
                "Scalar transition values and query must be finite."
            );
        }

        if (transition.domainBegin >= transition.domainEnd)
        {
            throw std::invalid_argument(
                "Scalar transition domain beginning must be less than its end."
            );
        }

        if (independentValue < transition.domainBegin
            || independentValue > transition.domainEnd)
        {
            throw std::out_of_range(
                "Scalar transition query must be inside its inclusive domain."
            );
        }

        if (independentValue == transition.domainBegin)
        {
            return transition.valueBegin;
        }

        if (independentValue == transition.domainEnd)
        {
            return transition.valueEnd;
        }

        const double normalizedProgress =
            (independentValue - transition.domainBegin)
            / (transition.domainEnd - transition.domainBegin);
        const double transitionProgress = evaluateTransition(
            transition.transitionType,
            normalizedProgress
        );

        return transition.valueBegin
            + (transition.valueEnd - transition.valueBegin)
                * transitionProgress;
    }

    double integrateScalarTransition(
        const ScalarTransition& transition,
        const double independentBegin,
        const double independentEnd
    )
    {
        const double domainLength = validatedDomainLength(transition);

        if (!std::isfinite(independentBegin)
            || !std::isfinite(independentEnd))
        {
            throw std::invalid_argument(
                "Scalar transition integration bounds must be finite."
            );
        }

        if (independentBegin > independentEnd)
        {
            throw std::invalid_argument(
                "Scalar transition integration bounds must be ordered."
            );
        }

        if (independentBegin < transition.domainBegin
            || independentEnd > transition.domainEnd)
        {
            throw std::out_of_range(
                "Scalar transition integration bounds must be inside its inclusive domain."
            );
        }

        if (independentBegin == independentEnd)
        {
            return 0.0;
        }

        const double normalizedBegin =
            (independentBegin - transition.domainBegin) / domainLength;
        const double normalizedEnd =
            (independentEnd - transition.domainBegin) / domainLength;
        const double transitionArea =
            evaluateTransitionIntegral(
                transition.transitionType,
                normalizedEnd
            )
            - evaluateTransitionIntegral(
                transition.transitionType,
                normalizedBegin
            );
        const double integral = transition.valueBegin
            * (independentEnd - independentBegin)
            + (transition.valueEnd - transition.valueBegin)
                * domainLength * transitionArea;

        if (!std::isfinite(integral))
        {
            throw std::domain_error(
                "Scalar transition integral is not representable as a finite value."
            );
        }

        return integral;
    }
}
