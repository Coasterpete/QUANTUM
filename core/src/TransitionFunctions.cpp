#include <quantum/math/TransitionFunctions.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace quantum::math
{
    namespace
    {
        enum class PowerDirection
        {
            EaseIn,
            EaseOut,
            EaseInOut
        };

        [[nodiscard]] constexpr double integerPower(
            const double value,
            const unsigned int exponent
        )
        {
            double result = 1.0;
            for (unsigned int index = 0; index < exponent; ++index)
            {
                result *= value;
            }
            return result;
        }

        [[nodiscard]] double evaluatePowerTransition(
            const double progress,
            const unsigned int order,
            const PowerDirection direction
        )
        {
            switch (direction)
            {
            case PowerDirection::EaseIn:
                return integerPower(progress, order);

            case PowerDirection::EaseOut:
                if (progress < 0.5)
                {
                    // x * sum((1-x)^k, k=0..n-1) is algebraically equal to
                    // 1 - (1-x)^n without its cancellation near x=0.
                    const double complement = 1.0 - progress;
                    double sum = 1.0;
                    double term = 1.0;
                    for (unsigned int index = 1; index < order; ++index)
                    {
                        term *= complement;
                        sum += term;
                    }
                    return progress * sum;
                }
                return 1.0 - integerPower(1.0 - progress, order);

            case PowerDirection::EaseInOut:
                if (progress < 0.5)
                {
                    return 0.5 * integerPower(2.0 * progress, order);
                }
                return 1.0 - 0.5 * integerPower(
                    2.0 * (1.0 - progress),
                    order
                );
            }

            throw std::invalid_argument("Unsupported power direction.");
        }

        [[nodiscard]] double easeOutPowerIntegral(
            const double progress,
            const unsigned int order
        )
        {
            // Expanded forms begin with x^2 and avoid cancellation in
            // x - (1 - (1-x)^(n+1)) / (n+1) near x=0.
            const double progressSquared = progress * progress;
            switch (order)
            {
            case 2:
                return progressSquared * (1.0 - progress / 3.0);
            case 3:
                return progressSquared * (
                    1.5 + progress * (-1.0 + 0.25 * progress)
                );
            case 4:
                return progressSquared * (
                    2.0 + progress * (
                        -2.0 + progress * (1.0 - 0.2 * progress)
                    )
                );
            case 5:
                return progressSquared * (
                    2.5 + progress * (
                        -10.0 / 3.0 + progress * (
                            2.5 + progress * (
                                -1.0 + progress / 6.0
                            )
                        )
                    )
                );
            }

            throw std::invalid_argument("Unsupported power order.");
        }

        [[nodiscard]] double integratePowerTransition(
            const double progress,
            const unsigned int order,
            const PowerDirection direction
        )
        {
            if (progress == 1.0)
            {
                switch (direction)
                {
                case PowerDirection::EaseIn:
                    return 1.0 / static_cast<double>(order + 1);
                case PowerDirection::EaseOut:
                    return static_cast<double>(order)
                        / static_cast<double>(order + 1);
                case PowerDirection::EaseInOut:
                    return 0.5;
                }
            }

            switch (direction)
            {
            case PowerDirection::EaseIn:
                return integerPower(progress, order + 1)
                    / static_cast<double>(order + 1);

            case PowerDirection::EaseOut:
                return easeOutPowerIntegral(progress, order);

            case PowerDirection::EaseInOut:
                if (progress < 0.5)
                {
                    return integerPower(2.0 * progress, order + 1)
                        / (4.0 * static_cast<double>(order + 1));
                }
                return progress - 0.5
                    + integerPower(
                        2.0 * (1.0 - progress),
                        order + 1
                    ) / (4.0 * static_cast<double>(order + 1));
            }

            throw std::invalid_argument("Unsupported power direction.");
        }

        [[nodiscard]] double angleMinusSine(const double angle)
        {
            if (angle <= 0.5)
            {
                // z - sin(z), evaluated as its alternating series where the
                // direct subtraction would lose precision near zero.
                const double angleSquared = angle * angle;
                return angle * angleSquared * (
                    1.0 / 6.0 + angleSquared * (
                        -1.0 / 120.0 + angleSquared * (
                            1.0 / 5'040.0 + angleSquared * (
                                -1.0 / 362'880.0 + angleSquared * (
                                    1.0 / 39'916'800.0
                                    - angleSquared / 6'227'020'800.0
                                )
                            )
                        )
                    )
                );
            }

            return angle - std::sin(angle);
        }
    }

    double evaluateTransition(
        const TransitionType type,
        const double normalizedProgress
    )
    {
        if (!std::isfinite(normalizedProgress))
        {
            throw std::invalid_argument(
                "Normalized transition progress must be finite."
            );
        }

        if (normalizedProgress < 0.0 || normalizedProgress > 1.0)
        {
            throw std::out_of_range(
                "Normalized transition progress must be in the inclusive range [0, 1]."
            );
        }

        // Return the normalized boundaries deliberately so every supported
        // formula has exact, shared endpoint behavior.
        if (normalizedProgress == 0.0)
        {
            return 0.0;
        }

        if (normalizedProgress == 1.0)
        {
            return 1.0;
        }

        switch (type)
        {
        case TransitionType::Linear:
            return normalizedProgress;

        case TransitionType::Smoothstep:
            return normalizedProgress
                * normalizedProgress
                * (3.0 - 2.0 * normalizedProgress);

        case TransitionType::Smootherstep:
            return normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * (normalizedProgress
                    * (6.0 * normalizedProgress - 15.0)
                    + 10.0);

        case TransitionType::SeventhOrderSmoothstep:
        {
            const bool useComplement = normalizedProgress > 0.5;
            const double progress = useComplement
                ? 1.0 - normalizedProgress
                : normalizedProgress;
            const double lowerValue = progress
                * progress
                * progress
                * progress
                * (35.0 + progress
                    * (-84.0 + progress * (70.0 - 20.0 * progress)));
            return useComplement ? 1.0 - lowerValue : lowerValue;
        }

        case TransitionType::CosineEaseInOut:
        {
            const double sine = std::sin(
                std::numbers::pi_v<double> * normalizedProgress / 2.0
            );
            return sine * sine;
        }

        case TransitionType::SineEaseIn:
        {
            const double sine = std::sin(
                std::numbers::pi_v<double> * normalizedProgress / 4.0
            );
            return 2.0 * sine * sine;
        }

        case TransitionType::SineEaseOut:
            return std::sin(
                std::numbers::pi_v<double> * normalizedProgress / 2.0
            );

        case TransitionType::QuadraticEaseIn:
            return evaluatePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseIn
            );
        case TransitionType::QuadraticEaseOut:
            return evaluatePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseOut
            );
        case TransitionType::QuadraticEaseInOut:
            return evaluatePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseInOut
            );
        case TransitionType::CubicEaseIn:
            return evaluatePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseIn
            );
        case TransitionType::CubicEaseOut:
            return evaluatePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseOut
            );
        case TransitionType::CubicEaseInOut:
            return evaluatePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseInOut
            );
        case TransitionType::QuarticEaseIn:
            return evaluatePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseIn
            );
        case TransitionType::QuarticEaseOut:
            return evaluatePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseOut
            );
        case TransitionType::QuarticEaseInOut:
            return evaluatePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseInOut
            );
        case TransitionType::QuinticEaseIn:
            return evaluatePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseIn
            );
        case TransitionType::QuinticEaseOut:
            return evaluatePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseOut
            );
        case TransitionType::QuinticEaseInOut:
            return evaluatePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseInOut
            );
        }

        throw std::invalid_argument("Unsupported transition type.");
    }

    double evaluateTransitionIntegral(
        const TransitionType type,
        const double normalizedProgress
    )
    {
        if (!std::isfinite(normalizedProgress))
        {
            throw std::invalid_argument(
                "Normalized transition progress must be finite."
            );
        }

        if (normalizedProgress < 0.0 || normalizedProgress > 1.0)
        {
            throw std::out_of_range(
                "Normalized transition progress must be in the inclusive range [0, 1]."
            );
        }

        switch (type)
        {
        case TransitionType::Linear:
            return 0.5 * normalizedProgress * normalizedProgress;

        case TransitionType::Smoothstep:
            return normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * (1.0 - 0.5 * normalizedProgress);

        case TransitionType::Smootherstep:
            return normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * normalizedProgress
                * (normalizedProgress
                    * (normalizedProgress - 3.0)
                    + 2.5);

        case TransitionType::SeventhOrderSmoothstep:
        {
            const bool useComplement = normalizedProgress > 0.5;
            const double progress = useComplement
                ? 1.0 - normalizedProgress
                : normalizedProgress;
            const double lowerIntegral = progress
                * progress
                * progress
                * progress
                * progress
                * (7.0 + progress
                    * (-14.0 + progress * (10.0 - 2.5 * progress)));
            return useComplement
                ? normalizedProgress - 0.5 + lowerIntegral
                : lowerIntegral;
        }

        case TransitionType::CosineEaseInOut:
            if (normalizedProgress == 1.0)
            {
                return 0.5;
            }
            return angleMinusSine(
                std::numbers::pi_v<double> * normalizedProgress
            ) / (2.0 * std::numbers::pi_v<double>);

        case TransitionType::SineEaseIn:
            if (normalizedProgress == 1.0)
            {
                return 1.0 - 2.0 / std::numbers::pi_v<double>;
            }
            return angleMinusSine(
                std::numbers::pi_v<double> * normalizedProgress / 2.0
            ) / (std::numbers::pi_v<double> / 2.0);

        case TransitionType::SineEaseOut:
            if (normalizedProgress == 1.0)
            {
                return 2.0 / std::numbers::pi_v<double>;
            }
            {
                const double sine = std::sin(
                    std::numbers::pi_v<double> * normalizedProgress / 4.0
                );
                return 4.0 * sine * sine
                    / std::numbers::pi_v<double>;
            }

        case TransitionType::QuadraticEaseIn:
            return integratePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseIn
            );
        case TransitionType::QuadraticEaseOut:
            return integratePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseOut
            );
        case TransitionType::QuadraticEaseInOut:
            return integratePowerTransition(
                normalizedProgress, 2, PowerDirection::EaseInOut
            );
        case TransitionType::CubicEaseIn:
            return integratePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseIn
            );
        case TransitionType::CubicEaseOut:
            return integratePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseOut
            );
        case TransitionType::CubicEaseInOut:
            return integratePowerTransition(
                normalizedProgress, 3, PowerDirection::EaseInOut
            );
        case TransitionType::QuarticEaseIn:
            return integratePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseIn
            );
        case TransitionType::QuarticEaseOut:
            return integratePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseOut
            );
        case TransitionType::QuarticEaseInOut:
            return integratePowerTransition(
                normalizedProgress, 4, PowerDirection::EaseInOut
            );
        case TransitionType::QuinticEaseIn:
            return integratePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseIn
            );
        case TransitionType::QuinticEaseOut:
            return integratePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseOut
            );
        case TransitionType::QuinticEaseInOut:
            return integratePowerTransition(
                normalizedProgress, 5, PowerDirection::EaseInOut
            );
        }

        throw std::invalid_argument("Unsupported transition type.");
    }
}
