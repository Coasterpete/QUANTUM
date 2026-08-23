#pragma once

#include <quantum/math/ScalarTransition.hpp>

#include <array>
#include <string_view>

namespace quantum::editor
{
    struct TransitionTypePreset
    {
        math::TransitionType type;
        std::string_view displayName;
    };

    inline constexpr std::array<TransitionTypePreset, 19>
        transitionTypePresets{{
            {math::TransitionType::Linear, "Linear"},
            {math::TransitionType::Smoothstep, "Smoothstep"},
            {math::TransitionType::Smootherstep, "Smootherstep"},
            {
                math::TransitionType::SeventhOrderSmoothstep,
                "Seventh Order Smoothstep"
            },
            {
                math::TransitionType::CosineEaseInOut,
                "Cosine Ease In/Out"
            },
            {math::TransitionType::SineEaseIn, "Sine Ease In"},
            {math::TransitionType::SineEaseOut, "Sine Ease Out"},
            {
                math::TransitionType::QuadraticEaseIn,
                "Quadratic Ease In"
            },
            {
                math::TransitionType::QuadraticEaseOut,
                "Quadratic Ease Out"
            },
            {
                math::TransitionType::QuadraticEaseInOut,
                "Quadratic Ease In/Out"
            },
            {math::TransitionType::CubicEaseIn, "Cubic Ease In"},
            {math::TransitionType::CubicEaseOut, "Cubic Ease Out"},
            {
                math::TransitionType::CubicEaseInOut,
                "Cubic Ease In/Out"
            },
            {math::TransitionType::QuarticEaseIn, "Quartic Ease In"},
            {math::TransitionType::QuarticEaseOut, "Quartic Ease Out"},
            {
                math::TransitionType::QuarticEaseInOut,
                "Quartic Ease In/Out"
            },
            {math::TransitionType::QuinticEaseIn, "Quintic Ease In"},
            {math::TransitionType::QuinticEaseOut, "Quintic Ease Out"},
            {
                math::TransitionType::QuinticEaseInOut,
                "Quintic Ease In/Out"
            }
        }};

    [[nodiscard]] constexpr const TransitionTypePreset*
    findTransitionTypePreset(const math::TransitionType type) noexcept
    {
        for (const TransitionTypePreset& preset : transitionTypePresets)
        {
            if (preset.type == type)
            {
                return &preset;
            }
        }

        return nullptr;
    }

    // Applies only choices represented by the current Editor preset table.
    // This keeps unsupported enum values out of authored transitions without
    // introducing a second transition enum.
    [[nodiscard]] constexpr bool trySetTransitionTypePreset(
        math::ScalarTransition& transition,
        const math::TransitionType type) noexcept
    {
        if (findTransitionTypePreset(type) == nullptr)
        {
            return false;
        }

        transition.transitionType = type;
        return true;
    }
}
