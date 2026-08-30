#pragma once

#include <quantum/coaster/TrackKinematics.hpp>

#include <cstddef>
#include <expected>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace quantum::coaster
{
    enum class TrackGenerationFailureReason
    {
        InvalidInput,
        EnergeticallyUnreachable,
        InsufficientSpeed,
        NonfiniteDerivedRates,
        IntegrationFailure
    };

    struct TrackGenerationFailure
    {
        TrackGenerationFailureReason reason;
        std::optional<std::size_t> sectionIndex;
        std::optional<double> localDistance;
        std::optional<double> cumulativeDistance;
        std::optional<double> speedSquared;
        std::string message;
    };

    using TrackGenerationResult = std::expected<
        std::vector<TrackKinematicState>, TrackGenerationFailure>;

    // Exception adapter for existing throwing geometry/visualization callers.
    // The result API retains the same failure without publishing partial data.
    class TrackGenerationError : public std::runtime_error
    {
    public:
        explicit TrackGenerationError(TrackGenerationFailure failure);
        [[nodiscard]] const TrackGenerationFailure& failure() const noexcept;

    private:
        TrackGenerationFailure failure_;
    };
}
