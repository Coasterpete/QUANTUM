#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::coaster
{
    // A roller coaster "setup" is the authored document configuration that
    // describes what kind of coaster this is: which style/capabilities apply,
    // which option values are set, the train configuration, and the heartline
    // settings. The style catalog is data, not code: adding a style or option
    // is a catalog entry, not a UI or serializer edit.
    //
    // The invisible style id is the stable persistence key; the display name
    // is presentation data. Capabilities are a bitmask so a single style can
    // expose several independent configuration surfaces.
    enum class CoasterCapability : std::uint8_t
    {
        None = 0,
        Launch = 1u << 0,
        TrainLayout = 1u << 1,
        Restraints = 1u << 2
    };

    constexpr CoasterCapability operator|(
        const CoasterCapability lhs,
        const CoasterCapability rhs) noexcept
    {
        return static_cast<CoasterCapability>(
            static_cast<std::uint8_t>(lhs)
            | static_cast<std::uint8_t>(rhs));
    }

    // An option with requiredCapability None is exposed by every style;
    // otherwise the style must hold the capability to expose the option.
    constexpr bool hasCapability(
        const CoasterCapability style,
        const CoasterCapability required) noexcept
    {
        return required == CoasterCapability::None
            || (static_cast<std::uint8_t>(style)
                    & static_cast<std::uint8_t>(required))
                == static_cast<std::uint8_t>(required);
    }

    // Human-readable comma-separated names for every flag in a capability set.
    [[nodiscard]] std::string coasterCapabilitySummary(
        CoasterCapability capabilities);

    enum class CoasterOptionKind : std::uint8_t
    {
        Boolean,
        Choice
    };

    // One selectable value of a Choice option. The train-layout choices carry
    // seat metadata so the editor can summarize the authored setup without
    // duplicating the option state.
    struct CoasterOptionChoice
    {
        std::string_view id;
        std::string_view label;
        std::uint32_t seatsAcross = 0;
        std::uint32_t rowsPerCar = 0;

        [[nodiscard]] friend bool operator==(
            const CoasterOptionChoice&,
            const CoasterOptionChoice&) = default;
    };

    struct CoasterOptionDefinition
    {
        std::string_view id;
        std::string_view label;
        CoasterOptionKind kind = CoasterOptionKind::Boolean;
        CoasterCapability requiredCapability = CoasterCapability::None;
        bool booleanDefault = false;
        std::string_view defaultChoiceId;
        std::vector<CoasterOptionChoice> choices;

        [[nodiscard]] friend bool operator==(
            const CoasterOptionDefinition&,
            const CoasterOptionDefinition&) = default;
    };

    // Per-style override of a globally defined option's default.
    struct CoasterStyleOptionDefault
    {
        std::string_view optionId;
        bool booleanValue = false;
        std::string_view choiceValue;

        [[nodiscard]] friend bool operator==(
            const CoasterStyleOptionDefault&,
            const CoasterStyleOptionDefault&) = default;
    };

    struct CoasterStyleDefinition
    {
        std::string_view id;
        std::string_view displayName;
        CoasterCapability capabilities = CoasterCapability::None;
        std::vector<CoasterStyleOptionDefault> optionDefaults;

        [[nodiscard]] friend bool operator==(
            const CoasterStyleDefinition&,
            const CoasterStyleDefinition&) = default;
    };

    inline constexpr std::string_view defaultCoasterStyleId =
        "placeholder-style-1";

    inline constexpr std::string_view trainLayoutOptionId = "train-layout";
    inline constexpr std::string_view launchOptionId = "launch";
    inline constexpr std::string_view restraintOptionId = "restraint";

    [[nodiscard]] std::span<const CoasterStyleDefinition>
    coasterStyleCatalog() noexcept;

    [[nodiscard]] std::span<const CoasterOptionDefinition>
    coasterOptionCatalog() noexcept;

    // Returns nullptr when the id is unknown.
    [[nodiscard]] const CoasterStyleDefinition* findCoasterStyle(
        std::string_view styleId) noexcept;

    [[nodiscard]] const CoasterOptionDefinition* findCoasterOption(
        std::string_view optionId) noexcept;

    [[nodiscard]] const CoasterOptionChoice* findCoasterOptionChoice(
        const CoasterOptionDefinition& option,
        std::string_view choiceId) noexcept;

    // Options a style exposes, in catalog order (see hasCapability).
    [[nodiscard]] std::vector<const CoasterOptionDefinition*>
    applicableOptionsForStyle(const CoasterStyleDefinition& style);

    // Heartline settings are authored document configuration. offsetMeters is
    // a non-negative distance from the solved track centerline along the
    // sample frame's local +up axis. The broad upper bound catches corrupt
    // input without asserting a final product-specific engineering limit.
    inline constexpr double minimumHeartlineOffsetMeters = 0.0;
    inline constexpr double maximumHeartlineOffsetMeters = 10.0;

    struct HeartlineSettings
    {
        bool enabled = true;
        double offsetMeters = 1.4;

        [[nodiscard]] friend bool operator==(
            const HeartlineSettings&,
            const HeartlineSettings&) = default;
    };

    // One authored value for an option. Boolean options carry booleanValue;
    // Choice options carry choiceValue. Validation enforces that the carried
    // field matches the option kind.
    struct CoasterOptionValue
    {
        std::string optionId;
        bool booleanValue = false;
        std::string choiceValue;

        [[nodiscard]] friend bool operator==(
            const CoasterOptionValue&,
            const CoasterOptionValue&) = default;
    };

    // The authored document configuration. A valid setup contains exactly one
    // value for every option applicable to styleId, in catalog order.
    struct CoasterSetup
    {
        std::string styleId = std::string(defaultCoasterStyleId);
        std::uint32_t carsPerTrain = 4;
        std::vector<CoasterOptionValue> options;
        HeartlineSettings heartline;

        [[nodiscard]] friend bool operator==(
            const CoasterSetup&,
            const CoasterSetup&) = default;
    };

    inline constexpr std::uint32_t minimumCarsPerTrain = 1;
    inline constexpr std::uint32_t maximumCarsPerTrain = 32;

    // Complete valid setup for a style, using the style's option defaults
    // (falling back to the global option defaults) and the canonical heartline
    // and train defaults. Throws std::invalid_argument for an unknown style id.
    [[nodiscard]] CoasterSetup createCoasterSetupForStyle(
        std::string_view styleId);

    // Lightweight UI summary of setup metadata. This is not a physical train
    // definition and is not consumed by SimulationPreview or TrainPhysics.
    struct CoasterSetupTrainSummary
    {
        std::optional<std::uint32_t> seatsAcross;
        std::optional<std::uint32_t> rowsPerCar;
        std::uint32_t carsPerTrain = 4;
        std::optional<std::string> restraintOptionId;

        [[nodiscard]] friend bool operator==(
            const CoasterSetupTrainSummary&,
            const CoasterSetupTrainSummary&) = default;
    };

    [[nodiscard]] CoasterSetupTrainSummary trainSummaryForCoasterSetup(
        const CoasterSetup& setup);

    // Validates a complete setup. Throws std::invalid_argument for an unknown
    // style or option, an option the style does not expose, a wrong field kind,
    // an unknown choice value, a missing or duplicate value for an applicable
    // option, an out-of-range carsPerTrain, or a non-finite/out-of-range
    // heartline offset.
    void validateCoasterSetup(const CoasterSetup& setup);
}
