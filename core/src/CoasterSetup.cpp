#include <quantum/coaster/CoasterSetup.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace quantum::coaster
{
    namespace
    {
        std::span<const CoasterStyleDefinition> styleCatalog() noexcept
        {
            // The catalog is the single source of style knowledge. Adding a
            // style here automatically drives the style selector, capability
            // gating, defaults, validation, and serialization.
            static const std::array<CoasterStyleDefinition, 5> styles = {
                CoasterStyleDefinition{
                    "placeholder-style-1", "Coaster Style 1 (Placeholder)",
                    CoasterCapability::TrainLayout, {}},
                CoasterStyleDefinition{
                    "placeholder-style-2", "Coaster Style 2 (Placeholder)",
                    CoasterCapability::Launch, {}},
                CoasterStyleDefinition{
                    "placeholder-style-3", "Coaster Style 3 (Placeholder)",
                    CoasterCapability::Restraints, {}},
                CoasterStyleDefinition{
                    "placeholder-style-4", "Coaster Style 4 (Placeholder)",
                    CoasterCapability::TrainLayout | CoasterCapability::Launch
                        | CoasterCapability::Restraints,
                    {{"train-layout", false, "layout-2"}}},
                CoasterStyleDefinition{
                    "custom", "Custom (Placeholder)",
                    CoasterCapability::TrainLayout | CoasterCapability::Launch
                        | CoasterCapability::Restraints,
                    {}}};
            return styles;
        }

        std::span<const CoasterOptionDefinition> optionCatalog() noexcept
        {
            static const std::array<CoasterOptionDefinition, 3> options = {
                CoasterOptionDefinition{
                    "train-layout", "Train Layout",
                    CoasterOptionKind::Choice,
                    CoasterCapability::TrainLayout,
                    false, "layout-1",
                    {{"layout-1", "Layout 1 (2 across, 1 row)", 2, 1},
                     {"layout-2", "Layout 2 (2 across, 3 rows)", 2, 3},
                     {"layout-3", "Layout 3 (3 across, 1 row)", 3, 1}}},
                CoasterOptionDefinition{
                    "launch", "Launch option enabled",
                    CoasterOptionKind::Boolean,
                    CoasterCapability::Launch, false, "",
                    {}},
                CoasterOptionDefinition{
                    "restraint", "Restraint Option",
                    CoasterOptionKind::Choice,
                    CoasterCapability::Restraints, false,
                    "restraint-option-1",
                    {{"restraint-option-1", "Restraint option 1", 0, 0},
                     {"restraint-option-2", "Restraint option 2", 0, 0},
                     {"restraint-option-3", "Restraint option 3", 0, 0}}}};
            return options;
        }
    }

    std::string coasterCapabilitySummary(
        const CoasterCapability capabilities)
    {
        if (capabilities == CoasterCapability::None)
        {
            return "none";
        }

        std::string result;
        const auto append = [&](const CoasterCapability capability,
                                const std::string_view name)
        {
            if (!hasCapability(capabilities, capability))
            {
                return;
            }
            if (!result.empty())
            {
                result += ", ";
            }
            result += name;
        };
        append(CoasterCapability::Launch, "launch");
        append(CoasterCapability::TrainLayout, "train layout");
        append(CoasterCapability::Restraints, "restraints");
        return result;
    }

    std::span<const CoasterStyleDefinition>
    coasterStyleCatalog() noexcept
    {
        return styleCatalog();
    }

    std::span<const CoasterOptionDefinition>
    coasterOptionCatalog() noexcept
    {
        return optionCatalog();
    }

    const CoasterStyleDefinition* findCoasterStyle(
        const std::string_view styleId) noexcept
    {
        for (const CoasterStyleDefinition& style : styleCatalog())
        {
            if (style.id == styleId)
            {
                return &style;
            }
        }
        return nullptr;
    }

    const CoasterOptionDefinition* findCoasterOption(
        const std::string_view optionId) noexcept
    {
        for (const CoasterOptionDefinition& option : optionCatalog())
        {
            if (option.id == optionId)
            {
                return &option;
            }
        }
        return nullptr;
    }

    const CoasterOptionChoice* findCoasterOptionChoice(
        const CoasterOptionDefinition& option,
        const std::string_view choiceId) noexcept
    {
        for (const CoasterOptionChoice& choice : option.choices)
        {
            if (choice.id == choiceId)
            {
                return &choice;
            }
        }
        return nullptr;
    }

    std::vector<const CoasterOptionDefinition*> applicableOptionsForStyle(
        const CoasterStyleDefinition& style)
    {
        std::vector<const CoasterOptionDefinition*> result;
        for (const CoasterOptionDefinition& option : optionCatalog())
        {
            if (hasCapability(style.capabilities, option.requiredCapability))
            {
                result.push_back(&option);
            }
        }
        return result;
    }

    CoasterSetup createCoasterSetupForStyle(const std::string_view styleId)
    {
        const CoasterStyleDefinition* style = findCoasterStyle(styleId);
        if (style == nullptr)
        {
            throw std::invalid_argument(
                "Unknown coaster style '" + std::string(styleId) + "'.");
        }

        CoasterSetup setup;
        setup.styleId = std::string(styleId);

        for (const CoasterOptionDefinition* option :
            applicableOptionsForStyle(*style))
        {
            CoasterOptionValue value;
            value.optionId = std::string(option->id);

            const CoasterStyleOptionDefault* overrideValue = nullptr;
            for (const CoasterStyleOptionDefault& candidate :
                style->optionDefaults)
            {
                if (candidate.optionId == option->id)
                {
                    overrideValue = &candidate;
                    break;
                }
            }

            if (option->kind == CoasterOptionKind::Boolean)
            {
                value.booleanValue = overrideValue != nullptr
                    ? overrideValue->booleanValue
                    : option->booleanDefault;
            }
            else
            {
                value.choiceValue = std::string(
                    overrideValue != nullptr
                        ? overrideValue->choiceValue
                        : option->defaultChoiceId);
            }

            setup.options.push_back(std::move(value));
        }

        return setup;
    }

    CoasterSetupTrainSummary trainSummaryForCoasterSetup(
        const CoasterSetup& setup)
    {
        CoasterSetupTrainSummary train;
        train.carsPerTrain = setup.carsPerTrain;

        for (const CoasterOptionValue& value : setup.options)
        {
            if (value.optionId == trainLayoutOptionId)
            {
                const CoasterOptionDefinition* option =
                    findCoasterOption(value.optionId);
                if (option != nullptr)
                {
                    const CoasterOptionChoice* choice =
                        findCoasterOptionChoice(*option, value.choiceValue);
                    if (choice != nullptr)
                    {
                        train.seatsAcross = choice->seatsAcross;
                        train.rowsPerCar = choice->rowsPerCar;
                    }
                }
            }
            else if (value.optionId == restraintOptionId)
            {
                train.restraintOptionId = value.choiceValue;
            }
        }

        return train;
    }

    void validateCoasterSetup(const CoasterSetup& setup)
    {
        const CoasterStyleDefinition* style = findCoasterStyle(setup.styleId);
        if (style == nullptr)
        {
            throw std::invalid_argument(
                "Unknown coaster style '" + setup.styleId + "'.");
        }

        if (setup.carsPerTrain < minimumCarsPerTrain
            || setup.carsPerTrain > maximumCarsPerTrain)
        {
            throw std::invalid_argument(
                "Cars per train must be within ["
                + std::to_string(minimumCarsPerTrain) + ", "
                + std::to_string(maximumCarsPerTrain) + "].");
        }

        if (!std::isfinite(setup.heartline.offsetMeters)
            || setup.heartline.offsetMeters < minimumHeartlineOffsetMeters
            || setup.heartline.offsetMeters > maximumHeartlineOffsetMeters)
        {
            throw std::invalid_argument(
                "Heartline offset must be finite and within ["
                + std::to_string(minimumHeartlineOffsetMeters) + ", "
                + std::to_string(maximumHeartlineOffsetMeters)
                + "] meters.");
        }

        const std::vector<const CoasterOptionDefinition*> applicable =
            applicableOptionsForStyle(*style);

        // Every value must name an option the style exposes and carry the
        // field matching that option's kind. Duplicate values are invalid.
        for (const CoasterOptionValue& value : setup.options)
        {
            const CoasterOptionDefinition* option =
                findCoasterOption(value.optionId);
            if (option == nullptr)
            {
                throw std::invalid_argument(
                    "Unknown coaster option '" + value.optionId + "'.");
            }
            if (!hasCapability(style->capabilities, option->requiredCapability))
            {
                throw std::invalid_argument(
                    "Style '" + setup.styleId
                    + "' does not expose option '" + value.optionId + "'.");
            }

            if (option->kind == CoasterOptionKind::Boolean)
            {
                if (!value.choiceValue.empty())
                {
                    throw std::invalid_argument(
                        "Boolean option '" + value.optionId
                        + "' must not carry a choice value.");
                }
            }
            else
            {
                if (value.booleanValue)
                {
                    throw std::invalid_argument(
                        "Choice option '" + value.optionId
                        + "' must not carry a boolean value.");
                }
                if (findCoasterOptionChoice(*option, value.choiceValue)
                    == nullptr)
                {
                    throw std::invalid_argument(
                        "Unknown choice '" + value.choiceValue
                        + "' for option '" + value.optionId + "'.");
                }
            }
        }

        // A valid setup carries exactly the options its style exposes.
        // Duplicates are caught by counting instead of the identity check.
        for (const CoasterOptionDefinition* option : applicable)
        {
            std::size_t count = 0;
            for (const CoasterOptionValue& value : setup.options)
            {
                if (value.optionId == option->id)
                {
                    ++count;
                }
            }
            if (count != 1)
            {
                throw std::invalid_argument(
                    "Coaster setup must carry exactly one value for option '"
                    + std::string(option->id) + "' (found "
                    + std::to_string(count) + ").");
            }
        }
    }
}
