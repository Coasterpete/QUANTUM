#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/CoasterSetup.hpp>
#include <quantum/editor/DocumentHistory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <expected>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::applicableOptionsForStyle;
    using quantum::coaster::CoasterCapability;
    using quantum::coaster::CoasterOptionDefinition;
    using quantum::coaster::coasterOptionCatalog;
    using quantum::coaster::CoasterOptionKind;
    using quantum::coaster::CoasterOptionValue;
    using quantum::coaster::CoasterSetup;
    using quantum::coaster::CoasterStyleDefinition;
    using quantum::coaster::coasterStyleCatalog;
    using quantum::coaster::createCoasterSetupForStyle;
    using quantum::coaster::defaultCoasterStyleId;
    using quantum::coaster::deserializeCoasterDocument;
    using quantum::coaster::findCoasterOption;
    using quantum::coaster::findCoasterOptionChoice;
    using quantum::coaster::findCoasterStyle;
    using quantum::coaster::hasCapability;
    using quantum::coaster::launchOptionId;
    using quantum::coaster::maximumCarsPerTrain;
    using quantum::coaster::maximumHeartlineOffsetMeters;
    using quantum::coaster::minimumCarsPerTrain;
    using quantum::coaster::minimumHeartlineOffsetMeters;
    using quantum::coaster::restraintOptionId;
    using quantum::coaster::serializeCoasterDocument;
    using quantum::coaster::trainSummaryForCoasterSetup;
    using quantum::coaster::trainLayoutOptionId;
    using quantum::coaster::validateCoasterSetup;
    using quantum::editor::DocumentHistory;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(
        const bool condition,
        const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename T>
    void requireValidDocument(
        const std::expected<T, std::string>& result,
        const std::string_view context)
    {
        if (!result.has_value())
        {
            throw TestFailure(
                std::string(context) + ": "
                + result.error());
        }
    }

    void requireThrowsInvalidArgument(
        const std::function<void()>& function,
        const std::string_view message)
    {
        try
        {
            function();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        catch (const std::exception& exception)
        {
            throw TestFailure(
                std::string(message) + ": wrong exception type: "
                + exception.what());
        }
        throw TestFailure(
            std::string(message) + ": expected std::invalid_argument");
    }

    [[nodiscard]] CoasterOptionValue optionValue(
        const CoasterSetup& setup,
        const std::string_view optionId)
    {
        for (const CoasterOptionValue& value : setup.options)
        {
            if (value.optionId == optionId)
            {
                return value;
            }
        }
        throw TestFailure(
            "setup missing option '" + std::string(optionId) + "'");
    }

    // Removes a top-level object field from the compact deterministic JSON
    // produced by serializeCoasterDocument, emulating a document written by a
    // version of the app that did not know the field yet.
    std::string removeRootField(
        const std::string& document,
        const std::string& key)
    {
        std::string output = document;
        const std::string needle = "\"" + key + "\":";
        const std::size_t keyPosition = output.find(needle);
        require(keyPosition != std::string::npos,
            "root field '" + key + "' not present");

        const std::size_t valueStart = output.find('{', keyPosition);
        require(valueStart != std::string::npos,
            "root field '" + key + "' value is not an object");

        int depth = 0;
        std::size_t valueEnd = valueStart;
        for (; valueEnd < output.size(); ++valueEnd)
        {
            if (output[valueEnd] == '{')
            {
                ++depth;
            }
            else if (output[valueEnd] == '}')
            {
                --depth;
                if (depth == 0)
                {
                    break;
                }
            }
        }
        require(depth == 0 && valueEnd < output.size(),
            "root field '" + key + "' value is unbalanced");

        // Scan back to the field's start: either it is preceded by the
        // object's opening brace (first field) or by a field separator.
        std::size_t begin = keyPosition;
        while (begin > 0 && output[begin - 1] != ','
            && output[begin - 1] != '{')
        {
            --begin;
        }
        bool leadingComma = false;
        if (begin > 0 && output[begin - 1] == ',')
        {
            leadingComma = true;
            --begin;
        }

        std::size_t end = valueEnd;
        if (!leadingComma && end + 1 < output.size()
            && output[end + 1] == ',')
        {
            // The field was the first in the object; drop the following
            // separator so the object does not end up with a stray comma.
            ++end;
        }

        output.erase(begin, end - begin + 1);
        return output;
    }

    // ================================================================
    // 1. Catalog consistency
    // ================================================================

    void catalogConsistency()
    {
        const auto styles = coasterStyleCatalog();
        const auto options = coasterOptionCatalog();
        require(styles.size() >= 5,
            "placeholder milestone must provide its five test styles");
        require(!options.empty(), "option catalog must not be empty");

        for (const CoasterStyleDefinition& style : styles)
        {
            require(!style.id.empty(), "style id must not be empty");
            require(!style.displayName.empty(),
                "style display name must not be empty");
            require(findCoasterStyle(style.id) == &style,
                "catalog and lookup must agree");
            require(std::count_if(styles.begin(), styles.end(),
                    [&style](const CoasterStyleDefinition& candidate)
                    {
                        return candidate.id == style.id;
                    }) == 1,
                "style ids must be unique");
        }

        for (const std::string_view placeholderId : {
                "placeholder-style-1", "placeholder-style-2",
                "placeholder-style-3", "placeholder-style-4", "custom"})
        {
            const CoasterStyleDefinition* placeholder =
                findCoasterStyle(placeholderId);
            require(placeholder != nullptr,
                "required neutral placeholder style must exist");
            require(placeholder->displayName.find("Placeholder")
                    != std::string_view::npos,
                "placeholder labels must be visibly provisional");
        }

        for (const CoasterOptionDefinition& option : options)
        {
            require(!option.id.empty(), "option id must not be empty");
            require(!option.label.empty(), "option label must not be empty");
            require(findCoasterOption(option.id) == &option,
                "option catalog and lookup must agree");
            require(std::count_if(options.begin(), options.end(),
                    [&option](const CoasterOptionDefinition& candidate)
                    {
                        return candidate.id == option.id;
                    }) == 1,
                "option ids must be unique");

            if (option.kind == CoasterOptionKind::Choice)
            {
                require(!option.choices.empty(),
                    "choice option '"
                    + std::string(option.id) + "' needs choices");
                bool defaultKnown = false;
                for (const auto& choice : option.choices)
                {
                    require(findCoasterOptionChoice(option, choice.id)
                            == &choice,
                        "choice lookup must agree for '"
                        + std::string(choice.id) + "'");
                    defaultKnown = defaultKnown
                        || choice.id == option.defaultChoiceId;
                    require(std::count_if(
                            option.choices.begin(),
                            option.choices.end(),
                            [&choice](const auto& candidate)
                            {
                                return candidate.id == choice.id;
                            }) == 1,
                        "choice ids must be unique within an option");
                }
                require(defaultKnown,
                    "default choice must exist for '"
                    + std::string(option.id) + "'");
            }
        }
    }

    void placeholderCapabilitiesExerciseDynamicOptions()
    {
        const auto optionCount = [](const std::string_view styleId)
        {
            const CoasterStyleDefinition* style = findCoasterStyle(styleId);
            require(style != nullptr, "placeholder style must exist");
            return applicableOptionsForStyle(*style).size();
        };
        require(optionCount("placeholder-style-1") == 1,
            "style 1 must exercise train-layout only");
        require(optionCount("placeholder-style-2") == 1,
            "style 2 must exercise launch only");
        require(optionCount("placeholder-style-3") == 1,
            "style 3 must exercise restraints only");
        require(optionCount("placeholder-style-4") == 3,
            "style 4 must exercise every capability");
        require(optionCount("custom") == 3,
            "custom must exercise every capability");

        require(quantum::coaster::coasterCapabilitySummary(
                    CoasterCapability::TrainLayout
                        | CoasterCapability::Launch
                        | CoasterCapability::Restraints)
                == "launch, train layout, restraints",
            "combined capabilities must all be visible in the UI summary");
    }

    void catalogDefaultsAreValid()
    {
        for (const CoasterStyleDefinition& style : coasterStyleCatalog())
        {
            // Every style can produce a fully valid setup.
            const CoasterSetup setup =
                createCoasterSetupForStyle(style.id);
            require(setup.styleId == style.id,
                "produced setup must use its own style id");
            validateCoasterSetup(setup);

            // Style defaults must reference options the style exposes and
            // match the option kind and available choices.
            for (const auto& optionDefault : style.optionDefaults)
            {
                const CoasterOptionDefinition* option =
                    findCoasterOption(optionDefault.optionId);
                require(option != nullptr,
                    "style default references an unknown option");
                require(hasCapability(style.capabilities,
                        option->requiredCapability),
                    "style default references an unexposed option");
                require(std::count_if(
                        style.optionDefaults.begin(),
                        style.optionDefaults.end(),
                        [&optionDefault](const auto& candidate)
                        {
                            return candidate.optionId
                                == optionDefault.optionId;
                        }) == 1,
                    "a style must not define an option default twice");
                if (option->kind == CoasterOptionKind::Choice)
                {
                    require(findCoasterOptionChoice(*option,
                            optionDefault.choiceValue) != nullptr,
                        "style default references an unknown choice");
                }
            }
        }
    }

    // ================================================================
    // 2. Defaults
    // ================================================================

    void defaultSetupMatchesAuthoredTrack()
    {
        const AuthoredTrack track;
        const CoasterSetup& setup = track.coasterSetup();
        require(setup.styleId == defaultCoasterStyleId,
            "new documents must default to the canonical style");
        const CoasterSetup canonical =
            createCoasterSetupForStyle(defaultCoasterStyleId);
        require(canonical == setup,
            "AuthoredTrack default setup must equal the catalog default");

        require(setup.carsPerTrain == 4,
            "default cars per train must be 4");
        require(setup.options.size() == 1,
            "placeholder style 1 must expose train-layout only");
        require(optionValue(setup, trainLayoutOptionId).choiceValue
                == "layout-1",
            "placeholder style 1 must use layout option 1");
        require(setup.heartline.enabled,
            "heartline must default to enabled");
        require(setup.heartline.offsetMeters == 1.4,
            "heartline must default to the 1.4 m reference offset");
    }

    void perStyleOptionDefaults()
    {
        const CoasterSetup styleFour = createCoasterSetupForStyle(
            "placeholder-style-4");
        require(optionValue(styleFour, trainLayoutOptionId).choiceValue
                == "layout-2",
            "placeholder style 4 must exercise a per-style default");
        require(optionValue(styleFour, launchOptionId).booleanValue == false,
            "launch must default disabled where exposed");
        require(optionValue(styleFour, restraintOptionId).choiceValue
                == "restraint-option-1",
            "restraint must use neutral option 1 by default");

        const CoasterSetup custom = createCoasterSetupForStyle("custom");
        require(optionValue(custom, launchOptionId).booleanValue == false,
            "custom must default launch to disabled");
        require(optionValue(custom, restraintOptionId).choiceValue
                == "restraint-option-1",
            "custom must default to neutral restraint option 1");
    }

    void setupTrainSummaryIsMetadataOnly()
    {
        CoasterSetup setup = createCoasterSetupForStyle("custom");
        auto train = trainSummaryForCoasterSetup(setup);
        require(train.seatsAcross == 2 && train.rowsPerCar == 1,
            "layout option 1 must summarize as 2-across and 1-row");
        require(train.restraintOptionId == "restraint-option-1",
            "neutral restraint option id must survive the summary");

        for (auto& value : setup.options)
        {
            if (value.optionId == trainLayoutOptionId)
            {
                value.choiceValue = "layout-3";
            }
            else if (value.optionId == restraintOptionId)
            {
                value.choiceValue = "restraint-option-2";
            }
        }
        train = trainSummaryForCoasterSetup(setup);
        require(train.seatsAcross == 3 && train.rowsPerCar == 1,
            "layout option 3 must summarize as 3-across and 1-row");
        require(train.restraintOptionId == "restraint-option-2",
            "restraint option must survive summary derivation");

        setup.carsPerTrain = 7;
        require(trainSummaryForCoasterSetup(setup).carsPerTrain == 7,
            "carsPerTrain must pass through derivation");

        const auto launchOnly = trainSummaryForCoasterSetup(
            createCoasterSetupForStyle("placeholder-style-2"));
        require(!launchOnly.seatsAcross.has_value()
                && !launchOnly.rowsPerCar.has_value()
                && !launchOnly.restraintOptionId.has_value(),
            "inapplicable train metadata must not receive plausible defaults");
    }

    void unknownStyleRejected()
    {
        requireThrowsInvalidArgument(
            [] { (void)createCoasterSetupForStyle("no-such-style"); },
            "createCoasterSetupForStyle must reject unknown styles");

        CoasterSetup setup;
        setup.styleId = "no-such-style";
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "validateCoasterSetup must reject unknown styles");
    }

    // ================================================================
    // 3. Validation
    // ================================================================

    CoasterSetup validAllCapabilitiesSetup()
    {
        return createCoasterSetupForStyle("placeholder-style-4");
    }

    void carsRangeValidation()
    {
        CoasterSetup setup = validAllCapabilitiesSetup();
        setup.carsPerTrain = 0;
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "zero cars must be rejected");
        setup.carsPerTrain = maximumCarsPerTrain + 1;
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "excessive cars must be rejected");
        setup.carsPerTrain = 1;
        validateCoasterSetup(setup);
        setup.carsPerTrain = maximumCarsPerTrain;
        validateCoasterSetup(setup);
    }

    void heartlineValidation()
    {
        CoasterSetup setup = validAllCapabilitiesSetup();
        setup.heartline.offsetMeters = minimumHeartlineOffsetMeters - 0.1;
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "negative heartline offset must be rejected");
        setup.heartline.offsetMeters = maximumHeartlineOffsetMeters + 0.1;
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "excessive heartline offset must be rejected");
        setup.heartline.offsetMeters =
            std::numeric_limits<double>::quiet_NaN();
        requireThrowsInvalidArgument(
            [&] { validateCoasterSetup(setup); },
            "non-finite heartline offset must be rejected");
    }

    void optionValidation()
    {
        // Unknown option id.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            setup.options.push_back(CoasterOptionValue{
                "no-such-option", false, ""});
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "an unknown option must be rejected");
        }

        // Option the style does not expose.
        {
            AuthoredTrack track;
            CoasterSetup setup = track.coasterSetup();
            setup.options.push_back(
                CoasterOptionValue{std::string(launchOptionId), true, ""});
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "placeholder style 1 must not expose the launch option");
        }

        // Boolean option carrying a choice value.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            for (auto& value : setup.options)
            {
                if (value.optionId == launchOptionId)
                {
                    value.choiceValue = "yes";
                }
            }
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "a boolean option carrying a choice must be rejected");
        }

        // Unknown choice value.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            for (auto& value : setup.options)
            {
                if (value.optionId == trainLayoutOptionId)
                {
                    value.choiceValue = "no-such-seating";
                }
            }
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "an unknown choice must be rejected");
        }

        // Missing value for an applicable option.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            setup.options.erase(
                std::remove_if(setup.options.begin(), setup.options.end(),
                    [](const CoasterOptionValue& value)
                    {
                        return value.optionId == restraintOptionId;
                    }),
                setup.options.end());
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "a missing applicable option must be rejected");
        }

        // Duplicate value for an applicable option.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            setup.options.push_back(
                CoasterOptionValue{std::string(trainLayoutOptionId), false,
                    "layout-2"});
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "a duplicate applicable option must be rejected");
        }

        // Choice option carrying the boolean field's non-default value.
        {
            CoasterSetup setup = validAllCapabilitiesSetup();
            for (auto& value : setup.options)
            {
                if (value.optionId == trainLayoutOptionId)
                {
                    value.booleanValue = true;
                }
            }
            requireThrowsInvalidArgument(
                [&] { validateCoasterSetup(setup); },
                "a choice option carrying a boolean must be rejected");
        }
    }

    void setCoasterSetupStrongGuarantee()
    {
        AuthoredTrack track;
        const CoasterSetup original = track.coasterSetup();

        CoasterSetup invalid = original;
        invalid.styleId = "no-such-style";
        requireThrowsInvalidArgument(
            [&] { track.setCoasterSetup(invalid); },
            "setCoasterSetup must reject invalid setups");
        require(track.coasterSetup() == original,
            "a rejected edit must leave the authored setup unchanged");

        const CoasterSetup edit = createCoasterSetupForStyle("custom");
        track.setCoasterSetup(edit);
        require(track.coasterSetup() == edit,
            "setCoasterSetup must apply valid edits");
    }

    // ================================================================
    // 4. Serialization
    // ================================================================

    void defaultSerializationRoundTrip()
    {
        const AuthoredTrack original =
            quantum::coaster::createDefaultAuthoredTrack();
        const std::string json = serializeCoasterDocument(original);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "default setup round-trip");
        require((*result).coasterSetup() == original.coasterSetup(),
            "serialize/deserialize must preserve the coaster setup");
        require(serializeCoasterDocument(*result) == json,
            "coaster setup serialization must be deterministic");
    }

    void customSerializationRoundTrip()
    {
        AuthoredTrack track =
            quantum::coaster::createDefaultAuthoredTrack();

        CoasterSetup setup = createCoasterSetupForStyle(
            "placeholder-style-4");
        setup.carsPerTrain = 6;
        for (auto& value : setup.options)
        {
            if (value.optionId == trainLayoutOptionId)
            {
                value.choiceValue = "layout-3";
            }
            else if (value.optionId == launchOptionId)
            {
                value.booleanValue = true;
            }
            else if (value.optionId == restraintOptionId)
            {
                value.choiceValue = "restraint-option-2";
            }
        }
        setup.heartline.enabled = false;
        setup.heartline.offsetMeters = 0.6;

        track.setCoasterSetup(setup);
        const std::string json = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "custom setup round-trip");
        require((*result).coasterSetup() == track.coasterSetup(),
            "custom setup must survive serialize/deserialize");
        require(serializeCoasterDocument(*result) == json,
            "custom setup serialization must be deterministic");
    }

    void stableIdsAreIndependentFromLabels()
    {
        AuthoredTrack track = quantum::coaster::createDefaultAuthoredTrack();
        const CoasterStyleDefinition* style =
            findCoasterStyle(track.coasterSetup().styleId);
        require(style != nullptr, "default style must exist");
        require(style->id != style->displayName,
            "stable style id must be independent from its display label");

        const std::string json = serializeCoasterDocument(track);
        require(json.find(std::string(style->id)) != std::string::npos,
            "serialized setup must contain the stable style id");
        require(json.find(std::string(style->displayName)) == std::string::npos,
            "serialized setup must not contain the style display label");

        const CoasterOptionDefinition* layout =
            findCoasterOption(trainLayoutOptionId);
        require(layout != nullptr && !layout->choices.empty(),
            "train layout option must have placeholder choices");
        for (const auto& choice : layout->choices)
        {
            require(json.find(std::string(choice.label)) == std::string::npos,
                "serialized setup must not contain option choice labels");
        }
    }

    void obsoletePersistedIdsAreRejected()
    {
        const AuthoredTrack track =
            quantum::coaster::createDefaultAuthoredTrack();
        const std::string json = serializeCoasterDocument(track);

        std::string unknownStyle = json;
        const std::size_t stylePosition = unknownStyle.find(
            std::string(defaultCoasterStyleId));
        require(stylePosition != std::string::npos,
            "serialized default style id must be present");
        unknownStyle.replace(
            stylePosition,
            defaultCoasterStyleId.size(),
            "obsolete-style-id");
        require(!deserializeCoasterDocument(unknownStyle).has_value(),
            "an obsolete persisted style id must be rejected safely");

        std::string unknownOption = json;
        const std::size_t optionPosition = unknownOption.find(
            std::string(trainLayoutOptionId));
        require(optionPosition != std::string::npos,
            "serialized default option id must be present");
        unknownOption.replace(
            optionPosition,
            trainLayoutOptionId.size(),
            "obsolete-option-id");
        require(!deserializeCoasterDocument(unknownOption).has_value(),
            "an obsolete persisted option id must be rejected safely");
    }

    void legacyDocumentKeepsDefaults()
    {
        const AuthoredTrack original =
            quantum::coaster::createDefaultAuthoredTrack();
        const std::string json = serializeCoasterDocument(original);
        require(json.find("\"coasterSetup\":") != std::string::npos,
            "new document must carry the coasterSetup field");

        const std::string legacyJson = removeRootField(json, "coasterSetup");
        auto result = deserializeCoasterDocument(legacyJson);
        requireValidDocument(result, "legacy document");
        require((*result).coasterSetup() == original.coasterSetup(),
            "a document without coasterSetup must fall back to defaults");
    }

    // ================================================================
    // 5. Document history integration
    // ================================================================

    [[nodiscard]] std::string snapshot(const AuthoredTrack& track)
    {
        return serializeCoasterDocument(track);
    }

    [[nodiscard]] AuthoredTrack requireState(
        std::optional<AuthoredTrack> state,
        const std::string_view message)
    {
        require(state.has_value(), message);
        return std::move(*state);
    }

    void undoRedoIntegration()
    {
        AuthoredTrack track = quantum::coaster::createNewDocument();
        DocumentHistory history;
        history.reset(track);
        require(!history.isDirty(),
            "a reset document history must start clean");
        const std::string baseline = snapshot(track);

        track.setCoasterSetup(createCoasterSetupForStyle("custom"));
        history.record(track);
        const std::string edited = snapshot(track);
        require(edited != baseline, "setup edit must change the document");
        require(history.isDirty(), "a setup edit must mark history dirty");

        require(history.canUndo() && !history.canRedo(),
            "a setup edit enables Undo only");
        require(snapshot(requireState(history.undo(), "Undo state missing"))
                == baseline,
            "Undo must restore the exact baseline document");
        require(!history.isDirty(),
            "Undo to the saved baseline must clear dirty state");
        require(snapshot(requireState(history.redo(), "Redo state missing"))
                == edited,
            "Redo must restore the exact edited document");
        require(history.isDirty(), "Redo must restore the dirty revision");
        history.markSaved();
        require(!history.isDirty(), "Save must clear dirty state");
    }

    // ================================================================
    // Test runner
    // ================================================================

    struct Test
    {
        std::string_view name;
        std::function<void()> function;
    };

    int runTests()
    {
        const std::vector<Test> tests = {
            {"CatalogConsistency",              catalogConsistency},
            {"PlaceholderCapabilities",        placeholderCapabilitiesExerciseDynamicOptions},
            {"CatalogDefaultsAreValid",         catalogDefaultsAreValid},
            {"DefaultSetupMatchesAuthoredTrack", defaultSetupMatchesAuthoredTrack},
            {"PerStyleOptionDefaults",          perStyleOptionDefaults},
            {"SetupTrainSummaryMetadata",       setupTrainSummaryIsMetadataOnly},
            {"UnknownStyleRejected",            unknownStyleRejected},
            {"CarsRangeValidation",             carsRangeValidation},
            {"HeartlineValidation",             heartlineValidation},
            {"OptionValidation",                optionValidation},
            {"SetCoasterSetupStrongGuarantee",  setCoasterSetupStrongGuarantee},
            {"DefaultSerializationRoundTrip",   defaultSerializationRoundTrip},
            {"CustomSerializationRoundTrip",    customSerializationRoundTrip},
            {"StableIdsIndependentFromLabels", stableIdsAreIndependentFromLabels},
            {"ObsoletePersistedIdsRejected",   obsoletePersistedIdsAreRejected},
            {"LegacyDocumentKeepsDefaults",     legacyDocumentKeepsDefaults},
            {"UndoRedoIntegration",             undoRedoIntegration},
        };

        std::size_t failures = 0;

        for (const auto& test : tests)
        {
            try
            {
                test.function();
                std::cout << "[PASS] " << test.name << '\n';
            }
            catch (const std::exception& exception)
            {
                ++failures;
                std::cerr << "[FAIL] " << test.name << ": "
                          << exception.what() << '\n';
            }
            catch (...)
            {
                ++failures;
                std::cerr << "[FAIL] " << test.name
                          << ": unknown exception\n";
            }
        }

        if (failures != 0)
        {
            std::cerr << failures << " test group(s) failed.\n";
            return 1;
        }

        std::cout << std::size(tests) << " test groups passed.\n";
        return 0;
    }
}

int main()
{
    return runTests();
}
