#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <exception>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::computeLayoutStatus;
    using quantum::coaster::createNewDocument;
    using quantum::coaster::deserializeCoasterDocument;
    using quantum::coaster::LayoutMode;
    using quantum::coaster::LayoutStatus;
    using quantum::coaster::layoutModeFromString;
    using quantum::coaster::layoutModeToString;
    using quantum::coaster::layoutStatusLabel;
    using quantum::coaster::serializeCoasterDocument;
    using quantum::coaster::TopologyKind;

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
                std::string("Expected valid document for ")
                + std::string(context) + ": " + result.error());
        }
    }

    // ================================================================
    // 1. layoutModeToString round-trip
    // ================================================================

    void test1_toStringCircuit()
    {
        require(
            std::string(layoutModeToString(LayoutMode::Circuit))
                == "Circuit",
            "Circuit string");
    }

    void test2_toStringShuttle()
    {
        require(
            std::string(layoutModeToString(LayoutMode::Shuttle))
                == "Shuttle",
            "Shuttle string");
    }

    // ================================================================
    // 3. layoutModeFromString valid inputs
    // ================================================================

    void test3_fromStringCircuit()
    {
        require(
            layoutModeFromString("Circuit") == LayoutMode::Circuit,
            "fromString Circuit");
    }

    void test4_fromStringShuttle()
    {
        require(
            layoutModeFromString("Shuttle") == LayoutMode::Shuttle,
            "fromString Shuttle");
    }

    // ================================================================
    // 5. layoutModeFromString rejects unknown strings
    // ================================================================

    void test5_fromStringUnknownThrows()
    {
        bool threw = false;
        try
        {
            (void)layoutModeFromString("Bogus");
        }
        catch (const std::invalid_argument&)
        {
            threw = true;
        }
        require(threw, "Unknown string must throw");
    }

    // ================================================================
    // 6. Default document defaults to Circuit
    // ================================================================

    void test6_newDocumentDefaultsToCircuit()
    {
        const AuthoredTrack track = createNewDocument();
        require(
            track.layoutMode() == LayoutMode::Circuit,
            "New document must default to Circuit");
    }

    // ================================================================
    // 7. AuthoredTrack setLayoutMode / layoutMode getter
    // ================================================================

    void test7_setAndGetLayoutMode()
    {
        AuthoredTrack track;
        require(
            track.layoutMode() == LayoutMode::Circuit,
            "Default must be Circuit");

        track.setLayoutMode(LayoutMode::Shuttle);
        require(
            track.layoutMode() == LayoutMode::Shuttle,
            "After set, must be Shuttle");

        track.setLayoutMode(LayoutMode::Circuit);
        require(
            track.layoutMode() == LayoutMode::Circuit,
            "Can set back to Circuit");
    }

    // ================================================================
    // 8. Serialization includes layoutMode
    // ================================================================

    void test8_serializationIncludesLayoutMode()
    {
        AuthoredTrack track;
        const std::string json = serializeCoasterDocument(track);
        require(
            json.find("\"layoutMode\": \"Circuit\"") != std::string::npos,
            "Default serialization must include layoutMode Circuit");
    }

    void test9_shuttleSerializesCorrectly()
    {
        AuthoredTrack track;
        track.setLayoutMode(LayoutMode::Shuttle);
        const std::string json = serializeCoasterDocument(track);
        require(
            json.find("\"layoutMode\": \"Shuttle\"") != std::string::npos,
            "Shuttle serialization must include layoutMode Shuttle");
    }

    // ================================================================
    // 10. Round-trip preserves Circuit
    // ================================================================

    void test10_roundTripCircuit()
    {
        AuthoredTrack original = createNewDocument();
        original.setLayoutMode(LayoutMode::Circuit);

        const std::string json = serializeCoasterDocument(original);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "round-trip Circuit");

        require(
            result->layoutMode() == LayoutMode::Circuit,
            "Circuit must survive round-trip");

        const std::string json2 = serializeCoasterDocument(*result);
        require(
            json == json2,
            "Circuit round-trip must be deterministic");
    }

    // ================================================================
    // 11. Round-trip preserves Shuttle
    // ================================================================

    void test11_roundTripShuttle()
    {
        AuthoredTrack original = createNewDocument();
        original.setLayoutMode(LayoutMode::Shuttle);

        const std::string json = serializeCoasterDocument(original);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "round-trip Shuttle");

        require(
            result->layoutMode() == LayoutMode::Shuttle,
            "Shuttle must survive round-trip");

        const std::string json2 = serializeCoasterDocument(*result);
        require(
            json == json2,
            "Shuttle round-trip must be deterministic");
    }

    // ================================================================
    // 12. Backward compat: missing layoutMode defaults to Circuit
    // ================================================================

    void test12_backwardCompatMissingLayoutMode()
    {
        const std::string legacyJson =
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})";

        auto result = deserializeCoasterDocument(legacyJson);
        requireValidDocument(result, "backward compat");
        require(
            result->layoutMode() == LayoutMode::Circuit,
            "Missing layoutMode must default to Circuit");
    }

    // ================================================================
    // 13. Backward compat: legacy JSON without layoutMode
    //      re-serializes WITH layoutMode
    // ================================================================

    void test13_backwardCompatReserializesWithLayoutMode()
    {
        const std::string legacyJson =
            R"({"formatVersion": 1, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})";

        auto result = deserializeCoasterDocument(legacyJson);
        requireValidDocument(result, "backward compat re-serialize");

        const std::string reserialized =
            serializeCoasterDocument(*result);
        require(
            reserialized.find("\"layoutMode\": \"Circuit\"")
                != std::string::npos,
            "Re-serialized legacy doc must include layoutMode");
        require(
            reserialized.find("\"formatVersion\": 1")
                != std::string::npos,
            "formatVersion must remain 1");
    }

    // ================================================================
    // 14. Deserialization rejects invalid layoutMode string
    // ================================================================

    void test14_rejectsInvalidLayoutModeString()
    {
        const std::string badJson =
            R"({"formatVersion": 1, "layoutMode": "Bogus", "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})";

        auto result = deserializeCoasterDocument(badJson);
        require(
            !result.has_value(),
            "Invalid layoutMode string must be rejected");
    }

    // ================================================================
    // 15. Deserialization rejects wrong-type layoutMode
    // ================================================================

    void test15_rejectsWrongTypeLayoutMode()
    {
        const std::string badJson =
            R"({"formatVersion": 1, "layoutMode": 42, "sections": [{"kind": "RateProfiles", "length": 60.0, "rateProfiles": {"roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0.0, "domainEnd": 60.0, "valueBegin": 0.0, "valueEnd": 0.0, "type": "Linear"}}]}}}]})";

        auto result = deserializeCoasterDocument(badJson);
        require(
            !result.has_value(),
            "Wrong-type layoutMode must be rejected");
    }

    // ================================================================
    // 16. LayoutStatus: Circuit + OpenLinear = CircuitIncomplete
    // ================================================================

    void test16_layoutStatusCircuitIncomplete()
    {
        require(
            computeLayoutStatus(
                LayoutMode::Circuit, TopologyKind::OpenLinear)
                == LayoutStatus::CircuitIncomplete,
            "Circuit + OpenLinear must be CircuitIncomplete");
    }

    // ================================================================
    // 17. LayoutStatus: Circuit + ClosedCircuit = CircuitComplete
    // ================================================================

    void test17_layoutStatusCircuitComplete()
    {
        require(
            computeLayoutStatus(
                LayoutMode::Circuit, TopologyKind::ClosedCircuit)
                == LayoutStatus::CircuitComplete,
            "Circuit + ClosedCircuit must be CircuitComplete");
    }

    // ================================================================
    // 18. LayoutStatus: Shuttle + OpenLinear = ShuttleValid
    // ================================================================

    void test18_layoutStatusShuttleValidOpen()
    {
        require(
            computeLayoutStatus(
                LayoutMode::Shuttle, TopologyKind::OpenLinear)
                == LayoutStatus::ShuttleValid,
            "Shuttle + OpenLinear must be ShuttleValid");
    }

    // ================================================================
    // 19. LayoutStatus: Shuttle + ClosedCircuit = ShuttleValid
    // ================================================================

    void test19_layoutStatusShuttleValidClosed()
    {
        require(
            computeLayoutStatus(
                LayoutMode::Shuttle, TopologyKind::ClosedCircuit)
                == LayoutStatus::ShuttleValid,
            "Shuttle + ClosedCircuit must be ShuttleValid");
    }

    // ================================================================
    // 20. layoutStatusLabel returns non-empty strings
    // ================================================================

    void test20_layoutStatusLabels()
    {
        require(
            std::string(layoutStatusLabel(LayoutStatus::CircuitIncomplete))
                .find("Circuit") != std::string::npos,
            "CircuitIncomplete label must contain 'Circuit'");
        require(
            std::string(layoutStatusLabel(LayoutStatus::CircuitComplete))
                .find("Complete") != std::string::npos,
            "CircuitComplete label must contain 'Complete'");
        require(
            std::string(layoutStatusLabel(LayoutStatus::ShuttleValid))
                .find("Shuttle") != std::string::npos,
            "ShuttleValid label must contain 'Shuttle'");
    }

    // ================================================================
    // 21. LayoutMode mutation does not affect section data
    // ================================================================

    void test21_layoutModeDoesNotAffectSections()
    {
        AuthoredTrack track = createNewDocument();
        const std::size_t originalCount = track.sectionCount();

        track.setLayoutMode(LayoutMode::Shuttle);
        require(
            track.sectionCount() == originalCount,
            "Section count must not change when layout mode changes");

        track.setLayoutMode(LayoutMode::Circuit);
        require(
            track.sectionCount() == originalCount,
            "Section count must not change on second mode change");
    }

    // ================================================================
    // 22. Shuttle layout round-trip with geometry section
    // ================================================================

    void test22_shuttleWithGeometrySection()
    {
        AuthoredTrack track = createNewDocument();
        track.setLayoutMode(LayoutMode::Shuttle);

        const std::string json = serializeCoasterDocument(track);
        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "Shuttle round-trip");
        require(
            result->layoutMode() == LayoutMode::Shuttle,
            "Shuttle mode must survive round-trip with geometry");
    }

    // ================================================================
    // 23. Layout mode default value of AuthoredTrack constructor
    // ================================================================

    void test23_defaultConstructorLayoutMode()
    {
        AuthoredTrack track;
        require(
            track.layoutMode() == LayoutMode::Circuit,
            "Default-constructed AuthoredTrack must be Circuit");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    auto run = [&](const char* name, void (*fn)())
    {
        try
        {
            fn();
            ++passed;
            std::fprintf(stdout, "  PASS  %s\n", name);
        }
        catch (const TestFailure& e)
        {
            ++failed;
            std::fprintf(stderr, "  FAIL  %s: %s\n", name, e.what());
        }
        catch (const std::exception& e)
        {
            ++failed;
            std::fprintf(
                stderr,
                "  FAIL  %s (exception): %s\n",
                name,
                e.what());
        }
    };

    std::fprintf(stdout, "Layout Mode Tests\n");

    run("layoutModeToString returns 'Circuit' for Circuit",
        test1_toStringCircuit);
    run("layoutModeToString returns 'Shuttle' for Shuttle",
        test2_toStringShuttle);
    run("layoutModeFromString parses 'Circuit'",
        test3_fromStringCircuit);
    run("layoutModeFromString parses 'Shuttle'",
        test4_fromStringShuttle);
    run("layoutModeFromString rejects unknown string",
        test5_fromStringUnknownThrows);
    run("New document defaults to Circuit layout mode",
        test6_newDocumentDefaultsToCircuit);
    run("setLayoutMode / layoutMode getter round-trip",
        test7_setAndGetLayoutMode);
    run("Serialization includes layoutMode Circuit",
        test8_serializationIncludesLayoutMode);
    run("Serialization includes layoutMode Shuttle",
        test9_shuttleSerializesCorrectly);
    run("Round-trip preserves Circuit mode",
        test10_roundTripCircuit);
    run("Round-trip preserves Shuttle mode",
        test11_roundTripShuttle);
    run("Backward compat: missing layoutMode defaults to Circuit",
        test12_backwardCompatMissingLayoutMode);
    run("Backward compat: legacy doc re-serializes with layoutMode",
        test13_backwardCompatReserializesWithLayoutMode);
    run("Rejects invalid layoutMode string",
        test14_rejectsInvalidLayoutModeString);
    run("Rejects wrong-type layoutMode",
        test15_rejectsWrongTypeLayoutMode);
    run("Circuit + OpenLinear = CircuitIncomplete",
        test16_layoutStatusCircuitIncomplete);
    run("Circuit + ClosedCircuit = CircuitComplete",
        test17_layoutStatusCircuitComplete);
    run("Shuttle + OpenLinear = ShuttleValid",
        test18_layoutStatusShuttleValidOpen);
    run("Shuttle + ClosedCircuit = ShuttleValid",
        test19_layoutStatusShuttleValidClosed);
    run("layoutStatusLabel returns expected content",
        test20_layoutStatusLabels);
    run("Layout mode change does not affect sections",
        test21_layoutModeDoesNotAffectSections);
    run("Shuttle mode round-trip with default geometry",
        test22_shuttleWithGeometrySection);
    run("Default-constructed AuthoredTrack is Circuit",
        test23_defaultConstructorLayoutMode);

    std::fprintf(
        stdout,
        "\nResults: %d passed, %d failed\n",
        passed,
        failed);

    return failed == 0 ? 0 : 1;
}
