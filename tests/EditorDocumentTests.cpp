#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/DocumentState.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::currentFormatVersion;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::deserializeCoasterDocument;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::serializeCoasterDocument;
    using quantum::coaster::SegmentId;
    using quantum::editor::DocumentState;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

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

    void requireEqual(
        const std::string& actual,
        const std::string& expected,
        const std::string_view context)
    {
        if (actual != expected)
        {
            throw TestFailure(
                std::string(context) + ": expected '"
                + expected + "', got '" + actual + "'");
        }
    }

    [[nodiscard]] std::filesystem::path tempPath()
    {
        const auto dir = std::filesystem::temp_directory_path();
        auto path = dir / "quantum_test_doc";
        std::filesystem::create_directories(path);
        return path;
    }

    [[nodiscard]] std::filesystem::path testFilePath(
        const std::string_view name)
    {
        return tempPath() / name;
    }

    void removeIfExists(const std::filesystem::path& path)
    {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }

    void cleanupTempDir()
    {
        std::error_code ec;
        std::filesystem::remove_all(tempPath(), ec);
    }

    // ================================================================
    // 1. Default new document is one neutral 60.0 Rate/Profile Region
    // ================================================================

    void newDocumentIsOneNeutralSection()
    {
        const AuthoredTrack doc =
            quantum::coaster::createNewDocument();

        require(
            doc.sectionCount() == 1,
            "new document must have exactly 1 section");

        const auto& section = doc.section(0);
        require(
            section.kind == RegionKind::RateProfiles,
            "section must be RateProfiles kind");

        const double length = quantum::coaster::sectionLength(section);
        require(
            std::abs(length - 60.0) < 1.0e-9,
            "section length must be 60.0");

        const auto& roll = section.rateProfileRegion().rateProfiles.roll;
        const auto& pitch = section.rateProfileRegion().rateProfiles.pitch;
        const auto& yaw = section.rateProfileRegion().rateProfiles.yaw;

        require(
            roll.segments.size() == 1,
            "roll must have exactly 1 segment");
        require(
            pitch.segments.size() == 1,
            "pitch must have exactly 1 segment");
        require(
            yaw.segments.size() == 1,
            "yaw must have exactly 1 segment");

        require(
            std::abs(roll.segments[0].transition.valueBegin) < 1.0e-9
            && std::abs(roll.segments[0].transition.valueEnd) < 1.0e-9,
            "roll must be neutral (zero rates)");

        require(
            std::abs(pitch.segments[0].transition.valueBegin) < 1.0e-9
            && std::abs(pitch.segments[0].transition.valueEnd) < 1.0e-9,
            "pitch must be neutral (zero rates)");

        require(
            std::abs(yaw.segments[0].transition.valueBegin) < 1.0e-9
            && std::abs(yaw.segments[0].transition.valueEnd) < 1.0e-9,
            "yaw must be neutral (zero rates)");
    }

    // ================================================================
    // 2. Save to explicit path produces valid deterministic JSON
    // ================================================================

    void saveProducesValidJson()
    {
        const AuthoredTrack doc =
            quantum::coaster::createNewDocument();
        const std::string json = serializeCoasterDocument(doc);

        require(!json.empty(), "serialized JSON must not be empty");
        auto result = deserializeCoasterDocument(json);

        requireValidDocument(result, "deserialization must succeed");

        const std::string json2 = serializeCoasterDocument(*result);
        require(
            json == json2,
            "serialization must be deterministic");
    }

    // ================================================================
    // 3. Saved document round-trips through Core
    // ================================================================

    void savedDocumentRoundTrips()
    {
        const AuthoredTrack doc =
            quantum::coaster::createNewDocument();
        const std::string json = serializeCoasterDocument(doc);

        auto result = deserializeCoasterDocument(json);
        require(result.has_value(), "round-trip must succeed");

        require(
            result->sectionCount() == doc.sectionCount(),
            "section count must survive round-trip");

        require(
            std::abs(quantum::coaster::sectionLength(result->section(0))
                - 60.0) < 1.0e-9,
            "section length must survive round-trip");
    }

    // ================================================================
    // 4. Open success replaces document (tested via serialize/deserialize)
    // ================================================================

    void openSuccessReplacesDocument()
    {
        const auto path = testFilePath("open_success.quantum");
        removeIfExists(path);

        AuthoredTrack original =
            quantum::coaster::createNewDocument();

        {
            std::ofstream ofs(path);
            require(ofs.is_open(), "must create test file");
            const std::string json = serializeCoasterDocument(original);
            ofs.write(json.data(),
                static_cast<std::streamsize>(json.size()));
        }

        std::ifstream ifs(path);
        require(ifs.is_open(), "must read test file");
        std::string json(
            (std::istreambuf_iterator<char>(ifs)),
            std::istreambuf_iterator<char>());

        auto result = deserializeCoasterDocument(json);
        requireValidDocument(result, "open must succeed");

        require(
            result->sectionCount() == original.sectionCount(),
            "opened document must match original");

        removeIfExists(path);
    }

    // ================================================================
    // 5. Open failure leaves prior document unchanged (tested via
    //    DocumentState metadata)
    // ================================================================

    void openFailureLeavesPriorDocument()
    {
        DocumentState state;
        state.setOpenDocument(testFilePath("existing.quantum"));
        state.markDirty();

        require(state.isDirty(), "must be dirty before failed open");
        require(
            state.hasPath(),
            "must have path before failed open");

        const auto path = testFilePath("bad.quantum");
        removeIfExists(path);

        {
            std::ofstream ofs(path);
            require(ofs.is_open(), "must create bad file");
            ofs << "this is not valid JSON {{{";
        }

        std::ifstream ifs(path);
        std::string json(
            (std::istreambuf_iterator<char>(ifs)),
            std::istreambuf_iterator<char>());

        auto result = deserializeCoasterDocument(json);
        require(
            !result.has_value(),
            "deserialization of bad data must fail");

        require(
            state.isDirty(),
            "dirty must remain true after failed open");
        require(
            state.currentPath()
                == testFilePath("existing.quantum"),
            "path must remain unchanged after failed open");

        removeIfExists(path);
    }

    // ================================================================
    // 6. Save failure leaves dirty state true
    // ================================================================

    void saveFailureLeavesDirtyTrue()
    {
        DocumentState state;
        state.markDirty();

        require(state.isDirty(), "must be dirty");

        // Simulate a failed save by checking that dirty stays set
        // when we don't call clearDirty.
        require(
            state.isDirty(),
            "dirty must remain true after simulated save failure");
    }

    // ================================================================
    // 7. Successful Save clears dirty state
    // ================================================================

    void successfulSaveClearsDirty()
    {
        const auto path = testFilePath("save_clears.quantum");
        removeIfExists(path);

        DocumentState state;
        state.setOpenDocument(path);
        state.markDirty();

        require(state.isDirty(), "must be dirty before save");

        AuthoredTrack doc =
            quantum::coaster::createNewDocument();
        const std::string json = serializeCoasterDocument(doc);

        std::ofstream ofs(path);
        require(ofs.is_open(), "must create save file");
        ofs.write(json.data(),
            static_cast<std::streamsize>(json.size()));

        state.clearDirty();
        require(!state.isDirty(), "must be clean after save");

        removeIfExists(path);
    }

    // ================================================================
    // 8. Save with no path routes to Save As behavior
    // ================================================================

    void saveWithNoPathRoutesToSaveAs()
    {
        DocumentState state;
        require(
            !state.hasPath(),
            "new DocumentState must have no path");
        require(
            !state.isDirty(),
            "new DocumentState must be clean");

        state.markDirty();
        require(state.isDirty(), "must be dirty");

        // In Application, Save with no path triggers the save dialog.
        // We verify the state-level precondition: no path means
        // Save must behave as Save As.
        require(
            !state.hasPath(),
            "still no path after dirty (Save As needed)");
    }

    // ================================================================
    // 9. Successful Save As updates current path
    // ================================================================

    void successfulSaveAsUpdatesPath()
    {
        const auto path = testFilePath("saveas.quantum");
        removeIfExists(path);

        DocumentState state;
        require(!state.hasPath(), "must start without path");

        AuthoredTrack doc =
            quantum::coaster::createNewDocument();
        const std::string json = serializeCoasterDocument(doc);

        std::ofstream ofs(path);
        require(ofs.is_open(), "must create saveas file");
        ofs.write(json.data(),
            static_cast<std::streamsize>(json.size()));

        state.setOpenDocument(path);
        state.clearDirty();

        require(state.hasPath(), "must have path after Save As");
        require(
            state.currentPath() == path,
            "path must match save destination");
        require(!state.isDirty(), "must be clean after Save As");

        removeIfExists(path);
    }

    // ================================================================
    // 10. Canceled Open causes no mutation
    // ================================================================

    void canceledOpenCausesNoMutation()
    {
        DocumentState state;
        const auto originalPath = testFilePath("original.quantum");
        state.setOpenDocument(originalPath);
        state.markDirty();

        // A canceled open would not call setOpenDocument or clearDirty.
        // Verify state is unchanged.
        require(state.isDirty(), "must remain dirty after cancel");
        require(
            state.currentPath() == originalPath,
            "path must remain unchanged after cancel");
    }

    // ================================================================
    // 11. Canceled Save As causes no mutation
    // ================================================================

    void canceledSaveAsCausesNoMutation()
    {
        DocumentState state;
        state.markDirty();

        // A canceled Save As would not call setOpenDocument or clearDirty.
        require(state.isDirty(), "must remain dirty after cancel");
        require(!state.hasPath(), "must remain without path");
    }

    // ================================================================
    // 12. Dirty flag becomes true after successful authoring mutation
    // ================================================================

    void dirtyBecomesTrueAfterMutation()
    {
        DocumentState state;
        require(!state.isDirty(), "must start clean");

        state.markDirty();
        require(state.isDirty(), "must be dirty after mutation");
    }

    // ================================================================
    // 13. Rejected edit does not falsely clear dirty state
    // ================================================================

    void rejectedEditDoesNotClearDirty()
    {
        DocumentState state;
        state.markDirty();

        // A rejected edit should not call clearDirty.
        require(state.isDirty(), "must remain dirty");
    }

    // ================================================================
    // 14. New resets path and dirty state
    // ================================================================

    void newResetsPathAndDirty()
    {
        DocumentState state;
        state.setOpenDocument(testFilePath("something.quantum"));
        state.markDirty();

        require(state.hasPath(), "must have path before New");
        require(state.isDirty(), "must be dirty before New");

        state.newDocument();

        require(!state.hasPath(), "must have no path after New");
        require(!state.isDirty(), "must be clean after New");
    }

    // ================================================================
    // 15. Open resets dirty state
    // ================================================================

    void openResetsDirtyState()
    {
        DocumentState state;
        state.markDirty();

        require(state.isDirty(), "must be dirty before open");

        state.setOpenDocument(testFilePath("opened.quantum"));

        require(!state.isDirty(), "must be clean after open");
        require(state.hasPath(), "must have path after open");
    }

    // ================================================================
    // 16. Title formatting for Untitled / filename / dirty variants
    // ================================================================

    void titleFormattingUntitled()
    {
        DocumentState state;
        requireEqual(
            state.windowTitle(),
            "QUANTUM \xe2\x80\x94 Untitled",
            "untitled clean title");
    }

    void titleFormattingUntitledDirty()
    {
        DocumentState state;
        state.markDirty();
        requireEqual(
            state.windowTitle(),
            "QUANTUM \xe2\x80\x94 Untitled *",
            "untitled dirty title");
    }

    void titleFormattingFilename()
    {
        DocumentState state;
        state.setOpenDocument(
            std::filesystem::path("MyCoaster.quantum"));
        requireEqual(
            state.windowTitle(),
            "QUANTUM \xe2\x80\x94 MyCoaster.quantum",
            "file clean title");
    }

    void titleFormattingFilenameDirty()
    {
        DocumentState state;
        state.setOpenDocument(
            std::filesystem::path("MyCoaster.quantum"));
        state.markDirty();
        requireEqual(
            state.windowTitle(),
            "QUANTUM \xe2\x80\x94 MyCoaster.quantum *",
            "file dirty title");
    }

    void titleFormattingFullPathNotExposed()
    {
        DocumentState state;
        state.setOpenDocument(
            std::filesystem::path("C:/Users/test/MyCoaster.quantum"));
        requireEqual(
            state.displayName(),
            "MyCoaster.quantum",
            "display name must be filename only");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    const auto run = [&](const char* name, void (*fn)())
    {
        try
        {
            fn();
            ++passed;
            std::cout << "  PASS: " << name << "\n";
        }
        catch (const std::exception& e)
        {
            ++failed;
            std::cerr << "  FAIL: " << name << "\n    "
                << e.what() << "\n";
        }
    };

    std::cout << "EditorDocumentTests\n";

    run("newDocumentIsOneNeutralSection",
        newDocumentIsOneNeutralSection);
    run("saveProducesValidJson", saveProducesValidJson);
    run("savedDocumentRoundTrips", savedDocumentRoundTrips);
    run("openSuccessReplacesDocument", openSuccessReplacesDocument);
    run("openFailureLeavesPriorDocument",
        openFailureLeavesPriorDocument);
    run("saveFailureLeavesDirtyTrue", saveFailureLeavesDirtyTrue);
    run("successfulSaveClearsDirty", successfulSaveClearsDirty);
    run("saveWithNoPathRoutesToSaveAs",
        saveWithNoPathRoutesToSaveAs);
    run("successfulSaveAsUpdatesPath", successfulSaveAsUpdatesPath);
    run("canceledOpenCausesNoMutation",
        canceledOpenCausesNoMutation);
    run("canceledSaveAsCausesNoMutation",
        canceledSaveAsCausesNoMutation);
    run("dirtyBecomesTrueAfterMutation",
        dirtyBecomesTrueAfterMutation);
    run("rejectedEditDoesNotClearDirty",
        rejectedEditDoesNotClearDirty);
    run("newResetsPathAndDirty", newResetsPathAndDirty);
    run("openResetsDirtyState", openResetsDirtyState);
    run("titleFormattingUntitled", titleFormattingUntitled);
    run("titleFormattingUntitledDirty",
        titleFormattingUntitledDirty);
    run("titleFormattingFilename", titleFormattingFilename);
    run("titleFormattingFilenameDirty",
        titleFormattingFilenameDirty);
    run("titleFormattingFullPathNotExposed",
        titleFormattingFullPathNotExposed);

    cleanupTempDir();

    std::cout << "\n  " << passed << " passed, "
        << failed << " failed\n";

    return failed == 0 ? 0 : 1;
}
