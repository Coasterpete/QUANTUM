#include <quantum/editor/ReadmeCapture.hpp>
#include <quantum/coaster/CoasterDocument.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
    using namespace quantum::editor;
    using nlohmann::json;

    void require(const bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    template<class Function>
    void rejects(Function function, const char* message)
    {
        try { function(); }
        catch (const std::exception&) { return; }
        throw std::runtime_error(message);
    }

    void arguments()
    {
        require(!parseReadmeCaptureArguments({}), "Normal startup must not capture.");
        const std::array normal{std::string_view("--log-level=debug")};
        require(!parseReadmeCaptureArguments(normal), "Normal logging must remain independent.");
        const std::array valid{std::string_view("--capture-screenshots"), std::string_view("with spaces/captures.json"),
            std::string_view("--log-level"), std::string_view("warning")};
        require(parseReadmeCaptureArguments(valid) == std::filesystem::path("with spaces/captures.json"),
            "Manifest path must preserve spaces.");
        for (const auto& invalid : std::vector<std::vector<std::string_view>>{
            {"--capture-screenshots"}, {"--capture-screenshots", ""},
            {"--capture-screenshots", "--log-level=info"}, {"--capture-unknown"},
            {"--capture-screenshots", "a", "--capture-screenshots", "b"},
            {"--capture-screenshots", "a", "--unknown"},
            {"--capture-screenshots=a"}, {"--capture-screenshots", "a", "--log-level"}})
            rejects([&] { (void)parseReadmeCaptureArguments(invalid); }, "Invalid arguments accepted.");
    }

    void manifestAndDocuments(const std::filesystem::path& directory)
    {
        const auto fixture = directory / "supplied document.quantum";
        auto track = quantum::coaster::createNewDocument();
        const auto original = quantum::coaster::serializeCoasterDocument(track);
        std::ofstream(fixture) << original;
        const auto path = directory / "captures.json";
        const json base{
            {"output_directory", "preview images"},
            {"scenarios", json::array({{{"name", "editor-overview"},
                {"document", "supplied document.quantum"}, {"region", 0}}})}
        };
        const auto load = [&](const json& value)
        {
            std::ofstream(path) << value.dump(2);
            return loadReadmeCaptureManifest(path);
        };
        const auto manifest = load(base);
        require(manifest.width == 1600 && manifest.height == 900 && manifest.settleFrames == 16,
            "Deterministic default dimensions/timing changed.");
        require(!manifest.overwrite && !manifest.scenarios.front().focusSelected
            && !manifest.scenarios.front().rotateGizmo, "Unsafe presentation defaults.");
        require(manifest.scenarios.front().document == std::filesystem::weakly_canonical(fixture),
            "Document must resolve relative to manifest, not working directory.");
        require(readmeCaptureOutputPath(manifest, manifest.scenarios.front())
            == directory / "preview images/editor-overview.png", "Incorrect output path.");
        validateReadmeCaptureDocument(manifest.scenarios.front(), track);

        const auto badField = [&](const char* field, const json& value)
        {
            auto invalid = base;
            invalid[field] = value;
            rejects([&] { (void)load(invalid); }, "Invalid manifest field accepted.");
        };
        badField("width", -1);
        badField("width", 1600.5);
        badField("height", 999999999999ULL);
        badField("settle_frames", 0);
        badField("settle_frames", 601);
        badField("output_directory", "");
        badField("output_directory", "supplied document.quantum");
        badField("overwrite", "yes");
        badField("unknown", 0);
        badField("scenarios", json::array());
        badField("scenarios", base["scenarios"][0]);
        badField("scenarios", json::array({base["scenarios"][0], base["scenarios"][0]}));
        for (const auto& [field, value] : std::vector<std::pair<std::string, json>>{
            {"name", "../escape"}, {"document", "missing.quantum"}, {"region", -1},
            {"region", 0.5}, {"framing", "random"}, {"tool", "move"}, {"camera", "top"}})
        {
            auto invalid = base;
            invalid["scenarios"][0][field] = value;
            rejects([&] { (void)load(invalid); }, "Invalid scenario field accepted.");
        }

        auto gizmo = base;
        gizmo["scenarios"][0]["name"] = "track-start-gizmo";
        gizmo["scenarios"][0]["tool"] = "rotate";
        gizmo["scenarios"][0]["framing"] = "selected";
        const auto presentation = load(gizmo).scenarios.front();
        require(presentation.kind == ReadmeCaptureKind::TrackStartGizmo
            && presentation.region == 0 && presentation.rotateGizmo && presentation.focusSelected,
            "Explicit gizmo/focus setup lost.");
        gizmo["scenarios"][0]["region"] = 1;
        rejects([&] { (void)load(gizmo); }, "Inconsistent Track Start selection accepted.");

        auto scenario = manifest.scenarios.front();
        scenario.region = track.sectionCount();
        rejects([&] { validateReadmeCaptureDocument(scenario, track); }, "Out-of-range region accepted.");
        scenario.region = 0;
        scenario.kind = ReadmeCaptureKind::GeometryRegions;
        rejects([&] { validateReadmeCaptureDocument(scenario, track); }, "Non-geometry region accepted.");
        scenario.kind = ReadmeCaptureKind::TransitionEditor;
        rejects([&] { validateReadmeCaptureDocument(scenario, track); }, "Flat profile accepted as meaningful curve.");
        // Validation-only state, never a public screenshot fixture.
        track.section(0).rateProfileRegion().rateProfiles.pitch.segments.front().transition.valueEnd = 0.01;
        const auto beforeValidation = quantum::coaster::serializeCoasterDocument(track);
        validateReadmeCaptureDocument(scenario, track);
        require(quantum::coaster::serializeCoasterDocument(track) == beforeValidation,
            "Scenario validation mutated authored data.");
        quantum::coaster::convertSectionToPlanarArc(track.section(0));
        rejects([&] { validateReadmeCaptureDocument(scenario, track); }, "Geometry accepted as editable profiles.");
        scenario.kind = ReadmeCaptureKind::GeometryRegions;
        validateReadmeCaptureDocument(scenario, track);

        std::filesystem::create_directory(manifest.outputDirectory);
        const auto output = readmeCaptureOutputPath(manifest, manifest.scenarios.front());
        std::ofstream(output) << "existing-image";
        rejects([&] { (void)load(base); }, "Existing output silently overwritten.");
        auto overwrite = base;
        overwrite["overwrite"] = true;
        (void)load(overwrite);

        auto collision = overwrite;
        collision["scenarios"][0]["document"] = "preview images/editor-overview.png";
        rejects([&] { (void)load(collision); }, "Output may not overwrite a supplied document with a PNG extension.");
        std::filesystem::remove(output);
        std::filesystem::create_hard_link(fixture, output);
        rejects([&] { (void)load(overwrite); }, "Output may not overwrite a hard link to a document.");
        std::ifstream saved(fixture);
        require(std::string(std::istreambuf_iterator<char>(saved), {}) == original,
            "Manifest loading altered the supplied document.");
    }
}

int main()
{
    const auto directory = std::filesystem::temp_directory_path()
        / ("quantum-capture-tests-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        std::filesystem::create_directory(directory);
        arguments();
        manifestAndDocuments(directory);
        std::filesystem::remove_all(directory);
        std::cout << "README capture tests passed.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::filesystem::remove_all(directory);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
