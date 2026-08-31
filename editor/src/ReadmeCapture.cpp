#include <quantum/editor/ReadmeCapture.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>

namespace quantum::editor
{
    namespace
    {
        constexpr std::array<std::string_view, 5> names{
            "editor-overview", "transition-editor", "geometry-regions",
            "track-start-gizmo", "force-diagnostics"
        };

        void requireKeys(const nlohmann::json& object,
            std::initializer_list<std::string_view> allowed)
        {
            if (!object.is_object())
                throw std::invalid_argument("Capture configuration must be a JSON object.");
            for (const auto& [key, value] : object.items())
            {
                if (std::find(allowed.begin(), allowed.end(), key) == allowed.end())
                    throw std::invalid_argument("Unknown capture field: " + key);
            }
        }

        int integer(const nlohmann::json& value, const int minimum,
            const int maximum, const char* field)
        {
            if (!value.is_number_integer() || value < minimum || value > maximum)
                throw std::invalid_argument(std::string("Capture ") + field
                    + " must be an integer in [" + std::to_string(minimum)
                    + ", " + std::to_string(maximum) + "].");
            return value.get<int>();
        }

        std::filesystem::path resolvePath(const std::filesystem::path& base,
            const nlohmann::json& value)
        {
            const auto text = value.get<std::string>();
            if (text.empty() || text.find('\0') != std::string::npos)
                throw std::invalid_argument("Capture paths must be nonempty and contain no NUL.");
            return std::filesystem::weakly_canonical(base / std::filesystem::u8path(text));
        }
    }

    std::optional<std::filesystem::path> parseReadmeCaptureArguments(
        const std::span<const std::string_view> arguments)
    {
        bool captureRequested = false;
        for (const auto argument : arguments)
            captureRequested |= argument.starts_with("--capture");
        if (!captureRequested)
            return std::nullopt;

        std::optional<std::filesystem::path> manifest;
        for (std::size_t index = 0; index < arguments.size(); ++index)
        {
            const auto argument = arguments[index];
            if (argument == "--capture-screenshots")
            {
                if (manifest || index + 1 == arguments.size()
                    || arguments[index + 1].empty() || arguments[index + 1].starts_with("--"))
                    throw std::invalid_argument("Use --capture-screenshots <manifest.json> exactly once.");
                manifest = std::filesystem::u8path(arguments[++index]);
            }
            else if (argument == "--log-level")
            {
                if (++index == arguments.size())
                    throw std::invalid_argument("--log-level requires a value.");
            }
            else if (!argument.starts_with("--log-level="))
                throw std::invalid_argument("Unknown capture argument: " + std::string(argument));
        }
        if (!manifest)
            throw std::invalid_argument("Use --capture-screenshots <manifest.json>.");
        return manifest;
    }

    std::string_view readmeCaptureName(const ReadmeCaptureKind kind)
    {
        return names.at(static_cast<std::size_t>(kind));
    }

    std::filesystem::path readmeCaptureOutputPath(
        const ReadmeCaptureManifest& manifest, const ReadmeCaptureScenario& scenario)
    {
        // Filenames come exclusively from the five presets; no traversal or source-file writes.
        return manifest.outputDirectory / (std::string(readmeCaptureName(scenario.kind)) + ".png");
    }

    ReadmeCaptureManifest loadReadmeCaptureManifest(const std::filesystem::path& path)
    {
        std::ifstream input(path);
        if (!input)
            throw std::runtime_error("Cannot open capture manifest: " + path.string());
        const auto json = nlohmann::json::parse(input);
        requireKeys(json, {"width", "height", "settle_frames", "output_directory", "overwrite", "scenarios"});
        ReadmeCaptureManifest manifest;
        manifest.width = integer(json.value("width", nlohmann::json(1600)), 960, 3840, "width");
        manifest.height = integer(json.value("height", nlohmann::json(900)), 720, 2160, "height");
        manifest.settleFrames = integer(json.value("settle_frames", nlohmann::json(16)), 8, 600, "settle_frames");
        manifest.overwrite = json.value("overwrite", false);
        const auto base = std::filesystem::absolute(path).parent_path();
        manifest.outputDirectory = resolvePath(base, json.at("output_directory"));
        if (std::filesystem::exists(manifest.outputDirectory)
            && !std::filesystem::is_directory(manifest.outputDirectory))
            throw std::invalid_argument("Capture output_directory is not a directory.");
        const auto& scenarios = json.at("scenarios");
        if (!scenarios.is_array() || scenarios.empty() || scenarios.size() > names.size())
            throw std::invalid_argument("Capture scenarios must contain 1 to 5 entries.");
        std::set<ReadmeCaptureKind> used;
        for (const auto& entry : scenarios)
        {
            requireKeys(entry, {"name", "document", "region", "framing", "tool"});
            const auto name = entry.at("name").get<std::string>();
            const auto found = std::find(names.begin(), names.end(), name);
            if (found == names.end())
                throw std::invalid_argument("Unknown capture scenario: " + name);
            ReadmeCaptureScenario scenario;
            scenario.kind = static_cast<ReadmeCaptureKind>(found - names.begin());
            if (!used.insert(scenario.kind).second)
                throw std::invalid_argument("Duplicate capture scenario: " + name);
            scenario.document = resolvePath(base, entry.at("document"));
            if (!std::filesystem::is_regular_file(scenario.document))
                throw std::invalid_argument("Supply a user-authored document for " + name
                    + ": " + scenario.document.string());
            scenario.region = static_cast<std::size_t>(integer(entry.at("region"), 0, 1000000, "region"));
            const auto framing = entry.value("framing", std::string("all"));
            if (framing != "all" && framing != "selected")
                throw std::invalid_argument("Capture framing must be all or selected.");
            scenario.focusSelected = framing == "selected";
            const auto tool = entry.value("tool", std::string("move"));
            if (tool != "move" && tool != "rotate")
                throw std::invalid_argument("Capture tool must be move or rotate.");
            scenario.rotateGizmo = tool == "rotate";
            if (entry.contains("tool") && scenario.kind != ReadmeCaptureKind::TrackStartGizmo)
                throw std::invalid_argument("Capture tool is only used by track-start-gizmo.");
            if (scenario.kind == ReadmeCaptureKind::TrackStartGizmo && scenario.region != 0)
                throw std::invalid_argument("Track Start selects region 0; use region: 0.");
            const auto output = readmeCaptureOutputPath(manifest, scenario);
            if (std::filesystem::exists(output)
                && (!manifest.overwrite || !std::filesystem::is_regular_file(output)))
                throw std::invalid_argument("Capture output already exists (set overwrite: true for files): "
                    + output.string());
            if (std::filesystem::is_symlink(output))
                throw std::invalid_argument("Capture output must not be a symlink: " + output.string());
            manifest.scenarios.push_back(std::move(scenario));
        }
        for (const auto& scenario : manifest.scenarios)
        {
            const auto output = readmeCaptureOutputPath(manifest, scenario);
            const auto sameFile = [&](const std::filesystem::path& inputPath)
            {
                return output == std::filesystem::weakly_canonical(inputPath)
                    || (std::filesystem::exists(output) && std::filesystem::equivalent(output, inputPath));
            };
            if (sameFile(path))
                throw std::invalid_argument("Capture output would overwrite its manifest: " + output.string());
            for (const auto& inputScenario : manifest.scenarios)
                if (sameFile(inputScenario.document))
                    throw std::invalid_argument("Capture output would overwrite a supplied document: " + output.string());
        }
        return manifest;
    }

    void validateReadmeCaptureDocument(
        const ReadmeCaptureScenario& scenario, const coaster::AuthoredTrack& track)
    {
        const std::string name(readmeCaptureName(scenario.kind));
        if (scenario.region >= track.sectionCount())
            throw std::invalid_argument(name + ": selected region is outside the supplied document.");
        const auto& section = track.section(scenario.region);
        if (scenario.kind == ReadmeCaptureKind::GeometryRegions
            && section.kind != coaster::RegionKind::Geometry)
            throw std::invalid_argument(name + ": select a Geometry region in the supplied document.");
        if (scenario.kind == ReadmeCaptureKind::TransitionEditor)
        {
            if (section.kind != coaster::RegionKind::RateProfiles)
                throw std::invalid_argument(name + ": select a Rate Profiles region.");
            const auto& profiles = section.rateProfileRegion().rateProfiles;
            bool varying = false;
            for (const auto* channel : {&profiles.roll, &profiles.pitch, &profiles.yaw})
                for (const auto& segment : channel->segments)
                    varying |= segment.transition.valueBegin != segment.transition.valueEnd;
            if (!varying)
                throw std::invalid_argument(name + ": supply a region with a varying authored profile curve.");
        }
    }
}
