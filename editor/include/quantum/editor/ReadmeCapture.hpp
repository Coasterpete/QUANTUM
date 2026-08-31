#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <filesystem>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace quantum::editor
{
    enum class ReadmeCaptureKind
    {
        EditorOverview,
        TransitionEditor,
        GeometryRegions,
        TrackStartGizmo,
        ForceDiagnostics
    };

    // Presentation only. Documents are supplied by the developer and never edited.
    struct ReadmeCaptureScenario
    {
        ReadmeCaptureKind kind = ReadmeCaptureKind::EditorOverview;
        std::filesystem::path document;
        std::size_t region = 0;
        bool focusSelected = false;
        bool rotateGizmo = false;
    };

    struct ReadmeCaptureManifest
    {
        int width = 1600;
        int height = 900;
        int settleFrames = 16;
        bool overwrite = false;
        std::filesystem::path outputDirectory;
        std::vector<ReadmeCaptureScenario> scenarios;
    };

    // Arguments exclude argv[0]. Normal non-capture startup keeps its existing parser.
    [[nodiscard]] std::optional<std::filesystem::path> parseReadmeCaptureArguments(
        std::span<const std::string_view> arguments);
    [[nodiscard]] ReadmeCaptureManifest loadReadmeCaptureManifest(
        const std::filesystem::path& path);
    [[nodiscard]] std::string_view readmeCaptureName(ReadmeCaptureKind kind);
    [[nodiscard]] std::filesystem::path readmeCaptureOutputPath(
        const ReadmeCaptureManifest& manifest, const ReadmeCaptureScenario& scenario);
    void validateReadmeCaptureDocument(
        const ReadmeCaptureScenario& scenario, const coaster::AuthoredTrack& track);
    [[nodiscard]] int runReadmeCapture(const ReadmeCaptureManifest& manifest);
}
