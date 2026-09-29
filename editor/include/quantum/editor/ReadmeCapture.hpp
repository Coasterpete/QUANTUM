#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <array>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
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
        ForceDiagnostics,
        ModernSteel,
        TrackStyleRegions,
        ForceDrivenAuthoring,
        // Supports M2A. These name the nine documentation views rather than
        // a workflow: one per structural family, a timber close-up, a tall
        // structure, the concrete foundations, and the same structure in two
        // tints. Each maps to its own output filename, so a manifest can
        // request all of them at once.
        SupportsSolidTraditional,
        SupportsSolidTwister,
        SupportsSolidPrefabricated,
        SupportsSolidHybrid,
        SupportsSolidCloseUp,
        SupportsSolidTall,
        SupportsSolidFoundations,
        SupportsSolidTintPine,
        SupportsSolidTintWeathered
    };

    // Presentation only. Documents are supplied by the developer and never edited.
    struct ReadmeCaptureScenario
    {
        ReadmeCaptureKind kind = ReadmeCaptureKind::EditorOverview;
        // Solid Supports presentation. Both default to the interactive
        // editor's state so an existing manifest keeps producing its current
        // appearance.
        bool supportSolidVisible = true;
        bool supportDebugLinesVisible = true;
        // Authored timber tint override for this capture only, in sRGB. A
        // negative component means "leave the document's value alone", which
        // is what lets a capture photograph the shipped default instead of
        // baking a color into the shared document.
        std::array<float, 3> supportTimberTint{-1.0F, -1.0F, -1.0F};
        std::filesystem::path document;
        std::size_t region = 0;
        bool focusSelected = false;
        bool rotateGizmo = false;
        bool msaaEnabled = true;
        bool environmentEnabled = true;
        // Selects a specific bundled sky. Empty means "use the first bundled
        // sky when environmentEnabled is true", which keeps manifests that
        // predate selectable HDRIs working unchanged.
        std::string environmentAsset;
        float environmentRotationDegrees = 0.0F;
        float environmentIntensity = 0.35F;
        bool skyVisible = true;
        float sunIntensity = 3.0F;
        double zoom = 1.0;
        // Ground Surface M0 presentation. Unspecified fields keep the same
        // defaults the interactive editor starts with, so an existing manifest
        // keeps producing the pre-ground appearance only when ground is false.
        bool groundEnabled = true;
        float groundElevation = 0.0F;
        float groundSize = 1200.0F;
        float groundRoughness = 0.85F;
        float groundMetallic = 0.0F;
        float groundUvTiling = 48.0F;
        std::array<float, 3> groundBaseColor{0.30F, 0.32F, 0.26F};
        std::string groundAlbedoTexture;
        std::string groundNormalTexture;
        std::string groundRoughnessTexture;
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
