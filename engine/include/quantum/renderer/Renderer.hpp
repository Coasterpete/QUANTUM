#pragma once

#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/renderer/FrameSynchronizationTelemetry.hpp>
#include <quantum/renderer/GroundSurface.hpp>
#include <quantum/renderer/StaticMeshAssets.hpp>
#include <quantum/renderer/ViewportTrackPresentation.hpp>

#include <SDL3/SDL_video.h>
#include <glm/vec3.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::renderer
{
    // Ground aids, reference curves, supports, and train diagnostics share
    // this line-list layout. The backend copies vertices on update.
    struct LineVertex
    {
        float x;
        float y;
        float z;
        std::array<float, 4> color;
    };

    // Reference curves are four equal-length runs in this order.
    inline constexpr std::uint32_t viewportLeftRailCurve = 0;
    inline constexpr std::uint32_t viewportRightRailCurve = 1;
    inline constexpr std::uint32_t viewportCenterlineCurve = 2;
    inline constexpr std::uint32_t viewportHeartlineCurve = 3;
    inline constexpr std::uint32_t viewportCurveCount = 4;
    inline constexpr std::uint32_t viewportAllCurvesVisibleMask = 0xFu;

    // RGBA8 pixels of the complete client area, top to bottom.
    struct FrameImage
    {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::vector<std::uint8_t> pixels;
    };

    struct DrawFrameCpuTelemetry
    {
        FrameSynchronizationTelemetry synchronization;
        double frameSlotWaitMilliseconds = 0.0;
        double deferredBufferReclaimMilliseconds = 0.0;
        double previewFrameSlotUpdateMilliseconds = 0.0;
        double acquireCallMilliseconds = 0.0;
        double presentCallMilliseconds = 0.0;
        double totalMilliseconds = 0.0;
        double gpuExecutionMilliseconds = 0.0;
        std::size_t deferredBufferCountBeforeReclaim = 0;
        std::uint64_t deferredBufferBytesBeforeReclaim = 0;
        std::size_t reclaimedBufferCount = 0;
        bool gpuTimingAvailable = false;
        bool previewStreamUpdated = false;
        bool swapchainRecreated = false;
        bool synchronousReadback = false;
    };

    struct RendererCapabilities
    {
        bool viewportMsaa4 = false;
        // The HDR/IBL pipeline is compiled in and the bundled sky registry is
        // available. Whether the currently selected sky actually loaded is
        // reported by environmentStatus().
        bool hdrEnvironment = false;
    };

    // Owns the renderer's GPU resources. Geometry inputs are copied or uploaded;
    // callers retain ownership of the authored document and generated data.
    class Renderer
    {
    public:
        using ViewportTargetRetirementCallback = void (*)(void*) noexcept;

        // What the HDR environment pipeline is actually doing. The requested
        // identifier is what the application selected (empty means "no
        // environment"); available is false when nothing is sampled, so the
        // caller can report constant ambient instead of a sky.
        struct EnvironmentStatus
        {
            std::string identifier;
            bool available = false;
            std::string detail;
        };

        virtual ~Renderer() = default;

        virtual void initialize(SDL_Window* window,
            std::span<const LineVertex> trackCurveVertices,
            std::uint32_t trackVerticesPerCurve,
            const coaster::RenderableTrack& renderableTrack,
            bool enableFrameReadback = false) = 0;
        virtual void drawFrame(FrameImage* readback = nullptr) = 0;
        virtual void shutdown() noexcept = 0;
        // Called after in-flight work completes and before the old target is
        // destroyed, so an embedding UI can release its texture reference.
        virtual void resizeViewportTarget(std::uint32_t width,
            std::uint32_t height,
            ViewportTargetRetirementCallback retirementCallback,
            void* userData, bool enableMsaa) = 0;
        [[nodiscard]] virtual RendererCapabilities capabilities() const noexcept = 0;
        [[nodiscard]] virtual bool viewportMsaaEnabled() const noexcept = 0;

        virtual void setViewportViewProjection(
            const std::array<float, 16>& viewProjection) = 0;
        virtual void setViewportCameraPosition(const glm::vec3& position) = 0;
        virtual void setSunlight(const glm::vec3& direction, float intensity) = 0;
        virtual void setExposure(float exposure) = 0;
        // Selects a bundled HDR sky by package-relative identifier, or the
        // empty string for "no environment" (constant ambient, no sky). An
        // unknown identifier is rejected; a bundled one that cannot be read is
        // reported through environmentStatus() rather than silently replaced.
        // Rotation, lighting intensity, and sky visibility travel through push
        // constants, so they never retire or recreate GPU images.
        virtual void setEnvironment(std::string_view identifier,
            float rotationDegrees, float lightingIntensity,
            bool skyVisible) = 0;
        [[nodiscard]] virtual EnvironmentStatus environmentStatus() const = 0;
        // Scene presentation only. This is renderer-neutral and has no
        // relationship to authored track geometry, physics, or document state.
        // The backend owns the ground's textures, descriptors, pipeline, and
        // buffers, and applies GPU work only when the settings change.
        virtual void setGroundSurface(
            const GroundSurfaceSettings& settings) = 0;
        // Load outcome for a currently selected ground map identifier. Empty
        // when that identifier is not part of the current ground settings.
        [[nodiscard]] virtual std::optional<GroundTextureLoadStatus>
            groundTextureLoadStatus(std::string_view identifier) const = 0;
        virtual void updateTrackCurveVertices(std::span<const LineVertex> vertices,
            std::uint32_t verticesPerCurve) = 0;
        virtual void updateRenderableTrack(
            const coaster::RenderableTrack& renderableTrack) = 0;
        virtual void updateRenderableTrackMesh(
            const coaster::ContinuousTrackMesh& mesh,
            std::span<const coaster::TrackMaterial> materials) = 0;
        virtual void updateTrackMaterials(
            std::span<const coaster::TrackSubmesh> submeshes,
            std::span<const coaster::TrackMaterial> materials) = 0;
        virtual void updateTrackHardware(
            std::span<const coaster::HardwareInstanceBatch> batches) = 0;
        virtual void updateTrackHardwareMaterials(
            std::span<const std::optional<coaster::TrackMaterial>> materials) = 0;
        virtual void reloadTrackHardwareAsset(std::string_view identifier,
            const coaster::RenderableTrack& renderableTrack) = 0;
        virtual void setTrackPresentationMode(TrackPresentationMode mode) = 0;
        virtual void updateTrainPreviewVertices(
            std::span<const LineVertex> vertices) = 0;
        // Replaces the Editor's renderer-neutral support-member line stream.
        // This stays available as the technical overlay and is independent of
        // the solid presentation below.
        virtual void updateSupportVertices(std::span<const LineVertex> vertices) = 0;
        // Replaces the solid timber presentation. The renderer owns the unit
        // meshes, the shared instance stream, and the timber textures; the
        // caller keeps ownership of the authored document.
        virtual void updateSupportSolidPresentation(
            std::span<const coaster::SupportSolidPresentation> presentations) = 0;
        // Solid and debug-line display are independent so a user can inspect
        // both presentation and authored topology at once.
        virtual void setSupportDisplay(bool solidVisible,
            bool debugLinesVisible) = 0;
        virtual void setViewportElementVisibility(bool gridVisible,
            std::uint32_t curveVisibilityMask) = 0;
        virtual void setTrackCurveHighlight(std::uint32_t firstVertex,
            std::uint32_t vertexCount) = 0;
        virtual void updateViewportAidReference(float centerX, float centerY,
            float referenceRadius) = 0;

        [[nodiscard]] virtual const DrawFrameCpuTelemetry&
            lastDrawFrameCpuTelemetry() const noexcept = 0;
        [[nodiscard]] virtual double lastFrameCompletionWaitMilliseconds()
            const noexcept = 0;
        [[nodiscard]] virtual const std::filesystem::path& runtimeAssetRoot()
            const noexcept = 0;
        [[nodiscard]] virtual std::optional<HardwareAssetLoadStatus>
            hardwareAssetLoadStatus(std::string_view identifier) const = 0;
    };
}
