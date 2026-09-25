#pragma once

#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/renderer/FrameSynchronizationTelemetry.hpp>
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
        bool hdrEnvironment = false;
    };

    // Owns the renderer's GPU resources. Geometry inputs are copied or uploaded;
    // callers retain ownership of the authored document and generated data.
    class Renderer
    {
    public:
        using ViewportTargetRetirementCallback = void (*)(void*) noexcept;
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
        virtual void setEnvironment(bool enabled, float rotationDegrees,
            float lightingIntensity, bool skyVisible) = 0;
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
        virtual void updateSupportVertices(std::span<const LineVertex> vertices) = 0;
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
