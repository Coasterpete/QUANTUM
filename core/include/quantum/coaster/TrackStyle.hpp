#pragma once

#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace quantum::coaster
{
    // A geometry family selects a continuous structural-generation strategy.
    // Named presets supply dimensions and assets without requiring another
    // renderer implementation.
    enum class TrackGeometryFamily : std::uint8_t
    {
        DualRailTubular
    };

    struct TrackMaterial
    {
        glm::vec4 baseColor{0.32F, 0.40F, 0.48F, 1.0F};
    };

    struct RailOffset
    {
        double lateral = 0.0;
        double vertical = 0.0;
    };

    enum class ContinuousSpineType : std::uint8_t
    {
        None,
        Tubular,
        Box
    };

    // This pass records the future continuous-spine intent but generates no
    // spine. Keeping the parameters in the preset avoids renderer constants
    // when a later geometry-family milestone implements one.
    struct ContinuousSpineStyle
    {
        bool enabled = false;
        ContinuousSpineType type = ContinuousSpineType::None;
        RailOffset offset{};
        glm::dvec2 dimensions{0.0, 0.0};
    };

    struct StaticMeshAssetReference
    {
        std::string path;
        // Diagnostic assets are replaceable development stand-ins, not
        // production Blender-authored track hardware.
        bool placeholder = false;
    };

    enum class HardwareFrameFollow : std::uint8_t
    {
        TrackFrame,
        WorldAligned
    };

    struct RepeatingHardwareStyle
    {
        bool enabled = true;
        StaticMeshAssetReference asset;
        double spacing = 1.5;
        double startOffset = 0.0;
        glm::dvec3 localPosition{0.0};
        // Local X/Y/Z Euler adjustments, in radians, applied in X-Y-Z order.
        glm::dvec3 localRotation{0.0};
        glm::dvec3 localScale{1.0};
        HardwareFrameFollow frameFollow = HardwareFrameFollow::TrackFrame;
        std::optional<TrackMaterial> materialOverride;
    };

    struct TrackStylePreset
    {
        std::string name;
        TrackGeometryFamily geometryFamily =
            TrackGeometryFamily::DualRailTubular;

        std::uint32_t railCount = 2;
        std::vector<RailOffset> railOffsets;
        double railRadius = 0.065;
        std::uint32_t railRadialSegments = 12;
        TrackMaterial railMaterial;

        ContinuousSpineStyle spine;
        std::vector<RepeatingHardwareStyle> repeatingHardware;
    };

    [[nodiscard]] TrackStylePreset createStandardDualRailPreset();

    // Throws std::invalid_argument for malformed values. Generation calls
    // this before allocating output, so invalid styles cannot publish partial
    // meshes or instance data.
    void validateTrackStyle(const TrackStylePreset& style);

    struct TrackMeshVertex
    {
        glm::vec3 position{0.0F};
        glm::vec3 normal{0.0F, 0.0F, 1.0F};
    };

    struct TrackSubmesh
    {
        std::uint32_t firstIndex = 0;
        std::uint32_t indexCount = 0;
        std::uint32_t materialIndex = 0;
        std::uint32_t componentId = 0;
    };

    struct ContinuousTrackMesh
    {
        std::vector<TrackMeshVertex> vertices;
        std::vector<std::uint32_t> triangleIndices;
        // Explicit mesh edges provide portable wireframe without depending on
        // the optional Vulkan fillModeNonSolid feature.
        std::vector<std::uint32_t> edgeIndices;
        std::vector<TrackSubmesh> submeshes;
    };

    // Rendering-only, contiguous instance payload. It deliberately contains
    // no pointers, editor state, or draw commands and can later be copied into
    // a persistent GPU instance buffer.
    struct HardwareInstance
    {
        glm::mat4 transform{1.0F};
        std::uint32_t componentId = 0;
        std::uint32_t objectId = 0;
        std::uint32_t flags = 0;
    };

    struct HardwareInstanceBatch
    {
        StaticMeshAssetReference asset;
        std::optional<TrackMaterial> materialOverride;
        std::vector<HardwareInstance> instances;
    };

    struct RenderableTrack
    {
        ContinuousTrackMesh continuousMesh;
        std::vector<TrackMaterial> materials;
        std::vector<HardwareInstanceBatch> hardwareBatches;
    };

    // Builds renderer-neutral indexed rail geometry and reusable-hardware
    // placement from the canonical centerline/frame samples. The samples are
    // not regenerated or re-framed here.
    [[nodiscard]] RenderableTrack generateRenderableTrack(
        std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style
    );
}
