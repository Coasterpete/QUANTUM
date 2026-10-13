#pragma once

#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/StaticMeshAsset.hpp>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cstdint>
#include <functional>
#include <memory>
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
        DualRailTubular,
        Components
    };

    struct TrackMaterial
    {
        // Editor colors are sRGB; the renderer decodes them before lighting.
        glm::vec4 baseColor{0.32F, 0.40F, 0.48F, 1.0F};
        float metallic = 0.0F;
        float roughness = 0.42F;
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

    struct ContinuousSpineStyle
    {
        bool enabled = false;
        ContinuousSpineType type = ContinuousSpineType::None;
        RailOffset offset{};
        // Tubular profiles use lateral/vertical diameters. Box profiles use
        // lateral width and vertical height.
        glm::dvec2 dimensions{0.0, 0.0};
        std::uint32_t radialSegments = 12;
        TrackMaterial material;
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

    // Slots are family-defined strings. Providers describe geometry rather than
    // catalog identity; a new type using these providers is a content operation.
    enum class TrackComponentProvider : std::uint8_t { Tube, Box, Member, Beam };
    enum class TrackComponentPlacement : std::uint8_t { Continuous, Repeated, Junction };

    struct TrackComponent
    {
        std::string id;
        std::string displayName;
        std::string materialSlot;
        TrackComponentProvider provider = TrackComponentProvider::Box;
        TrackComponentPlacement placement = TrackComponentPlacement::Continuous;
        bool enabled = true;
        bool runningSurface = false;
        bool required = false; // required slot identity; visibility is independent
        glm::dvec3 offset{0.0};
        // Member endpoint relative to the same track frame. X can span a bay.
        glm::dvec3 endOffset{0.0};
        // X = rigid length, Y = lateral width, Z = vertical height, metres.
        glm::dvec3 dimensions{0.1};
        double spacing = 1.0;
        double phase = 0.0;
        std::uint32_t radialSegments = 12;
        TrackMaterial material;
        StaticMeshAssetReference asset; // empty = procedural; retained on failure
        double assetLength = 1.0; // authored local-X tile length for curve deformation
        glm::dvec3 assetScale{1.0};
        bool useAssetMaterials = true;
    };

    struct TrackConstructionVariant
    {
        std::string id;
        std::string displayName;
        std::vector<TrackComponent> components;
        glm::dvec3 supportAttachment{0.0};
    };

    struct TrackConstructionInterval
    {
        std::uint64_t id = 0;
        double start = 0.0;
        double end = 1.0;
        std::string typeId;
        std::string variantId;
        // Optional complete component snapshot, independent of catalog availability.
        std::vector<TrackComponent> components;
    };

    struct TrackStylePreset
    {
        std::string name;
        TrackGeometryFamily geometryFamily =
            TrackGeometryFamily::DualRailTubular;

        bool visible = true;
        bool railsVisible = true;

        std::uint32_t railCount = 2;
        std::vector<RailOffset> railOffsets;
        double railRadius = 0.065;
        std::uint32_t railRadialSegments = 12;
        TrackMaterial railMaterial;

        ContinuousSpineStyle spine;
        std::vector<RepeatingHardwareStyle> repeatingHardware;
        std::string familyId;
        std::string typeId;
        std::string variantId;
        std::vector<TrackComponent> components;
        std::vector<TrackConstructionVariant> variants;
        std::vector<TrackConstructionInterval> constructionIntervals;
        std::uint64_t nextConstructionIntervalId = 1;
        glm::dvec3 supportAttachment{0.0};
    };

    // Semantic authoring properties that a concrete style configuration may
    // expose to sparse region overrides. Support is derived from the
    // configuration's structure, never from its preset name.
    enum class TrackStyleProperty : std::uint8_t
    {
        Visibility,
        RailsVisibility,
        RailRadius,
        RailCenterSpacing,
        RailVerticalOffset,
        SpineEnabled,
        SpineType,
        SpineRadius,
        SpineDimensions,
        SpineVerticalOffset,
        HardwareEnabled,
        HardwareSpacing,
        RailMaterial,
        SpineMaterial,
        HardwareMaterial
    };

    // A region owns only explicitly authored differences. std::optional is
    // the inheritance marker: disengaged means use the document value.
    struct RegionTrackStyleOverrides
    {
        bool enabled = false;
        std::optional<bool> visible;
        std::optional<bool> railsVisible;
        std::optional<double> railRadius;
        std::optional<double> railCenterSpacing;
        std::optional<double> railVerticalOffset;
        std::optional<bool> spineEnabled;
        std::optional<ContinuousSpineType> spineType;
        std::optional<double> spineRadius;
        std::optional<glm::dvec2> spineDimensions;
        std::optional<double> spineVerticalOffset;
        std::optional<bool> hardwareEnabled;
        std::optional<double> hardwareSpacing;
        std::optional<TrackMaterial> railMaterial;
        std::optional<TrackMaterial> spineMaterial;
        std::optional<TrackMaterial> hardwareMaterial;
    };

    [[nodiscard]] TrackStylePreset createStandardDualRailPreset();
    [[nodiscard]] TrackStylePreset createModernSteelPreset();

    [[nodiscard]] bool supportsTrackStyleProperty(
        const TrackStylePreset& style,
        TrackStyleProperty property) noexcept;

    // The single canonical document-style + region-overrides resolution
    // path. Throws std::invalid_argument for unsupported or invalid authored
    // overrides and otherwise returns a fully validated concrete style.
    [[nodiscard]] TrackStylePreset resolveTrackStyle(
        const TrackStylePreset& documentStyle,
        const RegionTrackStyleOverrides& overrides);

    [[nodiscard]] bool hasTrackStylePropertyOverrides(
        const RegionTrackStyleOverrides& overrides) noexcept;

    // Returns a canonical package-relative identifier for repeating track
    // hardware below assets://track/. The retained builtin diagnostic
    // placeholder is the only builtin identifier accepted as authored
    // hardware. File-backed paths share normalizeStaticMeshAssetIdentifier
    // with the support connector assets.
    [[nodiscard]] std::string normalizeTrackHardwareAssetIdentifier(
        std::string_view identifier);

    // Throws std::invalid_argument for malformed values. Generation calls
    // this before allocating output, so invalid styles cannot publish partial
    // meshes or instance data.
    void validateTrackStyle(const TrackStylePreset& style);

    struct TrackMeshVertex
    {
        glm::vec3 position{0.0F};
        glm::vec3 normal{0.0F, 0.0F, 1.0F};
        glm::vec2 uv{0.0F};
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

    struct TrackComponentMesh
    {
        ContinuousTrackMesh mesh;
        std::vector<TrackMaterial> materials;
    };
    using TrackComponentMeshResolver = std::function<
        std::shared_ptr<const TrackComponentMesh>(std::string_view)>;
    // Engine supplies this per calling thread. Core owns no platform/asset I/O.
    void setTrackComponentMeshResolver(TrackComponentMeshResolver resolver);
    [[nodiscard]] glm::dvec3 trackSupportAttachmentAt(
        const TrackStylePreset& style, double station);
    [[nodiscard]] TrackStylePreset componentStyleInCoordinateUnits(
        const TrackStylePreset& style, double metersPerCoordinateUnit);

    // Versioned component payload shared by catalog and document serialization.
    [[nodiscard]] std::string serializeTrackComponents(const TrackStylePreset& style);
    void deserializeTrackComponents(TrackStylePreset& style, std::string_view json);

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

    // Product-level builders used by presentation invalidation. Both consume
    // already solved centerline/frame samples and never advance authored-track
    // geometry or physics generation.
    [[nodiscard]] RenderableTrack generateContinuousTrackPresentation(
        std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style);
    [[nodiscard]] std::vector<HardwareInstanceBatch>
    generateTrackHardwarePresentation(
        std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style,
        bool includeHardwareAtEnd = true);

    // Builds renderer-neutral indexed rail geometry and reusable-hardware
    // placement from the canonical centerline/frame samples. The samples are
    // not regenerated or re-framed here.
    [[nodiscard]] RenderableTrack generateRenderableTrack(
        std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style,
        bool includeHardwareAtEnd = true
    );
}
