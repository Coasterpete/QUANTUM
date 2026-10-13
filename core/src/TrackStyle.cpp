#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/coaster/TrackComponentDiagnostics.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <set>
#include <map>
#include <array>

namespace quantum::coaster
{
    namespace
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr std::uint32_t firstRailComponentId = 1;
        constexpr std::uint32_t spineComponentId = 3;
        constexpr std::uint32_t firstHardwareComponentId = 1000;

        // Temporary presentation dimensions live together here until the
        // user's Blender master model supplies measured production values.
        constexpr double modernSteelRailDiameter = 0.15;
        constexpr double modernSteelRailSpacing = 1.10;
        constexpr std::uint32_t modernSteelRailRadialSegments = 16;
        constexpr glm::dvec2 modernSteelSpineDimensions{0.32, 0.42};
        constexpr double modernSteelSpineVerticalOffset = -0.46;
        constexpr double modernSteelCrosstieSpacing = 0.75;
        constexpr double modernSteelCrosstieVerticalOffset = -0.12;

        [[nodiscard]] bool finite(const glm::dvec2& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y);
        }

        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] bool finite(const glm::vec4& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z) && std::isfinite(value.w);
        }

        void validateMaterial(
            const TrackMaterial& material,
            const char* const context)
        {
            if (!finite(material.baseColor)
                || glm::any(glm::lessThan(material.baseColor, glm::vec4{0.0F}))
                || glm::any(glm::greaterThan(material.baseColor, glm::vec4{1.0F}))
                || !std::isfinite(material.metallic)
                || !std::isfinite(material.roughness)
                || material.metallic < 0.0F || material.metallic > 1.0F
                || material.roughness < 0.0F || material.roughness > 1.0F)
            {
                throw std::invalid_argument(
                    std::string(context)
                    + " material factors must contain finite values in [0, 1]."
                );
            }
        }

        [[nodiscard]] std::uint32_t checkedIndex(
            const std::size_t value,
            const char* const context)
        {
            if (value > std::numeric_limits<std::uint32_t>::max())
            {
                throw std::length_error(context);
            }
            return static_cast<std::uint32_t>(value);
        }

        [[nodiscard]] glm::vec3 finiteFloatVector(
            const glm::dvec3& value,
            const char* const context)
        {
            const glm::vec3 converted{value};
            if (!std::isfinite(converted.x) || !std::isfinite(converted.y)
                || !std::isfinite(converted.z))
            {
                throw std::runtime_error(context);
            }
            return converted;
        }

        void validateSamples(
            const std::span<const RiderLocalGeometryState> samples)
        {
            if (samples.size() < 2)
            {
                throw std::invalid_argument(
                    "Renderable track generation requires at least two centerline samples."
                );
            }

            for (std::size_t index = 0; index < samples.size(); ++index)
            {
                const RiderLocalGeometryState& sample = samples[index];
                if (!std::isfinite(sample.distance)
                    || !finite(sample.position)
                    || !finite(sample.frame.tangent)
                    || !finite(sample.frame.lateral)
                    || !finite(sample.frame.up))
                {
                    throw std::invalid_argument(
                        "Renderable track generation received a non-finite centerline sample."
                    );
                }
                if (index > 0
                    && sample.distance <= samples[index - 1].distance)
                {
                    throw std::invalid_argument(
                        "Renderable track centerline distances must increase strictly."
                    );
                }
            }
        }

        struct InterpolatedTrackFrame
        {
            glm::dvec3 position{0.0};
            glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        };

        [[nodiscard]] glm::dquat frameOrientation(
            const geometry::CurveFrame& frame)
        {
            const glm::dmat3 rotation{
                frame.tangent,
                frame.lateral,
                frame.up
            };
            return glm::normalize(glm::quat_cast(rotation));
        }

        [[nodiscard]] InterpolatedTrackFrame interpolateTrackFrame(
            const std::span<const RiderLocalGeometryState> samples,
            const double distance)
        {
            const auto upper = std::lower_bound(
                samples.begin(),
                samples.end(),
                distance,
                [](const RiderLocalGeometryState& sample, const double target)
                {
                    return sample.distance < target;
                }
            );

            if (upper == samples.begin())
            {
                return {upper->position, frameOrientation(upper->frame)};
            }
            if (upper == samples.end())
            {
                const auto& sample = samples.back();
                return {sample.position, frameOrientation(sample.frame)};
            }

            const RiderLocalGeometryState& after = *upper;
            const RiderLocalGeometryState& before = *(upper - 1);
            const double span = after.distance - before.distance;
            const double amount = std::clamp(
                (distance - before.distance) / span,
                0.0,
                1.0
            );

            glm::dquat beforeOrientation = frameOrientation(before.frame);
            glm::dquat afterOrientation = frameOrientation(after.frame);
            if (glm::dot(beforeOrientation, afterOrientation) < 0.0)
            {
                afterOrientation = -afterOrientation;
            }

            return {
                glm::mix(before.position, after.position, amount),
                glm::normalize(glm::slerp(
                    beforeOrientation,
                    afterOrientation,
                    amount
                ))
            };
        }

        [[nodiscard]] glm::dquat localAdjustment(
            const glm::dvec3& rotation)
        {
            const glm::dquat x = glm::angleAxis(
                rotation.x, glm::dvec3{1.0, 0.0, 0.0});
            const glm::dquat y = glm::angleAxis(
                rotation.y, glm::dvec3{0.0, 1.0, 0.0});
            const glm::dquat z = glm::angleAxis(
                rotation.z, glm::dvec3{0.0, 0.0, 1.0});
            return glm::normalize(z * y * x);
        }

        [[nodiscard]] bool finite(const glm::mat4& matrix) noexcept
        {
            for (int column = 0; column < 4; ++column)
            {
                for (int row = 0; row < 4; ++row)
                {
                    if (!std::isfinite(matrix[column][row]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        struct ProfileEdge
        {
            glm::dvec2 begin{0.0};
            glm::dvec2 end{0.0};
            glm::dvec2 beginNormal{0.0, 1.0};
            glm::dvec2 endNormal{0.0, 1.0};
        };

        [[nodiscard]] std::vector<ProfileEdge> ellipticalProfile(
            const glm::dvec2 diameters,
            const std::uint32_t segmentCount)
        {
            const glm::dvec2 radii = diameters * 0.5;
            std::vector<ProfileEdge> profile;
            profile.reserve(segmentCount);
            const auto point = [radii](const double angle)
            {
                return glm::dvec2{
                    radii.x * std::cos(angle),
                    radii.y * std::sin(angle)};
            };
            const auto normal = [radii](const double angle)
            {
                return glm::normalize(glm::dvec2{
                    std::cos(angle) / radii.x,
                    std::sin(angle) / radii.y});
            };
            for (std::uint32_t segment = 0; segment < segmentCount; ++segment)
            {
                const double beginAngle = 2.0 * pi
                    * static_cast<double>(segment)
                    / static_cast<double>(segmentCount);
                const double endAngle = 2.0 * pi
                    * static_cast<double>(segment + 1)
                    / static_cast<double>(segmentCount);
                profile.push_back({
                    point(beginAngle), point(endAngle),
                    normal(beginAngle), normal(endAngle)});
            }
            return profile;
        }

        [[nodiscard]] std::vector<ProfileEdge> boxProfile(
            const glm::dvec2 dimensions)
        {
            const glm::dvec2 half = dimensions * 0.5;
            return {
                {{half.x, -half.y}, {half.x, half.y}, {1.0, 0.0}, {1.0, 0.0}},
                {{half.x, half.y}, {-half.x, half.y}, {0.0, 1.0}, {0.0, 1.0}},
                {{-half.x, half.y}, {-half.x, -half.y}, {-1.0, 0.0}, {-1.0, 0.0}},
                {{-half.x, -half.y}, {half.x, -half.y}, {0.0, -1.0}, {0.0, -1.0}}
            };
        }

        void appendProfileSweep(
            ContinuousTrackMesh& mesh,
            const std::span<const RiderLocalGeometryState> samples,
            const std::span<const ProfileEdge> profile,
            const RailOffset offset,
            const std::uint32_t materialIndex,
            const std::uint32_t componentId)
        {
            const std::size_t verticesPerRing = profile.size() * 2;
            const std::size_t firstVertex = mesh.vertices.size();
            const std::uint32_t firstIndex = checkedIndex(
                mesh.triangleIndices.size(),
                "A profile-sweep triangle-index stream exceeds 32 bits.");
            static_cast<void>(checkedIndex(
                firstVertex + samples.size() * verticesPerRing,
                "A profile-sweep vertex stream exceeds 32 bits."));

            for (const RiderLocalGeometryState& sample : samples)
            {
                const glm::dvec3 center = sample.position
                    + sample.frame.lateral * offset.lateral
                    + sample.frame.up * offset.vertical;
                for (const ProfileEdge& edge : profile)
                {
                    for (const auto& [point, profileNormal] : {
                        std::pair{edge.begin, edge.beginNormal},
                        std::pair{edge.end, edge.endNormal}})
                    {
                        const glm::dvec3 position = center
                            + sample.frame.lateral * point.x
                            + sample.frame.up * point.y;
                        const glm::dvec3 normal = glm::normalize(
                            sample.frame.lateral * profileNormal.x
                            + sample.frame.up * profileNormal.y);
                        mesh.vertices.push_back({
                            finiteFloatVector(position,
                                "A generated profile-sweep position is outside the finite float range."),
                            finiteFloatVector(normal,
                                "A generated profile-sweep normal is outside the finite float range.")});
                    }
                }
            }

            for (std::size_t ring = 0; ring + 1 < samples.size(); ++ring)
            {
                for (std::size_t edge = 0; edge < profile.size(); ++edge)
                {
                    const std::uint32_t a = checkedIndex(
                        firstVertex + ring * verticesPerRing + edge * 2,
                        "A profile-sweep index exceeds 32 bits.");
                    const std::uint32_t d = a + 1;
                    const std::uint32_t b = checkedIndex(
                        firstVertex + (ring + 1) * verticesPerRing + edge * 2,
                        "A profile-sweep index exceeds 32 bits.");
                    const std::uint32_t c = b + 1;
                    mesh.triangleIndices.insert(
                        mesh.triangleIndices.end(), {a, c, b, a, d, c});
                    mesh.edgeIndices.insert(
                        mesh.edgeIndices.end(), {a, b, d, c});
                }
            }

            for (std::size_t ring = 0; ring < samples.size(); ++ring)
            {
                for (std::size_t edge = 0; edge < profile.size(); ++edge)
                {
                    const std::uint32_t begin = checkedIndex(
                        firstVertex + ring * verticesPerRing + edge * 2,
                        "A profile-sweep edge index exceeds 32 bits.");
                    mesh.edgeIndices.insert(
                        mesh.edgeIndices.end(), {begin, begin + 1});
                }
            }

            mesh.submeshes.push_back({
                firstIndex,
                checkedIndex(mesh.triangleIndices.size() - firstIndex,
                    "A profile-sweep submesh index count exceeds 32 bits."),
                materialIndex,
                componentId});
        }
    }

    TrackStylePreset createStandardDualRailPreset()
    {
        TrackStylePreset style;
        style.name = "StandardDualRail";
        style.geometryFamily = TrackGeometryFamily::DualRailTubular;
        style.railCount = 2;
        style.railOffsets = {
            RailOffset{-0.6, 0.0},
            RailOffset{0.6, 0.0}
        };
        style.railRadius = 0.065;
        style.railRadialSegments = 12;
        style.railMaterial.baseColor = {0.20F, 0.34F, 0.48F, 1.0F};

        RepeatingHardwareStyle testCrosstie;
        testCrosstie.asset = {
            "assets://track/test-crosstie-placeholder.glb",
            true
        };
        testCrosstie.spacing = 1.5;
        testCrosstie.startOffset = 0.0;
        testCrosstie.localPosition = {0.0, 0.0, -0.11};
        testCrosstie.localScale = {1.0, 1.0, 1.0};
        testCrosstie.materialOverride = TrackMaterial{
            glm::vec4{0.28F, 0.30F, 0.32F, 1.0F}
        };
        style.repeatingHardware.push_back(std::move(testCrosstie));
        return style;
    }

    TrackStylePreset createModernSteelPreset()
    {
        TrackStylePreset style;
        style.name = "ModernSteel";
        style.geometryFamily = TrackGeometryFamily::DualRailTubular;
        style.railCount = 2;
        style.railOffsets = {
            {-modernSteelRailSpacing * 0.5, 0.0},
            {modernSteelRailSpacing * 0.5, 0.0}};
        style.railRadius = modernSteelRailDiameter * 0.5;
        style.railRadialSegments = modernSteelRailRadialSegments;
        style.railMaterial.baseColor = {0.72F, 0.16F, 0.08F, 1.0F};
        style.railMaterial.roughness = 0.24F;

        style.spine.enabled = true;
        style.spine.type = ContinuousSpineType::Box;
        style.spine.offset = {0.0, modernSteelSpineVerticalOffset};
        style.spine.dimensions = modernSteelSpineDimensions;
        style.spine.radialSegments = 12;
        style.spine.material.baseColor = {0.20F, 0.23F, 0.27F, 1.0F};
        style.spine.material.metallic = 0.8F;
        style.spine.material.roughness = 0.58F;

        RepeatingHardwareStyle crosstie;
        crosstie.asset = {
            "assets://track/test-crosstie-placeholder.glb", true};
        crosstie.spacing = modernSteelCrosstieSpacing;
        crosstie.localPosition = {0.0, 0.0,
            modernSteelCrosstieVerticalOffset};
        crosstie.materialOverride = TrackMaterial{
            glm::vec4{0.36F, 0.39F, 0.43F, 1.0F}};
        style.repeatingHardware.push_back(std::move(crosstie));
        return style;
    }

    bool hasTrackStylePropertyOverrides(
        const RegionTrackStyleOverrides& overrides) noexcept
    {
        return overrides.visible.has_value()
            || overrides.railsVisible.has_value()
            || overrides.railRadius.has_value()
            || overrides.railCenterSpacing.has_value()
            || overrides.railVerticalOffset.has_value()
            || overrides.spineEnabled.has_value()
            || overrides.spineType.has_value()
            || overrides.spineRadius.has_value()
            || overrides.spineDimensions.has_value()
            || overrides.spineVerticalOffset.has_value()
            || overrides.hardwareEnabled.has_value()
            || overrides.hardwareSpacing.has_value()
            || overrides.railMaterial.has_value()
            || overrides.spineMaterial.has_value()
            || overrides.hardwareMaterial.has_value();
    }

    bool supportsTrackStyleProperty(
        const TrackStylePreset& style,
        const TrackStyleProperty property) noexcept
    {
        if (style.geometryFamily == TrackGeometryFamily::Components)
            return property == TrackStyleProperty::Visibility || property == TrackStyleProperty::RailsVisibility;


        const bool hasRails = style.railCount > 0
            && style.railOffsets.size() == style.railCount;
        const bool hasConfiguredSpine = style.spine.type
                != ContinuousSpineType::None
            && style.spine.dimensions.x > 0.0
            && style.spine.dimensions.y > 0.0;
        bool commonRailVerticalOffset = hasRails;
        if (hasRails)
        {
            const double vertical = style.railOffsets.front().vertical;
            for (const RailOffset& offset : style.railOffsets)
            {
                commonRailVerticalOffset = commonRailVerticalOffset
                    && offset.vertical == vertical;
            }
        }

        switch (property)
        {
        case TrackStyleProperty::Visibility:
            return true;
        case TrackStyleProperty::RailsVisibility:
        case TrackStyleProperty::RailMaterial:
            return hasRails;
        case TrackStyleProperty::RailRadius:
            return hasRails
                && style.geometryFamily == TrackGeometryFamily::DualRailTubular;
        case TrackStyleProperty::RailCenterSpacing:
            return style.railCount == 2 && style.railOffsets.size() == 2;
        case TrackStyleProperty::RailVerticalOffset:
            return commonRailVerticalOffset;
        case TrackStyleProperty::SpineEnabled:
        case TrackStyleProperty::SpineType:
        case TrackStyleProperty::SpineVerticalOffset:
        case TrackStyleProperty::SpineMaterial:
            return hasConfiguredSpine;
        case TrackStyleProperty::SpineRadius:
            return hasConfiguredSpine
                && style.spine.type == ContinuousSpineType::Tubular;
        case TrackStyleProperty::SpineDimensions:
            return hasConfiguredSpine
                && style.spine.type == ContinuousSpineType::Box;
        case TrackStyleProperty::HardwareEnabled:
        case TrackStyleProperty::HardwareSpacing:
            return !style.repeatingHardware.empty();
        case TrackStyleProperty::HardwareMaterial:
            return !style.repeatingHardware.empty()
                && std::ranges::all_of(style.repeatingHardware,
                    [](const RepeatingHardwareStyle& hardware)
                    {
                        return hardware.materialOverride.has_value();
                    });
        }
        return false;
    }

    TrackStylePreset resolveTrackStyle(
        const TrackStylePreset& documentStyle,
        const RegionTrackStyleOverrides& overrides)
    {
        validateTrackStyle(documentStyle);
        if (!overrides.enabled)
        {
            return documentStyle;
        }

        if (documentStyle.geometryFamily == TrackGeometryFamily::Components)
        {
            // Keep legacy dimensional overrides authored but inactive when a
            // document changes topology. Visibility remains meaningful; component
            // materials are edited in the numbered family's component inspector.
            // Switching back to the legacy configuration restores the overrides.
            auto resolved = documentStyle;
            if (overrides.visible) resolved.visible = *overrides.visible;
            if (overrides.railsVisible) resolved.railsVisible = *overrides.railsVisible;
            return resolved;
        }

        const auto requireSupported = [&documentStyle](
            const bool authored,
            const TrackStyleProperty property,
            const char* const name)
        {
            if (authored
                && !supportsTrackStyleProperty(documentStyle, property))
            {
                throw std::invalid_argument(
                    std::string("The current track style does not support a region ")
                    + name + " override.");
            }
        };

        requireSupported(overrides.visible.has_value(),
            TrackStyleProperty::Visibility, "visibility");
        requireSupported(overrides.railsVisible.has_value(),
            TrackStyleProperty::RailsVisibility, "rail-visibility");
        requireSupported(overrides.railRadius.has_value(),
            TrackStyleProperty::RailRadius, "rail-radius");
        requireSupported(overrides.railCenterSpacing.has_value(),
            TrackStyleProperty::RailCenterSpacing, "rail-spacing");
        requireSupported(overrides.railVerticalOffset.has_value(),
            TrackStyleProperty::RailVerticalOffset, "rail-offset");
        requireSupported(overrides.spineEnabled.has_value(),
            TrackStyleProperty::SpineEnabled, "spine-enabled");
        requireSupported(overrides.spineType.has_value(),
            TrackStyleProperty::SpineType, "spine-type");
        requireSupported(overrides.spineRadius.has_value(),
            TrackStyleProperty::SpineRadius, "spine-radius");
        requireSupported(overrides.spineDimensions.has_value(),
            TrackStyleProperty::SpineDimensions, "spine-dimensions");
        requireSupported(overrides.spineVerticalOffset.has_value(),
            TrackStyleProperty::SpineVerticalOffset, "spine-offset");
        requireSupported(overrides.hardwareEnabled.has_value(),
            TrackStyleProperty::HardwareEnabled, "hardware-enabled");
        requireSupported(overrides.hardwareSpacing.has_value(),
            TrackStyleProperty::HardwareSpacing, "hardware-spacing");
        requireSupported(overrides.railMaterial.has_value(),
            TrackStyleProperty::RailMaterial, "rail-material");
        requireSupported(overrides.spineMaterial.has_value(),
            TrackStyleProperty::SpineMaterial, "spine-material");
        requireSupported(overrides.hardwareMaterial.has_value(),
            TrackStyleProperty::HardwareMaterial, "hardware-material");

        TrackStylePreset resolved = documentStyle;
        if (overrides.visible) resolved.visible = *overrides.visible;
        if (overrides.railsVisible)
            resolved.railsVisible = *overrides.railsVisible;
        if (overrides.railRadius)
            resolved.railRadius = *overrides.railRadius;
        if (overrides.railCenterSpacing)
        {
            const double midpoint = (resolved.railOffsets[0].lateral
                + resolved.railOffsets[1].lateral) * 0.5;
            const double halfSpacing = *overrides.railCenterSpacing * 0.5;
            resolved.railOffsets[0].lateral = midpoint - halfSpacing;
            resolved.railOffsets[1].lateral = midpoint + halfSpacing;
        }
        if (overrides.railVerticalOffset)
        {
            for (RailOffset& offset : resolved.railOffsets)
                offset.vertical = *overrides.railVerticalOffset;
        }
        if (overrides.spineEnabled)
            resolved.spine.enabled = *overrides.spineEnabled;
        if (overrides.spineType)
            resolved.spine.type = *overrides.spineType;
        if (overrides.spineRadius)
            resolved.spine.dimensions = glm::dvec2{*overrides.spineRadius * 2.0};
        if (overrides.spineDimensions)
            resolved.spine.dimensions = *overrides.spineDimensions;
        if (overrides.spineVerticalOffset)
            resolved.spine.offset.vertical = *overrides.spineVerticalOffset;
        if (overrides.hardwareEnabled)
        {
            for (RepeatingHardwareStyle& hardware : resolved.repeatingHardware)
                hardware.enabled = *overrides.hardwareEnabled;
        }
        if (overrides.hardwareSpacing)
        {
            for (RepeatingHardwareStyle& hardware : resolved.repeatingHardware)
                hardware.spacing = *overrides.hardwareSpacing;
        }
        if (overrides.railMaterial)
            resolved.railMaterial = *overrides.railMaterial;
        if (overrides.spineMaterial)
            resolved.spine.material = *overrides.spineMaterial;
        if (overrides.hardwareMaterial)
        {
            for (RepeatingHardwareStyle& hardware : resolved.repeatingHardware)
                hardware.materialOverride = *overrides.hardwareMaterial;
        }

        validateTrackStyle(resolved);
        return resolved;
    }

    std::string normalizeTrackHardwareAssetIdentifier(
        const std::string_view identifier)
    {
        constexpr std::string_view diagnosticAsset =
            "builtin://diagnostic/track-hardware-placeholder";

        if (identifier.empty())
        {
            throw std::invalid_argument(
                "Track hardware asset identifier cannot be empty.");
        }

        std::string normalized{identifier};
        std::ranges::replace(normalized, '\\', '/');
        if (normalized == diagnosticAsset)
        {
            return normalized;
        }
        return normalizeStaticMeshAssetIdentifier(identifier, "track");
    }

    void validateTrackStyle(const TrackStylePreset& style)
    {
        if (style.name.empty())
        {
            throw std::invalid_argument("A track-style preset requires a name.");
        }

        switch (style.geometryFamily)
        {
        case TrackGeometryFamily::DualRailTubular:
        case TrackGeometryFamily::Components:
            break;
        default:
            throw std::invalid_argument("The track geometry family is invalid.");
        }

        if ((style.geometryFamily == TrackGeometryFamily::DualRailTubular && style.railCount != 2)
            || style.railOffsets.size() != style.railCount)
        {
            throw std::invalid_argument(
                "The DualRailTubular family requires exactly two rail offsets."
            );
        }
        if (!std::isfinite(style.railRadius) || style.railRadius <= 0.0)
        {
            throw std::invalid_argument("Rail radius must be finite and positive.");
        }
        if (style.railRadialSegments < 3
            || style.railRadialSegments > 128)
        {
            throw std::invalid_argument(
                "Rail radial tessellation must be in [3, 128]."
            );
        }
        for (const RailOffset& offset : style.railOffsets)
        {
            if (!std::isfinite(offset.lateral)
                || !std::isfinite(offset.vertical))
            {
                throw std::invalid_argument("Rail offsets must be finite.");
            }
        }
        validateMaterial(style.railMaterial, "Rail material");

        if (!std::isfinite(style.spine.offset.lateral)
            || !std::isfinite(style.spine.offset.vertical)
            || !finite(style.spine.dimensions))
        {
            throw std::invalid_argument("Continuous-spine parameters must be finite.");
        }
        switch (style.spine.type)
        {
        case ContinuousSpineType::None:
        case ContinuousSpineType::Tubular:
        case ContinuousSpineType::Box:
            break;
        default:
            throw std::invalid_argument("The continuous-spine type is invalid.");
        }
        if (style.spine.enabled
            && (style.spine.type == ContinuousSpineType::None
                || style.spine.dimensions.x <= 0.0
                || style.spine.dimensions.y <= 0.0))
        {
            throw std::invalid_argument(
                "An enabled continuous spine requires a type and positive dimensions."
            );
        }
        if (style.spine.enabled
            && style.spine.type == ContinuousSpineType::Tubular
            && (style.spine.radialSegments < 3
                || style.spine.radialSegments > 128))
        {
            throw std::invalid_argument(
                "Tubular spine radial tessellation must be in [3, 128].");
        }
        validateMaterial(style.spine.material, "Spine material");

        if (style.geometryFamily == TrackGeometryFamily::Components)
        {
            if (style.familyId.empty() || style.typeId.empty() || style.components.empty()
                || !finite(style.supportAttachment))
                throw std::invalid_argument("Component track requires identity, geometry and finite attachment.");
            const auto validateComponents = [](const std::vector<TrackComponent>& components)
            {
                std::set<std::string> ids;
                for (const auto& c : components)
                {
                    if (c.id.empty() || !ids.insert(c.id).second || c.materialSlot.empty()
                        || !finite(c.offset) || !finite(c.endOffset) || !finite(c.dimensions)
                        || glm::any(glm::lessThanEqual(c.dimensions, glm::dvec3{0}))
                        || !std::isfinite(c.spacing) || c.spacing <= 0
                        || !std::isfinite(c.phase) || c.phase < 0
                        || c.radialSegments < 3 || c.radialSegments > 128
                        || !std::isfinite(c.assetLength) || c.assetLength <= 0
                        || !finite(c.assetScale) || glm::any(glm::lessThanEqual(c.assetScale, glm::dvec3{0})))
                        throw std::invalid_argument("Invalid track component: " + c.id);
                    if (c.placement == TrackComponentPlacement::Continuous && c.offset.x != 0)
                        throw std::invalid_argument("Continuous component X offset must be zero; X spans the tile/route.");
                    if (c.provider > TrackComponentProvider::Beam || c.placement > TrackComponentPlacement::Junction)
                        throw std::invalid_argument("Unsupported component provider/placement.");
                    if ((c.provider == TrackComponentProvider::Member || c.provider == TrackComponentProvider::Beam) && c.placement == TrackComponentPlacement::Continuous)
                        throw std::invalid_argument("A point-to-point member requires repeated or junction placement.");
                    if ((c.provider == TrackComponentProvider::Member || c.provider == TrackComponentProvider::Beam)
                        && glm::length(c.endOffset - c.offset) < 1e-8)
                        throw std::invalid_argument("Structural member requires distinct endpoints.");
                    if (!c.asset.path.empty()) static_cast<void>(normalizeTrackHardwareAssetIdentifier(c.asset.path));
                    validateMaterial(c.material, "Component");
                }
            };
            validateComponents(style.components);
            std::set<std::string> variants;
            for (const auto& v : style.variants)
            {
                if (v.id.empty() || !variants.insert(v.id).second || v.components.empty() || !finite(v.supportAttachment))
                    throw std::invalid_argument("Invalid construction variant.");
                validateComponents(v.components);
            }
            if (!style.variantId.empty() && !variants.contains(style.variantId))
                throw std::invalid_argument("Unknown base construction variant.");
            std::set<std::uint64_t> ids;
            double lastEnd = -1;
            for (const auto& i : style.constructionIntervals)
            {
                if (!i.id || !ids.insert(i.id).second || !std::isfinite(i.start) || !std::isfinite(i.end)
                    || i.start < 0 || i.end <= i.start || i.start < lastEnd
                    || i.id >= style.nextConstructionIntervalId || i.typeId != style.typeId
                    || !variants.contains(i.variantId))
                    throw std::invalid_argument("Construction intervals require ordered nonoverlapping ranges, stable IDs and a supported same-type variant.");
                validateComponents(i.components);
                lastEnd = i.end;
            }
        }

        for (const RepeatingHardwareStyle& hardware : style.repeatingHardware)
        {
            if (!hardware.enabled)
            {
                continue;
            }
            if (hardware.asset.path.empty())
            {
                throw std::invalid_argument(
                    "Enabled repeating hardware requires an asset reference."
                );
            }
            static_cast<void>(normalizeTrackHardwareAssetIdentifier(
                hardware.asset.path));
            if (!std::isfinite(hardware.spacing) || hardware.spacing <= 0.0)
            {
                throw std::invalid_argument(
                    "Repeating-hardware spacing must be finite and positive."
                );
            }
            if (!std::isfinite(hardware.startOffset)
                || hardware.startOffset < 0.0)
            {
                throw std::invalid_argument(
                    "Repeating-hardware start offset must be finite and non-negative."
                );
            }
            if (!finite(hardware.localPosition)
                || !finite(hardware.localRotation))
            {
                throw std::invalid_argument(
                    "Repeating-hardware position and rotation must be finite."
                );
            }
            if (!finite(hardware.localScale)
                || hardware.localScale.x <= 0.0
                || hardware.localScale.y <= 0.0
                || hardware.localScale.z <= 0.0)
            {
                throw std::invalid_argument(
                    "Repeating-hardware scale must be finite and positive."
                );
            }
            switch (hardware.frameFollow)
            {
            case HardwareFrameFollow::TrackFrame:
            case HardwareFrameFollow::WorldAligned:
                break;
            default:
                throw std::invalid_argument(
                    "Repeating-hardware frame-follow behavior is invalid."
                );
            }
            if (hardware.materialOverride.has_value())
            {
                validateMaterial(*hardware.materialOverride, "Hardware material override");
            }
        }
    }


    namespace
    {
        thread_local TrackComponentMeshResolver componentMeshResolver;
        thread_local TrackComponentDiagnosticSink componentDiagnosticSink;

        RiderLocalGeometryState componentSample(std::span<const RiderLocalGeometryState> samples, double station)
        {
            const auto interpolated = interpolateTrackFrame(samples, station);
            auto result = samples.front();
            result.distance = station;
            result.position = interpolated.position;
            result.frame.tangent = interpolated.orientation * glm::dvec3{1,0,0};
            result.frame.lateral = interpolated.orientation * glm::dvec3{0,1,0};
            result.frame.up = interpolated.orientation * glm::dvec3{0,0,1};
            return result;
        }

        std::vector<RiderLocalGeometryState> componentRange(std::span<const RiderLocalGeometryState> samples, double start, double end)
        {
            std::vector<RiderLocalGeometryState> result;
            result.push_back(componentSample(samples, start));
            for (const auto& sample : samples)
                if (sample.distance > start && sample.distance < end) result.push_back(sample);
            result.push_back(componentSample(samples, end));
            return result;
        }

        const TrackConstructionVariant* componentVariant(const TrackStylePreset& style, std::string_view id)
        {
            for (const auto& variant : style.variants) if (variant.id == id) return &variant;
            return nullptr;
        }

        const std::vector<TrackComponent>& componentsAt(const TrackStylePreset& style, double station)
        {
            for (const auto& interval : style.constructionIntervals)
                if (station >= interval.start && station < interval.end)
                {
                    if (!interval.components.empty()) return interval.components;
                    return componentVariant(style, interval.variantId)->components;
                }
            return style.components;
        }

        void appendRigidBox(ContinuousTrackMesh& mesh, const glm::dmat4& transform,
            glm::dvec3 dimensions, std::uint32_t material, std::uint32_t component)
        {
            const auto first = checkedIndex(mesh.triangleIndices.size(), "Component index overflow.");
            // Six independent flat-shaded faces; explicit caps avoid open ties.
            for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1})
            {
                glm::dvec3 n{0}; n[axis] = sign;
                const int a = (axis + 1) % 3, b = (axis + 2) % 3;
                const auto base = checkedIndex(mesh.vertices.size(), "Component vertex overflow.");
                for (const glm::ivec2 corner : {glm::ivec2{-1,-1}, {1,-1}, {1,1}, {-1,1}})
                {
                    glm::dvec3 point{0}; point[axis] = sign * dimensions[axis] * .5;
                    point[a] = corner.x * dimensions[a] * .5;
                    point[b] = corner.y * sign * dimensions[b] * .5;
                    mesh.vertices.push_back({finiteFloatVector(glm::dvec3{transform * glm::dvec4{point,1}}, "Component position overflow."),
                        finiteFloatVector(glm::normalize(glm::dmat3{transform} * n), "Component normal overflow.")});
                }
                for (auto i : {0u,1u,2u,0u,2u,3u}) mesh.triangleIndices.push_back(base+i);
                for (auto i : {0u,1u,1u,2u,2u,3u,3u,0u}) mesh.edgeIndices.push_back(base+i);
            }
            mesh.submeshes.push_back({first,36,material,component});
        }

        std::uint32_t componentMaterialIndex(RenderableTrack& result, const TrackMaterial& material)
        {
            for (std::size_t index=0; index<result.materials.size(); ++index)
            {
                const auto& existing=result.materials[index];
                if (existing.baseColor==material.baseColor && existing.metallic==material.metallic && existing.roughness==material.roughness)
                    return checkedIndex(index,"Component material overflow.");
            }
            const auto index=checkedIndex(result.materials.size(),"Component material overflow.");
            result.materials.push_back(material);
            return index;
        }

        void appendComponentAsset(RenderableTrack& result, const TrackComponentMesh& asset,
            const TrackComponent& c, std::span<const RiderLocalGeometryState> samples,
            double tileStart, double start, double end, bool deform,
            std::uint32_t material, std::uint32_t component)
        {
            // Clip each triangle to the active tile's local-X domain before
            // deforming. This avoids duplicate/clamped triangles at interval ends.
            const double scaleX = c.assetScale.x;
            const double low = deform ? (start-tileStart)/scaleX : -std::numeric_limits<double>::infinity();
            const double high = deform ? (end-tileStart)/scaleX : std::numeric_limits<double>::infinity();
            for (const auto& sub : asset.mesh.submeshes)
            {
                auto effectiveMaterial=material;
                if (c.useAssetMaterials && sub.materialIndex<asset.materials.size())
                {
                    effectiveMaterial=componentMaterialIndex(result,asset.materials[sub.materialIndex]);
                }
                const auto first = checkedIndex(result.continuousMesh.triangleIndices.size(), "Custom component overflow.");
                for (std::uint32_t tri = sub.firstIndex; tri < sub.firstIndex+sub.indexCount; tri += 3)
                {
                    std::vector<TrackMeshVertex> polygon;
                    for (unsigned k=0; k<3; ++k) polygon.push_back(asset.mesh.vertices.at(asset.mesh.triangleIndices.at(tri+k)));
                    for (const auto& boundary : {std::pair{low,true}, std::pair{high,false}})
                    {
                        if (!std::isfinite(boundary.first)) continue;
                        std::vector<TrackMeshVertex> clipped;
                        for (std::size_t k=0; k<polygon.size(); ++k)
                        {
                            const auto& a=polygon[k]; const auto& b=polygon[(k+1)%polygon.size()];
                            const bool inA=boundary.second ? a.position.x>=boundary.first : a.position.x<=boundary.first;
                            const bool inB=boundary.second ? b.position.x>=boundary.first : b.position.x<=boundary.first;
                            if (inA) clipped.push_back(a);
                            if (inA!=inB)
                            {
                                const float t=static_cast<float>((boundary.first-a.position.x)/(b.position.x-a.position.x));
                                clipped.push_back({glm::mix(a.position,b.position,t),glm::mix(a.normal,b.normal,t),glm::mix(a.uv,b.uv,t)});
                            }
                        }
                        polygon=std::move(clipped);
                    }
                    if (polygon.size()<3) continue;
                    const auto base=checkedIndex(result.continuousMesh.vertices.size(), "Custom vertex overflow.");
                    for (const auto& vertex : polygon)
                    {
                        glm::dvec3 local=glm::dvec3{vertex.position}*c.assetScale;
                        const auto frame=interpolateTrackFrame(samples, deform ? tileStart+local.x : tileStart);
                        if (deform) local.x=0;
                        local+=c.offset;
                        const auto position=frame.position+frame.orientation*local;
                        const auto normal=glm::normalize(frame.orientation*(glm::dvec3{vertex.normal}/c.assetScale));
                        result.continuousMesh.vertices.push_back({finiteFloatVector(position,"Custom position overflow."),finiteFloatVector(normal,"Custom normal overflow."),vertex.uv});
                    }
                    for (std::uint32_t k=1; k+1<polygon.size(); ++k)
                        for (auto i : {base,base+k,base+k+1}) result.continuousMesh.triangleIndices.push_back(i);
                    for (std::uint32_t k=0; k<polygon.size(); ++k)
                    {
                        result.continuousMesh.edgeIndices.push_back(base+k);
                        result.continuousMesh.edgeIndices.push_back(base+(k+1)%static_cast<std::uint32_t>(polygon.size()));
                    }
                }
                const auto count=checkedIndex(result.continuousMesh.triangleIndices.size(), "Custom index overflow.")-first;
                if (count) result.continuousMesh.submeshes.push_back({first,count,effectiveMaterial,component});
            }
        }

        RenderableTrack generateComponentTrack(std::span<const RiderLocalGeometryState> samples, const TrackStylePreset& style)
        {
            RenderableTrack result;
            std::vector<double> cuts{samples.front().distance,samples.back().distance};
            for (const auto& i : style.constructionIntervals)
                for (double station : {i.start,i.end})
                    if (station>cuts.front() && station<samples.back().distance) cuts.push_back(station);
            std::ranges::sort(cuts); cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
            std::map<std::string,std::uint32_t> componentIds;
            for (std::size_t range=0; range+1<cuts.size(); ++range)
            {
                const double start=cuts[range], end=cuts[range+1];
                const auto clipped=componentRange(samples,start,end);
                for (const auto& c : componentsAt(style,(start+end)*.5))
                {
                    if (!c.enabled || (c.runningSurface && !style.railsVisible)) continue;
                    const auto [idIt, inserted]=componentIds.try_emplace(c.id,static_cast<std::uint32_t>(componentIds.size()+1));
                    const auto component=idIt->second;
                    const auto material=componentMaterialIndex(result,c.material);
                    std::shared_ptr<const TrackComponentMesh> custom;
                    if (!c.asset.path.empty() && componentMeshResolver) custom=componentMeshResolver(c.asset.path);
                    if (c.placement==TrackComponentPlacement::Continuous)
                    {
                        if (custom)
                        {
                            double assetMin=std::numeric_limits<double>::infinity(),assetMax=-assetMin;
                            for (const auto& vertex:custom->mesh.vertices)
                            { assetMin=std::min(assetMin,static_cast<double>(vertex.position.x)); assetMax=std::max(assetMax,static_cast<double>(vertex.position.x)); }
                            if (std::abs(assetMin)>1e-5 || std::abs(assetMax-c.assetLength)>1e-5*std::max(1.0,c.assetLength))
                            {
                                if (componentDiagnosticSink) componentDiagnosticSink(c.asset.path,
                                    "Slot '"+c.id+"' uses procedural fallback: continuous GLB X bounds must be [0, tile length].");
                                custom.reset();
                            }
                            else if (componentDiagnosticSink) componentDiagnosticSink(c.asset.path,{});
                        }
                        if (custom)
                        {
                            const double length=c.assetLength*c.assetScale.x;
                            const auto tiles=std::ceil((end-start)/length)+1;
                            if (tiles>1000000) throw std::length_error("Too many custom component tiles.");
                            for (double tile=std::floor(start/length)*length; tile<end; tile+=length)
                                appendComponentAsset(result,*custom,c,samples,tile,std::max(start,tile),std::min(end,tile+length),true,material,component);
                        }
                        else
                        {
                            const auto profile=c.provider==TrackComponentProvider::Tube
                                ? ellipticalProfile({c.dimensions.y,c.dimensions.z},c.radialSegments)
                                : boxProfile({c.dimensions.y,c.dimensions.z});
                            appendProfileSweep(result.continuousMesh,clipped,profile,{c.offset.y,c.offset.z},material,component);
                        }
                        continue;
                    }
                    if (c.placement==TrackComponentPlacement::Junction && start>1e-9
                        && std::ranges::none_of(style.constructionIntervals,[start](const auto& interval) {
                            return std::abs(interval.start-start)<1e-9 || std::abs(interval.end-start)<1e-9;
                        })) continue;
                    double first=c.phase+std::ceil((start-c.phase)/c.spacing-1e-10)*c.spacing;
                    if (c.placement==TrackComponentPlacement::Junction) first=start;
                    if ((end-start)/c.spacing>1000000) throw std::length_error("Too many repeated track members.");
                    for (double station=first; station<end-1e-9; station+=c.spacing)
                    {
                        if (custom)
                        {
                            // Repeated custom components retain one GPU mesh and
                            // use the established instance/retirement pipeline.
                            HardwareInstanceBatch batch;
                            batch.asset=c.asset;
                            if (!c.useAssetMaterials) batch.materialOverride=c.material;
                            auto frame=interpolateTrackFrame(samples,station);
                            auto transform=glm::translate(glm::dmat4{1},frame.position)*glm::mat4_cast(frame.orientation);
                            transform=glm::translate(transform,c.offset); transform=glm::scale(transform,c.assetScale);
                            batch.instances.push_back({glm::mat4{transform},component,0,0});
                            // Coalesce matching slot instances within this range.
                            if (!result.hardwareBatches.empty() && result.hardwareBatches.back().asset==c.asset
                                && result.hardwareBatches.back().instances.front().componentId==component
                                && result.hardwareBatches.back().materialOverride.has_value()==batch.materialOverride.has_value()
                                && (!batch.materialOverride || (
                                    result.hardwareBatches.back().materialOverride->baseColor==batch.materialOverride->baseColor
                                    && result.hardwareBatches.back().materialOverride->metallic==batch.materialOverride->metallic
                                    && result.hardwareBatches.back().materialOverride->roughness==batch.materialOverride->roughness)))
                                result.hardwareBatches.back().instances.push_back(batch.instances.front());
                            else result.hardwareBatches.push_back(std::move(batch));
                        }
                        else if (c.provider==TrackComponentProvider::Member || c.provider==TrackComponentProvider::Beam)
                        {
                            if (station+c.offset.x<start-1e-9 || station+c.endOffset.x<start-1e-9
                                || station+c.offset.x>end+1e-9 || station+c.endOffset.x>end+1e-9) continue; // no brace crossing a variant boundary
                            const auto a=componentSample(samples,station+c.offset.x);
                            const auto b=componentSample(samples,station+c.endOffset.x);
                            const auto pa=a.position+a.frame.lateral*c.offset.y+a.frame.up*c.offset.z;
                            const auto pb=b.position+b.frame.lateral*c.endOffset.y+b.frame.up*c.endOffset.z;
                            const auto tangent=glm::normalize(pb-pa);
                            auto lateral=glm::cross(a.frame.up,tangent);
                            if (glm::length(lateral)<1e-8) lateral=glm::cross(a.frame.lateral,tangent);
                            lateral=glm::normalize(lateral);
                            auto p=a,q=b; p.position=pa;q.position=pb;
                            p.frame.tangent=q.frame.tangent=tangent;
                            p.frame.lateral=q.frame.lateral=lateral;
                            p.frame.up=q.frame.up=glm::normalize(glm::cross(tangent,lateral));
                            const std::array pair{p,q};
                            const auto profile=c.provider==TrackComponentProvider::Beam
                                ? boxProfile({c.dimensions.y,c.dimensions.z})
                                : ellipticalProfile({c.dimensions.y,c.dimensions.z},8);
                            appendProfileSweep(result.continuousMesh,pair,profile,{},material,component);
                        }
                        else
                        {
                            // Trim rigid ties at boundaries; prevents overlap into
                            // a neighboring construction interval.
                            const double length=c.placement==TrackComponentPlacement::Junction ? c.dimensions.x
                                : std::min(c.dimensions.x,2*std::min(station-start,end-station));
                            if (length>1e-8)
                            {
                                const auto frame=interpolateTrackFrame(samples,station);
                                auto transform=glm::translate(glm::dmat4{1},frame.position)*glm::mat4_cast(frame.orientation);
                                transform=glm::translate(transform,c.offset);
                                if (c.provider==TrackComponentProvider::Tube)
                                {
                                    auto a=componentSample(samples,station),b=a;
                                    a.position=glm::dvec3{transform*glm::dvec4{-length*.5,0,0,1}};
                                    b.position=glm::dvec3{transform*glm::dvec4{length*.5,0,0,1}};
                                    const std::array pair{a,b};
                                    appendProfileSweep(result.continuousMesh,pair,
                                        ellipticalProfile({c.dimensions.y,c.dimensions.z},c.radialSegments),{},material,component);
                                }
                                else appendRigidBox(result.continuousMesh,transform,{length,c.dimensions.y,c.dimensions.z},material,component);
                            }
                        }
                        if (c.placement==TrackComponentPlacement::Junction) break;
                    }
                }
            }
            // Batch the baked repeated blockouts by contiguous component and
            // material. Thousands of ties do not become thousands of draws.
            std::vector<TrackSubmesh> batches;
            for (const auto& sub:result.continuousMesh.submeshes)
            {
                if (!batches.empty() && batches.back().componentId==sub.componentId
                    && batches.back().materialIndex==sub.materialIndex
                    && batches.back().firstIndex+batches.back().indexCount==sub.firstIndex)
                    batches.back().indexCount+=sub.indexCount;
                else batches.push_back(sub);
            }
            result.continuousMesh.submeshes=std::move(batches);
            return result;
        }
    }

    TrackStylePreset componentStyleInCoordinateUnits(const TrackStylePreset& style, double metersPerCoordinateUnit)
    {
        if (!std::isfinite(metersPerCoordinateUnit) || metersPerCoordinateUnit<=0)
            throw std::invalid_argument("Track visual scale must be finite and positive.");
        auto result=style;
        if (style.geometryFamily!=TrackGeometryFamily::Components) return result;
        const double scale=1/metersPerCoordinateUnit;
        const auto convert=[scale](std::vector<TrackComponent>& components) {
            for (auto& c:components)
            {
                c.offset*=scale; c.endOffset*=scale; c.dimensions*=scale;
                c.spacing*=scale; c.phase*=scale; c.assetScale*=scale;
            }
        };
        convert(result.components);
        result.supportAttachment*=scale;
        for (auto& v:result.variants) { convert(v.components); v.supportAttachment*=scale; }
        for (auto& i:result.constructionIntervals) { i.start*=scale; i.end*=scale; convert(i.components); }
        // Reference overlays derive from authored components, so a gauge/offset
        // edit cannot leave a competing stale rail-position cache.
        result.railOffsets.clear();
        for (const auto& component:result.components)
            if (component.runningSurface) result.railOffsets.push_back({component.offset.y,component.offset.z});
        result.railCount=static_cast<std::uint32_t>(result.railOffsets.size());
        return result;
    }

    void setTrackComponentDiagnosticSink(TrackComponentDiagnosticSink sink)
    { componentDiagnosticSink=std::move(sink); }

    void setTrackComponentMeshResolver(TrackComponentMeshResolver resolver)
    { componentMeshResolver=std::move(resolver); }

    glm::dvec3 trackSupportAttachmentAt(const TrackStylePreset& style, double station)
    {
        for (const auto& i : style.constructionIntervals)
            if (station>=i.start && station<i.end)
                if (const auto* v=componentVariant(style,i.variantId)) return v->supportAttachment;
        return style.supportAttachment;
    }

    RenderableTrack generateContinuousTrackPresentation(
        const std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style)
    {
        validateTrackStyle(style);
        validateSamples(samples);

        RenderableTrack result;
        if (!style.visible)
        {
            return result;
        }
        if (style.geometryFamily == TrackGeometryFamily::Components)
            return generateComponentTrack(samples, style);
        result.materials.push_back(style.railMaterial);
        result.continuousMesh.submeshes.reserve(
            (style.railsVisible ? style.railCount : 0)
                + (style.spine.enabled ? 1 : 0));

        if (style.railsVisible)
        {
            const std::vector<ProfileEdge> railProfile = ellipticalProfile(
                glm::dvec2{style.railRadius * 2.0},
                style.railRadialSegments);
            for (std::size_t rail = 0; rail < style.railCount; ++rail)
            {
                appendProfileSweep(
                    result.continuousMesh, samples, railProfile,
                    style.railOffsets[rail], 0,
                    firstRailComponentId + static_cast<std::uint32_t>(rail));
            }
        }

        if (style.spine.enabled)
        {
            result.materials.push_back(style.spine.material);
            const std::vector<ProfileEdge> spineProfile =
                style.spine.type == ContinuousSpineType::Tubular
                ? ellipticalProfile(
                    style.spine.dimensions, style.spine.radialSegments)
                : boxProfile(style.spine.dimensions);
            appendProfileSweep(
                result.continuousMesh, samples, spineProfile,
                style.spine.offset, 1, spineComponentId);
        }

        return result;
    }

    std::vector<HardwareInstanceBatch> generateTrackHardwarePresentation(
        const std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style,
        const bool includeHardwareAtEnd)
    {
        validateTrackStyle(style);
        validateSamples(samples);

        std::vector<HardwareInstanceBatch> result;
        if (!style.visible)
        {
            return result;
        }

        const double trackBegin = samples.front().distance;
        const double trackEnd = samples.back().distance;
        std::uint32_t nextObjectId = 1;
        for (std::size_t hardwareIndex = 0;
            hardwareIndex < style.repeatingHardware.size();
            ++hardwareIndex)
        {
            const RepeatingHardwareStyle& hardware =
                style.repeatingHardware[hardwareIndex];
            if (!hardware.enabled)
            {
                continue;
            }

            HardwareInstanceBatch batch;
            batch.asset = hardware.asset;
            batch.materialOverride = hardware.materialOverride;
            const double availableLength = trackEnd - trackBegin
                - hardware.startOffset;
            if (availableLength >= 0.0)
            {
                const std::size_t count = static_cast<std::size_t>(
                    std::floor(availableLength / hardware.spacing + 1.0e-10))
                    + 1;
                if (count > std::numeric_limits<std::uint32_t>::max())
                {
                    throw std::length_error(
                        "Repeating-hardware instance count exceeds 32 bits."
                    );
                }
                batch.instances.reserve(count);

                for (std::size_t instanceIndex = 0;
                    instanceIndex < count;
                    ++instanceIndex)
                {
                    const double distance = trackBegin
                        + hardware.startOffset
                        + static_cast<double>(instanceIndex)
                            * hardware.spacing;
                    if (!includeHardwareAtEnd
                        && distance >= trackEnd
                            - 1.0e-9 * std::max(1.0, std::abs(trackEnd)))
                    {
                        continue;
                    }
                    const InterpolatedTrackFrame frame =
                        interpolateTrackFrame(samples, distance);
                    const glm::dquat orientation =
                        hardware.frameFollow == HardwareFrameFollow::TrackFrame
                        ? frame.orientation : glm::dquat{1.0, 0.0, 0.0, 0.0};

                    glm::dmat4 transform{1.0};
                    transform = glm::translate(transform, frame.position);
                    transform *= glm::mat4_cast(orientation);
                    transform = glm::translate(
                        transform, hardware.localPosition);
                    transform *= glm::mat4_cast(
                        localAdjustment(hardware.localRotation));
                    transform = glm::scale(transform, hardware.localScale);

                    const glm::mat4 gpuTransform{transform};
                    if (!finite(gpuTransform))
                    {
                        throw std::runtime_error(
                            "A repeating-hardware transform is outside the finite float range."
                        );
                    }
                    batch.instances.push_back({
                        gpuTransform,
                        firstHardwareComponentId
                            + static_cast<std::uint32_t>(hardwareIndex),
                        nextObjectId++,
                        hardware.asset.placeholder ? 1u : 0u
                    });
                }
            }
            result.push_back(std::move(batch));
        }

        return result;
    }

    RenderableTrack generateRenderableTrack(
        const std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style,
        const bool includeHardwareAtEnd)
    {
        RenderableTrack result =
            generateContinuousTrackPresentation(samples, style);
        if (style.geometryFamily != TrackGeometryFamily::Components)
            result.hardwareBatches = generateTrackHardwarePresentation(
                samples, style, includeHardwareAtEnd);
        return result;
    }
}
