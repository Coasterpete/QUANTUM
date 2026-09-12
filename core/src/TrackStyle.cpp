#include <quantum/coaster/TrackStyle.hpp>

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
                || glm::any(glm::greaterThan(material.baseColor, glm::vec4{1.0F})))
            {
                throw std::invalid_argument(
                    std::string(context)
                    + " base color must contain finite values in [0, 1]."
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
                        mesh.triangleIndices.end(), {a, b, c, a, c, d});
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

        style.spine.enabled = true;
        style.spine.type = ContinuousSpineType::Box;
        style.spine.offset = {0.0, modernSteelSpineVerticalOffset};
        style.spine.dimensions = modernSteelSpineDimensions;
        style.spine.radialSegments = 12;
        style.spine.material.baseColor = {0.20F, 0.23F, 0.27F, 1.0F};

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
            break;
        default:
            throw std::invalid_argument("The track geometry family is invalid.");
        }

        if (style.railCount != 2
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

    RenderableTrack generateRenderableTrack(
        const std::span<const RiderLocalGeometryState> samples,
        const TrackStylePreset& style)
    {
        validateTrackStyle(style);
        validateSamples(samples);

        RenderableTrack result;
        result.materials.push_back(style.railMaterial);
        result.continuousMesh.submeshes.reserve(
            style.railCount + (style.spine.enabled ? 1 : 0));

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
            result.hardwareBatches.push_back(std::move(batch));
        }

        return result;
    }
}
