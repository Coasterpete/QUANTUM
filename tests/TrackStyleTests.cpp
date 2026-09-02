#include <quantum/coaster/TrackStyle.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const char* const message)
    {
        if (glm::length(actual - expected) > tolerance)
        {
            throw std::runtime_error(message);
        }
    }

    [[nodiscard]] std::vector<RiderLocalGeometryState> straightSamples(
        const quantum::geometry::CurveFrame& frame = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}})
    {
        return {
            {0.0, {0.0, 0.0, 0.0}, frame},
            {1.0, {1.0, 0.0, 0.0}, frame},
            {2.0, {2.0, 0.0, 0.0}, frame}
        };
    }

    [[nodiscard]] glm::dvec3 ringCenter(
        const ContinuousTrackMesh& mesh,
        const std::size_t first,
        const std::size_t sides)
    {
        glm::dvec3 center{0.0};
        for (std::size_t index = 0; index < sides; ++index)
        {
            center += glm::dvec3{mesh.vertices[first + index].position};
        }
        return center / static_cast<double>(sides);
    }

    void defaultPresetValidates()
    {
        const TrackStylePreset style = createStandardDualRailPreset();
        validateTrackStyle(style);
        require(style.name == "StandardDualRail", "default preset name");
        require(style.geometryFamily == TrackGeometryFamily::DualRailTubular,
            "default geometry family");
        require(style.railCount == 2 && style.railOffsets.size() == 2,
            "default dual rail count");
        require(style.repeatingHardware.size() == 1
                && style.repeatingHardware.front().asset.placeholder
                && style.repeatingHardware.front().asset.path
                    == "assets://track/test-crosstie-placeholder.glb",
            "default hardware is the explicit file-backed test placeholder");
    }

    void meshGenerationIsDeterministicAndIndexed()
    {
        const auto samples = straightSamples();
        const auto style = createStandardDualRailPreset();
        const RenderableTrack first = generateRenderableTrack(samples, style);
        const RenderableTrack second = generateRenderableTrack(samples, style);

        require(first.continuousMesh.vertices.size()
                == second.continuousMesh.vertices.size(),
            "deterministic vertex count");
        require(first.continuousMesh.triangleIndices
                == second.continuousMesh.triangleIndices,
            "deterministic triangle indices");
        require(first.continuousMesh.edgeIndices
                == second.continuousMesh.edgeIndices,
            "deterministic edge indices");
        for (std::size_t index = 0;
            index < first.continuousMesh.vertices.size(); ++index)
        {
            require(first.continuousMesh.vertices[index].position
                    == second.continuousMesh.vertices[index].position,
                "deterministic positions");
            require(first.continuousMesh.vertices[index].normal
                    == second.continuousMesh.vertices[index].normal,
                "deterministic normals");
        }

        const std::size_t vertexCount = first.continuousMesh.vertices.size();
        require(vertexCount == 2 * samples.size() * style.railRadialSegments,
            "indexed tube vertex count");
        require(!first.continuousMesh.triangleIndices.empty()
                && first.continuousMesh.triangleIndices.size() % 3 == 0,
            "triangle list is complete");
        require(!first.continuousMesh.edgeIndices.empty()
                && first.continuousMesh.edgeIndices.size() % 2 == 0,
            "edge list is complete");
        for (const std::uint32_t index : first.continuousMesh.triangleIndices)
        {
            require(index < vertexCount, "triangle index is in range");
        }
        for (const std::uint32_t index : first.continuousMesh.edgeIndices)
        {
            require(index < vertexCount, "edge index is in range");
        }
        for (const TrackSubmesh& submesh : first.continuousMesh.submeshes)
        {
            require(submesh.indexCount > 0
                    && submesh.firstIndex + submesh.indexCount
                        <= first.continuousMesh.triangleIndices.size(),
                "submesh index range is valid");
            require(submesh.materialIndex < first.materials.size(),
                "submesh material is valid");
        }
    }

    void railOffsetsFollowTheAuthoredFrame()
    {
        const auto samples = straightSamples();
        auto style = createStandardDualRailPreset();
        const std::size_t sides = style.railRadialSegments;
        RenderableTrack track = generateRenderableTrack(samples, style);
        const std::size_t verticesPerRail = samples.size() * sides;

        requireNear(ringCenter(track.continuousMesh, 0, sides),
            {0.0, -0.6, 0.0}, 1.0e-6, "left rail lateral offset");
        requireNear(ringCenter(track.continuousMesh, verticesPerRail, sides),
            {0.0, 0.6, 0.0}, 1.0e-6, "right rail lateral offset");

        style.railOffsets[0].vertical = 0.25;
        style.railOffsets[1].vertical = -0.15;
        track = generateRenderableTrack(samples, style);
        requireNear(ringCenter(track.continuousMesh, 0, sides),
            {0.0, -0.6, 0.25}, 1.0e-6, "left rail vertical offset");
        requireNear(ringCenter(track.continuousMesh, verticesPerRail, sides),
            {0.0, 0.6, -0.15}, 1.0e-6, "right rail vertical offset");

        const quantum::geometry::CurveFrame banked{
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0},
            {0.0, -1.0, 0.0}
        };
        const auto bankedSamples = straightSamples(banked);
        style = createStandardDualRailPreset();
        track = generateRenderableTrack(bankedSamples, style);
        const glm::dvec3 left = ringCenter(track.continuousMesh, 0, sides);
        const glm::dvec3 right = ringCenter(
            track.continuousMesh, verticesPerRail, sides);
        requireNear(glm::normalize(right - left), banked.lateral, 1.0e-6,
            "banking rotates the rail pair with the lateral frame");
    }

    void generatedDataIsFiniteAndNormalsAreUseful()
    {
        const RenderableTrack track = generateRenderableTrack(
            straightSamples(), createStandardDualRailPreset());
        for (const TrackMeshVertex& vertex : track.continuousMesh.vertices)
        {
            for (const float value : {
                vertex.position.x, vertex.position.y, vertex.position.z,
                vertex.normal.x, vertex.normal.y, vertex.normal.z})
            {
                require(std::isfinite(value), "generated mesh value is finite");
            }
            require(glm::length(vertex.normal) > 0.99F,
                "generated normal is non-degenerate");
        }
    }

    template<typename Edit>
    void requireInvalid(Edit&& edit, const char* const message)
    {
        auto style = createStandardDualRailPreset();
        edit(style);
        bool rejected = false;
        try
        {
            validateTrackStyle(style);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, message);
    }

    void invalidStylesAreRejected()
    {
        requireInvalid([](auto& style) { style.railRadius = 0.0; },
            "zero radius rejected");
        requireInvalid([](auto& style) { style.railRadius = -1.0; },
            "negative radius rejected");
        requireInvalid([](auto& style) { style.railCount = 0; },
            "zero rail count rejected");
        requireInvalid([](auto& style) { style.railCount = 3; },
            "rail count/offset mismatch rejected");
        requireInvalid([](auto& style) {
            style.railOffsets[0].lateral =
                std::numeric_limits<double>::quiet_NaN();
        }, "non-finite rail offset rejected");
        requireInvalid([](auto& style) { style.railRadialSegments = 2; },
            "low tessellation rejected");
        requireInvalid([](auto& style) { style.railRadialSegments = 129; },
            "excessive tessellation rejected");
        requireInvalid([](auto& style) {
            style.repeatingHardware[0].asset.path.clear();
        }, "missing required asset reference rejected");
        requireInvalid([](auto& style) {
            style.repeatingHardware[0].asset.path =
                "C:/art/crosstie.glb";
        }, "absolute hardware path rejected");
        requireInvalid([](auto& style) {
            style.repeatingHardware[0].asset.path =
                "assets://track/../../outside.glb";
        }, "hardware path traversal rejected");
        requireInvalid([](auto& style) {
            style.repeatingHardware[0].asset.path =
                "assets://models/crosstie.glb";
        }, "hardware outside assets track rejected");
        requireInvalid([](auto& style) {
            style.repeatingHardware[0].asset.path =
                "assets://track/crosstie.gltf";
        }, "non-GLB hardware rejected");

        require(normalizeTrackHardwareAssetIdentifier(
                "assets://track\\standard-dual-rail\\.\\crosstie.glb")
                == "assets://track/standard-dual-rail/crosstie.glb",
            "logical track hardware identity normalizes package separators");
        require(normalizeTrackHardwareAssetIdentifier(
                "builtin://diagnostic/track-hardware-placeholder")
                == "builtin://diagnostic/track-hardware-placeholder",
            "diagnostic builtin remains a valid authored hardware identity");
    }
}

int main()
{
    try
    {
        defaultPresetValidates();
        meshGenerationIsDeterministicAndIndexed();
        railOffsetsFollowTheAuthoredFrame();
        generatedDataIsFiniteAndNormalsAreUseful();
        invalidStylesAreRejected();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Track style test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Track style tests passed.\n";
    return 0;
}
