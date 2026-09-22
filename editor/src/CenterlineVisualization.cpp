#include <quantum/editor/CenterlineVisualization.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace quantum::editor
{
    namespace
    {
        inline constexpr std::array<float, 4> leftRailColor{
            1.00F, 0.45F, 0.15F, 1.0F};
        inline constexpr std::array<float, 4> rightRailColor{
            0.35F, 0.95F, 0.40F, 1.0F};
        inline constexpr std::array<float, 4> centerlineCurveColor{
            0.20F, 0.90F, 1.00F, 1.0F};
        inline constexpr std::array<float, 4> heartlineColor{
            0.90F, 0.35F, 0.95F, 1.0F};

        inline constexpr double sectionBoundaryToleranceRelative = 1.0e-9;

        [[nodiscard]] double boundaryTolerance(const double distance) noexcept
        {
            return sectionBoundaryToleranceRelative
                * std::max(1.0, std::abs(distance));
        }

        [[nodiscard]] std::uint32_t checkedVertexCount(
            const std::size_t value,
            const char* const context)
        {
            if (value > std::numeric_limits<std::uint32_t>::max())
            {
                throw std::length_error(context);
            }

            return static_cast<std::uint32_t>(value);
        }

        // Appends one polyline as explicit line segments spanning every
        // consecutive solved state pair. The solved states chain across all
        // authored sections, so each derived curve is continuous across
        // section boundaries by construction.
        template<typename OffsetFn>
        void appendReferenceCurveSegmentRange(
            std::vector<renderer::LineVertex>& vertices,
            const std::vector<coaster::RiderLocalGeometryState>& states,
            const std::size_t firstState,
            const std::size_t lastState,
            const std::array<float, 4>& color,
            OffsetFn&& offsetForState)
        {
            for (std::size_t index = firstState; index < lastState; ++index)
            {
                for (const std::size_t endpoint : {index, index + 1})
                {
                    const glm::dvec3 position =
                        offsetForState(states[endpoint]);
                    const renderer::LineVertex vertex{
                        static_cast<float>(position.x),
                        static_cast<float>(position.y),
                        static_cast<float>(position.z),
                        color};
                    if (!std::isfinite(vertex.x)
                        || !std::isfinite(vertex.y)
                        || !std::isfinite(vertex.z))
                    {
                        throw std::runtime_error(
                            "A track reference-curve position is outside the renderer's finite float range.");
                    }
                    vertices.push_back(vertex);
                }
            }
        }

        template<typename OffsetFn>
        void appendReferenceCurveSegments(
            std::vector<renderer::LineVertex>& vertices,
            const std::vector<coaster::RiderLocalGeometryState>& states,
            const std::array<float, 4>& color,
            OffsetFn&& offsetForState)
        {
            appendReferenceCurveSegmentRange(vertices, states, 0,
                states.size() - 1, color,
                std::forward<OffsetFn>(offsetForState));
        }

        void appendRenderableTrack(
            coaster::RenderableTrack& destination,
            coaster::RenderableTrack source,
            std::uint32_t& nextHardwareObjectId)
        {
            const std::uint32_t vertexOffset = checkedVertexCount(
                destination.continuousMesh.vertices.size(),
                "A combined track mesh exceeds the renderer's vertex range.");
            const std::uint32_t triangleIndexOffset = checkedVertexCount(
                destination.continuousMesh.triangleIndices.size(),
                "A combined track mesh exceeds the renderer's index range.");
            const std::uint32_t materialOffset = checkedVertexCount(
                destination.materials.size(),
                "A combined track mesh exceeds the renderer's material range.");

            destination.continuousMesh.vertices.insert(
                destination.continuousMesh.vertices.end(),
                source.continuousMesh.vertices.begin(),
                source.continuousMesh.vertices.end());
            for (std::uint32_t index : source.continuousMesh.triangleIndices)
                destination.continuousMesh.triangleIndices.push_back(
                    vertexOffset + index);
            for (std::uint32_t index : source.continuousMesh.edgeIndices)
                destination.continuousMesh.edgeIndices.push_back(
                    vertexOffset + index);
            for (coaster::TrackSubmesh submesh :
                source.continuousMesh.submeshes)
            {
                submesh.firstIndex += triangleIndexOffset;
                submesh.materialIndex += materialOffset;
                destination.continuousMesh.submeshes.push_back(submesh);
            }
            destination.materials.insert(destination.materials.end(),
                source.materials.begin(), source.materials.end());
            for (coaster::HardwareInstanceBatch& batch : source.hardwareBatches)
            {
                for (coaster::HardwareInstance& instance : batch.instances)
                    instance.objectId = nextHardwareObjectId++;
                destination.hardwareBatches.push_back(std::move(batch));
            }
        }

        [[nodiscard]] std::span<const coaster::RiderLocalGeometryState>
        sectionSamples(
            const CenterlineVisualization& visualization,
            const CenterlineSectionSlice& slice)
        {
            const std::size_t firstState = slice.firstVertex / 2;
            const std::size_t stateCount = slice.vertexCount / 2 + 1;
            if (slice.firstVertex % 2 != 0 || slice.vertexCount == 0
                || slice.vertexCount % 2 != 0
                || firstState > visualization.samples.size()
                || stateCount > visualization.samples.size() - firstState)
            {
                throw std::logic_error(
                    "Cached centerline section samples are inconsistent.");
            }
            return {visualization.samples.data() + firstState, stateCount};
        }

        void appendEngineeringRailCurves(
            std::vector<renderer::LineVertex>& vertices,
            const CenterlineVisualization& visualization,
            const std::vector<coaster::TrackStylePreset>& resolvedStyles)
        {
            for (const std::size_t rail : {std::size_t{0}, std::size_t{1}})
            {
                const auto& color = rail == 0 ? leftRailColor : rightRailColor;
                for (std::size_t region = 0;
                    region < visualization.sectionSlices.size(); ++region)
                {
                    const CenterlineSectionSlice& slice =
                        visualization.sectionSlices[region];
                    const std::size_t firstState = slice.firstVertex / 2;
                    const std::size_t lastState = firstState
                        + slice.vertexCount / 2;
                    const coaster::RailOffset offset =
                        resolvedStyles.at(region).railOffsets.at(rail);
                    appendReferenceCurveSegmentRange(
                        vertices, visualization.samples, firstState, lastState,
                        color,
                        [offset](const coaster::RiderLocalGeometryState& state)
                        {
                            return state.position
                                + state.frame.lateral * offset.lateral
                                + state.frame.up * offset.vertical;
                        });
                }
            }
        }

        [[nodiscard]] std::vector<coaster::TrackStylePreset>
        resolveRegionStyles(
            const coaster::AuthoredTrack& track,
            const std::size_t expectedSectionCount)
        {
            if (track.sectionCount() != expectedSectionCount)
            {
                throw std::logic_error(
                    "Presentation-only style rebuild requires unchanged track sections.");
            }
            std::vector<coaster::TrackStylePreset> styles;
            styles.reserve(track.sectionCount());
            for (std::size_t index = 0; index < track.sectionCount(); ++index)
            {
                styles.push_back(coaster::resolveTrackStyle(
                    track.trackStyle(),
                    track.section(index).trackStyleOverrides));
            }
            return styles;
        }
    }

    CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track)
    {
        return createCenterlineVisualization(track, track.trackStyle());
    }

    CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track,
        const coaster::TrackStylePreset& style)
    {
        CenterlineVisualization visualization;

        if (track.sectionCount() == 0)
        {
            return visualization;
        }

        visualization.anchors = createViewportTrackAnchors(track);

        visualization.samples = coaster::integrateAuthoredTrack(
            track,
            centerlineVisualizationSampleSpacing
        );
        const std::vector<coaster::RiderLocalGeometryState>& states =
            visualization.samples;

        if (states.size() < 2)
        {
            throw std::runtime_error(
                "QuantumCore generated fewer than two centerline samples."
            );
        }

        for (const coaster::RiderLocalGeometryState& state : states)
        {
            if (!std::isfinite(state.distance)
                || !std::isfinite(state.position.x)
                || !std::isfinite(state.position.y)
                || !std::isfinite(state.position.z)
                || !std::isfinite(state.frame.tangent.x)
                || !std::isfinite(state.frame.tangent.y)
                || !std::isfinite(state.frame.tangent.z)
                || !std::isfinite(state.frame.lateral.x)
                || !std::isfinite(state.frame.lateral.y)
                || !std::isfinite(state.frame.lateral.z)
                || !std::isfinite(state.frame.up.x)
                || !std::isfinite(state.frame.up.y)
                || !std::isfinite(state.frame.up.z))
            {
                throw std::runtime_error(
                    "QuantumCore generated a non-finite centerline state."
                );
            }
        }

        visualization.minimumPosition = glm::dvec3{
            std::numeric_limits<double>::max()
        };
        visualization.maximumPosition = glm::dvec3{
            std::numeric_limits<double>::lowest()
        };

        for (const coaster::RiderLocalGeometryState& state : states)
        {
            visualization.minimumPosition = glm::min(
                visualization.minimumPosition,
                state.position
            );
            visualization.maximumPosition = glm::max(
                visualization.maximumPosition,
                state.position
            );
        }

        const std::size_t segmentCount = states.size() - 1;
        visualization.verticesPerCurve = checkedVertexCount(
            2 * segmentCount,
            "The centerline visualization exceeds the renderer's draw range."
        );

        // Section membership comes from authored data: solved distances are
        // cumulative from the track start, and the section boundaries are
        // the prefix sums of the authored lengths. The joint sample between
        // consecutive sections appears once in `samples`, while the line-list
        // representation duplicates it as the end of one visible segment and
        // the start of the next.
        const std::size_t authoredSectionCount = track.sectionCount();
        std::vector<double> sectionBoundaryDistances;
        sectionBoundaryDistances.reserve(authoredSectionCount + 1);
        sectionBoundaryDistances.push_back(0.0);
        double runningLength = 0.0;
        for (std::size_t index = 0; index < authoredSectionCount; ++index)
        {
            runningLength += coaster::sectionLength(track.section(index));
            sectionBoundaryDistances.push_back(runningLength);
        }

        if (std::abs(states.front().distance) > boundaryTolerance(0.0))
        {
            throw std::runtime_error(
                "QuantumCore centerline samples do not start at distance zero."
            );
        }

        std::vector<std::size_t> boundaryStateIndices;
        boundaryStateIndices.resize(sectionBoundaryDistances.size());
        std::size_t searchIndex = 0;

        for (std::size_t boundary = 1;
            boundary < sectionBoundaryDistances.size();
            ++boundary)
        {
            const double targetDistance =
                sectionBoundaryDistances[boundary];
            const double tolerance = boundaryTolerance(targetDistance);

            while (searchIndex + 1 < states.size()
                && states[searchIndex].distance
                    < targetDistance - tolerance)
            {
                ++searchIndex;
            }

            if (std::abs(states[searchIndex].distance - targetDistance)
                > tolerance)
            {
                throw std::runtime_error(
                    "QuantumCore centerline samples do not contain an authored section boundary."
                );
            }

            boundaryStateIndices[boundary] = searchIndex;
        }

        visualization.sectionSlices.resize(authoredSectionCount);
        visualization.resolvedRegionStyles.reserve(authoredSectionCount);
        std::uint32_t nextHardwareObjectId = 1;
        for (std::size_t index = 0; index < authoredSectionCount; ++index)
        {
            CenterlineSectionSlice& slice = visualization.sectionSlices[index];
            const std::size_t firstState = boundaryStateIndices[index];
            const std::size_t lastState = boundaryStateIndices[index + 1];

            if (lastState <= firstState)
            {
                throw std::runtime_error(
                    "QuantumCore generated no centerline segment for an authored section."
                );
            }

            // Each segment s starts at vertex 2*s inside every curve run.
            slice.firstVertex = checkedVertexCount(
                2 * firstState,
                "A centerline section slice starts outside the renderer's draw range."
            );
            slice.vertexCount = checkedVertexCount(
                2 * (lastState - firstState),
                "A centerline section slice exceeds the renderer's draw range."
            );

            const coaster::RiderLocalGeometryState& start =
                states[firstState];
            const coaster::RiderLocalGeometryState& end = states[lastState];
            slice.startDistance = start.distance;
            slice.endDistance = end.distance;
            slice.startPosition = start.position;
            slice.endPosition = end.position;
            const glm::dvec3 tangentLength = start.frame.tangent;
            const double tangentMagnitude = glm::length(tangentLength);
            slice.startTangent = tangentMagnitude > 1.0e-9
                ? tangentLength / tangentMagnitude
                : glm::dvec3{1.0, 0.0, 0.0};

            slice.minimumPosition = glm::dvec3{
                std::numeric_limits<double>::max()
            };
            slice.maximumPosition = glm::dvec3{
                std::numeric_limits<double>::lowest()
            };

            for (std::size_t stateIndex = firstState;
                stateIndex <= lastState;
                ++stateIndex)
            {
                slice.minimumPosition = glm::min(
                    slice.minimumPosition,
                    states[stateIndex].position
                );
                slice.maximumPosition = glm::max(
                    slice.maximumPosition,
                    states[stateIndex].position
                );
            }

            coaster::TrackStylePreset resolvedStyle =
                coaster::resolveTrackStyle(
                    style,
                    track.section(index).trackStyleOverrides);
            appendRenderableTrack(
                visualization.renderableTrack,
                coaster::generateRenderableTrack(
                    std::span<const coaster::RiderLocalGeometryState>{
                        states.data() + firstState,
                        lastState - firstState + 1},
                    resolvedStyle,
                    index + 1 == authoredSectionCount),
                nextHardwareObjectId);
            visualization.resolvedRegionStyles.push_back(
                std::move(resolvedStyle));
        }

        visualization.vertices.reserve(
            renderer::viewportCurveCount * 2 * segmentCount
        );

        // Each region contributes only its own rail segments, so an offset
        // override cannot leak across a boundary.
        appendEngineeringRailCurves(visualization.vertices, visualization,
            visualization.resolvedRegionStyles);

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            centerlineCurveColor,
            [](const coaster::RiderLocalGeometryState& state)
            {
                return state.position;
            });

        // Coaster Setup stores SI metres while solved positions use document
        // coordinate units. This affects only the viewport reference curve;
        // centerline integration, rail meshing, and TrainPhysics retain their
        // existing authoritative inputs.
        const coaster::HeartlineSettings& heartline =
            track.coasterSetup().heartline;
        const double heartlineOffsetCoordinateUnits = heartline.enabled
            ? heartline.offsetMeters
                / track.physicalSettings().metersPerCoordinateUnit
            : 0.0;

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            heartlineColor,
            [heartlineOffsetCoordinateUnits](
                const coaster::RiderLocalGeometryState& state)
            {
                return state.position + state.frame.up
                    * heartlineOffsetCoordinateUnits;
            });

        return visualization;
    }

    TrackStylePresentationCandidate createTrackStylePresentationCandidate(
        const coaster::AuthoredTrack& track,
        const CenterlineVisualization& cachedVisualization,
        const TrackStylePresentationImpact impact)
    {
        if (impact.requiresFullRegeneration())
        {
            throw std::invalid_argument(
                "A full-regeneration impact cannot use the presentation-only path.");
        }
        if (cachedVisualization.samples.size() < 2
            || cachedVisualization.sectionSlices.empty()
            || cachedVisualization.vertices.size()
                != static_cast<std::size_t>(
                    cachedVisualization.verticesPerCurve)
                    * renderer::viewportCurveCount)
        {
            throw std::logic_error(
                "Presentation-only style rebuild requires a valid solved visualization cache.");
        }

        TrackStylePresentationCandidate candidate;
        candidate.impact = impact;
        candidate.documentStyle = track.trackStyle();
        candidate.resolvedRegionStyles = resolveRegionStyles(
            track, cachedVisualization.sectionSlices.size());

        const bool rebuildMesh = impact.affects(
            TrackStylePresentationProduct::RenderableMesh);
        const bool rebuildHardware = impact.affects(
            TrackStylePresentationProduct::HardwareInstances);
        if (rebuildMesh)
        {
            candidate.continuousMesh.emplace();
            candidate.trackMaterials.emplace();
        }
        else if (impact.affects(
            TrackStylePresentationProduct::TrackMaterials))
        {
            candidate.trackMaterials.emplace();
        }
        if (rebuildHardware)
        {
            candidate.hardwareBatches.emplace();
        }
        else if (impact.affects(
            TrackStylePresentationProduct::HardwareMaterials))
        {
            candidate.hardwareMaterials.emplace();
        }

        coaster::RenderableTrack combinedMesh;
        std::uint32_t nextHardwareObjectId = 1;
        for (std::size_t region = 0;
            region < cachedVisualization.sectionSlices.size(); ++region)
        {
            const auto samples = sectionSamples(
                cachedVisualization,
                cachedVisualization.sectionSlices[region]);
            const auto& style = candidate.resolvedRegionStyles[region];
            const bool includeHardwareAtEnd = region + 1
                == cachedVisualization.sectionSlices.size();

            if (rebuildMesh)
            {
                appendRenderableTrack(combinedMesh,
                    coaster::generateContinuousTrackPresentation(
                        samples, style),
                    nextHardwareObjectId);
            }
            else if (candidate.trackMaterials.has_value()
                && style.visible)
            {
                // Material order is the same compact order emitted by the
                // continuous presentation builder and consumed by submeshes.
                candidate.trackMaterials->push_back(style.railMaterial);
                if (style.spine.enabled)
                    candidate.trackMaterials->push_back(style.spine.material);
            }

            if (rebuildHardware)
            {
                coaster::RenderableTrack regionalHardware;
                regionalHardware.hardwareBatches =
                    coaster::generateTrackHardwarePresentation(
                        samples, style, includeHardwareAtEnd);
                appendRenderableTrack(combinedMesh,
                    std::move(regionalHardware), nextHardwareObjectId);
            }
            else if (candidate.hardwareMaterials.has_value()
                && style.visible)
            {
                for (const auto& hardware : style.repeatingHardware)
                {
                    if (hardware.enabled)
                    {
                        candidate.hardwareMaterials->push_back(
                            hardware.materialOverride);
                    }
                }
            }
        }

        if (rebuildMesh)
        {
            *candidate.continuousMesh =
                std::move(combinedMesh.continuousMesh);
            *candidate.trackMaterials = std::move(combinedMesh.materials);
        }
        if (rebuildHardware)
        {
            *candidate.hardwareBatches =
                std::move(combinedMesh.hardwareBatches);
        }

        if (impact.affects(TrackStylePresentationProduct::EngineeringRails))
        {
            candidate.referenceCurveVertices = cachedVisualization.vertices;
            std::vector<renderer::LineVertex> engineeringRails;
            engineeringRails.reserve(
                2 * cachedVisualization.verticesPerCurve);
            appendEngineeringRailCurves(engineeringRails, cachedVisualization,
                candidate.resolvedRegionStyles);
            if (engineeringRails.size()
                != 2 * static_cast<std::size_t>(
                    cachedVisualization.verticesPerCurve))
            {
                throw std::logic_error(
                    "Regional engineering-rail rebuild changed curve topology.");
            }
            std::copy(engineeringRails.begin(), engineeringRails.end(),
                candidate.referenceCurveVertices->begin());
        }

        return candidate;
    }

    std::pair<glm::dvec3, glm::dvec3> referenceCurveBounds(
        const CenterlineVisualization& visualization,
        const CenterlineSectionSlice* const slice)
    {
        const std::size_t first = slice != nullptr ? slice->firstVertex : 0;
        const std::size_t count = slice != nullptr
            ? slice->vertexCount : visualization.verticesPerCurve;
        if (count == 0 || first + count > visualization.verticesPerCurve
            || visualization.vertices.size() != static_cast<std::size_t>(
                visualization.verticesPerCurve) * renderer::viewportCurveCount)
        {
            throw std::invalid_argument("Reference curve bounds require a valid nonempty curve range.");
        }
        glm::dvec3 minimum{std::numeric_limits<double>::max()};
        glm::dvec3 maximum{std::numeric_limits<double>::lowest()};
        for (std::size_t curve = 0; curve < renderer::viewportCurveCount; ++curve)
        {
            const std::size_t begin = curve * visualization.verticesPerCurve + first;
            for (std::size_t index = begin; index < begin + count; ++index)
            {
                const auto& vertex = visualization.vertices[index];
                const glm::dvec3 position{vertex.x, vertex.y, vertex.z};
                minimum = glm::min(minimum, position);
                maximum = glm::max(maximum, position);
            }
        }
        return {minimum, maximum};
    }

    void CenterlineVisualizationCache::markDirty() noexcept
    {
        dirty_ = true;
    }

    void CenterlineVisualizationCache::setTrackStyle(
        coaster::TrackStylePreset style)
    {
        coaster::validateTrackStyle(style);
        trackStyle_ = std::move(style);
        markDirty();
    }

    bool CenterlineVisualizationCache::rebuildIfDirty(
        const coaster::AuthoredTrack& track)
    {
        if (!dirty_)
        {
            return false;
        }

        replace(createCenterlineVisualization(track, trackStyle_));
        return true;
    }

    void CenterlineVisualizationCache::replace(
        CenterlineVisualization visualization)
    {
        visualization_ = std::move(visualization);
        dirty_ = false;
        ++generation_;
        ++presentationGeneration_;
    }

    void CenterlineVisualizationCache::applyTrackStylePresentation(
        TrackStylePresentationCandidate candidate)
    {
        if (candidate.impact.requiresFullRegeneration())
        {
            throw std::invalid_argument(
                "A full-regeneration impact cannot be applied as presentation-only.");
        }
        if (candidate.continuousMesh.has_value())
        {
            visualization_.renderableTrack.continuousMesh =
                std::move(*candidate.continuousMesh);
        }
        if (candidate.trackMaterials.has_value())
        {
            visualization_.renderableTrack.materials =
                std::move(*candidate.trackMaterials);
        }
        if (candidate.hardwareBatches.has_value())
        {
            visualization_.renderableTrack.hardwareBatches =
                std::move(*candidate.hardwareBatches);
        }
        if (candidate.hardwareMaterials.has_value())
        {
            auto& batches = visualization_.renderableTrack.hardwareBatches;
            if (batches.size() != candidate.hardwareMaterials->size())
            {
                throw std::logic_error(
                    "Hardware material update no longer matches cached batches.");
            }
            for (std::size_t index = 0; index < batches.size(); ++index)
            {
                batches[index].materialOverride =
                    std::move((*candidate.hardwareMaterials)[index]);
            }
        }
        if (candidate.referenceCurveVertices.has_value())
        {
            visualization_.vertices =
                std::move(*candidate.referenceCurveVertices);
        }
        visualization_.resolvedRegionStyles =
            std::move(candidate.resolvedRegionStyles);
        trackStyle_ = std::move(candidate.documentStyle);
        if (!candidate.impact.empty())
        {
            ++presentationGeneration_;
        }
    }

    bool CenterlineVisualizationCache::isDirty() const noexcept
    {
        return dirty_;
    }

    std::uint64_t CenterlineVisualizationCache::generation() const noexcept
    {
        return generation_;
    }

    std::uint64_t CenterlineVisualizationCache::presentationGeneration()
        const noexcept
    {
        return presentationGeneration_;
    }

    const CenterlineVisualization&
    CenterlineVisualizationCache::visualization() const noexcept
    {
        return visualization_;
    }

    const coaster::TrackStylePreset&
    CenterlineVisualizationCache::trackStyle() const noexcept
    {
        return trackStyle_;
    }
}
