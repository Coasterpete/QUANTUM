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
        // Temporary stand-in track-style values so the viewport can render
        // rails and the heartline before track style/vehicle authoring
        // exists. When real style data arrives it replaces these at the
        // single call site below; the curve construction itself is unchanged.
        // gauge: lateral rail-to-rail distance. heartlineOffset: distance
        // from the track center to the rider heart point along the solved,
        // banking local up axis (not a world-vertical offset).
        inline constexpr double temporaryTrackGauge = 1.2;
        inline constexpr double temporaryHeartlineOffset = 1.4;

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
        void appendReferenceCurveSegments(
            std::vector<renderer::LineVertex>& vertices,
            const std::vector<coaster::RiderLocalGeometryState>& states,
            const std::array<float, 4>& color,
            OffsetFn&& offsetForState)
        {
            for (std::size_t index = 0; index + 1 < states.size(); ++index)
            {
                for (const std::size_t endpoint : {index, index + 1})
                {
                    const glm::dvec3 position =
                        offsetForState(states[endpoint]);

                    // QuantumCore remains double precision. This explicit
                    // conversion is the application-to-renderer boundary for
                    // static GPU positions.
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
                            "A track reference-curve position is outside the renderer's finite float range."
                        );
                    }

                    vertices.push_back(vertex);
                }
            }
        }
    }

    CenterlineVisualization createCenterlineVisualization(
        const coaster::AuthoredTrack& track)
    {
        CenterlineVisualization visualization;

        if (track.sectionCount() == 0)
        {
            return visualization;
        }

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
        }

        visualization.vertices.reserve(
            renderer::viewportCurveCount * 2 * segmentCount
        );

        const double halfGauge = temporaryTrackGauge * 0.5;

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            leftRailColor,
            [halfGauge](const coaster::RiderLocalGeometryState& state)
            {
                return state.position - state.frame.lateral * halfGauge;
            });

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            rightRailColor,
            [halfGauge](const coaster::RiderLocalGeometryState& state)
            {
                return state.position + state.frame.lateral * halfGauge;
            });

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            centerlineCurveColor,
            [](const coaster::RiderLocalGeometryState& state)
            {
                return state.position;
            });

        appendReferenceCurveSegments(
            visualization.vertices,
            states,
            heartlineColor,
            [](const coaster::RiderLocalGeometryState& state)
            {
                return state.position + state.frame.up
                    * temporaryHeartlineOffset;
            });

        return visualization;
    }

    void CenterlineVisualizationCache::markDirty() noexcept
    {
        dirty_ = true;
    }

    bool CenterlineVisualizationCache::rebuildIfDirty(
        const coaster::AuthoredTrack& track)
    {
        if (!dirty_)
        {
            return false;
        }

        replace(createCenterlineVisualization(track));
        return true;
    }

    void CenterlineVisualizationCache::replace(
        CenterlineVisualization visualization)
    {
        visualization_ = std::move(visualization);
        dirty_ = false;
        ++generation_;
    }

    bool CenterlineVisualizationCache::isDirty() const noexcept
    {
        return dirty_;
    }

    std::uint64_t CenterlineVisualizationCache::generation() const noexcept
    {
        return generation_;
    }

    const CenterlineVisualization&
    CenterlineVisualizationCache::visualization() const noexcept
    {
        return visualization_;
    }
}
