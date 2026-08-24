#include <quantum/editor/CenterlineVisualization.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

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
        constexpr double outputSpacing = 0.75;

        const std::vector<coaster::RiderLocalGeometryState> states =
            coaster::integrateAuthoredTrack(track, outputSpacing);

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

        CenterlineVisualization visualization;
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

        // Section membership comes from authored data: solved distances are
        // cumulative from the track start, and the section boundaries are
        // the prefix sums of the authored lengths. The joint sample between
        // consecutive sections appears exactly once and belongs to the
        // earlier section here.
        const std::size_t authoredSectionCount = track.sectionCount();
        std::vector<double> sectionEndDistances;
        sectionEndDistances.reserve(authoredSectionCount);
        double runningLength = 0.0;
        for (std::size_t index = 0; index < authoredSectionCount; ++index)
        {
            runningLength += coaster::sectionLength(track.section(index));
            sectionEndDistances.push_back(runningLength);
        }

        visualization.sectionSlices.resize(authoredSectionCount);
        for (CenterlineSectionSlice& slice : visualization.sectionSlices)
        {
            slice.minimumPosition = glm::dvec3{
                std::numeric_limits<double>::max()
            };
            slice.maximumPosition = glm::dvec3{
                std::numeric_limits<double>::lowest()
            };
        }

        std::size_t currentSection = 0;
        std::vector<std::size_t> firstStateOfSection;
        firstStateOfSection.reserve(authoredSectionCount);
        firstStateOfSection.push_back(0);

        for (std::size_t index = 0; index < states.size(); ++index)
        {
            while (currentSection + 1 < authoredSectionCount
                && states[index].distance
                    > sectionEndDistances[currentSection])
            {
                ++currentSection;
                firstStateOfSection.push_back(index);
            }

            CenterlineSectionSlice& slice =
                visualization.sectionSlices[currentSection];
            slice.minimumPosition = glm::min(
                slice.minimumPosition,
                states[index].position
            );
            slice.maximumPosition = glm::max(
                slice.maximumPosition,
                states[index].position
            );
        }

        // Each state s starts one segment pair at vertex 2*s, so a slice's
        // vertex range spans [2*first, 2*firstOfNext).
        for (std::size_t index = 0; index < authoredSectionCount; ++index)
        {
            CenterlineSectionSlice& slice = visualization.sectionSlices[index];
            const std::size_t firstState = firstStateOfSection[index];
            const std::size_t nextState = index + 1
                    < authoredSectionCount
                ? firstStateOfSection[index + 1]
                : states.size();

            slice.firstVertex = static_cast<std::uint32_t>(2 * firstState);
            slice.vertexCount = static_cast<std::uint32_t>(
                2 * (nextState - firstState)
            );

            const coaster::RiderLocalGeometryState& start =
                states[firstState];
            slice.startPosition = start.position;
            const glm::dvec3 tangentLength = start.frame.tangent;
            const double tangentMagnitude = glm::length(tangentLength);
            slice.startTangent = tangentMagnitude > 1.0e-9
                ? tangentLength / tangentMagnitude
                : glm::dvec3{1.0, 0.0, 0.0};
        }

        visualization.verticesPerCurve = static_cast<std::uint32_t>(
            2 * (states.size() - 1)
        );

        visualization.vertices.reserve(4 * 2 * (states.size() - 1));

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
}
