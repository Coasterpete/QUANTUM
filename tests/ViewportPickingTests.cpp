#include <quantum/editor/RegionSelection.hpp>
#include <quantum/editor/ViewportPicking.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::editor::CenterlineSectionSlice;
    using quantum::editor::CenterlineVisualization;
    using quantum::editor::ViewportCamera;
    using quantum::editor::ViewportCameraPose;
    using quantum::editor::ViewportCameraPreset;
    using quantum::editor::ViewportProjection;
    using quantum::editor::ViewportRay;

    constexpr double pi = 3.14159265358979323846;
    constexpr std::uint32_t viewportHeight = 1000;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    [[nodiscard]] quantum::renderer::LineVertex vertex(
        const glm::dvec3& position)
    {
        return {
            static_cast<float>(position.x),
            static_cast<float>(position.y),
            static_cast<float>(position.z),
            {1.0F, 1.0F, 1.0F, 1.0F}
        };
    }

    [[nodiscard]] CenterlineVisualization visualizationFromSegments(
        const std::vector<std::pair<glm::dvec3, glm::dvec3>>& segments)
    {
        CenterlineVisualization visualization;
        visualization.verticesPerCurve = static_cast<std::uint32_t>(
            2 * segments.size()
        );

        for (std::size_t index = 0; index < segments.size(); ++index)
        {
            visualization.sectionSlices.push_back(CenterlineSectionSlice{
                .firstVertex = static_cast<std::uint32_t>(2 * index),
                .vertexCount = 2
            });
        }

        for (std::uint32_t curve = 0;
            curve < quantum::renderer::viewportCurveCount;
            ++curve)
        {
            for (const auto& [begin, end] : segments)
            {
                visualization.vertices.push_back(vertex(begin));
                visualization.vertices.push_back(vertex(end));
            }
        }
        return visualization;
    }

    [[nodiscard]] ViewportCamera topCamera(
        const ViewportProjection projection =
            ViewportProjection::Orthographic)
    {
        ViewportCamera camera;
        camera.setBounds({-20.0, -20.0, -10.0}, {20.0, 20.0, 10.0});
        camera.setPose(ViewportCameraPose{
            .focus = {0.0, 0.0, 0.0},
            .yaw = 0.0,
            .pitch = 0.5 * pi,
            .distance = 20.0
        });
        camera.setProjection(projection);
        return camera;
    }

    [[nodiscard]] constexpr std::uint32_t centerlineOnlyMask() noexcept
    {
        return 1u << quantum::renderer::viewportCenterlineCurve;
    }

    void multiRegionMappingAndMiss()
    {
        const CenterlineVisualization visualization =
            visualizationFromSegments({
                {{-8.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}},
                {{2.0, 0.0, 0.0}, {8.0, 0.0, 0.0}}
            });
        const ViewportCamera camera = topCamera();

        const auto left = quantum::editor::pickViewportSection(
            visualization,
            camera,
            ViewportRay{{-5.0, 0.0, 20.0}, {0.0, 0.0, -1.0}},
            viewportHeight,
            centerlineOnlyMask()
        );
        const auto right = quantum::editor::pickViewportSection(
            visualization,
            camera,
            ViewportRay{{5.0, 0.0, 20.0}, {0.0, 0.0, -1.0}},
            viewportHeight,
            centerlineOnlyMask()
        );
        const auto miss = quantum::editor::pickViewportSection(
            visualization,
            camera,
            ViewportRay{{0.0, 5.0, 20.0}, {0.0, 0.0, -1.0}},
            viewportHeight,
            centerlineOnlyMask()
        );

        require(left.has_value() && left->sectionIndex == 0,
            "left segment must map to region zero");
        require(right.has_value() && right->sectionIndex == 1,
            "right segment must map to region one");
        require(!miss.has_value(),
            "empty viewport space must report a miss");
    }

    void frontmostValidHitWins()
    {
        const CenterlineVisualization visualization =
            visualizationFromSegments({
                {{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                {{-2.0, 0.0, 6.0}, {2.0, 0.0, 6.0}}
            });
        const ViewportCamera camera = topCamera(
            ViewportProjection::Perspective
        );
        const auto hit = quantum::editor::pickViewportSection(
            visualization,
            camera,
            ViewportRay{{0.0, 0.0, 20.0}, {0.0, 0.0, -1.0}},
            viewportHeight,
            centerlineOnlyMask()
        );

        require(hit.has_value() && hit->sectionIndex == 1,
            "frontmost valid region must hide a farther overlapping region");
    }

    void sharedBoundaryUsesDeterministicOrdering()
    {
        const CenterlineVisualization visualization =
            visualizationFromSegments({
                {{-5.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}
            });
        const ViewportCamera camera = topCamera();

        for (int attempt = 0; attempt < 20; ++attempt)
        {
            const auto hit = quantum::editor::pickViewportSection(
                visualization,
                camera,
                ViewportRay{{0.0, 0.0, 20.0}, {0.0, 0.0, -1.0}},
                viewportHeight,
                centerlineOnlyMask()
            );
            require(hit.has_value() && hit->sectionIndex == 0,
                "an exact shared boundary must deterministically choose the lower region index");
        }
    }

    void toleranceAndVisibilityAreRespected()
    {
        const CenterlineVisualization visualization =
            visualizationFromSegments({
                {{-5.0, 0.12, 0.0}, {5.0, 0.12, 0.0}}
            });
        const ViewportCamera camera = topCamera();
        const ViewportRay ray{{0.0, 0.0, 20.0}, {0.0, 0.0, -1.0}};

        require(quantum::editor::pickViewportSection(
            visualization,
            camera,
            ray,
            viewportHeight,
            centerlineOnlyMask()
        ).has_value(), "a line inside the eight-pixel radius must hit");
        require(!quantum::editor::pickViewportSection(
            visualization,
            camera,
            ray,
            viewportHeight,
            centerlineOnlyMask(),
            2.0
        ).has_value(), "the same line must miss with a tighter tolerance");
        require(!quantum::editor::pickViewportSection(
            visualization,
            camera,
            ray,
            viewportHeight,
            0
        ).has_value(), "hidden curves must not be selectable");
    }

    void selectionMutationMappingPreservesIdentity()
    {
        using quantum::editor::selectionAfterInsertion;
        using quantum::editor::selectionAfterMove;
        using quantum::editor::selectionAfterRemoval;

        require(selectionAfterInsertion(2, 0) == 3,
            "prepending before selection must shift its index");
        require(selectionAfterInsertion(2, 3) == 2,
            "appending after selection must preserve its index");
        require(selectionAfterRemoval(3, 1, 4) == 2,
            "removing an earlier region must shift selection back");
        require(selectionAfterRemoval(2, 2, 4) == 2,
            "removing the selected region must select its successor");
        require(selectionAfterRemoval(4, 4, 4) == 3,
            "removing the selected tail must select the prior tail");
        require(selectionAfterMove(2, 2, 0) == 0,
            "moving the selected region must follow it");
        require(selectionAfterMove(2, 0, 3) == 1,
            "moving an earlier region past selection must update its index");
        require(selectionAfterMove(2, 4, 1) == 3,
            "moving a later region before selection must update its index");
    }
}

int main()
{
    try
    {
        multiRegionMappingAndMiss();
        frontmostValidHitWins();
        sharedBoundaryUsesDeterministicOrdering();
        toleranceAndVisibilityAreRespected();
        selectionMutationMappingPreservesIdentity();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Viewport picking test failure: "
                  << exception.what() << '\n';
        return 1;
    }

    std::cout << "Viewport picking tests passed.\n";
    return 0;
}
