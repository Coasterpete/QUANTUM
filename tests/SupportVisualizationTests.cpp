#include <quantum/editor/SupportPicking.hpp>
#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
    using namespace quantum;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    coaster::SupportStructure& addStructure(
        coaster::SupportCollection& supports,
        const std::string& name)
    {
        const auto id = coaster::allocateSupportStructureId(supports);
        supports.structures.push_back({id, name});
        return supports.structures.back();
    }

    coaster::SupportElementId addNode(
        coaster::SupportStructure& structure,
        const glm::dvec3& position)
    {
        const auto id = coaster::allocateSupportElementId(structure);
        structure.nodes.push_back({id, position});
        return id;
    }

    coaster::SupportElementId addMember(
        coaster::SupportStructure& structure,
        const coaster::SupportElementId start,
        const coaster::SupportElementId end)
    {
        const auto id = coaster::allocateSupportElementId(structure);
        structure.members.push_back({id, start, end, {}});
        return id;
    }

    editor::ViewportCamera camera()
    {
        editor::ViewportCamera result;
        result.setBounds({-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0});
        result.applyPreset(editor::ViewportCameraPreset::Top);
        result.frame(1.0);
        return result;
    }

    void visualizationIsDeterministicAndExact()
    {
        const auto empty = editor::createSupportVisualization({});
        require(empty.memberVertices.empty() && empty.members.empty()
                && empty.nodes.empty(),
            "empty supports must produce empty visualization");

        coaster::SupportCollection supports;
        auto& first = addStructure(supports, "First");
        const auto firstStructureId = first.id;
        const auto firstA = addNode(first, {-1.0, 0.0, 0.0});
        const auto firstB = addNode(first, {1.0, 0.0, 0.0});
        const auto firstMember = addMember(first, firstA, firstB);
        auto& second = addStructure(supports, "Second");
        const auto secondStructureId = second.id;
        const auto secondA = addNode(second, {2.0, 1.0, 3.0});
        const auto secondB = addNode(second, {4.0, 5.0, 6.0});
        const auto secondMember = addMember(second, secondA, secondB);

        const auto visualization = editor::createSupportVisualization(supports);
        require(visualization.memberVertices.size() == 4
                && visualization.members.size() == 2,
            "each member must produce exactly one line-list segment");
        require(visualization.members[0].selection.structureId == firstStructureId
                && visualization.members[0].selection.elementId == firstMember
                && visualization.members[1].selection.structureId == secondStructureId
                && visualization.members[1].selection.elementId == secondMember,
            "structures and members must retain authored ordering and IDs");
        require(visualization.members[1].startPosition
                    == glm::dvec3{2.0, 1.0, 3.0}
                && visualization.members[1].endPosition
                    == glm::dvec3{4.0, 5.0, 6.0},
            "member endpoints must exactly resolve node positions");
        require(visualization.memberVertices[2].x == 2.0F
                && visualization.memberVertices[2].y == 1.0F
                && visualization.memberVertices[2].z == 3.0F
                && visualization.memberVertices[3].x == 4.0F
                && visualization.memberVertices[3].y == 5.0F
                && visualization.memberVertices[3].z == 6.0F,
            "renderer vertices must be float conversions of node positions");
    }

    void pickingReturnsStableIdsWithNodePriority()
    {
        coaster::SupportCollection supports;
        auto& structure = addStructure(supports, "Pick");
        const auto left = addNode(structure, {-2.0, 0.0, 0.0});
        const auto center = addNode(structure, {0.0, 0.0, 0.0});
        const auto right = addNode(structure, {2.0, 0.0, 0.0});
        const auto member = addMember(structure, left, right);
        const auto tiedMember = addMember(structure, left, right);
        const auto visualization = editor::createSupportVisualization(supports);
        const auto view = camera();
        const auto centerProjection = editor::projectViewportPoint(
            view, {0.0, 0.0, 0.0}, 1.0);
        require(centerProjection.has_value(), "pick fixture must project");

        const auto nodeHit = editor::pickSupport(
            visualization, view, centerProjection->normalizedPosition,
            800, 800);
        require(nodeHit.has_value()
                && nodeHit->selection.kind
                    == editor::SupportSelectionKind::Node
                && nodeHit->selection.structureId == structure.id
                && nodeHit->selection.elementId == center,
            "node must win over an overlapping member using stable IDs");

        auto membersOnly = visualization;
        membersOnly.nodes.clear();
        std::reverse(membersOnly.members.begin(), membersOnly.members.end());
        const auto memberHit = editor::pickSupport(
            membersOnly, view, centerProjection->normalizedPosition,
            800, 800);
        require(memberHit.has_value()
                && memberHit->selection.kind
                    == editor::SupportSelectionKind::Member
                && memberHit->selection.elementId == member,
            "member ties must return the lower stable ID, not vector position");
        require(tiedMember > member,
            "member tie fixture must allocate distinct ordered IDs");
    }

    void pickingAndRefreshTieBreakDeterministically()
    {
        coaster::SupportCollection supports;
        auto& first = addStructure(supports, "First");
        const auto firstStructureId = first.id;
        const auto lowId = addNode(first, {0.0, 0.0, 0.0});
        auto& second = addStructure(supports, "Second");
        const auto secondStructureId = second.id;
        const auto highStructureNode = addNode(second, {0.0, 0.0, 0.0});
        auto visualization = editor::createSupportVisualization(supports);
        std::reverse(visualization.nodes.begin(), visualization.nodes.end());
        const auto view = camera();
        const auto projection = editor::projectViewportPoint(
            view, {0.0, 0.0, 0.0}, 1.0);
        const auto hit = editor::pickSupport(
            visualization, view, projection->normalizedPosition, 800, 800);
        require(hit.has_value()
                && hit->selection.structureId == firstStructureId
                && hit->selection.elementId == lowId,
            "exact picking ties must prefer lower stable IDs");

        const editor::SupportSelection retained{
            secondStructureId, editor::SupportSelectionKind::Node,
            highStructureNode};
        require(editor::supportSelectionExists(supports, retained),
            "selection must survive refresh while its IDs remain present");
        supports.structures.pop_back();
        require(!editor::supportSelectionExists(supports, retained),
            "selection must clear when its stable IDs disappear");
    }
}

int main()
{
    try
    {
        visualizationIsDeterministicAndExact();
        pickingReturnsStableIdsWithNodePriority();
        pickingAndRefreshTieBreakDeterministically();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support visualization test failure: "
                  << error.what() << '\n';
        return 1;
    }
}
