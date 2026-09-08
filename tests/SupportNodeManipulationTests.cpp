#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/SupportNodeManipulation.hpp>
#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

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

    template<typename Function>
    void requireInvalid(Function&& function, const std::string_view message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw std::runtime_error(std::string(message));
    }

    [[nodiscard]] editor::ViewportCamera camera()
    {
        editor::ViewportCamera result;
        result.setBounds({-10.0, -10.0, -10.0}, {10.0, 10.0, 10.0});
        result.applyPreset(editor::ViewportCameraPreset::Top);
        result.frame(1.0);
        return result;
    }

    [[nodiscard]] editor::SupportSelection nodeSelection(
        const coaster::SupportStructureId structureId,
        const coaster::SupportElementId nodeId)
    {
        return {structureId, editor::SupportSelectionKind::Node, nodeId};
    }

    void movementIntentUsesStableIdsAndCoreValidation()
    {
        coaster::AuthoredTrack track;
        const auto firstStructure = track.createSupportStructure("First");
        const auto movedNode = track.createSupportNode(
            firstStructure, {1.0, 2.0, 3.0});
        const auto neighbor = track.createSupportNode(
            firstStructure, {4.0, 5.0, 6.0});
        static_cast<void>(
            track.createSupportMember(firstStructure, movedNode, neighbor));
        const auto otherStructure = track.createSupportStructure("Other");
        const auto otherNode = track.createSupportNode(
            otherStructure, {-1.0, -2.0, -3.0});
        const std::string before = coaster::serializeCoasterDocument(track);

        const glm::dvec3 moved = editor::translateSupportNode(
            {1.0, 2.0, 3.0}, editor::SupportMoveAxis::X, 2.5);
        editor::AuthoredTrackEditTransaction transaction{track};
        transaction.candidate().setSupportNodePosition(
            firstStructure, movedNode, moved);
        transaction.commit(track);

        const auto& structures = track.supports().structures;
        require(structures[0].nodes[0].id == movedNode
                && structures[0].nodes[0].position
                    == glm::dvec3{3.5, 2.0, 3.0},
            "move intent must target the selected stable node ID");
        require(structures[0].nodes[1].id == neighbor
                && structures[0].nodes[1].position
                    == glm::dvec3{4.0, 5.0, 6.0}
                && structures[1].nodes[0].id == otherNode
                && structures[1].nodes[0].position
                    == glm::dvec3{-1.0, -2.0, -3.0}
                && structures[0].members.size() == 1,
            "moving one node must not alter unrelated graph data");

        requireInvalid([] {
            static_cast<void>(editor::translateSupportNode(
                {0.0, 0.0, 0.0}, editor::SupportMoveAxis::Y,
                std::numeric_limits<double>::infinity()));
        }, "non-finite move intent must be rejected");

        const std::string accepted = coaster::serializeCoasterDocument(track);
        requireInvalid([&] {
            track.setSupportNodePosition(
                firstStructure, 999999, {7.0, 8.0, 9.0});
        }, "a stale selected node ID must be rejected");
        require(coaster::serializeCoasterDocument(track) == accepted,
            "a stale selected ID must not mutate the document");
        require(before != accepted,
            "the valid stable-ID movement must change the document");
    }

    void snapSelectionIsScreenSpaceAndDeterministic()
    {
        const auto view = camera();
        editor::SupportVisualization visualization;
        const editor::SupportSelection dragged = nodeSelection(7, 11);
        visualization.nodes.push_back({dragged, {-4.0, 0.0, 0.0}});
        visualization.nodes.push_back({nodeSelection(7, 12), {0.25, 0.0, 0.0}});
        visualization.nodes.push_back({nodeSelection(8, 1), {0.0, 0.0, 0.0}});

        const auto center = editor::projectViewportPoint(
            view, {0.0, 0.0, 0.0}, 1.0);
        require(center.has_value(), "snap fixture must project");
        const auto nearest = editor::pickSupportNodeSnapTarget(
            visualization, view, center->normalizedPosition,
            800, 800, dragged, 30.0);
        require(nearest.has_value()
                && nearest->selection == nodeSelection(8, 1),
            "nearest eligible projected node must win and dragged node must be excluded");

        editor::SupportVisualization tied;
        tied.nodes.push_back({dragged, {-4.0, 0.0, 0.0}});
        tied.nodes.push_back({nodeSelection(9, 3), {0.0, 0.0, 0.0}});
        tied.nodes.push_back({nodeSelection(8, 20), {0.0, 0.0, 0.0}});
        std::reverse(tied.nodes.begin(), tied.nodes.end());
        const auto idTie = editor::pickSupportNodeSnapTarget(
            tied, view, center->normalizedPosition, 800, 800, dragged);
        require(idTie.has_value()
                && idTie->selection == nodeSelection(8, 20),
            "equal screen/depth targets must prefer structure then element ID");

        const editor::ViewportRay ray = view.viewportRay(0.5, 0.5, 1.0);
        editor::SupportVisualization depthFixture;
        depthFixture.nodes.push_back({dragged, {-4.0, 0.0, 0.0}});
        depthFixture.nodes.push_back({nodeSelection(2, 2),
            {0.0, 0.0, 0.0}});
        depthFixture.nodes.push_back({nodeSelection(3, 3),
            {0.0, 0.0, 5.0}});
        const auto depthTie = editor::pickSupportNodeSnapTarget(
            depthFixture, view, {0.5, 0.5}, 800, 800, dragged);
        require(depthTie.has_value()
                && depthTie->selection == nodeSelection(3, 3),
            "equal screen-distance targets must prefer frontmost depth");

        editor::SupportVisualization visibilityFixture;
        visibilityFixture.nodes.push_back({dragged, {-4.0, 0.0, 0.0}});
        visibilityFixture.nodes.push_back({nodeSelection(1, 2),
            ray.origin - 2.0 * ray.direction});
        require(!editor::pickSupportNodeSnapTarget(
                visibilityFixture, view, {0.5, 0.5},
                800, 800, dragged, 100.0).has_value(),
            "a target behind the camera must be ignored");

        require(nearest->selection.structureId != dragged.structureId,
            "fixture must prove cross-structure positional snap eligibility");
    }

    void connectedCollapseTargetsAreExcluded()
    {
        const auto view = camera();
        const editor::SupportSelection dragged = nodeSelection(4, 1);
        const editor::SupportSelection connected = nodeSelection(4, 2);
        editor::SupportVisualization visualization;
        visualization.nodes = {
            {dragged, {-2.0, 0.0, 0.0}},
            {connected, {0.0, 0.0, 0.0}}};
        visualization.members.push_back({
            {4, editor::SupportSelectionKind::Member, 3},
            dragged.elementId,
            connected.elementId,
            {-2.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}});
        const auto projected = editor::projectViewportPoint(
            view, {0.0, 0.0, 0.0}, 1.0);
        require(projected.has_value(), "collapse fixture must project");
        require(!editor::pickSupportNodeSnapTarget(
                visualization, view, projected->normalizedPosition,
                800, 800, dragged).has_value(),
            "a directly connected zero-length snap target must be excluded");
    }

    void snapResultsAreExactAndPrioritized()
    {
        const editor::SupportNodeSnapTarget target{
            nodeSelection(22, 31),
            {0.1, std::nextafter(2.0, 3.0), -7.25},
            1.0,
            0.5};
        const auto node = editor::resolveSupportPositionSnap(
            {9.0, 8.0, 0.01}, editor::SupportMoveAxis::Z, target,
            true, true, 0.1);
        require(node.kind == editor::SupportPositionSnapKind::Node
                && node.position == target.position
                && node.nodeTarget == target.selection,
            "node snap must win over ground and preserve exact authored doubles");

        const auto ground = editor::resolveSupportPositionSnap(
            {9.0, 8.0, 0.05}, editor::SupportMoveAxis::Z, std::nullopt,
            true, true, 0.1);
        require(ground.kind == editor::SupportPositionSnapKind::Ground
                && ground.position == glm::dvec3{9.0, 8.0, 0.0},
            "eligible ground snap must set exact zero and preserve X/Y");
        require(editor::resolveSupportPositionSnap(
                {9.0, 8.0, 0.05}, editor::SupportMoveAxis::Z, std::nullopt,
                true, false, 0.1).position.z == 0.05,
            "disabled ground snap must not modify Z");
        require(editor::resolveSupportPositionSnap(
                {9.0, 8.0, 0.2}, editor::SupportMoveAxis::Z, std::nullopt,
                true, true, 0.1).position.z == 0.2,
            "ground snap must not pull nodes outside its threshold");
        require(editor::resolveSupportPositionSnap(
                {9.0, 8.0, 0.05}, editor::SupportMoveAxis::X, std::nullopt,
                true, true, 0.1).position.z == 0.05,
            "ground snap must apply only to Z-axis movement");
    }

    void transactionVisualizationAndHistoryRemainAtomic()
    {
        coaster::AuthoredTrack track;
        const auto structureId = track.createSupportStructure("Bent");
        const auto movedId = track.createSupportNode(
            structureId, {-1.0, 0.0, 2.0});
        const auto fixedId = track.createSupportNode(
            structureId, {1.0, 0.0, 2.0});
        static_cast<void>(
            track.createSupportMember(structureId, movedId, fixedId));
        const auto targetStructure = track.createSupportStructure("Target");
        const glm::dvec3 exactTarget{
            0.1, std::nextafter(3.0, 4.0), 0.0};
        const auto targetId = track.createSupportNode(
            targetStructure, exactTarget);

        const std::string initial = coaster::serializeCoasterDocument(track);
        editor::DocumentHistory history;
        history.reset(track);
        for (const glm::dvec3 position : {
            glm::dvec3{-0.5, 0.0, 2.0},
            glm::dvec3{0.0, 0.0, 1.0},
            exactTarget})
        {
            editor::AuthoredTrackEditTransaction transaction{track};
            transaction.candidate().setSupportNodePosition(
                structureId, movedId, position);
            const auto candidateVisualization =
                editor::createSupportVisualization(
                    transaction.candidate().supports());
            require(candidateVisualization.members.front().startPosition
                    == position,
                "accepted movement must regenerate attached member endpoints");
            transaction.commit(track);
            history.record(track, true);
        }
        history.endContinuousEdit();

        require(history.size() == 2,
            "one live drag must coalesce to one history entry");
        const auto& supports = track.supports();
        require(supports.structures[0].nodes[0].position == exactTarget
                && supports.structures[0].nodes[0].id == movedId
                && supports.structures[1].nodes[0].id == targetId
                && supports.structures[0].members.size() == 1
                && supports.structures[1].members.empty(),
            "positional snap must not merge IDs, add members, or change ownership");

        auto undone = history.undo();
        require(undone.has_value()
                && coaster::serializeCoasterDocument(*undone) == initial,
            "Undo must restore the exact pre-drag document");
        auto redone = history.redo();
        require(redone.has_value()
                && redone->supports().structures[0].nodes[0].position
                    == exactTarget,
            "Redo must restore the exact snapped double position");

        const auto committedVisualization =
            editor::createSupportVisualization(track.supports());
        const std::string committed = coaster::serializeCoasterDocument(track);
        requireInvalid([&] {
            editor::AuthoredTrackEditTransaction rejected{track};
            rejected.candidate().setSupportNodePosition(
                structureId, movedId,
                {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        }, "failed movement must be rejected by Core");
        require(coaster::serializeCoasterDocument(track) == committed
                && editor::createSupportVisualization(track.supports())
                        .members.front().startPosition
                    == committedVisualization.members.front().startPosition,
            "failed movement must preserve document and visualization state");
    }

    void connectModeAndStaleSelectionGateTheGizmo()
    {
        editor::SupportVisualization visualization;
        const auto live = nodeSelection(3, 9);
        visualization.nodes.push_back({live, {0.0, 0.0, 0.0}});
        require(editor::supportNodeManipulationAvailable(
                visualization, live, false),
            "a live selected node must expose the Move gizmo");
        require(!editor::supportNodeManipulationAvailable(
                visualization, live, true),
            "Connect Nodes waiting state must disable the Move gizmo");
        require(editor::supportNodeManipulationAvailable(
                visualization, live, false),
            "canceling Connect Nodes must restore manipulation");
        require(!editor::supportNodeManipulationAvailable(
                visualization, nodeSelection(3, 99), false),
            "a stale selected ID must not expose manipulation");
        require(!editor::supportNodeManipulationAvailable(
                visualization,
                editor::SupportSelection{
                    3, editor::SupportSelectionKind::Member, 9},
                false),
            "members must not expose the node Move gizmo");
    }
}

int main()
{
    try
    {
        movementIntentUsesStableIdsAndCoreValidation();
        snapSelectionIsScreenSpaceAndDeterministic();
        connectedCollapseTargetsAreExcluded();
        snapResultsAreExactAndPrioritized();
        transactionVisualizationAndHistoryRemainAtomic();
        connectModeAndStaleSelectionGateTheGizmo();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Support node manipulation test failure: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "Support node manipulation tests passed.\n";
}
