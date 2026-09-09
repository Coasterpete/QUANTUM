#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/Supports.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/SupportVisualization.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

    [[nodiscard]] coaster::AuthoredTrack requireState(
        std::optional<coaster::AuthoredTrack> state,
        const std::string_view message)
    {
        require(state.has_value(), message);
        return std::move(*state);
    }

    [[nodiscard]] std::string snapshot(const coaster::AuthoredTrack& track)
    {
        return coaster::serializeCoasterDocument(track);
    }

    void connectCheckClassifiesEveryCase()
    {
        coaster::SupportCollection supports;
        const coaster::SupportStructureId firstId =
            coaster::createSupportStructure(supports, "First");
        const coaster::SupportStructureId secondId =
            coaster::createSupportStructure(supports, "Second");
        const auto firstA = coaster::createSupportNode(
            supports, firstId, {-2.0, 0.0, 0.0});
        const auto firstB = coaster::createSupportNode(
            supports, firstId, {0.0, 0.0, 0.0});
        const auto firstC = coaster::createSupportNode(
            supports, firstId, {2.0, 0.0, 0.0});
        coaster::createSupportMember(
            supports, firstId, firstA, firstB);
        coaster::createSupportNode(
            supports, secondId, {10.0, 0.0, 0.0});

        require(editor::checkSupportNodeConnection(
                    supports, firstId, firstA, firstC)
                == editor::SupportConnectCheck::Ok,
            "a valid new pair must be connectable");
        require(editor::checkSupportNodeConnection(
                    supports, 999, firstA, firstC)
                == editor::SupportConnectCheck::UnknownStructure,
            "an unknown structure must be reported");
        require(editor::checkSupportNodeConnection(
                    supports, firstId, 999, 777)
                == editor::SupportConnectCheck::UnknownNode,
            "endpoints that exist nowhere must be reported unknown");
        require(editor::checkSupportNodeConnection(
                    supports, secondId, firstA, firstB)
                == editor::SupportConnectCheck::DifferentStructure,
            "an endpoint owned by another structure must be reported");
        require(editor::checkSupportNodeConnection(
                    supports, firstId, firstA, firstA)
                == editor::SupportConnectCheck::SameNode,
            "a node connected to itself must be reported");
        require(editor::checkSupportNodeConnection(
                    supports, firstId, firstA, firstB)
                == editor::SupportConnectCheck::AlreadyConnected,
            "an existing unordered pair must be reported");

        for (const editor::SupportConnectCheck check :
            {editor::SupportConnectCheck::UnknownStructure,
                editor::SupportConnectCheck::UnknownNode,
                editor::SupportConnectCheck::SameNode,
                editor::SupportConnectCheck::DifferentStructure,
                editor::SupportConnectCheck::AlreadyConnected})
        {
            require(!editor::supportConnectCheckMessage(check).empty(),
                "every rejection must carry an actionable message");
        }
    }

    void transactionPublishesManualGraphAndRecordsHistory()
    {
        coaster::AuthoredTrack track;
        editor::DocumentHistory history;
        history.reset(track);
        const auto baseline = snapshot(track);

        editor::AuthoredTrackEditTransaction first{track};
        const auto structureId =
            first.candidate().createSupportStructure("Wood bent");
        first.commit(track);
        history.record(track);
        require(track.supports().structures.size() == 1
                && track.supports().structures.front().id == structureId,
            "structure creation must publish through the transaction");

        editor::AuthoredTrackEditTransaction second{track};
        const auto left = second.candidate().createSupportNode(
            structureId, {-2.0, 0.0, 0.0});
        const auto right = second.candidate().createSupportNode(
            structureId, {2.0, 0.0, 0.0});
        const auto top = second.candidate().createSupportNode(
            structureId, {0.0, 0.0, 6.0});
        const auto member = second.candidate().createSupportMember(
            structureId, left, top);
        requireInvalid([&] {
            second.candidate().createSupportMember(
                structureId, left, top);
        }, "a duplicate member must be rejected against the candidate");
        second.commit(track);
        history.record(track);

        const auto& bent = track.supports().structures.front();
        require(bent.nodes.size() == 3 && bent.members.size() == 1
                && bent.members.front().id == member,
            "the accepted network must match the published candidate");
        require(snapshot(track) != baseline,
            "the committed graph must alter the document");
        require(history.canUndo() && !history.canRedo(),
            "two committed graph edits must enable Undo only");

        track = requireState(history.undo(), "member Undo missing");
        require(track.supports().structures.front().members.empty(),
            "Undo must remove the member while retaining its nodes");
        track = requireState(history.undo(), "structure Undo missing");
        require(track.supports().empty(),
            "Undo must return the document to its untouched state");
        track = requireState(history.redo(), "Redo missing");
        require(track.supports().structures.size() == 1,
            "Redo must restore the committed graph state");
    }

    void anchorAndConnectionAuthoringUsesTransactionsAndHistory()
    {
        coaster::AuthoredTrack track = coaster::createNewDocument();
        editor::AuthoredTrackEditTransaction setup{track};
        const auto structureId =
            setup.candidate().createSupportStructure("M2C fixture");
        const auto trackNode = setup.candidate().createSupportNode(
            structureId, {-100.0, -100.0, -100.0});
        const glm::dvec3 foundationPosition{2.0, 3.0, 4.0};
        const auto foundationNode = setup.candidate().createSupportNode(
            structureId, foundationPosition);
        const auto memberId = setup.candidate().createSupportMember(
            structureId, trackNode, foundationNode);
        setup.commit(track);

        editor::DocumentHistory history;
        history.reset(track);

        editor::AuthoredTrackEditTransaction attach{track};
        attach.candidate().setSupportTrackAttachment(
            structureId, trackNode, {10.0, 0.0, 0.0});
        attach.commit(track);
        history.record(track);
        require(track.supports().structures.front().nodes.front()
                    .trackAttachment
                    == coaster::TrackAttachment{10.0, 0.0, 0.0},
            "first track pick must author cumulative station with zero offsets");

        editor::AuthoredTrackEditTransaction stationOne{track};
        stationOne.candidate().setSupportTrackAttachment(
            structureId, trackNode, {11.0, 1.5, -0.25});
        stationOne.commit(track);
        history.record(track, true);
        const std::size_t coalescedSize = history.size();
        editor::AuthoredTrackEditTransaction stationTwo{track};
        stationTwo.candidate().setSupportTrackAttachment(
            structureId, trackNode, {12.0, 2.5, -0.5});
        stationTwo.commit(track);
        history.record(track, true);
        history.endContinuousEdit();
        require(history.size() == coalescedSize,
            "continuous attachment numeric edits must coalesce");
        track = requireState(history.undo(), "attachment numeric Undo missing");
        require(track.supports().structures.front().nodes.front()
                    .trackAttachment->station == 10.0,
            "one Undo must restore the pre-gesture attachment");
        track = requireState(history.redo(), "attachment numeric Redo missing");
        require(track.supports().structures.front().nodes.front()
                    .trackAttachment
                    == coaster::TrackAttachment{12.0, 2.5, -0.5},
            "Redo must restore the exact final attachment gesture state");

        const coaster::SupportCollection beforeRejectedStation =
            track.supports();
        requireInvalid([&] {
            editor::AuthoredTrackEditTransaction rejected{track};
            rejected.candidate().setSupportTrackAttachment(
                structureId, trackNode, {1.0e12, 0.0, 0.0});
        }, "an out-of-domain station must be rejected by Core");
        require(track.supports() == beforeRejectedStation,
            "a rejected attachment candidate must not change the document");

        editor::AuthoredTrackEditTransaction foundation{track};
        foundation.candidate().setSupportFoundation(
            structureId, foundationNode);
        foundation.commit(track);
        history.record(track);
        const auto& anchoredNodes = track.supports().structures.front().nodes;
        require(anchoredNodes[1].foundation.has_value()
                && !anchoredNodes[1].trackAttachment.has_value()
                && anchoredNodes[1].position == foundationPosition,
            "Foundation must preserve position and remain exclusive");

        editor::AuthoredTrackEditTransaction clearFoundation{track};
        clearFoundation.candidate().clearSupportFoundation(
            structureId, foundationNode);
        clearFoundation.commit(track);
        history.record(track);
        require(!track.supports().structures.front().nodes[1]
                    .foundation.has_value(),
            "clearing Foundation must remove only its semantic marker");
        track = requireState(history.undo(), "Foundation clear Undo missing");
        require(track.supports().structures.front().nodes[1]
                    .foundation.has_value(),
            "Undo must restore Foundation");
        track = requireState(history.redo(), "Foundation clear Redo missing");
        require(!track.supports().structures.front().nodes[1]
                    .foundation.has_value(),
            "Redo must clear Foundation again");
        track = requireState(history.undo(), "Foundation restore missing");

        const auto& structureBeforeConnections =
            track.supports().structures.front();
        const coaster::SupportElementId allocatorBeforeConnections =
            structureBeforeConnections.nextElementId;
        editor::AuthoredTrackEditTransaction startConnection{track};
        startConnection.candidate().setSupportMemberEndConnection(
            structureId, memberId, coaster::SupportMemberEnd::Start,
            {coaster::SupportMemberEndTreatment::Saddle});
        startConnection.commit(track);
        history.record(track);
        editor::AuthoredTrackEditTransaction endConnection{track};
        endConnection.candidate().setSupportMemberEndConnection(
            structureId, memberId, coaster::SupportMemberEnd::End,
            {coaster::SupportMemberEndTreatment::Footing});
        endConnection.commit(track);
        history.record(track);
        const auto& connectedMember =
            track.supports().structures.front().members.front();
        require(connectedMember.startConnection->treatment
                    == coaster::SupportMemberEndTreatment::Saddle
                && connectedMember.endConnection->treatment
                    == coaster::SupportMemberEndTreatment::Footing,
            "Start and End treatments must target independent member ends");
        require(track.supports().structures.front().nextElementId
                    == allocatorBeforeConnections,
            "connection authoring must not allocate support element IDs");

        coaster::SupportMemberEndConnection startWithAsset =
            *connectedMember.startConnection;
        startWithAsset.asset = coaster::StaticMeshAssetReference{
            "assets://support/connectors/saddle.glb"};
        editor::AuthoredTrackEditTransaction asset{track};
        asset.candidate().setSupportMemberEndConnection(
            structureId, memberId, coaster::SupportMemberEnd::Start,
            startWithAsset);
        asset.commit(track);
        history.record(track);
        require(track.supports().structures.front().members.front()
                    .startConnection->asset->path
                    == "assets://support/connectors/saddle.glb",
            "a valid logical support asset must be retained");

        coaster::SupportMemberEndConnection startWithoutAsset =
            *track.supports().structures.front().members.front()
                .startConnection;
        startWithoutAsset.asset.reset();
        editor::AuthoredTrackEditTransaction clearAsset{track};
        clearAsset.candidate().setSupportMemberEndConnection(
            structureId, memberId, coaster::SupportMemberEnd::Start,
            startWithoutAsset);
        clearAsset.commit(track);
        history.record(track);
        require(!track.supports().structures.front().members.front()
                    .startConnection->asset.has_value(),
            "a blank asset edit must preserve treatment and clear the asset");
        track = requireState(history.undo(), "asset clear Undo missing");
        require(track.supports().structures.front().members.front()
                    .startConnection->asset.has_value(),
            "Undo must restore the connector asset");
        track = requireState(history.redo(), "asset clear Redo missing");
        require(!track.supports().structures.front().members.front()
                    .startConnection->asset.has_value(),
            "Redo must clear the connector asset again");
        track = requireState(history.undo(), "asset restore missing");

        const coaster::SupportCollection beforeRejectedAsset =
            track.supports();
        requireInvalid([&] {
            coaster::SupportMemberEndConnection malformed =
                *track.supports().structures.front().members.front()
                    .startConnection;
            malformed.asset = coaster::StaticMeshAssetReference{
                "C:\\connectors\\saddle.glb"};
            editor::AuthoredTrackEditTransaction rejected{track};
            rejected.candidate().setSupportMemberEndConnection(
                structureId, memberId, coaster::SupportMemberEnd::Start,
                malformed);
        }, "a Windows connector path must be rejected");
        require(track.supports() == beforeRejectedAsset,
            "a malformed asset candidate must roll back atomically");

        const geometry::CurveFrame frame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}};
        const std::vector<coaster::RiderLocalGeometryState> states{
            {0.0, {0.0, 0.0, 0.0}, frame},
            {60.0, {60.0, 0.0, 0.0}, frame}};
        const editor::SupportVisualization beforeClear =
            editor::createSupportVisualization(track, states);
        editor::AuthoredTrackEditTransaction clearStart{track};
        clearStart.candidate().clearSupportMemberEndConnection(
            structureId, memberId, coaster::SupportMemberEnd::Start);
        clearStart.commit(track);
        history.record(track);
        const editor::SupportVisualization afterClear =
            editor::createSupportVisualization(track, states);
        const auto& clearedMember =
            track.supports().structures.front().members.front();
        require(!clearedMember.startConnection.has_value()
                && clearedMember.endConnection.has_value(),
            "None must clear only the selected member end");
        require(beforeClear.memberVertices.size()
                    == afterClear.memberVertices.size()
                && beforeClear.members.front().startPosition
                    == afterClear.members.front().startPosition
                && beforeClear.members.front().endPosition
                    == afterClear.members.front().endPosition,
            "connection metadata must not create fake support geometry");
        require(afterClear.members.front().startPosition
                    != track.supports().structures.front().nodes.front().position
                && afterClear.members.front().endPosition == foundationPosition,
            "track attachments must move connected visualization while "
            "Foundation stays put");

        track = requireState(history.undo(), "connection clear Undo missing");
        require(track.supports().structures.front().members.front()
                    .startConnection->asset.has_value(),
            "Undo must restore the cleared connection and asset");
        track = requireState(history.redo(), "connection clear Redo missing");
        require(!track.supports().structures.front().members.front()
                    .startConnection.has_value(),
            "Redo must clear the same end again");
    }
}

int main()
{
    connectCheckClassifiesEveryCase();
    transactionPublishesManualGraphAndRecordsHistory();
    anchorAndConnectionAuthoringUsesTransactionsAndHistory();
}
