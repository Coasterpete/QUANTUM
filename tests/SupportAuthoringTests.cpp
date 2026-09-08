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
}

int main()
{
    connectCheckClassifiesEveryCase();
    transactionPublishesManualGraphAndRecordsHistory();
}