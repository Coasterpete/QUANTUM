#include <quantum/editor/ApplicationService.hpp>
#include <algorithm>
#include <stdexcept>

namespace quantum::editor
{
    std::uint32_t ApplicationService::editSupport(const SupportEdit& request,
        const SupportPublish& publish)
    {
        using Operation = SupportOperation;
        AuthoredTrackEditTransaction transaction{track_};
        auto& candidate = transaction.candidate();
        std::uint32_t created = 0;
        // Work on native values only. Final setSupports validates anchor and
        // connection invariants even for fields without a dedicated Core setter.
        auto collection = candidate.supports();
        const auto structure = [&]() -> coaster::SupportStructure&
        {
            const auto found = std::ranges::find(collection.structures, request.structureId,
                &coaster::SupportStructure::id);
            if (found == collection.structures.end()) throw std::invalid_argument("Unknown support structure ID.");
            return *found;
        };
        const auto member = [&]() -> coaster::SupportMember&
        {
            auto& members = structure().members;
            const auto found = std::ranges::find(members, request.elementId, &coaster::SupportMember::id);
            if (found == members.end()) throw std::invalid_argument("Unknown support member ID.");
            return *found;
        };
        const auto connection = [&]() -> coaster::SupportMemberEndConnection
        {
            if (request.memberEnd != coaster::SupportMemberEnd::Start
                && request.memberEnd != coaster::SupportMemberEnd::End)
                throw std::invalid_argument("Unknown support member end.");
            const auto& m = member();
            return (request.memberEnd == coaster::SupportMemberEnd::Start
                ? m.startConnection : m.endConnection).value_or(coaster::SupportMemberEndConnection{});
        };
        switch (request.operation)
        {
        case Operation::CreateStructure:
            created = coaster::createSupportStructure(collection, request.text); break;
        case Operation::DeleteStructure:
            coaster::removeSupportStructure(collection, request.structureId); break;
        case Operation::RenameStructure:
            if (request.text.empty()) throw std::invalid_argument("A support structure name is required.");
            structure().name = request.text; break;
        case Operation::CreateNode:
            created = coaster::createSupportNode(collection, request.structureId, request.position); break;
        case Operation::DeleteNode:
            coaster::removeSupportNode(collection, request.structureId, request.elementId); break;
        case Operation::MoveNode:
            // Authored fallback position has the same meaning as in ImGui;
            // attached world positions are derived, never overwritten here.
            candidate.setSupportNodePosition(request.structureId, request.elementId, request.position);
            collection = candidate.supports(); break;
        case Operation::SetAttachment:
            coaster::setSupportTrackAttachment(collection, request.structureId, request.elementId, request.attachment); break;
        case Operation::ClearAttachment:
            coaster::clearSupportTrackAttachment(collection, request.structureId, request.elementId); break;
        case Operation::SetFoundation:
            coaster::setSupportFoundation(collection, request.structureId, request.elementId); break;
        case Operation::ClearFoundation:
            coaster::clearSupportFoundation(collection, request.structureId, request.elementId); break;
        case Operation::CreateMember:
            created = coaster::createSupportMember(collection, request.structureId,
                request.startNodeId, request.endNodeId, request.profile); break;
        case Operation::DeleteMember:
            coaster::removeSupportMember(collection, request.structureId, request.elementId); break;
        case Operation::SetProfile:
            member().profile = request.profile; break;
        case Operation::SetTreatment:
        {
            auto value = connection(); value.treatment = request.treatment;
            coaster::setSupportMemberEndConnection(collection, request.structureId, request.elementId, request.memberEnd, value); break;
        }
        case Operation::ClearConnection:
            coaster::clearSupportMemberEndConnection(collection, request.structureId, request.elementId, request.memberEnd); break;
        case Operation::SetConnectorAsset:
        case Operation::ClearConnectionPlacement:
        {
            const auto& m = member();
            if (!(request.memberEnd == coaster::SupportMemberEnd::Start ? m.startConnection : m.endConnection))
                throw std::invalid_argument("Choose a treatment before editing its connector.");
            auto value = connection();
            if (request.operation == Operation::ClearConnectionPlacement) value.localPlacement.reset();
            else if (request.text.empty()) value.asset.reset();
            else value.asset = coaster::StaticMeshAssetReference{request.text};
            coaster::setSupportMemberEndConnection(collection, request.structureId, request.elementId, request.memberEnd, value); break;
        }
        default: throw std::invalid_argument("Unknown support operation.");
        }
        candidate.setSupports(collection);
        if (candidate.supports() == track_.supports()) return created;
        auto visualization = createSupportVisualization(candidate, centerline_.visualization().samples);
        auto history = history_;
        history.record(candidate);
        if (publish) publish(visualization);
        transaction.commit(track_);
        supports_ = std::move(visualization);
        history_ = std::move(history);
        profileDrag_.reset();
        ++revision_;
        synchronizeDirtyState();
        return created;
    }
}
