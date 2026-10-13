#include <quantum/editor/ApplicationService.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/WoodenSupportGenerator.hpp>
#include <quantum/editor/SupportPicking.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace quantum;
    void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
    std::string serialized(editor::ApplicationService& service) { return coaster::serializeCoasterDocument(service.track()); }
    std::uint32_t roundtrip(editor::ApplicationService& service, const editor::SupportEdit& request)
    {
        const auto before = serialized(service);
        const auto revision = service.revision();
        const auto generation = service.centerline().generation();
        const auto created = service.editSupport(request);
        const auto after = serialized(service);
        require(after != before && service.revision() == revision + 1, "One accepted support mutation/revision");
        require(service.centerline().generation() == generation, "Support-only edit retains solved track");
        service.restoreHistory(true); require(serialized(service) == before, "Exact support Undo");
        service.restoreHistory(false); require(serialized(service) == after, "Exact support Redo");
        return created;
    }
    void rejected(editor::ApplicationService& service, const editor::SupportEdit& request)
    {
        const auto before = serialized(service); const auto revision = service.revision();
        const auto redo = service.history().canRedo();
        bool failed = false;
        try { static_cast<void>(service.editSupport(request)); } catch (const std::invalid_argument&) { failed = true; }
        require(failed && serialized(service) == before && service.revision() == revision
            && service.history().canRedo() == redo, "Rejected support edit preserves document, revision and history");
    }
}
int main()
{
    using namespace quantum;
    using Operation = editor::SupportOperation;
    try
    {
        editor::ApplicationService service;
        editor::SupportEdit edit{Operation::CreateStructure}; edit.text = "A-frame";
        const auto structure = roundtrip(service, edit);
        edit = {Operation::RenameStructure, structure}; edit.text = "Authored steel frame"; roundtrip(service, edit);
        edit = {Operation::CreateNode, structure}; edit.position = {0, 0, -5}; const auto start = roundtrip(service, edit);
        edit.position = {2, 0, 1}; const auto end = roundtrip(service, edit);
        edit = {Operation::MoveNode, structure, end}; edit.position = {3, 1, 2}; roundtrip(service, edit);
        service.play(); service.update(1.0 / 60.0);
        const auto playback = service.preview().playbackState();
        const auto tick = service.preview().dynamicsState()->tick;
        edit.position = {4, 1, 2}; static_cast<void>(service.editSupport(edit));
        require(service.preview().playbackState() == playback && service.preview().dynamicsState()->tick == tick,
            "Support editing preserves live fixed-step simulation state");
        service.pause();
        edit = {Operation::SetFoundation, structure, start}; roundtrip(service, edit);
        edit = {Operation::SetAttachment, structure, end}; edit.attachment = {10, 1, -.5}; roundtrip(service, edit);
        const auto expected = coaster::resolveSupportTrackAttachment(service.centerline().visualization().samples, edit.attachment).position;
        require(glm::length(service.supports().nodes[1].position - expected) < 1e-12, "Native track frame supplies attached position");
        edit.attachment = {12, 2, -1}; roundtrip(service, edit);
        edit.attachment.station = 99999; rejected(service, edit);
        edit = {Operation::SetFoundation, structure, end}; rejected(service, edit);
        edit = {Operation::CreateMember, structure}; edit.startNodeId = start; edit.endNodeId = end;
        const auto member = roundtrip(service, edit); rejected(service, edit);
        edit = {Operation::DeleteNode, structure, start}; rejected(service, edit);
        edit = {Operation::SetProfile, structure, member};
        edit.profile = {coaster::SupportMemberProfileShape::Rectangular, {.3, .4}, .02}; roundtrip(service, edit);
        edit.profile.wallThickness = .2; rejected(service, edit);
        edit.profile = {coaster::SupportMemberProfileShape::Circular, {.3, .3}, .01}; roundtrip(service, edit);
        // All nine native treatments, using the anchor required by each end.
        for (unsigned treatment = 0; treatment < 9; ++treatment)
        {
            edit = {Operation::SetTreatment, structure, member};
            edit.treatment = static_cast<coaster::SupportMemberEndTreatment>(treatment);
            edit.memberEnd = treatment == 5 || treatment == 6 ? coaster::SupportMemberEnd::End : coaster::SupportMemberEnd::Start;
            roundtrip(service, edit);
        }
        edit = {Operation::ClearAttachment, structure, end}; rejected(service, edit);
        edit = {Operation::ClearFoundation, structure, start}; rejected(service, edit);
        edit = {Operation::SetConnectorAsset, structure, member}; edit.text = "assets://support/base.glb"; roundtrip(service, edit);
        edit.text = "assets://track/invalid.glb"; rejected(service, edit);
        edit = {Operation::ClearConnection, structure, member}; roundtrip(service, edit);
        edit.memberEnd = coaster::SupportMemberEnd::End; roundtrip(service, edit);
        edit = {Operation::ClearAttachment, structure, end}; roundtrip(service, edit);
        require(service.supports().nodes[1].position == service.track().supports().structures[0].nodes[1].position,
            "Detach restores authored fallback rather than persisting resolved position");
        edit = {Operation::SetAttachment, structure, end}; edit.attachment = {12, 2, -1}; roundtrip(service, edit);
        edit = {Operation::ClearFoundation, structure, start}; roundtrip(service, edit);
        edit = {Operation::SetFoundation, structure, start}; roundtrip(service, edit);

        // Seed existing richer metadata and verify focused form changes retain it.
        auto collection = service.track().supports(); auto& existing = collection.structures[0].members[0];
        existing.role = coaster::SupportMemberRole::Brace; existing.orientation = coaster::SupportMemberOrientation::BentDiagonal;
        existing.orientationReference = glm::dvec3{0, 1, 0};
        coaster::SupportMemberEndConnection connection; connection.treatment = coaster::SupportMemberEndTreatment::Plate;
        connection.localPlacement = coaster::SupportMemberEndPlacement{};
        existing.startConnection = connection;
        auto authored = service.track(); authored.setSupports(collection);
        editor::ApplicationService preserved(std::move(authored));
        edit = {Operation::SetTreatment, structure, member}; edit.treatment = coaster::SupportMemberEndTreatment::Flange;
        roundtrip(preserved, edit);
        const auto& retained = preserved.track().supports().structures[0].members[0];
        require(retained.role == existing.role && retained.orientation == existing.orientation
            && retained.orientationReference == existing.orientationReference
            && retained.startConnection->localPlacement == connection.localPlacement, "Treatment preserves native orientation and placement");
        edit = {Operation::ClearConnectionPlacement, structure, member}; roundtrip(preserved, edit);
        const auto beforeFailure = serialized(preserved); const auto revision = preserved.revision();
        bool uploadFailed = false;
        edit = {Operation::MoveNode, structure, start}; edit.position = {2, 1, -5};
        try { static_cast<void>(preserved.editSupport(edit, [](const auto&) { throw std::runtime_error("Rejected support upload"); })); }
        catch (const std::runtime_error&) { uploadFailed = true; }
        require(uploadFailed && serialized(preserved) == beforeFailure && preserved.revision() == revision,
            "Support publication failure precedes history/document commit");

        const auto file = std::filesystem::temp_directory_path() / ("quantum-support-service-"
            + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
        preserved.save(file); editor::ApplicationService loaded;
        try { loaded.open(file); } catch (...) { std::filesystem::remove(file); throw; }
        std::filesystem::remove(file);
        require(serialized(loaded) == serialized(preserved) && !loaded.history().canUndo(), "Native support save/reload exact document");
        editor::ViewportCamera camera;
        camera.setBounds({-10, -10, -10}, {40, 10, 10}); camera.frame(1.0);
        const auto& visualNode = loaded.supports().nodes[1];
        const auto pointer = editor::projectViewportPoint(camera, visualNode.position, 1.0);
        require(pointer.has_value(), "Attached support node projects through native camera");
        const auto picked = editor::pickSupport(loaded.supports(), camera, pointer->normalizedPosition, 800, 800);
        require(picked && picked->selection == visualNode.selection, "Picking reuses resolved native node metadata");

        edit = {Operation::DeleteMember, structure, member}; roundtrip(service, edit);
        edit = {Operation::DeleteNode, structure, start}; roundtrip(service, edit);
        edit = {Operation::DeleteStructure, structure}; roundtrip(service, edit);
        edit = {Operation::CreateStructure}; const auto nextStructure = service.editSupport(edit);
        require(nextStructure > structure, "Deleted structure IDs are not reused");
        const auto noOpRevision = service.revision();
        edit = {Operation::RenameStructure, nextStructure}; edit.text = service.track().supports().structures[0].name;
        static_cast<void>(service.editSupport(edit)); require(service.revision() == noOpRevision, "No-op has no history/revision churn");

        auto large = coaster::createNewDocument();
        for (int s = 0; s < 3; ++s)
        {
            const auto sid = large.createSupportStructure("Support set " + std::to_string(s));
            coaster::SupportElementId previous = 0;
            for (int n = 0; n < 40; ++n)
            {
                const auto id = large.createSupportNode(sid, {static_cast<double>(n), static_cast<double>(s), -5});
                if (previous) static_cast<void>(large.createSupportMember(sid, previous, id));
                previous = id;
            }
        }
        editor::ApplicationService largeService(std::move(large));
        const auto begin = std::chrono::steady_clock::now();
        edit = {Operation::MoveNode, 2, 1}; edit.position = {0, 1, -6}; roundtrip(largeService, edit);
        require(largeService.supports().nodes.size() == 120 && largeService.supports().members.size() == 117,
            "Representative support set retains complete presentation");
        std::cout << "Supports workspace native service passed; 120 nodes / 117 members edit + Undo/Redo "
            << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count() << " ms.\n";

        auto generatedTrack = coaster::createNewDocument();
        coaster::WoodenSupportRunRecipe recipe; recipe.endStation = 10; recipe.foundationElevation = -5;
        recipe.family = coaster::TimberSupportFamily::HybridTimberLattice;
        const auto generatedId = coaster::generateWoodenSupportRun(generatedTrack, recipe);
        const auto originalStructure = generatedTrack.supports().structures[0];
        const auto originalMember = std::ranges::find_if(originalStructure.members,
            [](const auto& m) { return m.startConnection && m.startConnection->mounting; });
        require(originalMember != originalStructure.members.end(), "Generated fixture carries concrete native mounting");
        editor::ApplicationService generatedService(std::move(generatedTrack));
        edit = {Operation::SetProfile, generatedId, originalMember->id};
        edit.profile = originalMember->profile; edit.profile.outerDimensions *= 1.1; roundtrip(generatedService, edit);
        edit = {Operation::SetTreatment, generatedId, originalMember->id}; edit.treatment = coaster::SupportMemberEndTreatment::Plate;
        roundtrip(generatedService, edit);
        const auto& current = generatedService.track().supports().structures[0];
        const auto changed = std::ranges::find(current.members, originalMember->id, &coaster::SupportMember::id);
        require(current.generatedWoodenRun == originalStructure.generatedWoodenRun && current.appearance == originalStructure.appearance
            && changed->role == originalMember->role && changed->orientation == originalMember->orientation
            && changed->orientationReference == originalMember->orientationReference
            && changed->startConnection->mounting == originalMember->startConnection->mounting,
            "Profile and treatment forms retain generated recipe, appearance, directed frame and mounting");
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
