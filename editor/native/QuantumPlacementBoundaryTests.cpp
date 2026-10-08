#include "QuantumNative.h"
#include <quantum/coaster/CoasterDocument.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    QnError error{};
    void require(bool v, const char* text) { if (!v) throw std::runtime_error(std::string(text) + ": " + error.message); }
    void ok(QnResult r) { require(r == QN_OK, "Placement ABI call"); }
    QnPlacementSnapshot read(QnSession session)
    { QnPlacementSnapshot result{sizeof(QnPlacementSnapshot), 1}; ok(qn_placement_snapshot(session, &result, &error)); return result; }
    void select(QnSession session, uint32_t mode = 1)
    { ok(qn_track_start_select(session, read(session).revision, 1, mode, &error)); }
    uint64_t begin(QnSession session, uint32_t mode = 1, uint32_t axis = 2)
    {
        const auto state = read(session);
        QnTransformCommand command{state.revision, state.selection_revision, 0, mode, axis, 0};
        uint64_t token = 0; ok(qn_transform_begin(session, &command, &token, &error)); return token;
    }
}
int main()
{
    QnSession session = 0;
    const auto path = std::filesystem::temp_directory_path() / ("quantum-placement-abi-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
    const auto utf8 = path.u8string(); const auto* file = reinterpret_cast<const char*>(utf8.c_str());
    try
    {
        auto track = quantum::coaster::createNewDocument();
        auto physical = track.physicalSettings(); physical.metersPerCoordinateUnit = .25; track.setPhysicalSettings(physical);
        quantum::coaster::setSectionLength(track.section(0), 240);
        { std::ofstream stream(path); stream << quantum::coaster::serializeCoasterDocument(track); }
        ok(qn_create(&session, &error)); ok(qn_open(session, file, &error));
        auto state = read(session); require(state.authored.qw == 1 && state.authored.z == 0 && state.pitch == 0, "Canonical default snapshot");
        QnPlacementSnapshot invalid{1, 1}; require(qn_placement_snapshot(session, &invalid, &error) == QN_INVALID_ARGUMENT, "Placement ABI validates size/version");
        const auto originalRevision = state.revision;
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            const double x = 2, y = axis > 0 ? -3 : 0, z = axis > 1 ? 15 : 0;
            ok(qn_placement_edit(session, state.revision, 0, x, y, z, &error)); state = read(session);
            require(state.revision == originalRevision + axis + 1 && state.authored.x == x && state.authored.y == y && state.authored.z == z,
                "Numeric XYZ are SI metres even at non-unit Core scale");
        }
        require(qn_placement_edit(session, originalRevision, 0, 0, 0, 0, &error) == QN_CONFLICT, "Stale numeric edit rejects");
        require(qn_placement_edit(session, state.revision, 0, 0, 0, std::numeric_limits<double>::infinity(), &error) == QN_INVALID_ARGUMENT,
            "Non-finite position rejects without changing state");
        ok(qn_placement_edit(session, state.revision, 1, 10, 20, 5, &error)); state = read(session);
        require(std::abs(state.pitch - 10) < 1e-12 && std::abs(state.yaw - 20) < 1e-12 && std::abs(state.roll - 5) < 1e-12,
            "Native Inspector conversion roundtrips orientation parameters");
        const auto unchanged = state.revision;
        ok(qn_placement_edit(session, state.revision, 1, state.pitch, state.yaw, state.roll, &error));
        require(read(session).revision == unchanged, "Unchanged displayed orientation creates no edit");
        select(session); state = read(session); auto token = begin(session);
        ok(qn_transform_preview(session, token, 4, &error)); auto preview = read(session);
        require(preview.revision == state.revision && preview.authored.z == 15 && preview.candidate.z == 19 && preview.preview
            && preview.presentation_revision > state.presentation_revision, "Native proposal is transient and advances presentation only");
        require(qn_transform_preview(session, token, std::numeric_limits<double>::quiet_NaN(), &error) == QN_INVALID_ARGUMENT,
            "Invalid native transform proposal rejects");
        require(read(session).candidate.z == 19, "Invalid proposal retains last accepted candidate");
        ok(qn_transform_finish(session, token, 1, &error)); state = read(session);
        require(state.revision == preview.revision + 1 && state.authored.z == 19 && !state.preview, "Commit publishes one authored revision");
        QnCommand history{QN_UNDO}; ok(qn_command(session, &history, &error)); require(read(session).authored.z == 15, "One Undo removes entire drag");
        history.type = QN_REDO; ok(qn_command(session, &history, &error)); require(read(session).authored.z == 19, "Redo restores native proposal");
        for (uint32_t axis = 0; axis < 3; ++axis)
        {
            token = begin(session, 2, axis); ok(qn_transform_preview(session, token, .03, &error));
            ok(qn_transform_finish(session, token, 1, &error));
        }
        state = read(session); token = begin(session);
        ok(qn_transform_preview(session, token, 0, &error)); ok(qn_transform_finish(session, token, 1, &error));
        require(read(session).revision == state.revision, "No-op native gesture has no history revision");
        token = begin(session); ok(qn_transform_preview(session, token, 20, &error));
        ok(qn_transform_finish(session, token, 0, &error)); require(read(session).revision == state.revision, "Canceled gesture has no history revision");
        token = begin(session); ok(qn_transform_preview(session, token, 1, &error));
        ok(qn_placement_edit(session, state.revision, 0, 3, -3, 19, &error));
        require(qn_transform_finish(session, token, 1, &error) == QN_CONFLICT && read(session).authored.x == 3 && !read(session).preview,
            "Stale release cannot overwrite a newer authored edit");
        token = begin(session); const auto selectionBefore = read(session);
        ok(qn_support_workspace(session, 1, &error));
        require(!read(session).preview && read(session).selected && !read(session).active, "Workspace switch cancels gesture and retains Track Start identity");
        ok(qn_support_workspace(session, 0, &error));
        QnTransformCommand stale{read(session).revision, selectionBefore.selection_revision, 0, 1, 2, 0};
        require(qn_transform_begin(session, &stale, &token, &error) == QN_CONFLICT, "Stale viewport selection revision rejects safely");
        token = begin(session); ok(qn_track_start_select(session, read(session).revision, 0, 0, &error));
        require(!read(session).preview && !read(session).selected, "Selection change cancels native gesture");
        const auto beforeSave = read(session); ok(qn_save(session, file, &error)); ok(qn_new(session, &error)); ok(qn_open(session, file, &error));
        const auto restored = read(session);
        require(restored.authored.x == beforeSave.authored.x && restored.authored.z == beforeSave.authored.z
            && std::abs(restored.authored.qw - beforeSave.authored.qw) < 1e-15, "Existing .quantum placement survives save/new/reload");
        ok(qn_destroy(session, &error)); session = 0; std::filesystem::remove(path);
        std::cout << "Track placement ABI checks passed\n";
    }
    catch (const std::exception& failure)
    { if (session) qn_destroy(session, &error); std::filesystem::remove(path); std::cerr << failure.what() << '\n'; return 1; }
}
