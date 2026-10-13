#include "QuantumNative.h"
#include <quantum/coaster/CoasterDocument.hpp>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    QnError error{};
    void require(bool condition, const char* message)
    { if (!condition) throw std::runtime_error(std::string(message) + ": " + error.message); }
    void ok(QnResult result) { require(result == QN_OK, "Support boundary call"); }
    struct Snapshot
    {
        QnSupportSnapshot summary{sizeof(QnSupportSnapshot), 1};
        std::vector<QnSupportStructure> structures;
        std::vector<QnSupportNode> nodes;
        std::vector<QnSupportMember> members;
    };
    Snapshot read(QnSession session)
    {
        Snapshot result;
        const auto size = qn_support_snapshot(session, &result.summary, nullptr, 0, nullptr, 0, nullptr, 0, 0, &error);
        require(size == QN_OK || size == QN_BUFFER_TOO_SMALL, "Support sizing returns all capacities");
        result.structures.resize(result.summary.structure_count); result.nodes.resize(result.summary.node_count);
        result.members.resize(result.summary.member_count);
        ok(qn_support_snapshot(session, &result.summary, result.structures.data(), result.summary.structure_count,
            result.nodes.data(), result.summary.node_count, result.members.data(), result.summary.member_count, 0, &error));
        return result;
    }
    uint32_t edit(QnSession session, QnSupportCommand command, const char* text = nullptr)
    {
        command.expected_revision = read(session).summary.revision;
        uint32_t created = 0; ok(qn_support_command(session, &command, text, &created, &error)); return created;
    }
}
int main()
{
    QnSession session = 0;
    try
    {
        ok(qn_create(&session, &error));
        const auto initial = read(session); require(initial.structures.empty(), "Empty native support baseline");
        QnSupportCommand command{}; command.operation = QN_SUPPORT_CREATE_STRUCTURE;
        const auto sid = edit(session, command, "Unicode frame \xc3\xa9");
        command = {}; command.operation = QN_SUPPORT_CREATE_NODE; command.structure_id = sid; command.z = -5;
        const auto a = edit(session, command); command.z = 2; const auto b = edit(session, command);
        command = {}; command.operation = QN_SUPPORT_CREATE_MEMBER; command.structure_id = sid;
        command.start_node = a; command.end_node = b; command.width = command.height = .1;
        const auto member = edit(session, command);
        auto before = read(session);
        require(before.nodes.size() == 2 && before.members.size() == 1 && before.members[0].id == member,
            "Boundary exposes stable native identities");
        require(std::strcmp(before.structures[0].name, "Unicode frame \xc3\xa9") == 0, "UTF-8 support name");
        ok(qn_support_workspace(session, 1, &error));
        ok(qn_select_support(session, before.summary.revision, 3, sid, member, &error));
        auto selected = read(session);
        require(selected.summary.revision == before.summary.revision && selected.summary.selection_revision > before.summary.selection_revision
            && selected.summary.presentation_revision == before.summary.presentation_revision && selected.summary.selection_element == member,
            "Selection changes its own snapshot counter only");
        QnSupportStructure structureSentinel{}; structureSentinel.id = 999;
        QnSupportNode nodeSentinel{}; nodeSentinel.id = 999;
        QnSupportMember memberSentinel{}; memberSentinel.id = 999;
        auto cached = selected.summary;
        ok(qn_support_snapshot(session, &cached, &structureSentinel, 1, &nodeSentinel, 1, &memberSentinel, 1,
            cached.revision, &error));
        require(structureSentinel.id == 999 && nodeSentinel.id == 999 && memberSentinel.id == 999, "Known revision preserves all support buffers");
        cached.revision = 0;
        require(qn_support_snapshot(session, &cached, &structureSentinel, 1, &nodeSentinel, 1, &memberSentinel, 1,
            0, &error) == QN_BUFFER_TOO_SMALL && structureSentinel.id == 999 && memberSentinel.id == 999,
            "Insufficient capacity copies summary only, never partial arrays");
        command = {}; command.expected_revision = initial.summary.revision; command.operation = QN_SUPPORT_DELETE_STRUCTURE;
        command.structure_id = sid; uint32_t created = 77;
        require(qn_support_command(session, &command, nullptr, &created, &error) == QN_CONFLICT && created == 0, "Stale support mutation rejects and clears output");
        command.expected_revision = before.summary.revision; command.operation = QN_SUPPORT_MOVE_NODE;
        command.element_id = b; command.x = std::numeric_limits<double>::quiet_NaN();
        require(qn_support_command(session, &command, nullptr, &created, &error) == QN_INVALID_ARGUMENT, "Nonfinite node mutation rejects");
        const auto invalid = read(session);
        require(invalid.summary.revision == selected.summary.revision && invalid.summary.selection_revision == selected.summary.selection_revision,
            "Invalid mutation retains authored and selection snapshot");
        require(qn_select_support(session, invalid.summary.revision, 2, sid, member, &error) == QN_INVALID_ARGUMENT,
            "Member ID cannot identify a node");
        require(qn_select_support(session, invalid.summary.revision - 1, 1, sid, 0, &error) == QN_CONFLICT, "Stale selection rejects");
        command = {}; command.operation = QN_SUPPORT_SET_ATTACHMENT; command.structure_id = sid; command.element_id = b;
        command.x = 10; command.y = 1; command.z = -.5; edit(session, command);
        const auto attached = read(session);
        require(attached.nodes[1].anchor == 1 && attached.nodes[1].station == 10
            && attached.nodes[1].resolved_x != attached.nodes[1].x
            && attached.summary.presentation_revision > invalid.summary.presentation_revision
            && attached.summary.selection_element == member, "Attachment updates native resolved presentation without changing selection");
        command = {}; command.operation = QN_SUPPORT_DELETE_MEMBER; command.structure_id = sid; command.element_id = member; edit(session, command);
        require(read(session).summary.selection_kind == 1 && read(session).summary.selection_structure == sid,
            "Deleted element selection returns to its surviving structure");
        QnCommand undo{QN_UNDO}; ok(qn_command(session, &undo, &error));
        require(read(session).members[0].id == member && read(session).summary.selection_kind == 1,
            "Undo restores stable member data while retaining its structure context");
        ok(qn_select_support(session, read(session).summary.revision, 2, sid, a, &error));
        ok(qn_support_workspace(session, 0, &error));
        require(read(session).summary.selection_element == a, "Workspace switching retains transient support selection");
        ok(qn_new(session, &error)); require(read(session).summary.selection_kind == 0, "New clears selection despite possible ID reuse");
        auto malformed = read(session).summary; malformed.version = 2;
        require(qn_support_snapshot(session, &malformed, nullptr, 0, nullptr, 0, nullptr, 0, 0, &error) == QN_INVALID_ARGUMENT,
            "Snapshot version validation");
        // Core generates the test document; production managed code never
        // serializes supports. This crosses a non-unit-scale ABI both ways.
        auto scaled = quantum::coaster::createNewDocument();
        auto physical = scaled.physicalSettings(); physical.metersPerCoordinateUnit = 2; scaled.setPhysicalSettings(physical);
        const auto scaledStructure = scaled.createSupportStructure("Scaled");
        const auto scaledA = scaled.createSupportNode(scaledStructure, {2, 1, -4});
        const auto scaledB = scaled.createSupportNode(scaledStructure, {3, 1, -4});
        const auto scaledMember = scaled.createSupportMember(scaledStructure, scaledA, scaledB);
        const auto file = std::filesystem::temp_directory_path() / ("quantum-support-boundary-"
            + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
        const auto path = file.u8string(); const auto utf8 = reinterpret_cast<const char*>(path.c_str());
        try
        {
            { std::ofstream output(file); output << quantum::coaster::serializeCoasterDocument(scaled); }
            ok(qn_open(session, utf8, &error));
            const auto scaledSnapshot = read(session);
            require(scaledSnapshot.nodes[0].x == 4 && scaledSnapshot.nodes[0].z == -8 && scaledSnapshot.members[0].width == .2,
                "Support snapshot converts Core positions and dimensions to metres");
            command = {}; command.operation = QN_SUPPORT_MOVE_NODE; command.structure_id = scaledStructure;
            command.element_id = scaledA; command.x = 6; command.y = 4; command.z = -10; edit(session, command);
            command = {}; command.operation = QN_SUPPORT_SET_ATTACHMENT; command.structure_id = scaledStructure;
            command.element_id = scaledB; command.x = 20; command.y = 4; command.z = -2; edit(session, command);
            command = {}; command.operation = QN_SUPPORT_SET_PROFILE; command.structure_id = scaledStructure;
            command.element_id = scaledMember; command.shape = 1; command.width = .6; command.height = .8; command.wall = .04; edit(session, command);
            ok(qn_save(session, utf8, &error));
            std::ifstream input(file); const std::string json{std::istreambuf_iterator<char>(input), {}};
            const auto restored = quantum::coaster::deserializeCoasterDocument(json);
            require(restored.has_value(), "Scaled native saved document reloads");
            const auto& graph = restored->supports().structures[0];
            require(graph.nodes[0].position == glm::dvec3(3, 2, -5)
                && *graph.nodes[1].trackAttachment == quantum::coaster::TrackAttachment{10, 2, -1}
                && graph.members[0].profile.outerDimensions == glm::dvec2(.3, .4)
                && graph.members[0].profile.wallThickness == .02, "Metre commands retain exact native serialized Core-unit semantics");
        }
        catch (...) { std::filesystem::remove(file); throw; }
        std::filesystem::remove(file);
        ok(qn_destroy(session, &error)); session = 0;
        std::cout << "Support boundary snapshot/revision/selection checks passed.\n";
        return 0;
    }
    catch (const std::exception& failure)
    {
        if (session) qn_destroy(session, &error);
        std::cerr << failure.what() << '\n'; return 1;
    }
}
