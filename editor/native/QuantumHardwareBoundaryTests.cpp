#include "QuantumNative.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace
{
    QnError error{}; int checks = 0;
    void require(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
    void ok(QnResult result) { if (result != QN_OK) throw std::runtime_error(error.message); }
    QnSnapshot summary(QnSession session)
    {
        QnSnapshot s{}; s.size = sizeof(s); s.version = 1;
        auto result = qn_snapshot(session, &s, nullptr, 0, 0, &error);
        if (result != QN_BUFFER_TOO_SMALL) ok(result);
        std::vector<QnRegion> regions(s.region_count);
        ok(qn_snapshot(session, &s, regions.data(), s.region_count, 0, &error)); return s;
    }
    nlohmann::json hardware(QnSession session, uint64_t revision)
    {
        uint32_t required = 0;
        require(qn_hardware_snapshot(session, revision, nullptr, 0, &required, &error) == QN_BUFFER_TOO_SMALL, "Hardware sizing call");
        std::vector<char> text(required); ok(qn_hardware_snapshot(session, revision, text.data(), required, &required, &error));
        return nlohmann::json::parse(text.data());
    }
}
int main()
{
    QnSession session = 0;
    try
    {
        ok(qn_create(&session, &error));
        for (uint32_t kind = 0; kind <= 8; ++kind)
        {
            auto s = summary(session); uint64_t id = 0; ok(qn_device_create(session, s.revision, kind, &id, &error));
            auto after = summary(session); auto data = hardware(session, after.revision);
            require(data["hardware"].back()["id"] == id && data["hardware"].back()["configuration"].is_object(), "Hardware-specific native configuration snapshot");
            require(qn_hardware_command(session, s.revision, id, 0, 1, 0, &error) == QN_CONFLICT, "Stale runtime command rejected");
        }
        auto baseline = summary(session); const auto revision = baseline.revision;
        auto authored = hardware(session, revision); auto gate = authored["hardware"].back()["id"].get<uint64_t>();
        require(qn_hardware_command(session, revision, gate, 4, 0, 0, &error) == QN_INVALID_ARGUMENT, "Locked gate opening rejected");
        ok(qn_hardware_command(session, revision, gate, 7, 0, 0, &error));
        ok(qn_hardware_command(session, revision, gate, 4, 0, 0, &error));
        require(summary(session).revision == revision, "Runtime gate commands do not edit document");
        require(hardware(session, revision)["hardware"].back()["state"] == "Opening", "Native gate telemetry exposes runtime state");
        auto config = authored["hardware"].back()["configuration"]; config["motion"] = 1;
        const auto text = config.dump(); ok(qn_hardware_configuration(session, revision, gate, text.c_str(), &error));
        auto changed = summary(session);
        require(changed.revision > revision && hardware(session, changed.revision)["hardware"].back()["configuration"]["motion"] == 1, "Gate config edit is authored");
        require(qn_hardware_configuration(session, revision, gate, text.c_str(), &error) == QN_CONFLICT, "Stale configuration edit rejected");
        config["unexpected"] = 1;
        require(qn_hardware_configuration(session, changed.revision, gate, config.dump().c_str(), &error) == QN_INVALID_ARGUMENT, "Malformed typed config rejected at C boundary");
        require(summary(session).revision == changed.revision, "Malformed config preserves revision");
        QnCommand command{}; command.type = QN_UNDO; ok(qn_command(session, &command, &error));
        auto undo = summary(session); require(hardware(session, undo.revision)["hardware"].back()["configuration"]["motion"] == 0, "Gate config Undo");
        command.type = QN_REDO; ok(qn_command(session, &command, &error));
        auto redo = summary(session); require(hardware(session, redo.revision)["hardware"].back()["configuration"]["motion"] == 1, "Gate config Redo");
        require(qn_device_create(session, redo.revision, 9, &gate, &error) == QN_INVALID_ARGUMENT, "Unknown hardware kind rejected");
        ok(qn_destroy(session, &error)); session = 0;
        std::cout << "QuantumHardwareBoundaryTests passed " << checks << " checks\n"; return 0;
    }
    catch (const std::exception& e) { if (session) qn_destroy(session, &error); std::cerr << e.what() << '\n'; return 1; }
}
