#include "QuantumNative.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <iostream>
#include <stdexcept>

namespace
{
    QnError error{}; int checks = 0;
    void require(bool value, const char* text) { ++checks; if (!value) throw std::runtime_error(text); }
    void ok(QnResult result) { if (result != QN_OK) throw std::runtime_error(error.message); }
    QnSnapshot snapshot(QnSession handle)
    {
        QnSnapshot result{}; result.size = sizeof(result); result.version = 1;
        auto code = qn_snapshot(handle, &result, nullptr, 0, 0, &error);
        if (code != QN_BUFFER_TOO_SMALL) ok(code);
        std::vector<QnRegion> regions(result.region_count);
        ok(qn_snapshot(handle, &result, regions.data(), result.region_count, 0, &error)); return result;
    }
    nlohmann::json train(QnSession handle)
    {
        uint32_t size = 0; auto revision = snapshot(handle).revision;
        require(qn_train_snapshot(handle, revision, nullptr, 0, &size, &error) == QN_BUFFER_TOO_SMALL, "Train sizing");
        std::vector<char> text(size); ok(qn_train_snapshot(handle, revision, text.data(), size, &size, &error));
        return nlohmann::json::parse(text.data());
    }
}
int main()
{
    QnSession handle = 0;
    try
    {
        ok(qn_create(&handle, &error)); auto initial = train(handle); auto revision = snapshot(handle).revision;
        auto id = initial["configuration"]["cars"][0]["id"].get<uint64_t>();
        ok(qn_train_view(handle, revision, 1, id, 31, &error));
        for (uint32_t action = 14; action <= 18; ++action)
            require(qn_viewport_input(handle, action, .5, .5, 0, 0, &error) == QN_UNAVAILABLE,
                "Train camera action passes validation before reporting absent viewport");
        require(qn_viewport_input(handle, 19, 0, 0, 0, 0, &error) == QN_INVALID_ARGUMENT, "Unknown camera action rejected");
        require(qn_viewport_input(handle, 18, 2, .5, 0, 0, &error) == QN_INVALID_ARGUMENT, "Car pick normalized coordinate checked");
        require(snapshot(handle).revision == revision && train(handle)["selectedCar"] == id, "Selection/overlays are transient");
        uint32_t size = 0;
        require(qn_train_snapshot(handle, revision, nullptr, 20, &size, &error) == QN_INVALID_ARGUMENT, "Invalid buffer rejected");
        require(qn_train_view(handle, revision, 1, 999, 31, &error) == QN_INVALID_ARGUMENT, "Unknown car selection rejected");
        ok(qn_train_edit(handle, revision, 3, id, 0, nullptr, &error)); auto copied = train(handle);
        require(copied["configuration"]["cars"].size() == 5 && copied["configuration"]["cars"][1]["id"] != id, "Boundary duplicate has fresh stable identity");
        require(qn_train_edit(handle, revision, 4, id, 0, nullptr, &error) == QN_CONFLICT, "Stale Train edit rejected");
        revision = snapshot(handle).revision;
        auto proposal = copied["configuration"]; proposal["cars"][0]["physical"]["dryMassKilograms"] = -2;
        const auto text = proposal.dump();
        require(qn_train_edit(handle, revision, 0, 0, 0, text.c_str(), &error) == QN_INVALID_ARGUMENT, "Invalid native Train proposal rejected");
        require(snapshot(handle).revision == revision, "Rejected proposal preserves revision");
        require(qn_train_validate(handle, revision, text.c_str(), nullptr, 0, &size, &error) == QN_BUFFER_TOO_SMALL, "Proposal diagnostics sizing");
        std::vector<char> buffer(size); ok(qn_train_validate(handle, revision, text.c_str(), buffer.data(), size, &size, &error));
        require(nlohmann::json::parse(buffer.data())["diagnostics"][0]["severity"] == "ERROR", "Errors distinguished from warnings");
        auto current = copied["configuration"]; current["cars"][0]["physical"]["dryMassKilograms"] = 900;
        current["cars"][0]["visualAsset"] = "assets://train/custom-missing.glb";
        ok(qn_train_edit(handle, revision, 0, 0, 0, current.dump().c_str(), &error));
        require(train(handle)["configuration"] == current, "Full physical/visual proposal roundtrips through C boundary");
        QnCommand command{}; command.type = QN_UNDO; ok(qn_command(handle, &command, &error));
        require(train(handle)["configuration"] == copied["configuration"], "Train Undo at native boundary");
        command.type = QN_REDO; ok(qn_command(handle, &command, &error));
        require(train(handle)["configuration"] == current, "Train Redo at native boundary");
        require(qn_train_edit(handle, snapshot(handle).revision, 99, id, 0, nullptr, &error) == QN_INVALID_ARGUMENT, "Unknown edit operation rejected");
        ok(qn_destroy(handle, &error)); handle = 0;
        require(qn_train_view(handle, revision, 1, id, 31, &error) == QN_INVALID_HANDLE, "Destroyed Train session rejected");
        std::cout << "Train boundary passed " << checks << " checks\n"; return 0;
    }
    catch (const std::exception& failure)
    { if (handle) qn_destroy(handle, &error); std::cerr << "Train boundary failed: " << failure.what() << '\n'; return 1; }
}
