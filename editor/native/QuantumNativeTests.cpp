#include "QuantumNative.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
    QnError error{};
    void require(bool condition, std::string_view message)
    {
        if (!condition) throw std::runtime_error(std::string(message) + ": " + error.message);
    }
    void ok(QnResult result) { require(result == QN_OK, "Native operation failed"); }
    QnSnapshot snapshot(QnSession session, std::vector<QnRegion>& regions)
    {
        QnSnapshot out{sizeof(QnSnapshot), 1};
        const auto sizing = qn_snapshot(session, &out, nullptr, 0, 0, &error);
        require(sizing == QN_BUFFER_TOO_SMALL || sizing == QN_OK, "Snapshot sizing");
        regions.resize(out.region_count);
        ok(qn_snapshot(session, &out, regions.data(), static_cast<uint32_t>(regions.size()), 0, &error));
        return out;
    }
    void command(QnSession session, uint32_t type, uint64_t revision = 0, double value = 0, uint32_t index = 0)
    {
        QnCommand cmd{type, index, revision, value};
        ok(qn_command(session, &cmd, &error));
    }
}

int main(int argc, char** argv)
{
    try
    {
        require(argc >= 2 && qn_abi_version() == 1, "Test input/ABI");
        QnSession session = 0;
        require(qn_create(nullptr, &error) == QN_INVALID_ARGUMENT, "Missing output");
        ok(qn_create(&session, &error));
        std::vector<QnRegion> regions;
        auto state = snapshot(session, regions);
        require(!state.dirty && !state.can_undo && state.region_count > 0, "New baseline");
        const auto originalLength = regions[0].length;
        command(session, QN_SET_REGION_LENGTH, state.revision, originalLength + 2);
        state = snapshot(session, regions);
        require(state.dirty && state.can_undo && regions[0].length == originalLength + 2, "Accepted edit snapshot");
        command(session, QN_UNDO);
        state = snapshot(session, regions);
        require(!state.dirty && state.can_redo && regions[0].length == originalLength, "Undo baseline");
        command(session, QN_REDO);
        state = snapshot(session, regions);
        require(state.dirty && regions[0].length == originalLength + 2, "Redo edit");
        QnCommand invalid{QN_SET_REGION_LENGTH, 0, state.revision, -1};
        require(qn_command(session, &invalid, &error) == QN_INVALID_ARGUMENT, "Invalid length");
        invalid.value = 10; invalid.expected_revision = state.revision - 1;
        require(qn_command(session, &invalid, &error) == QN_CONFLICT, "Stale edit");
        invalid.expected_revision = state.revision; invalid.region_index = 999999;
        require(qn_command(session, &invalid, &error) == QN_INVALID_ARGUMENT, "Invalid Region");
        const auto afterFailure = snapshot(session, regions);
        require(afterFailure.revision == state.revision && afterFailure.dirty, "Rejected commands preserve state");
        QnRegion sentinel{}; sentinel.length = -123;
        auto telemetry = state;
        ok(qn_snapshot(session, &telemetry, &sentinel, 1, state.revision, &error));
        require(sentinel.length == -123, "Unchanged Route storage stays untouched");

        const auto directory = std::filesystem::temp_directory_path() /
            ("quantum-native-tests-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directory(directory);
        const auto output = directory / "roundtrip.quantum";
        const auto outputUtf8 = output.u8string();
        ok(qn_save(session, reinterpret_cast<const char*>(outputUtf8.c_str()), &error));
        state = snapshot(session, regions);
        require(!state.dirty && state.can_undo, "Save marks exact revision");
        command(session, QN_UNDO);
        require(snapshot(session, regions).dirty, "Undo away from saved revision is dirty");
        command(session, QN_REDO);
        require(!snapshot(session, regions).dirty, "Redo to saved revision is clean");
        ok(qn_new(session, &error));
        require(!snapshot(session, regions).can_undo, "New resets history");
        ok(qn_open(session, reinterpret_cast<const char*>(outputUtf8.c_str()), &error));
        state = snapshot(session, regions);
        require(!state.dirty && !state.can_undo && regions[0].length == originalLength + 2, "Serialized edit roundtrip");
        const auto invalidPath = directory / "invalid.quantum";
        { std::ofstream file(invalidPath); file << "invalid document"; }
        const auto invalidUtf8 = invalidPath.u8string();
        require(qn_open(session, reinterpret_cast<const char*>(invalidUtf8.c_str()), &error) != QN_OK, "Failed file load");
        require(snapshot(session, regions).revision == state.revision, "Failed load preserves document");
        require(qn_open(session, "missing.quantum", &error) != QN_OK, "Missing file error");

        ok(qn_open(session, argv[1], &error));
        state = snapshot(session, regions);
        require(state.available && state.car_count > 0, "Real train preview");
        QnAuthoringSnapshot authoring{sizeof(QnAuthoringSnapshot), 1};
        require(qn_authoring_snapshot(session, &authoring, nullptr, 0, nullptr, 0, 0, &error) == QN_BUFFER_TOO_SMALL,
            "Authoring snapshot sizing");
        std::vector<QnProfileSegment> profiles(authoring.profile_count);
        std::vector<QnDevice> devices(authoring.device_count);
        const auto refreshProfiles = [&] {
            authoring = {sizeof(QnAuthoringSnapshot), 1};
            const auto result = qn_authoring_snapshot(session, &authoring, profiles.data(), static_cast<uint32_t>(profiles.size()),
                devices.data(), static_cast<uint32_t>(devices.size()), 0, &error);
            if (result == QN_BUFFER_TOO_SMALL)
            {
                profiles.resize(authoring.profile_count); devices.resize(authoring.device_count);
                ok(qn_authoring_snapshot(session, &authoring, profiles.data(), static_cast<uint32_t>(profiles.size()),
                    devices.data(), static_cast<uint32_t>(devices.size()), 0, &error));
            }
            else ok(result);
        };
        refreshProfiles();
        require(authoring.profile_count > 0 && profiles[0].samples[0] == profiles[0].value_begin
            && profiles[0].samples[32] == profiles[0].value_end, "Native analytic samples preserve endpoints");
        uint32_t typeCount = 0;
        require(qn_transition_types(nullptr, 0, &typeCount, &error) == QN_BUFFER_TOO_SMALL && typeCount == 19,
            "Existing transition catalog");
        std::vector<QnTransitionType> types(typeCount);
        ok(qn_transition_types(types.data(), typeCount, &typeCount, &error));
        QnProfileCommand profileEdit{0, profiles[0].channel, profiles[0].segment_id, 2, 0, 0,
            profiles[0].owner_id, authoring.revision, 0};
        for (const auto& type : types)
        {
            profileEdit.expected_revision = snapshot(session, regions).revision; profileEdit.transition_type = type.type;
            ok(qn_profile_command(session, &profileEdit, &error));
            refreshProfiles(); require(profiles[0].transition_type == type.type, "Transition type roundtrip");
            if (snapshot(session, regions).can_undo) command(session, QN_UNDO);
        }
        refreshProfiles();
        profileEdit.expected_revision = authoring.revision; profileEdit.transition_type = 999;
        require(qn_profile_command(session, &profileEdit, &error) == QN_INVALID_ARGUMENT, "Unknown transition rejected");
        profileEdit.operation = 5; profileEdit.value = 1;
        require(qn_profile_command(session, &profileEdit, &error) == QN_INVALID_ARGUMENT, "Outer domain boundary stays pinned");
        profileEdit.operation = 3; profileEdit.value = (profiles[0].begin_meters + profiles[0].end_meters) / 2;
        ok(qn_profile_command(session, &profileEdit, &error));
        const auto previousCount = authoring.profile_count; refreshProfiles();
        require(authoring.profile_count == previousCount + 1, "Profile split uses native segment IDs");
        require(qn_profile_command(session, &profileEdit, &error) == QN_CONFLICT, "Stale profile edit rejected");
        command(session, QN_UNDO);
        command(session, QN_PLAY);
        ok(qn_update(session, 1.0 / 60, &error));
        state = snapshot(session, regions);
        require(state.tick > 0 && state.playback == 1, "Native fixed-step playback");
        command(session, QN_PAUSE);
        const auto pausedTick = state.tick;
        ok(qn_update(session, 1.0 / 60, &error));
        require(snapshot(session, regions).tick == pausedTick, "Pause preserves tick");
        command(session, QN_RESET);
        state = snapshot(session, regions);
        require(state.tick == 0 && state.playback == 0, "Reset restores initial train");
        require(qn_update(session, std::nan(""), &error) == QN_INVALID_ARGUMENT, "NaN delta rejected");
        QnSnapshot bad{0, 1};
        require(qn_snapshot(session, &bad, nullptr, 0, 0, &error) == QN_INVALID_ARGUMENT, "Struct size validation");

        if (argc > 2 && std::string_view(argv[2]) == "--vulkan")
        {
            for (int cycle = 0; cycle < 3; ++cycle)
            {
                ok(qn_viewport_open(session, 0, 0, &error));
                require(qn_destroy(session, &error) == QN_BUSY, "Shutdown ordering");
                QnResult wrongThread = QN_OK;
                std::thread worker([&] { QnError local; wrongThread = qn_viewport_close(session, &local); });
                worker.join();
                require(wrongThread == QN_INVALID_ARGUMENT, "Viewport thread affinity");
                ok(qn_viewport_frame(session, 0, 0, &error));
                require(qn_viewport_presentation(session, 4, &error) == QN_INVALID_ARGUMENT, "Invalid display mode");
                for (int frame = 0; frame < 15; ++frame)
                {
                    ok(qn_viewport_frame(session, 800 + (frame % 3) * 80, 500 + (frame % 4) * 30, &error));
                    ok(qn_viewport_input(session, 1, .01, .005, 0, 0, &error));
                    ok(qn_viewport_input(session, 2, 2, 2, 0, 0, &error));
                    ok(qn_viewport_input(session, 3, .1, 0, 0, 0, &error));
                    ok(qn_viewport_presentation(session, static_cast<uint32_t>(frame % 4), &error));
                }
                ok(qn_viewport_close(session, &error));
                ok(qn_viewport_close(session, &error));
            }
        }
        const auto forceFixture = (std::filesystem::path(argv[1]).parent_path() / "geometry-force-driven-profiles.quantum").u8string();
        ok(qn_open(session, reinterpret_cast<const char*>(forceFixture.c_str()), &error));
        refreshProfiles();
        bool normalTarget = false, lateralTarget = false;
        for (uint32_t i = 0; i < authoring.profile_count; ++i)
        {
            const auto& profile = profiles[i];
            normalTarget |= profile.unit == 1 && profile.channel == 1;
            lateralTarget |= profile.unit == 1 && profile.channel == 2;
        }
        require(normalTarget && lateralTarget, "Force-driven channels remain native G targets");
        ok(qn_destroy(session, &error));
        require(qn_destroy(session, &error) == QN_INVALID_HANDLE, "Double destroy");
        require(qn_new(session, &error) == QN_INVALID_HANDLE, "Stale handle");
        require(qn_new(UINT64_MAX, &error) == QN_INVALID_HANDLE, "Fabricated handle");
        for (int cycle = 0; cycle < 10; ++cycle)
        {
            QnSession next; ok(qn_create(&next, &error)); require(next > session, "Tokens never reused");
            ok(qn_destroy(next, &error));
        }
        std::filesystem::remove_all(directory);
        std::cout << "Native application boundary checks passed.\n";
        return 0;
    }
    catch (const std::exception& failure) { std::cerr << failure.what() << '\n'; return 1; }
}
