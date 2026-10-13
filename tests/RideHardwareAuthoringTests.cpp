#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/TrackDevicePresentation.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <iostream>
#include <stdexcept>
namespace
{
    using namespace quantum;
    using namespace quantum::coaster;
    using namespace quantum::editor;
    int checks = 0;
    void require(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
    template<class F> void rejects(F f) { try { f(); } catch (const std::invalid_argument&) { ++checks; return; } throw std::runtime_error("Invalid authoring accepted"); }
    std::string document(ApplicationService& app) { return serializeCoasterDocument(app.track()); }
    void history(ApplicationService& app, const std::string& before)
    {
        const auto after = document(app);
        app.restoreHistory(true); require(document(app) == before, "Authored Undo exact equality");
        app.restoreHistory(false); require(document(app) == after, "Authored Redo exact equality");
        auto reload = deserializeCoasterDocument(after); require(reload && serializeCoasterDocument(*reload) == after, "Hardware save/reload exact equality");
    }
    void authoring()
    {
        ApplicationService app;
        TrackDeviceId block = 0, gate = 0;
        for (auto kind : {TrackDeviceKind::Launch, TrackDeviceKind::Brake, TrackDeviceKind::Transport,
            TrackDeviceKind::ChainLift, TrackDeviceKind::CableLift, TrackDeviceKind::Station,
            TrackDeviceKind::Block, TrackDeviceKind::Transfer, TrackDeviceKind::Access})
        {
            auto before = document(app); const auto id = app.createDevice(kind); history(app, before);
            require(app.track().trackDevices().devices.back().id == id, "Stable ID and kind survive history");
            if (kind == TrackDeviceKind::Block) block = id;
            if (kind == TrackDeviceKind::Access) gate = id;
            before = document(app);
            app.setDeviceProperties(id, false, 1, 40, 12000); history(app, before);
            before = document(app);
            auto config = app.track().trackDevices().devices.back().configuration;
            std::visit([](auto& c)
            {
                using T = std::decay_t<decltype(c)>;
                if constexpr (std::is_same_v<T, LaunchConfiguration>) c.targetSpeedMetersPerSecond = 10;
                else if constexpr (std::is_same_v<T, BrakeConfiguration>) c.holding = true;
                else if constexpr (std::is_same_v<T, TransportConfiguration>) { c.speedMetersPerSecond = 1; c.direction = HardwareDirection::Reverse; }
                else if constexpr (std::is_same_v<T, ChainLiftConfiguration>) c.speedMetersPerSecond = 4;
                else if constexpr (std::is_same_v<T, CableLiftConfiguration>) c.returnSeconds = 3;
                else if constexpr (std::is_same_v<T, StationConfiguration>) c.stopStationMeters = 20;
                else if constexpr (std::is_same_v<T, TransferConfiguration>) { c.alignments.push_back({3, "Maintenance", -3, DestinationKind::Maintenance}); c.nextAlignmentId = 4; }
                else if constexpr (std::is_same_v<T, AccessConfiguration>) { c.motion = AccessMotion::Slide; c.meshAsset = "assets/gates/test.glb"; }
            }, config);
            app.setHardwareConfiguration(id, config);
            if (document(app) != before) history(app, before);
            if (supportsAccelerationProfile(kind)) { before = document(app); app.setDeviceAcceleration(id, true, 2); history(app, before); }
            else rejects([&] { app.setDeviceAcceleration(id, true, 2); });
        }
        auto station = app.track().trackDevices().devices[5];
        auto configuration = std::get<StationConfiguration>(station.configuration);
        configuration.downstreamBlock = block; configuration.accessGates = {gate};
        auto before = document(app); app.setHardwareConfiguration(station.id, configuration); history(app, before);
        auto invalid = configuration; invalid.accessGates = {block};
        before = document(app); rejects([&] { app.setHardwareConfiguration(station.id, invalid); });
        require(document(app) == before, "Invalid station reference preserves accepted state");
        rejects([&] { app.deleteDevice(gate); }); require(document(app) == before, "Referenced gate cannot be deleted silently");
        auto transfer = app.track().trackDevices().devices[7]; auto destinations = std::get<TransferConfiguration>(transfer.configuration); destinations.accessGates = {gate};
        app.setHardwareConfiguration(transfer.id, destinations);
        destinations.alignments.pop_back();
        before = document(app); app.setHardwareConfiguration(transfer.id, destinations); history(app, before);
        auto reused = destinations; reused.alignments.push_back({3, "Reused", -3, DestinationKind::Maintenance});
        before = document(app); rejects([&] { app.setHardwareConfiguration(transfer.id, reused); });
        require(document(app) == before, "Deleted alignment identity cannot be reassigned");
        reused = destinations; reused.nextAlignmentId = 3;
        rejects([&] { app.setHardwareConfiguration(transfer.id, reused); });
        destinations.alignments.push_back({destinations.nextAlignmentId++, "New maintenance", -3, DestinationKind::Maintenance});
        app.setHardwareConfiguration(transfer.id, destinations); history(app, before);
        TrackInfrastructure catwalk; catwalk.id = 1; catwalk.name = "Lift catwalk"; catwalk.startStationMeters = 0; catwalk.endStationMeters = 30;
        catwalk.accessGates = {gate};
        before = document(app); app.setInfrastructure({catwalk}, 2); history(app, before);
        for (unsigned kind = 1; kind <= 4; ++kind)
        {
            auto item = catwalk; item.id = kind + 1; item.kind = static_cast<InfrastructureKind>(kind); item.name = "Access infrastructure";
            auto items = app.track().trackDevices().infrastructure; items.push_back(item);
            before = document(app); app.setInfrastructure(items, kind + 2); history(app, before);
        }
        auto infrastructure = app.track().trackDevices().infrastructure;
        infrastructure.pop_back();
        before = document(app); app.setInfrastructure(infrastructure, 6); history(app, before);
        before = document(app); rejects([&] { app.setInfrastructure(infrastructure, 5); });
        auto reusedInfrastructure = catwalk; reusedInfrastructure.id = 5; infrastructure.push_back(reusedInfrastructure);
        rejects([&] { app.setInfrastructure(infrastructure, 6); });
        require(document(app) == before, "Deleted infrastructure identity cannot be reassigned");
        infrastructure.back().id = 6; app.setInfrastructure(infrastructure, 7); history(app, before);
        before = document(app); const auto entries = app.history().size(), revision = app.revision();
        app.preview().hardwareCommand(gate, physics::HardwareCommand::SetEnabled, 1);
        app.preview().hardwareCommand(gate, physics::HardwareCommand::UnlockAccess);
        app.preview().hardwareCommand(gate, physics::HardwareCommand::OpenAccess);
        require(app.history().size() == entries && app.revision() == revision && document(app) == before, "Runtime commands never enter authored history");
        app.reset();
        const auto& runtime = app.preview().operations().hardware().back();
        require(!runtime.enabled && std::get<physics::AccessRuntime>(runtime.mechanism).locked, "Preview reset restores authored gate state");
        before = document(app); app.deleteDevice(app.track().trackDevices().devices[0].id); history(app, before);
    }
    void presentation()
    {
        auto track = createNewDocument(); auto physical = track.physicalSettings(); physical.metersPerCoordinateUnit = 2; track.setPhysicalSettings(physical);
        auto gate = makeTrackDevice(TrackDeviceKind::Access); gate.startStationMeters = 10; gate.endStationMeters = 11;
        auto id = track.addTrackDevice(gate); auto geometry = createCenterlineVisualization(track);
        physics::HardwareRuntime h; h.id = id; h.mechanism = physics::AccessRuntime{};
        const auto closed = createHardwareMechanismVertices(track, geometry, id, &h);
        std::get<physics::AccessRuntime>(h.mechanism).fraction = 1;
        const auto open = createHardwareMechanismVertices(track, geometry, id, &h);
        require(closed.size() == 10 && open.size() == closed.size() && closed[1].x != open[1].x, "Hinged gate geometry follows real runtime position");
        auto authored = track.trackDevices().devices.front(); std::get<AccessConfiguration>(authored.configuration).motion = AccessMotion::Slide;
        track.updateTrackDevice(authored);
        const auto slide = createHardwareMechanismVertices(track, geometry, id, &h);
        require(slide[0].y != closed[0].y, "Sliding gate moves rather than swings");
        auto transfer = makeTrackDevice(TrackDeviceKind::Transfer); transfer.startStationMeters = 20; transfer.endStationMeters = 30;
        id = track.addTrackDevice(transfer); h.id = id; h.mechanism = physics::TransferRuntime{};
        const auto home = createHardwareMechanismVertices(track, geometry, id, &h);
        std::get<physics::TransferRuntime>(h.mechanism).positionMeters = 4;
        const auto storage = createHardwareMechanismVertices(track, geometry, id, &h);
        require(home.size() == storage.size() && home.back().y != storage.back().y, "Transfer engineering outline follows native lateral travel");
    }
}
int main()
{
    try { authoring(); presentation(); std::cout << "RideHardwareAuthoringTests passed " << checks << " checks\n"; return 0; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
