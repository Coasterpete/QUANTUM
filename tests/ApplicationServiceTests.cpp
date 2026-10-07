#include <quantum/editor/ApplicationService.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }
}
int main(int argc, char** argv)
{
    try
    {
        quantum::editor::ApplicationService service;
        const auto original = quantum::coaster::serializeCoasterDocument(service.track());
        const auto revision = service.revision();
        try
        {
            service.setRegionLength(0, 62, [](const auto&) { throw std::runtime_error("Rejected GPU publication"); });
            throw std::logic_error("Publication failure was ignored");
        }
        catch (const std::runtime_error&) { }
        require(service.revision() == revision && !service.history().canUndo()
            && quantum::coaster::serializeCoasterDocument(service.track()) == original,
            "Rejected publication must preserve document/history");
        service.setRegionLength(0, 62);
        require(service.history().isDirty(), "Accepted edit uses native history");
        service.restoreHistoryWithPublication(true, [&](const auto& track, const auto)
        {
            auto prepared = quantum::editor::ApplicationService::prepare(track);
            service.track() = track;
            service.centerline().replace(std::move(prepared.centerline));
            service.supports() = std::move(prepared.supports);
        });
        require(!service.history().isDirty()
            && quantum::coaster::serializeCoasterDocument(service.track()) == original,
            "Frontend-specific publication retains shared history semantics");
        try
        {
            service.restoreHistoryWithPublication(false, [](const auto&, auto) { throw std::runtime_error("Rejected history publication"); });
        }
        catch (const std::runtime_error&) { }
        require(service.history().canRedo() && !service.history().isDirty(), "Failed restoration preserves real cursor");
        service.restoreHistory(false);
        require(service.track().section(0).length == 62, "Both restore paths agree");

        auto scaledTrack = quantum::coaster::createNewDocument();
        auto physical = scaledTrack.physicalSettings(); physical.metersPerCoordinateUnit = 2;
        scaledTrack.setPhysicalSettings(physical);
        quantum::editor::ApplicationService scaled(std::move(scaledTrack));
        scaled.editRegionProfile(0, quantum::editor::ProfileChannel::Roll,
            {1, quantum::editor::ProfileEditOperation::BeginValue, 1});
        require(std::abs(scaled.track().section(0).rateProfileRegion().rateProfiles.roll.segments[0].transition.valueBegin
            - quantum::editor::radiansPerDegree * 2) < 1e-15, "Native coordinate-scale conversion is preserved");
        scaled.restoreHistory(true);
        require(!scaled.history().isDirty(), "Profile edit shares document history");

        auto deviceTrack = quantum::coaster::createNewDocument();
        quantum::coaster::TrackDevice device; device.name = "Acceleration regression fixture";
        device.startStationMeters = 0; device.endStationMeters = 20;
        const auto deviceId = deviceTrack.addTrackDevice(device);
        quantum::editor::ApplicationService deviceService(std::move(deviceTrack));
        const auto deviceBaseline = quantum::coaster::serializeCoasterDocument(deviceService.track());
        deviceService.setDeviceAcceleration(deviceId, true, 2);
        deviceService.editDeviceProfile(deviceId, {1, quantum::editor::ProfileEditOperation::EndValue, 3});
        deviceService.editDeviceProfile(deviceId, {1, quantum::editor::ProfileEditOperation::Split, 10});
        deviceService.editDeviceProfile(deviceId, {1, quantum::editor::ProfileEditOperation::MoveEnd, 5});
        const auto deviceBeforeFailure = quantum::coaster::serializeCoasterDocument(deviceService.track());
        try { deviceService.editDeviceProfile(deviceId, {1, quantum::editor::ProfileEditOperation::BeginValue, -1}); }
        catch (const std::invalid_argument&) { }
        require(quantum::coaster::serializeCoasterDocument(deviceService.track()) == deviceBeforeFailure,
            "Invalid acceleration preserves native state");
        for (int edit = 0; edit < 4; ++edit) deviceService.restoreHistory(true);
        require(quantum::coaster::serializeCoasterDocument(deviceService.track()) == deviceBaseline,
            "Constant/custom device edits share exact history baseline");

        auto large = quantum::coaster::createNewDocument();
        for (int i = 0; i < 199; ++i) large.appendSection();
        const auto begin = std::chrono::steady_clock::now();
        quantum::editor::ApplicationService largeService(std::move(large));
        const auto milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        require(largeService.track().sectionCount() == 200
            && largeService.centerline().visualization().sectionSlices.size() == 200,
            "Large authored Route remains native");
        if (argc == 2) largeService.save(argv[1]);
        largeService.setRegionLength(199, 61);
        largeService.restoreHistory(true);
        require(largeService.track().section(199).length == 60, "Large Route edit/history roundtrip");
        std::cout << "Application service checks passed; 200-Region preparation " << milliseconds << " ms.\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
