#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/TrackDevicePresentation.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace quantum::editor;
    int checks = 0;
    void require(bool condition, const char* message)
    {
        ++checks;
        if (!condition) throw std::runtime_error(message);
    }
    template<class F> void rejects(F action)
    {
        bool rejected = false;
        try { action(); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Invalid candidate must be rejected");
    }
    std::string document(ApplicationService& app) { return coaster::serializeCoasterDocument(app.track()); }
    void roundtripHistory(ApplicationService& app, const std::string& before, const std::string& after)
    {
        app.restoreHistory(true); require(document(app) == before, "Undo exact authored state");
        app.restoreHistory(false); require(document(app) == after, "Redo exact authored state");
    }
    void regionGestures()
    {
        ApplicationService app;
        for (auto channel : {ProfileChannel::Roll, ProfileChannel::Pitch, ProfileChannel::Yaw})
        {
            const ProfileTarget target{ProfileOwner::Region, 0, channel};
            const auto before = document(app); const auto size = app.history().size(); const auto revision = app.revision();
            const auto token = app.beginProfileDrag(target, {1, ProfileEditOperation::EndValue});
            const auto start = std::chrono::steady_clock::now();
            for (int move = 1; move <= 100; ++move)
            {
                const auto& preview = app.previewProfileDrag(token, move * .001);
                require(std::abs(preview.segments[0].transition.valueEnd - move * .001 * radiansPerDegree) < 1e-15,
                    "Angular preview uses existing deg/m conversion");
            }
            const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            require(app.revision() == revision && app.history().size() == size && document(app) == before,
                "One hundred previews preserve document and history");
            app.commitProfileDrag(token);
            require(app.history().size() == size + 1, "Entire value drag records one history entry");
            const auto after = document(app); roundtripHistory(app, before, after);
            std::cout << "Native channel candidate (100 previews): " << elapsed << " ms\n";
            const auto canceled = app.beginProfileDrag(target, {1, ProfileEditOperation::BeginValue});
            static_cast<void>(app.previewProfileDrag(canceled, .2)); app.cancelProfileDrag(canceled);
            require(document(app) == after, "Escape/capture-loss cancellation leaves accepted document intact");
            const auto noMove = app.beginProfileDrag(target, {1, ProfileEditOperation::EndValue});
            const auto noMoveRevision = app.revision(); app.commitProfileDrag(noMove);
            require(app.revision() == noMoveRevision, "Click with no drag creates no history");
            rejects([&] { static_cast<void>(app.beginProfileDrag(target, {1, ProfileEditOperation::MoveBegin})); });
            rejects([&] { static_cast<void>(app.beginProfileDrag(target, {1, ProfileEditOperation::MoveEnd})); });
            app.editRegionProfile(0, channel, {1, ProfileEditOperation::Split, 30});
            const auto split = document(app); const auto splitCount = app.history().size();
            const auto boundary = app.beginProfileDrag(target, {1, ProfileEditOperation::MoveEnd});
            const auto& preview = app.previewProfileDrag(boundary, 24.7, .5);
            require(preview.segments[0].transition.domainEnd == 24.5 && preview.segments[1].transition.domainBegin == 24.5,
                "Native optional distance snapping and shared boundaries");
            const auto ids = std::pair{preview.segments[0].id, preview.segments[1].id};
            app.commitProfileDrag(boundary);
            require(app.history().size() == splitCount + 1, "Boundary drag is one history entry");
            const auto moved = document(app); roundtripHistory(app, split, moved);
            const auto clamp = app.beginProfileDrag(target, {1, ProfileEditOperation::MoveEnd});
            const auto& clamped = app.previewProfileDrag(clamp, 10000);
            require(clamped.segments[0].transition.domainEnd < 60 && clamped.segments[0].id == ids.first
                && clamped.segments[1].id == ids.second, "Native boundary clamping prevents inversion and retains IDs");
            app.cancelProfileDrag(clamp);
            const auto invalid = app.beginProfileDrag(target, {1, ProfileEditOperation::EndValue});
            rejects([&] { static_cast<void>(app.previewProfileDrag(invalid, std::nan(""))); });
            require(document(app) == moved && app.history().size() == splitCount + 1, "Invalid preview preserves committed state/history");
            const auto stale = app.beginProfileDrag(target, {1, ProfileEditOperation::EndValue});
            app.setInitialSpeed(app.track().physicalSettings().initialSpeed + .1);
            const auto external = document(app);
            rejects([&] { static_cast<void>(app.previewProfileDrag(stale, 0)); });
            require(document(app) == external, "Stale drag preserves newer edit");
            app.editRegionProfile(0, channel, {1, ProfileEditOperation::TransitionType, 0, math::TransitionType::Linear});
            const auto beforeRemove = document(app);
            app.editRegionProfile(0, channel, {ids.second, ProfileEditOperation::Remove});
            roundtripHistory(app, beforeRemove, document(app));
        }
        const ProfileTarget target{ProfileOwner::Region, 0, ProfileChannel::Roll};
        const auto before = document(app); const auto count = app.history().size();
        const auto failed = app.beginProfileDrag(target, {1, ProfileEditOperation::EndValue});
        static_cast<void>(app.previewProfileDrag(failed, .25));
        try { app.commitProfileDrag(failed, [](const auto&) { throw std::runtime_error("GPU publication failure"); }); }
        catch (const std::runtime_error&) { }
        require(document(app) == before && app.history().size() == count, "Failed full publication discards drag without a history entry");
    }
    void devices()
    {
        auto track = coaster::createNewDocument(); track.setLayoutMode(coaster::LayoutMode::Shuttle);
        ApplicationService app(std::move(track));
        const auto before = document(app);
        const auto id = app.createDevice(coaster::TrackDeviceKind::Launch);
        const auto created = document(app); roundtripHistory(app, before, created);
        require(app.track().trackDevices().devices[0].id == id, "Creation uses native stable ID");
        app.setDeviceProperties(id, false, 3, 23, 8000);
        const auto placed = document(app); roundtripHistory(app, created, placed);
        const auto& device = app.track().trackDevices().devices[0];
        require(!device.enabled && device.startStationMeters == 3 && device.endStationMeters == 23 && device.maximumForceNewtons == 8000,
            "Placement/enable/force properties use native data");
        rejects([&] { app.setDeviceProperties(id, true, 23, 3, 8000); });
        require(document(app) == placed, "Invalid interval preserved");
        app.setDeviceAcceleration(id, true, 2);
        const auto custom = document(app); roundtripHistory(app, placed, custom);
        app.editDeviceProfile(id, {1, ProfileEditOperation::Split, 10});
        const auto split = document(app);
        const auto token = app.beginProfileDrag({ProfileOwner::Device, id}, {1, ProfileEditOperation::EndValue});
        const auto history = app.history().size();
        for (int move = 1; move <= 50; ++move)
        {
            const auto& profile = app.previewProfileDrag(token, 2 + move * .01);
            require(profile.segments[0].transition.valueEnd == profile.segments[1].transition.valueBegin, "Custom preview remains C0 continuous");
        }
        app.commitProfileDrag(token);
        require(app.history().size() == history + 1, "Custom acceleration drag is one history entry");
        const auto edited = document(app); roundtripHistory(app, split, edited);
        const auto invalid = app.beginProfileDrag({ProfileOwner::Device, id}, {1, ProfileEditOperation::BeginValue});
        rejects([&] { static_cast<void>(app.previewProfileDrag(invalid, -1)); });
        require(document(app) == edited, "Negative custom candidate preserves document");
        app.setDeviceProperties(id, true, 3, 28, 8000);
        const auto& resized = *app.track().trackDevices().devices[0].accelerationProfile;
        require(resized.segments[0].transition.domainEnd == 12.5 && resized.segments[1].transition.domainEnd == 25,
            "Interval resize preserves normalized profile timing");
        const auto resizedState = document(app);
        app.setDeviceProperties(id, true, 3, 8, 8000);
        const auto& shrunk = *app.track().trackDevices().devices[0].accelerationProfile;
        require(shrunk.segments[0].transition.domainEnd == 2.5 && shrunk.segments[1].transition.domainEnd == 5,
            "Shrinking before an old split retimes both segments without inversion");
        const auto shrunkState = document(app);
        roundtripHistory(app, resizedState, shrunkState);
        rejects([&] { app.setDeviceProperties(id, true, 3, 3, 8000); });
        require(document(app) == shrunkState, "Empty custom extent remains rejected atomically");
        app.restoreHistory(true);
        require(document(app) == resizedState, "Undo shrink restores the exact larger profile");
        const auto boundary = app.beginProfileDrag({ProfileOwner::Device, id}, {1, ProfileEditOperation::MoveEnd});
        static_cast<void>(app.previewProfileDrag(boundary, 7)); app.commitProfileDrag(boundary);
        roundtripHistory(app, resizedState, document(app));
        app.editRegionProfile(0, ProfileChannel::Roll, {1, ProfileEditOperation::BeginValue, .1});
        const auto complete = document(app);
        const auto path = std::filesystem::temp_directory_path() / ("quantum-authoring-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
        app.save(path); app.newDocument(); app.open(path); std::filesystem::remove(path);
        require(document(app) == complete && !app.history().isDirty(), "Device + Region .quantum Save/New/Reload exact authored state");
        require(app.preview().isAvailable(), "Reloaded authored device has a native simulation preview");
        app.play(); app.update(.05);
        const auto& forces = app.preview().lastDeviceForces().devices;
        require(!forces.empty() && forces[0].id == id && forces[0].occupiedBogieCount > 0 && forces[0].appliedForceNewtons > 0
            && forces[0].commandedAccelerationMetersPerSecondSquared > 2,
            "Train bogies consume the authored Custom profile rather than its Constant seed");
        app.pause(); app.reset();
        app.setDeviceAcceleration(id, false, 2);
        const auto constant = document(app); roundtripHistory(app, complete, constant);
        require(!app.track().trackDevices().devices[0].accelerationProfile, "Constant mode removes custom data natively");
        app.deleteDevice(id); roundtripHistory(app, constant, document(app));
        require(app.track().trackDevices().devices.empty(), "Delete stays deleted after Redo");
        const auto brake = app.createDevice(coaster::TrackDeviceKind::Brake);
        require(brake > id && app.track().trackDevices().devices[0].kind == coaster::TrackDeviceKind::Brake, "Deleted IDs not reused, Brake creation retained");
    }
    void presentation()
    {
        auto track = coaster::createNewDocument();
        auto physical = track.physicalSettings(); physical.metersPerCoordinateUnit = 2; track.setPhysicalSettings(physical);
        track.setLayoutMode(coaster::LayoutMode::Circuit);
        coaster::TrackDevice d; d.name = "Wrapped highlight"; d.startStationMeters = 110; d.endStationMeters = 10;
        const auto id = track.addTrackDevice(d);
        auto geometry = createCenterlineVisualization(track);
        const auto lines = createTrackDeviceSelectionVertices(track, geometry, id);
        require(!lines.empty() && lines.size() % 2 == 0, "Native selection presentation line list");
        bool start = false, end = false, acrossGap = false;
        for (const auto& v : lines)
        {
            start |= std::abs(v.x - 55) < 1e-5; end |= std::abs(v.x - 5) < 1e-5;
            acrossGap |= v.x > 6 && v.x < 54;
        }
        require(start && end && !acrossGap, "Wrapped highlight clips both intervals in native coordinate units");
        require(createTrackDeviceSelectionVertices(track, geometry, 0).empty(), "Cleared selection has no viewport lines");
    }
}
int main()
{
    try { regionGestures(); devices(); presentation(); std::cout << "Timeline authoring service: " << checks << " checks passed.\n"; return 0; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
