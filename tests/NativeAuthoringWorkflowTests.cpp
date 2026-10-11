#include <quantum/editor/ApplicationService.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace coaster;
    using namespace editor;
    void require(bool condition, const char* message)
    { if (!condition) throw std::runtime_error(message); }
    void near(double value, double expected, double tolerance, const char* message)
    { require(std::abs(value - expected) <= tolerance, message); }
    AuthoredTrack mixed()
    {
        auto track = createNewDocument();
        track.setLayoutMode(LayoutMode::Shuttle);
        track.section(0) = createRateProfileSection(80);
        convertSectionToPlanarArc(track.section(0));
        setPlanarArcRadius(track.section(0), 1000);
        track.appendSection();
        track.section(1) = createForceDrivenSection(80);
        track.appendSection();
        track.section(2) = createRateProfileSection(160);
        return track;
    }
    void editorInitialization()
    {
        auto track = mixed();
        for (double scale : {0.5, 1.0, 2.0})
        {
            auto physical = track.physicalSettings(); physical.metersPerCoordinateUnit = scale;
            track.setPhysicalSettings(physical);
            for (int repeat = 0; repeat < 3; ++repeat)
                for (std::size_t i : {0u, 1u, 2u, 1u, 0u})
                {
                    const auto values = regionEditorValues(track.section(i), scale);
                    near(values.length, track.section(i).length, 0, "All region kinds seed length");
                    if (i == 0)
                    {
                        near(values.arcParameters[0], 1000, 0, "Arc-first radius is seeded without profiles");
                        near(values.arcParameters[1], .08 * degreesPerRadian, 1e-12, "Arc sweep uses degrees");
                        require(values.channelEnds == std::array<double, 3>{}, "Arc selection clears stale channel buffers");
                    }
                    if (i == 1)
                    {
                        near(values.channelEnds[static_cast<int>(ProfileChannel::Pitch)], 1, 0, "Force startup normal channel is G, not degrees");
                        require(values.arcParameters == std::array<double, 4>{}, "Force selection clears stale arc buffers");
                    }
                }
            auto& roll = sectionProfileChannel(track.section(2), ProfileChannel::Roll);
            roll.segments.back().transition.valueEnd = .001;
            const auto values = regionEditorValues(track.section(2), scale);
            near(values.channelEnds[static_cast<int>(ProfileChannel::Roll)],
                .001 * degreesPerRadian / scale, 1e-14, "Profile startup uses the document's physical scale");
        }
    }
    void conversionsPreserveAppearance()
    {
        auto section = createRateProfileSection(60);
        section.trackStyleOverrides.enabled = true;
        section.trackStyleOverrides.railRadius = .07;
        const auto check = [&]
        {
            require(section.trackStyleOverrides.enabled
                && section.trackStyleOverrides.railRadius == .07, "Region conversion must retain authored appearance");
            near(sectionLength(section), 60, 1e-12, "Region conversion retains length");
        };
        convertSectionToPlanarArc(section); check();
        convertSectionToRateProfiles(section); check();
        convertSectionToForceDriven(section); check();
        convertSectionToPlanarArc(section); check();
    }
    void authorToSimulation()
    {
        ApplicationService app(mixed());
        require(app.rebuildPreview(), "Mixed Arc-first track has a usable preview");
        const auto baseline = serializeCoasterDocument(app.track());
        require(app.centerline().visualization().sectionSlices.size() == 3, "All mixed regions publish geometry");
        const auto& slices = app.centerline().visualization().sectionSlices;
        for (std::size_t i = 1; i < slices.size(); ++i)
            require(glm::length(slices[i].startPosition - slices[i - 1].endPosition) < 1e-9, "Connected mixed-region endpoints remain continuous");

        auto id = sectionProfileChannel(app.track().section(2), ProfileChannel::Pitch).segments[0].id;
        app.editRegionProfile(2, ProfileChannel::Pitch, {id, ProfileEditOperation::Split, 40});
        app.editRegionProfile(2, ProfileChannel::Pitch, {id, ProfileEditOperation::EndValue, .01});
        app.editRegionProfile(2, ProfileChannel::Pitch, {id, ProfileEditOperation::TransitionType, 0, math::TransitionType::Smootherstep});
        id = sectionProfileChannel(app.track().section(1), ProfileChannel::Pitch).segments[0].id;
        app.editRegionProfile(1, ProfileChannel::Pitch, {id, ProfileEditOperation::EndValue, 1.01});
        app.setRegionLength(0, 90);
        require(app.preview().isAvailable(), "Profile, Force and Arc length edits rebuild accepted preview");
        auto train = app.track().trainConfiguration();
        train.cars[0].physical.car.dryMassKilograms += 123;
        train.cars[0].physical.loadout.massKilograms += 12;
        train.cars[0].visualAlignment.translationMeters.y = .125;
        train.cars[0].visualAlignment.rotationDegrees.z = 3;
        const auto carId = train.cars[0].id;
        const auto generation = app.centerline().generation();
        app.play(); app.update(physics::defaultFixedTimeStepSeconds * 1.5);
        app.setTrainConfiguration(train);
        require(app.centerline().generation() == generation, "Native train Apply reuses track geometry");
        require(app.preview().dynamicsState()->tick == 0 && app.preview().playbackState() == SimulationPreview::PlaybackState::Stopped,
            "Native train Apply resets the actual simulation");
        near(app.preview().trainDefinition().cars[0].car.dryMassKilograms,
            train.cars[0].physical.car.dryMassKilograms, 0, "Authored car mass reaches runtime physics");
        require(app.preview().meshInstances()[0].assetIdentifier == train.cars[0].visualAsset, "Authored visual assignment reaches preview");
        const auto valid = serializeCoasterDocument(app.track());
        const auto historySize = app.history().size();
        auto invalid = train; invalid.cars[0].physical.car.dryMassKilograms = -1;
        bool rejected = false;
        try { app.setTrainConfiguration(invalid); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected && app.history().size() == historySize && serializeCoasterDocument(app.track()) == valid,
            "Invalid native train draft preserves accepted document/history");
        app.restoreHistory(true); app.restoreHistory(false);
        require(serializeCoasterDocument(app.track()) == valid && app.track().trainConfiguration().cars[0].id == carId,
            "Undo/Redo restores the full car definition and stable IDs");

        const auto launch = app.createDevice(TrackDeviceKind::Launch);
        app.setDeviceProperties(launch, true, 0, 100, 1000000);
        app.setDeviceAcceleration(launch, false, 3);
        LaunchConfiguration config; config.targetSpeedMetersPerSecond = 30;
        app.setHardwareConfiguration(launch, config);
        const auto withLaunch = serializeCoasterDocument(app.track());
        app.play(); const double startSpeed = app.preview().speedMetersPerSecond();
        for (int tick = 0; tick < 30; ++tick) app.update(physics::defaultFixedTimeStepSeconds);
        require(app.preview().dynamicsState()->tick == 30, "Native preview retains deterministic 240 Hz stepping");
        require(app.preview().speedMetersPerSecond() > startSpeed, "Authored launch produces an observed physical speed increase");
        require(!app.preview().lastDeviceForces().devices.empty()
            && app.preview().lastDeviceForces().devices[0].appliedForceNewtons > 0, "Native operations report actual force");
        require(!app.preview().operations().hardware()[0].occupancy.clear(), "Native operations report real train occupancy");
        app.pause(); const auto tick = app.preview().dynamicsState()->tick;
        app.update(.05); require(app.preview().dynamicsState()->tick == tick, "Returning to editing/pause preserves committed physics state");
        app.preview().hardwareCommand(launch, physics::HardwareCommand::SetEnabled, 0);
        require(!app.preview().operations().hardware()[0].enabled && serializeCoasterDocument(app.track()) == withLaunch,
            "Runtime operation commands do not mutate authored data");
        app.reset(); require(app.preview().operations().hardware()[0].enabled, "Reset restores authored operational state");

        for (auto kind : {TrackDeviceKind::Brake, TrackDeviceKind::Transport, TrackDeviceKind::ChainLift, TrackDeviceKind::CableLift,
            TrackDeviceKind::Station, TrackDeviceKind::Block})
            static_cast<void>(app.createDevice(kind));
        const auto saved = serializeCoasterDocument(app.track());
        const auto path = std::filesystem::temp_directory_path() / ("quantum-native-workflow-"
            + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
        app.save(path);
        ApplicationService reopened; reopened.open(path);
        std::filesystem::remove(path);
        require(serializeCoasterDocument(reopened.track()) == saved && !reopened.history().isDirty(),
            "Real file save/reopen retains mixed regions, train visuals/physics and all supported devices");
        require(reopened.preview().isAvailable() && reopened.preview().operations().hardware().size() == 7,
            "Reopened authored operations and train are available to the simulator");
        reopened.play(); reopened.update(physics::defaultFixedTimeStepSeconds * 1.5); reopened.pause();
        require(serializeCoasterDocument(reopened.track()) == saved, "Simulation and return preserve the complete authored document");
        require(baseline != saved, "Workflow performed real document edits");
    }
}
int main(int argc, char** argv)
{
    try
    {
        if (argc == 3 && std::string_view(argv[1]) == "--write-fixture")
        { std::ofstream out(argv[2]); out << quantum::coaster::serializeCoasterDocument(mixed()); require(static_cast<bool>(out), "Fixture write failed"); return 0; }
        editorInitialization(); conversionsPreserveAppearance(); authorToSimulation();
        std::cout << "Native mixed-region authoring/train/operations workflow passed\n"; return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
