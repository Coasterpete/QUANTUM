#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <nlohmann/json.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace quantum::coaster;
    using namespace quantum::physics;
    using namespace quantum::editor;
    int checks = 0;
    void require(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
    void near(double a, double b, double tolerance, const char* message) { require(std::abs(a - b) <= tolerance, message); }
    template<class F> void rejects(F operation)
    { try { operation(); } catch (const std::invalid_argument&) { ++checks; return; } throw std::runtime_error("Invalid Train proposal accepted"); }
    TrainConfiguration heterogeneous(bool zero = false)
    {
        auto c = createDefaultOrderedTrainConfiguration();
        c.trainId = 73; c.name = "Train 2 engineering acceptance";
        const std::array roles{TrainCarRole::Lead, TrainCarRole::Middle, TrainCarRole::Trailer, TrainCarRole::Rear};
        for (std::size_t i = 0; i < c.cars.size(); ++i)
        {
            auto car = createTrainCarPrototype(roles[i]); car.id = c.cars[i].id;
            auto& p = car.physical.car;
            p.dryMassKilograms = 650 + i * 175;
            p.bodyDimensionsMeters = {3.2 + i * .25, 1.3 + i * .05, 1.2 + i * .1};
            p.dryCenterOfGravityMeters = {i * .02, i * .01, .45 + i * .05};
            p.dryInertiaTensorBodyKgM2 = makeUniformBoxInertiaTensorBodyKgM2(p.dryMassKilograms, p.bodyDimensionsMeters);
            p.frontHitchPositionMeters = {p.bodyDimensionsMeters.x * .5, 0, .2};
            p.rearHitchPositionMeters = {-p.bodyDimensionsMeters.x * .5, 0, .2};
            p.bogies[0].referencePositionMeters = {p.bodyDimensionsMeters.x * .3, 0, -.1 * i};
            p.bogies[1].referencePositionMeters = {-p.bodyDimensionsMeters.x * .3, 0, -.1 * i};
            car.physical.loadout.massKilograms = i == 2 ? 0 : 120 + i * 30;
            car.physical.loadout.centerOfMassMeters = {0, 0, .85 + i * .02};
            // Explicit, shared alignment of temporary art; no per-file hacks.
            car.visualAlignment.translationMeters.z = .7;
            c.cars[i] = std::move(car);
        }
        if (zero)
        {
            c.cars[2].role = TrainCarRole::Middle;
            c.cars[2].name = "Middle";
            static_cast<void>(insertTrainCar(c, 0, createTrainCarPrototype(TrainCarRole::Zero)));
        }
        c.resistance = {0, 0, 1.225, 0, 0};
        validateTrainConfiguration(c);
        return c;
    }
    AuthoredTrack trackWith(const TrainConfiguration& train)
    {
        auto track = createNewDocument(); track.setLayoutMode(LayoutMode::Shuttle);
        setSectionLength(track.section(0), 300);
        track.setTrainConfiguration(train);
        return track;
    }
    TrackDevice device(TrackDeviceKind kind, TrackDeviceId id, double start, double end)
    {
        auto d = makeTrackDevice(kind); d.id = id; d.startStationMeters = start; d.endStationMeters = end;
        d.targetAccelerationMetersPerSecondSquared = 4; d.maximumForceNewtons = 100000;
        return d;
    }
    CompiledPhysicsTrack rail()
    {
        const geometry::CurveFrame frame{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        return {std::vector<TrackKinematicState>{{0, {0, 0, 0}, frame, {0, 0, 0}},
            {300, {300, 0, 0}, frame, {0, 0, 0}}}, 1, TopologyKind::OpenLinear};
    }
    void orderedModelAndSummary()
    {
        auto c = heterogeneous(); const auto originalIds = c.cars;
        const auto def = resolveTrainConfiguration(c);
        const auto pose = solveTrainPose(rail(), def, {primaryTrackPathId, 100});
        double dry = 0, loaded = 0;
        for (std::size_t i = 0; i < c.cars.size(); ++i)
        {
            require(def.cars[i].authoredCarId == c.cars[i].id, "Ordered physical IDs match authored records");
            const auto& car = c.cars[i].physical;
            dry += car.car.dryMassKilograms; loaded += car.car.dryMassKilograms + car.loadout.massKilograms;
            near(pose.cars()[i].loadedMassKilograms(), car.car.dryMassKilograms + car.loadout.massKilograms, 1e-10, "Heterogeneous loaded masses reach solver");
            require(glm::length(pose.cars()[i].loadedLocalCenterOfGravityMeters()
                - loadedCarCenterOfGravityMeters(car.car, car.loadout)) < 1e-10, "Heterogeneous COG reaches solver");
            near(glm::length(pose.cars()[i].carPose().frontBogie().worldPositionMeters()
                - pose.cars()[i].carPose().transformLocalPoint(car.car.bogies[0].referencePositionMeters)), 0, 1e-7, "Each car's authored bogie position is solved");
        }
        require(pose.maximumAbsoluteConnectorResidualMeters() <= connectorLengthToleranceMeters, "Heterogeneous hitch closure keeps native tolerance");
        const auto s = summarizeTrainConfiguration(c);
        near(s.dryMassKilograms, dry, 1e-10, "Native dry mass summary"); near(s.loadedMassKilograms, loaded, 1e-10, "Native loaded mass summary");
        require(s.carCount == 4 && s.bogieCount == 8 && s.connectorCount == 3 && s.straightAssemblyAvailable, "Native summary counts");
        const auto defaults = summarizeTrainConfiguration(createDefaultOrderedTrainConfiguration());
        near(defaults.bodyEnvelopeLengthMeters, 17.5, 1e-7, "Length is body-envelope span including connectors");
        for (std::size_t i = 0; i + 1 < s.cars.size(); ++i) require(s.cars[i].gapToNextMeters > .45, "Unequal body and hitch spacing remains separated");
        reorderTrainCar(c, c.cars[2].id, 0);
        require(c.cars.front().id == originalIds[2].id && resolveTrainConfiguration(c).cars.front().car.dryMassKilograms == originalIds[2].physical.car.dryMassKilograms, "Reorder moves physical record and stable ID");
        const auto copy = c.cars[1]; const auto id = insertTrainCar(c, 2, copy);
        require(id != copy.id && c.cars[2].physical.car.dryMassKilograms == copy.physical.car.dryMassKilograms && c.cars[2].visualAsset == copy.visualAsset, "Duplicate copies configuration with new identity");
        removeTrainCar(c, id); const auto next = c.nextCarId;
        require(insertTrainCar(c, 1, copy) == next, "Removed identity is never reused");
        auto overlap = createDefaultOrderedTrainConfiguration(2);
        overlap.cars[0].physical.car.bodyDimensionsMeters.x = 5.36;
        const auto diagnostics = summarizeTrainConfiguration(overlap).diagnostics;
        require(std::ranges::any_of(diagnostics, [](const auto& d) { return !d.error && d.message.find("overlaps") != std::string::npos; }), "Overlap is a warning and never auto-corrected");
        near(summarizeTrainConfiguration(overlap).cars[0].gapToNextMeters, -.18, 1e-7, "Overlap amount is measured in native meters");
        validateTrainConfiguration(overlap);
        auto bad = c; bad.cars[0].id = bad.cars[1].id; rejects([&] { validateTrainConfiguration(bad); });
        bad = c; bad.cars[0].physical.car.bogies.resize(1); rejects([&] { validateTrainConfiguration(bad); });
        bad = c; bad.cars[0].physical.car.dryMassKilograms = 0; rejects([&] { validateTrainConfiguration(bad); });
        bad = c; bad.cars[0].physical.car.bodyDimensionsMeters.z = -1; rejects([&] { validateTrainConfiguration(bad); });
        bad = c; bad.cars[0].rearConnection.rigidLengthMeters = -1; rejects([&] { validateTrainConfiguration(bad); });
        bad = c; bad.repeatedCar.loadout.massKilograms = -1; rejects([&] { validateTrainConfiguration(bad); });
    }
    void changedConsistLifecycle()
    {
        auto track = createNewDocument();
        auto settings = track.physicalSettings(); settings.initialSpeed = 0; track.setPhysicalSettings(settings);
        auto launch = makeTrackDevice(TrackDeviceKind::Launch); launch.startStationMeters = 0; launch.endStationMeters = 45;
        launch.maximumForceNewtons = 90000; launch.targetAccelerationMetersPerSecondSquared = 11.794;
        (void)track.addTrackDevice(launch);
        ApplicationService app{track}; require(app.rebuildPreview(), "Lifecycle four-car preview initializes");
        app.play(); app.update(.1);
        auto train = app.track().trainConfiguration(); resizeTrainConsist(train, 8);
        const auto before = serializeCoasterDocument(app.track());
        app.setTrainConfiguration(train);
        require(app.preview().dynamicsState()->tick == 0 && app.preview().playbackState() == SimulationPreview::PlaybackState::Stopped,
            "Four-to-eight Apply stops and resets active simulation");
        require(app.preview().dynamicsDiagnostics().cars.size() == 8 && app.preview().meshInstances().size() == 8, "Physics and visuals both rebuild eight cars");
        require(!app.preview().dynamicsDiagnostics().hasStep && app.preview().lastDeviceForces().devices.empty(), "Apply clears old forces");
        app.play(); app.update(defaultFixedTimeStepSeconds);
        require(app.preview().lastDeviceForces().devices.front().occupiedBogieCount == 16 && app.preview().dynamicsDiagnostics().loadedMassKilograms == 8000,
            "Accepted eight-car state owns mass and device occupancy");
        const auto eight = serializeCoasterDocument(app.track());
        app.restoreHistory(true); require(serializeCoasterDocument(app.track()) == before, "Undo restores four cars");
        app.restoreHistory(false); require(serializeCoasterDocument(app.track()) == eight, "Redo restores eight cars");
        auto reopened = deserializeCoasterDocument(eight); require(bool(reopened), "Eight-car save reopens");
        ApplicationService loaded{*reopened}; require(loaded.rebuildPreview() && loaded.preview().dynamicsDiagnostics().cars.size() == 8, "Reopen reconstructs eight-car physics");
        train = app.track().trainConfiguration(); resizeTrainConsist(train, 16);
        app.setTrainConfiguration(train);
        require(app.track().trainConfiguration().cars.size() == 16 && !app.preview().isAvailable(), "Valid oversize authored consist is retained with unavailable simulation");
        require(app.preview().error().find("69.800") != std::string::npos, "Oversize Apply explains required bogie distance");
        const auto oversize = serializeCoasterDocument(app.track()); require(bool(deserializeCoasterDocument(oversize)), "Oversize train remains saveable");
        app.restoreHistory(true); require(app.preview().isAvailable() && app.preview().dynamicsDiagnostics().cars.size() == 8, "Undo invalid placement recovers working preview");
        app.restoreHistory(false); require(!app.preview().isAvailable() && serializeCoasterDocument(app.track()) == oversize, "Redo restores diagnosis and authored data");
    }

    void persistenceHistoryAndPreview()
    {
        for (bool zero : {false, true})
        {
            auto c = heterogeneous(zero); auto track = trackWith(c);
            ApplicationService app{track}; require(app.rebuildPreview(), "Scenario A/B preview solves");
            const auto history = [&](const std::string& before) {
                const auto after = serializeCoasterDocument(app.track());
                app.restoreHistory(true); require(serializeCoasterDocument(app.track()) == before, "Train Undo restores exact authored state");
                app.restoreHistory(false); require(serializeCoasterDocument(app.track()) == after, "Train Redo restores exact authored state");
                auto reload = deserializeCoasterDocument(after);
                require(reload && serializeCoasterDocument(*reload) == after, "Complete train deterministic roundtrip");
            };
            for (int operation = 0; operation < 13; ++operation)
            {
                const auto before = serializeCoasterDocument(app.track()); c = app.track().trainConfiguration();
                auto& car = c.cars[1];
                switch (operation)
                {
                case 0: static_cast<void>(insertTrainCar(c, 1, createTrainCarPrototype(TrainCarRole::Zero))); break;
                case 1: removeTrainCar(c, car.id); break;
                case 2: reorderTrainCar(c, c.cars.back().id, 1); break;
                case 3: static_cast<void>(insertTrainCar(c, 2, car)); break;
                case 4: car.role = TrainCarRole::Custom; break;
                case 5: car.physical.car.dryMassKilograms += 50; break;
                case 6: car.physical.car.dryCenterOfGravityMeters.z += .05; break;
                case 7: car.physical.car.bogies[0].referencePositionMeters.x += .05; break;
                case 8: car.physical.car.frontHitchPositionMeters.x += .1; break;
                case 9: car.visualAsset = "assets://train/custom-regression.glb"; break;
                case 10: car.visualAlignment.rotationDegrees.z = 7; break;
                case 11: car.physical.loadout.massKilograms += 25; break;
                case 12: car.physical.loadout.centerOfMassMeters.x += .1; break;
                }
                const auto generation = app.centerline().generation();
                app.setTrainConfiguration(c); history(before);
                require(app.preview().trainDefinition().cars.size() == c.cars.size(), "Reset/history reconstruct current topology");
                // restoreHistory prepares track products; the Train edit itself
                // is checked separately below to avoid mistaking restore work.
                static_cast<void>(generation);
            }
            c = app.track().trainConfiguration(); const auto generation = app.centerline().generation();
            c.cars[0].name += " edited"; app.play(); app.update(1.5 * defaultFixedTimeStepSeconds);
            app.setTrainConfiguration(c);
            require(app.centerline().generation() == generation, "Train-only edit retains track presentation");
            require(app.preview().dynamicsState()->tick == 0 && app.preview().playbackState() == SimulationPreview::PlaybackState::Stopped, "Live edit stops and resets simulation");
            const auto initial = app.preview().pose()->cars().front().carPose().bodyWorldPositionMeters();
            app.play(); app.update(1.5 * defaultFixedTimeStepSeconds); app.reset();
            require(app.preview().dynamicsState()->tick == 0 && glm::length(app.preview().pose()->cars().front().carPose().bodyWorldPositionMeters() - initial) < 1e-10, "Reset uses edited car definitions");
            const auto size = app.history().size();
            app.preview().setPhysicsDiagnosticsVisible(false);
            app.preview().setTrainInspection(true, c.cars[0].id, 31);
            require(!app.preview().vertices().empty(), "Selected car engineering overlays exist");
            app.preview().setTrainInspection(false, 0);
            require(app.preview().vertices().empty() && app.history().size() == size, "Display changes never enter history");
            for (std::size_t i = 0; i < c.cars.size(); ++i)
            {
                const auto& instance = app.preview().meshInstances()[i];
                require(instance.assetIdentifier == c.cars[i].visualAsset, "Visual order belongs to matching physical car");
                const auto transformed = glm::dvec3(instance.transform * glm::vec4{0, 0, 0, 1});
                const auto expected = app.preview().renderPose()->cars()[i].carPose().transformLocalPoint(c.cars[i].visualAlignment.translationMeters);
                require(glm::length(transformed - expected) < 1e-5, "Visual alignment applies at physical body origin only");
            }
        }
        auto legacy = nlohmann::json::parse(serializeCoasterDocument(trackWith(createDefaultOrderedTrainConfiguration())));
        legacy.erase("train"); const auto migrated = deserializeCoasterDocument(legacy.dump());
        require(migrated && migrated->trainConfiguration().cars.size() == 4, "Historical repeated train expands to ordered cars");
        SimulationPreview before, after; require(before.rebuild(*migrated), "Migrated preview available");
        const auto saved = deserializeCoasterDocument(serializeCoasterDocument(*migrated));
        require(saved && after.rebuild(*saved), "Migrated save/reload preview available");
        for (std::size_t i = 0; i < 4; ++i)
            require(glm::length(before.pose()->cars()[i].carPose().bodyWorldPositionMeters() - after.pose()->cars()[i].carPose().bodyWorldPositionMeters()) < 1e-8, "Legacy pose equivalence within tolerance");
        before.play(); after.play(); before.update(2.5 * defaultFixedTimeStepSeconds); after.update(2.5 * defaultFixedTimeStepSeconds);
        near(before.speedMetersPerSecond(), after.speedMetersPerSecond(), 1e-10, "Migration preserves simulation response");
        legacy["train"] = nlohmann::json::parse(serializeTrainConfiguration(heterogeneous()));
        legacy["train"]["cars"][0]["unexpected"] = 1;
        require(!deserializeCoasterDocument(legacy.dump()), "Strict train schema rejects unknown fields");
    }
    void hardwareMassAndIdentity()
    {
        const auto track = rail(); const auto c = heterogeneous(true); const auto def = resolveTrainConfiguration(c);
        const auto pose = solveTrainPose(track, def, {primaryTrackPathId, 60});
        const auto observation = observeTrain(pose, 1, def.trainId, &def);
        require(observation.id == 73 && observation.bogies.size() == 10, "Zero-car observation has stable train and all bogies");
        auto sensor = device(TrackDeviceKind::Block, 1, 59.8, 61);
        RideOperations occupancy; occupancy.reset(std::span(&sensor, 1)); occupancy.step(std::span(&sensor, 1), std::span(&observation, 1), 300, false);
        require(occupancy.hardware()[0].occupancy.bogies.size() == 1 && occupancy.hardware()[0].occupancy.bogies[0].carId == c.cars[0].id,
            "Partial occupancy reports stable Zero car ID and bogie-local identity");
        for (auto kind : {TrackDeviceKind::Launch, TrackDeviceKind::Brake, TrackDeviceKind::Transport, TrackDeviceKind::ChainLift, TrackDeviceKind::CableLift})
        {
            auto hardware = device(kind, 1, 0, 120);
            RideOperations operations; operations.reset(std::span(&hardware, 1));
            operations.step(std::span(&hardware, 1), std::span(&observation, 1), 300, false);
            operations.step(std::span(&hardware, 1), std::span(&observation, 1), 300, false, defaultFixedTimeStepSeconds);
            operations.step(std::span(&hardware, 1), std::span(&observation, 1), 300, false, defaultFixedTimeStepSeconds);
            const auto forces = evaluateTrackDeviceForces(std::span(&hardware, 1), operations.forceControls(), track, def, pose, 1);
            require(forces.applications.size() == 10, "Each force device addresses heterogeneous bogies");
            std::vector<double> perCar(def.cars.size());
            for (const auto& force : forces.applications) perCar[force.carIndex] += glm::length(force.worldForceNewtons);
            const auto ratio = perCar[0] / pose.cars()[0].loadedMassKilograms();
            require(ratio > 0, "Hardware exerts a native force");
            for (std::size_t i = 1; i < perCar.size(); ++i)
                near(perCar[i] / pose.cars()[i].loadedMassKilograms(), ratio, 1e-8, "Device force scales by occupied car mass, never equal shares");
            hardware.maximumForceNewtons = 300;
            const auto limited = evaluateTrackDeviceForces(std::span(&hardware, 1), operations.forceControls(), track, def, pose, 1);
            near(std::abs(limited.devices[0].appliedForceNewtons), 300, 1e-8, "Heterogeneous device aggregate force cap");
            TrainDynamicsState state{{primaryTrackPathId, 60}, 1, 0, 0, FollowerRunState::Running};
            const auto stepped = stepTrain(track, def, {}, state, {}, forces.applications);
            require(stepped.state.tick == 1 && std::isfinite(stepped.state.signedVelocityMetersPerSecond), "Heterogeneous force input reaches specialized step");
            if (kind == TrackDeviceKind::ChainLift || kind == TrackDeviceKind::CableLift)
            {
                auto moving = stepped.state; auto movingPose = stepped.telemetry.pose;
                for (int tick = 0; tick < 60; ++tick)
                {
                    const auto observed = observeTrain(movingPose, moving.signedVelocityMetersPerSecond, def.trainId, &def);
                    operations.step(std::span(&hardware, 1), std::span(&observed, 1), 300, false, defaultFixedTimeStepSeconds);
                    auto input = evaluateTrackDeviceForces(std::span(&hardware, 1), operations.forceControls(), track, def, movingPose, moving.signedVelocityMetersPerSecond);
                    auto result = stepTrain(track, def, {}, moving, {}, input.applications);
                    moving = result.state; movingPose = std::move(result.telemetry.pose);
                }
                require(moving.signedVelocityMetersPerSecond > 1 && operations.hardware()[0].state == "Pulling", "Heterogeneous lift engages and advances using native states");
            }
        }
    }
    void previewPickingAndFraming()
    {
        const auto c = heterogeneous();
        const auto pose = solveTrainPose(rail(), resolveTrainConfiguration(c), {primaryTrackPathId, 100});
        std::vector<TrainPreviewCar> cars;
        for (std::size_t i = 0; i < c.cars.size(); ++i)
        {
            const auto& body = pose.cars()[i].carPose();
            cars.push_back({c.cars[i].id, i, body.bodyWorldPositionMeters(),
                c.cars[i].physical.car.bodyDimensionsMeters,
                pose.cars()[i].loadedWorldCenterOfGravityMeters(), body.bodyOrientation()});
        }
        for (const auto& car : cars)
            require(pickTrainPreviewCar(cars, car.center + glm::dvec3{0, 0, 10}, {0, 0, -1}) == car.id,
                "Native envelope picking selects each heterogeneous physical car ID");
        require(!pickTrainPreviewCar(cars, {0, 50, 10}, {0, 0, -1}), "Empty viewport ray leaves selection alone");
        require(!trainPreviewBounds({}, {}) && !trainPreviewBounds(cars, 999), "Missing train/selection does not frame unrelated geometry");
        cars[1].orientation = glm::normalize(glm::angleAxis(.35, glm::dvec3{0, 1, 0})
            * glm::angleAxis(.6, glm::dvec3{0, 0, 1}));
        const auto& tilted = cars[1];
        require(pickTrainPreviewCar(cars, tilted.center + tilted.orientation * glm::dvec3{0, 0, 10},
            tilted.orientation * glm::dvec3{0, 0, -1}) == tilted.id, "Picking uses rotated body bounds rather than GLB bounds");
        ViewportCamera camera;
        const auto wholeBounds = trainPreviewBounds(cars);
        camera.setBounds(wholeBounds->first, wholeBounds->second);
        for (auto projection : {ViewportProjection::Perspective, ViewportProjection::Orthographic})
        {
            camera.setProjection(projection);
            for (auto selected : {std::optional<std::uint64_t>{}, std::optional<std::uint64_t>{tilted.id}})
            {
                const auto bounds = trainPreviewBounds(cars, selected);
                const double yaw = camera.yaw(), pitch = camera.pitch();
                require(bounds && camera.frameBounds(bounds->first, bounds->second, 16. / 9.), "Train/selected framing uses native bounds");
                near(camera.yaw(), yaw, 1e-12, "Train framing preserves camera yaw");
                near(camera.pitch(), pitch, 1e-12, "Train framing preserves camera pitch");
                for (const auto& car : cars)
                {
                    if (selected && car.id != *selected) continue;
                    for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1})
                    {
                        const auto corner = car.center + car.orientation * (car.dimensions * glm::dvec3{x, y, z} * .5);
                        require(glm::all(glm::greaterThanEqual(corner, bounds->first - glm::dvec3{1e-10}))
                            && glm::all(glm::lessThanEqual(corner, bounds->second + glm::dvec3{1e-10})), "Framing contains all transformed body corners");
                        const auto projected = projectViewportPoint(camera, corner, 16. / 9.);
                        require(projected && projected->normalizedPosition.x >= 0 && projected->normalizedPosition.x <= 1
                            && projected->normalizedPosition.y >= 0 && projected->normalizedPosition.y <= 1, "Framed train corners stay inside viewport");
                    }
                }
            }
        }
        const std::array aligned{TrainPreviewCar{10, 0, {0, 0, 0}, {2, 2, 2}, {}, {1, 0, 0, 0}},
            TrainPreviewCar{20, 1, {5, 0, 0}, {2, 2, 2}, {}, {1, 0, 0, 0}}};
        require(pickTrainPreviewCar(aligned, {-10, 0, 0}, {1, 0, 0}) == 10, "Overlapping ray chooses nearest physical envelope");
        require(!pickTrainPreviewCar(aligned, {10, 0, 0}, {1, 0, 0}), "Car behind the camera cannot be picked");
    }
    void integratedOperations()
    {
        auto c = heterogeneous(); const auto def = resolveTrainConfiguration(c); const auto track = rail();
        auto station = device(TrackDeviceKind::Station, 1, 0, 45); std::get<StationConfiguration>(station.configuration).stopStationMeters = 25;
        auto tires = device(TrackDeviceKind::Transport, 2, 40, 70); std::get<TransportConfiguration>(tires.configuration).speedMetersPerSecond = 6;
        auto launch = device(TrackDeviceKind::Launch, 3, 65, 115); std::get<LaunchConfiguration>(launch.configuration).targetSpeedMetersPerSecond = 10;
        auto brake = device(TrackDeviceKind::Brake, 4, 110, 145); std::get<BrakeConfiguration>(brake.configuration).targetSpeedMetersPerSecond = 5;
        auto arrival = device(TrackDeviceKind::Station, 5, 140, 185); std::get<StationConfiguration>(arrival.configuration).stopStationMeters = 165;
        const std::vector hardware{station, tires, launch, brake, arrival};
        RideOperations operations; operations.reset(hardware);
        TrainDynamicsState state{{primaryTrackPathId, 25}, 0, 0, 0, FollowerRunState::Resting};
        auto pose = solveTrainPose(track, def, state.generalizedReferenceLocation);
        const auto refresh = [&](double seconds) {
            const auto observation = observeTrain(pose, state.signedVelocityMetersPerSecond, def.trainId, &def);
            operations.step(hardware, std::span(&observation, 1), 300, false, seconds);
        };
        refresh(0); require(operations.hardware()[0].aligned && operations.hardware()[0].stopped, "Heterogeneous train aligns at car-0 generalized reference");
        for (int i = 0; i < 30; ++i)
        {
            refresh(defaultFixedTimeStepSeconds);
            auto forces = evaluateTrackDeviceForces(hardware, operations.forceControls(), track, def, pose, state.signedVelocityMetersPerSecond);
            auto step = stepTrain(track, def, {}, state, {}, forces.applications); state = step.state; pose = std::move(step.telemetry.pose);
        }
        near(state.signedVelocityMetersPerSecond, 0, 1e-9, "Station holds whole heterogeneous train");
        operations.command(1, HardwareCommand::RequestDispatch);
        std::set<std::size_t> occupied; bool arrived = false, partial = false; double peak = 0;
        for (int i = 0; i < 14000; ++i)
        {
            refresh(defaultFixedTimeStepSeconds);
            const auto forces = evaluateTrackDeviceForces(hardware, operations.forceControls(), track, def, pose, state.signedVelocityMetersPerSecond);
            operations.recordForces(forces);
            for (std::size_t h = 0; h < hardware.size(); ++h)
                if (!operations.hardware()[h].occupancy.clear())
                {
                    occupied.insert(h);
                    if (!operations.hardware()[h].occupancy.trains[0].complete) partial = true;
                    for (const auto& bogie : operations.hardware()[h].occupancy.bogies)
                        require(bogie.carId == c.cars[bogie.car].id && bogie.train == c.trainId, "Integrated hardware retains ordered stable identities");
                }
            auto step = stepTrain(track, def, {}, state, {}, forces.applications); state = step.state; pose = std::move(step.telemetry.pose);
            peak = std::max(peak, state.signedVelocityMetersPerSecond);
            if (operations.hardware()[4].aligned && operations.hardware()[4].stopped) { arrived = true; break; }
        }
        std::cout << "Scenario C station/transport/launch/brake/station: tick=" << state.tick << " q=" << state.generalizedReferenceLocation.stationMeters << " peak=" << peak << '\n';
        require(arrived && occupied.size() == 5 && partial && peak > 9.8, "Scenario C heterogeneous station dispatch, hardware traversal and arrival");
    }
    AuthoredTrack acceptance()
    {
        auto track = trackWith(heterogeneous()); auto physical = track.physicalSettings(); physical.initialSpeed = 0; track.setPhysicalSettings(physical);
        auto station = makeTrackDevice(TrackDeviceKind::Station); station.startStationMeters = 0; station.endStationMeters = 45;
        station.name = "Departure · car 0 reference"; std::get<StationConfiguration>(station.configuration).stopStationMeters = 25;
        static_cast<void>(track.addTrackDevice(station));
        for (const auto& [kind, start, end] : {std::tuple{TrackDeviceKind::Transport, 40., 70.}, {TrackDeviceKind::Launch, 65., 115.},
            {TrackDeviceKind::Brake, 110., 145.}, {TrackDeviceKind::Station, 140., 185.}})
        {
            auto d = makeTrackDevice(kind); d.startStationMeters = start; d.endStationMeters = end; d.maximumForceNewtons = 100000;
            d.targetAccelerationMetersPerSecondSquared = 4;
            if (kind == TrackDeviceKind::Transport) std::get<TransportConfiguration>(d.configuration).speedMetersPerSecond = 6;
            if (kind == TrackDeviceKind::Launch) std::get<LaunchConfiguration>(d.configuration).targetSpeedMetersPerSecond = 10;
            if (kind == TrackDeviceKind::Brake) std::get<BrakeConfiguration>(d.configuration).targetSpeedMetersPerSecond = 5;
            if (kind == TrackDeviceKind::Station) std::get<StationConfiguration>(d.configuration).stopStationMeters = 165;
            static_cast<void>(track.addTrackDevice(d));
        }
        return track;
    }
}
int main(int argc, char** argv)
{
    try
    {
        if (argc == 3 && std::string_view(argv[1]) == "--write-fixture")
        { std::ofstream file(argv[2]); file << serializeCoasterDocument(acceptance()); if (!file) throw std::runtime_error("Fixture write failed"); return 0; }
        orderedModelAndSummary(); changedConsistLifecycle(); persistenceHistoryAndPreview(); hardwareMassAndIdentity(); previewPickingAndFraming(); integratedOperations();
        std::cout << "Train2Tests passed " << checks << " checks\n"; return 0;
    }
    catch (const std::exception& error) { std::cerr << "Train2Tests failed: " << error.what() << '\n'; return 1; }
}
