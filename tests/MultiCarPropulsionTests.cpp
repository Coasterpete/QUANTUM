#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/TrainConfiguration.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/physics/TrainPhysics.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum;
    int checks = 0, scenarios = 0;
    void require(bool condition, const std::string& message)
    { ++checks; if (!condition) throw std::runtime_error(message); }
    void near(double actual, double expected, double tolerance, const std::string& message)
    { require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
        message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual)); }

    coaster::AuthoredTrack fixture(double length, std::size_t count, double slopeDegrees = 0,
        double speed = 0, double force = 90000, double massScale = 1, bool connected = false)
    {
        auto track = coaster::createNewDocument();
        track.setLayoutMode(coaster::LayoutMode::Shuttle);
        coaster::setSectionLength(track.section(0), connected ? length * .5 : length);
        if (connected) track.insertSectionAfter(0, coaster::createRateProfileSection(length * .5));
        auto start = track.startPose();
        start.orientation = glm::angleAxis(-slopeDegrees * std::numbers::pi / 180., glm::dvec3{0, 1, 0});
        track.setStartPose(start);
        auto settings = track.physicalSettings(); settings.initialSpeed = speed; track.setPhysicalSettings(settings);
        auto train = coaster::createDefaultOrderedTrainConfiguration(count);
        for (auto& car : train.cars)
        {
            car.physical.car.dryMassKilograms *= massScale;
            car.physical.car.dryInertiaTensorBodyKgM2 *= massScale;
            car.physical.loadout.massKilograms *= massScale;
        }
        track.setTrainConfiguration(train);
        auto launch = coaster::makeTrackDevice(coaster::TrackDeviceKind::Launch);
        launch.startStationMeters = 0; launch.endStationMeters = std::min(45., length);
        launch.maximumForceNewtons = std::max(1., force); launch.enabled = force > 0;
        launch.targetAccelerationMetersPerSecondSquared = 11.794;
        std::get<coaster::LaunchConfiguration>(launch.configuration).targetSpeedMetersPerSecond = 100;
        (void)track.addTrackDevice(launch);
        return track;
    }

    void rigid(const editor::SimulationPreview& preview)
    {
        const auto& d = preview.dynamicsDiagnostics();
        require(d.placementValid && d.cars.size() == preview.trainDefinition().cars.size(), "Committed car diagnostics match train");
        require(d.tick == preview.dynamicsState()->tick, "Diagnostics use committed tick");
        for (std::size_t i = 0; i < d.cars.size(); ++i)
        {
            const auto& car = d.cars[i];
            require(car.frontBogieStationMeters >= -1e-8 && car.frontBogieStationMeters <= d.trackLengthMeters + 1e-8
                && car.rearBogieStationMeters >= -1e-8 && car.rearBogieStationMeters <= d.trackLengthMeters + 1e-8,
                "Every bogie remains inside supported track range");
            if (i)
            {
                const auto& definition = preview.trainDefinition();
                const double spacing = definition.cars[i].car.frontHitchPositionMeters.x
                    - definition.cars[i-1].car.rearHitchPositionMeters.x + definition.connections[i-1].rigidLengthMeters;
                near(d.cars[i-1].referenceStationMeters - car.referenceStationMeters, spacing, 1e-7,
                    "Car spacing retains non-overlapping connector branch");
            }
        }
        require(d.maximumConnectorResidualMeters <= physics::connectorLengthToleranceMeters, "Connector closure tolerance");
    }

    double analyticalVelocity(const editor::SimulationPreview& preview, double previousSpeed, double slopeDegrees)
    {
        const auto& d = preview.dynamicsDiagnostics();
        const auto& resistance = preview.trainDefinition().resistance;
        const double gravity = -d.loadedMassKilograms * 9.80665 * std::sin(slopeDegrees * std::numbers::pi / 180.);
        const double supply = preview.lastDeviceForces().devices.front().appliedForceNewtons;
        const double dryCapacity = resistance.constantMechanicalForceNewtons
            + resistance.rollingResistanceCoefficient * d.loadedMassKilograms * 9.80665;
        const double dry = previousSpeed == 0 ? -std::clamp(supply + gravity, -dryCapacity, dryCapacity)
            : -std::copysign(dryCapacity, previousSpeed);
        const double linear = -resistance.linearResistanceCoefficientNewtonSecondsPerMeter * previousSpeed;
        const double drag = -.5 * resistance.airDensityKilogramsPerCubicMeter * resistance.dragAreaSquareMeters
            * previousSpeed * std::abs(previousSpeed);
        near(d.gravityForceNewtons, gravity, .02, "Distributed gravity on straight slope");
        near(d.externalForceNewtons, supply, .02, "Device cap supplied once to physics");
        near(d.resistanceForceNewtons, dry + linear + drag, 1e-7, "Actual resistance agrees with modeled law");
        near(d.effectiveMassKilograms, d.loadedMassKilograms, .002, "No artificial effective mass from connector branch switching");
        near(d.netForceNewtons, d.gravityForceNewtons + d.externalForceNewtons + d.resistanceForceNewtons
            + d.massGradientForceNewtons + d.constraintForceNewtons, .002, "Resolved force balance closes");
        double velocity = previousSpeed + (supply + gravity + dry + linear + drag) / d.loadedMassKilograms / 240.;
        if (previousSpeed * velocity < 0 || std::abs(velocity) <= physics::followerRestSpeedToleranceMetersPerSecond) velocity = 0;
        return velocity;
    }

    void matrix()
    {
        std::cout << "cars,length_m,slope_deg,regions,mass_kg,initial_v,cap_N,ticks,v_mps,a_mps2,request_N,supply_N,Q_N,gravity_N,resistance_N,constraint_N,connector_residual_m,bogie_min_m,bogie_max_m\n";
        for (int geometry = 0; geometry < 6; ++geometry)
        for (std::size_t count : {1, 2, 4, 8, 12})
        for (double force : {0., 2000., 90000., 1000000.})
        for (double speed : {0., 5.})
        for (double massScale : {1., 2.})
        {
            const double length = geometry == 0 ? 60 : geometry == 2 ? 500 : 200;
            const double slope = geometry == 3 ? 10 : geometry == 4 ? 60 : 0;
            auto track = fixture(length, count, slope, speed, force, massScale, geometry == 5);
            editor::SimulationPreview preview;
            require(preview.rebuild(track), "Matrix placement: " + preview.error());
            const auto& d = preview.dynamicsDiagnostics();
            near(d.loadedMassKilograms, count * 1000. * massScale, 1e-8, "Loaded mass scales");
            rigid(preview); preview.play();
            for (int tick = 0; tick < 24; ++tick)
            {
                const double speedBefore = preview.dynamicsState()->signedVelocityMetersPerSecond;
                double expectedRequest = 0;
                std::size_t expectedOccupancy = 0;
                for (const auto& car : preview.pose()->cars())
                    for (const auto* bogie : {&car.carPose().frontBogie(), &car.carPose().rearBogie()})
                        if (bogie->location().stationMeters >= 0 && bogie->location().stationMeters < 45)
                        { ++expectedOccupancy; if (force > 0) expectedRequest += car.loadedMassKilograms() * .5 * 11.794; }
                preview.update(physics::defaultFixedTimeStepSeconds);
                require(preview.isAvailable(), "Production preview remains available: " + preview.error());
                require(d.hasStep && d.boundary == physics::TrackBoundary::None, "Valid train is not endpoint-blocked");
                near(preview.dynamicsState()->signedVelocityMetersPerSecond, analyticalVelocity(preview, speedBefore, slope), 2e-6, "Analytical F_net / mass");
                rigid(preview);
                const auto& device = preview.lastDeviceForces().devices.front();
                require(device.occupiedBogieCount == expectedOccupancy, "Device occupancy uses every committed bogie");
                near(device.requestedForceNewtons, expectedRequest, 1e-6, "Acceleration demand scales with engaged mass");
                near(device.appliedForceNewtons, std::min(expectedRequest, std::max(1., force)), 1e-6, "Total mechanism cap distributed once");
                require(device.generalizedForceNewtons.has_value(), "Projection published after actual step");
                near(*device.generalizedForceNewtons, d.externalForceNewtons, 1e-9, "Actual per-device contribution");
                require(device.forceLimitReached == (device.requestedAbsoluteForceNewtons > std::max(1., force)), "Saturation meaning");
                if (force == 0) near(device.appliedForceNewtons, 0, 0, "Disabled launch supplies no force");
            }
            const auto& device = preview.lastDeviceForces().devices.front();
            std::cout << count << ',' << length << ',' << slope << ',' << track.sectionCount() << ',' << d.loadedMassKilograms << ',' << speed << ',' << force
                << ',' << d.tick << ',' << d.signedSpeedMetersPerSecond << ',' << d.accelerationMetersPerSecondSquared << ',' << device.requestedForceNewtons
                << ',' << device.appliedForceNewtons << ',' << d.externalForceNewtons << ',' << d.gravityForceNewtons << ',' << d.resistanceForceNewtons
                << ',' << d.constraintForceNewtons << ',' << d.maximumConnectorResidualMeters << ',' << d.minimumBogieStationMeters << ',' << d.maximumBogieStationMeters << '\n';
            ++scenarios;
        }
    }

    void spacingAndInitialization()
    {
        for (double bodyLength : {3., 4., 6.}) for (double bogieHalfSpacing : {.3, 1.15, 1.9}) for (double connector : {0., .5, 2.})
        {
            auto track = fixture(200, 8); auto train = track.trainConfiguration();
            std::cout << "spacing body=" << bodyLength << " bogie_half=" << bogieHalfSpacing << " connector=" << connector << '\n';
            for (auto& car : train.cars)
            {
                car.physical.car.bodyDimensionsMeters.x = bodyLength;
                car.physical.car.frontHitchPositionMeters.x = bodyLength * .5;
                car.physical.car.rearHitchPositionMeters.x = -bodyLength * .5;
                car.physical.car.bogies[0].referencePositionMeters.x = bogieHalfSpacing;
                car.physical.car.bogies[1].referencePositionMeters.x = -bogieHalfSpacing;
                car.rearConnection.rigidLengthMeters = connector;
            }
            track.setTrainConfiguration(train);
            editor::SimulationPreview preview; require(preview.rebuild(track), "Spacing fixture: " + preview.error());
            rigid(preview); preview.play();
            for (int i = 0; i < 24; ++i) { const double speed = preview.dynamicsState()->signedVelocityMetersPerSecond; preview.update(1./240.); rigid(preview);
                near(preview.dynamicsState()->signedVelocityMetersPerSecond, analyticalVelocity(preview, speed, 0), 2e-6, "Spacing-independent acceleration"); }
            ++scenarios;
        }
        std::cout << "branch transition sweep\n";
        auto track = fixture(60, 8); const auto train = coaster::resolveTrainConfiguration(track.trainConfiguration());
        const auto rail = physics::CompiledPhysicsTrack(coaster::integrateAuthoredTrackKinematics(track, .5), track.physicalSettings(), track.layoutMode(), coaster::TopologyKind::OpenLinear);
        bool rejected = false;
        try { (void)physics::solveTrainPose(rail, train, {physics::primaryTrackPathId, 31.75}); } catch (const std::domain_error&) { rejected = true; }
        require(rejected, "Original compressed initialization rejected");
        for (double q = 32.650001; q < 34.; q += .002)
        {
            const auto kin = physics::evaluateTrainKinematics(rail, train, {}, {physics::primaryTrackPathId, q});
            near(kin.effectiveGeneralizedMassKilograms, 8000, .002, "Endpoint samples stay on physical branch");
        }
        editor::SimulationPreview preview;
        require(preview.rebuild(fixture(4, 1)), "Fitting one-car short track no longer searches only beyond 14.75 m");
        auto tooShort = fixture(60, 16); const auto saved = coaster::serializeCoasterDocument(tooShort);
        require(!preview.rebuild(tooShort) && !preview.dynamicsState(), "Oversize train invalidates playback");
        require(preview.error().find("69.800") != std::string::npos && preview.error().find("60.000") != std::string::npos,
            "Failure gives required straight span and available length");
        require(saved == coaster::serializeCoasterDocument(tooShort), "Rejection preserves authored document");
        preview.play(); preview.update(1); require(!preview.isAvailable(), "Invalid placement cannot play");
    }

    void launchLongRunAndReset()
    {
        for (std::size_t count : {4, 8, 12})
        {
            auto track = fixture(200, count); editor::SimulationPreview preview; require(preview.rebuild(track), "Long launch rebuild");
            const auto run = [&] {
                preview.reset(); require(!preview.dynamicsDiagnostics().hasStep && preview.lastDeviceForces().devices.empty(), "Reset clears stale forces");
                preview.play();
                for (int i = 0; i < 240; ++i) { const double speed = preview.dynamicsState()->signedVelocityMetersPerSecond; preview.update(1./240.);
                    near(preview.dynamicsState()->signedVelocityMetersPerSecond, analyticalVelocity(preview, speed, 0), 2e-6, "One-second analytical launch"); rigid(preview); }
                return std::pair(preview.dynamicsState()->signedVelocityMetersPerSecond, preview.pose()->generalizedReferenceLocation().stationMeters);
            };
            const auto first = run(); require(first == run(), "Reset replay deterministic");
            std::cout << "one_second cars=" << count << " v=" << first.first << " q=" << first.second << '\n';
        }
        editor::SimulationPreview preview; require(preview.rebuild(fixture(60, 8)), "Original 60 m regression"); preview.play();
        for (int i = 0; i < 1200 && preview.playbackState() == editor::SimulationPreview::PlaybackState::Playing; ++i) preview.update(1./240.);
        require(preview.dynamicsDiagnostics().boundary == physics::TrackBoundary::End && preview.dynamicsState()->signedVelocityMetersPerSecond == 0, "Complete consist stops at leading-bogie boundary");
        require(preview.lastDeviceForces().devices.front().generalizedForceNewtons.has_value(), "Endpoint retains supplied force independently of correction");
        near(preview.dynamicsDiagnostics().maximumBogieStationMeters, 60, 1e-7, "No bogie beyond endpoint");
    }

    void reverseAndConnectorBalance()
    {
        auto authored = fixture(200, 8); auto train = coaster::resolveTrainConfiguration(authored.trainConfiguration()); train.resistance = {};
        const auto rail = physics::CompiledPhysicsTrack(coaster::integrateAuthoredTrackKinematics(authored, .5), authored.physicalSettings(), authored.layoutMode(), coaster::TopologyKind::OpenLinear);
        for (auto direction : {physics::TravelDirection::IncreasingStation, physics::TravelDirection::DecreasingStation})
        {
            physics::TrainDynamicsState state; state.generalizedReferenceLocation = {physics::primaryTrackPathId, 100, direction}; state.signedVelocityMetersPerSecond = -5;
            const std::vector<physics::ExternalForceApplication> forces{{0, {0,0,0}, {90000,0,0}}};
            const auto result = physics::stepTrain(rail, train, {}, state, {}, forces);
            near(result.state.signedVelocityMetersPerSecond, -5 + 90000./8000./240., 1e-8, "Reverse speed integrates force with fixed facing");
            require(result.state.generalizedReferenceLocation.direction == direction, "Facing separate from signed travel");
            const auto loads = physics::evaluateRigidConnectorLoads(rail, train, {}, result.state, forces);
            require(loads.status() == physics::RigidConnectorLoadRecoveryStatus::Available, "Connector balance available with aggregate resistance disabled");
            require(loads.balanceResidualNewtons() && std::abs(*loads.balanceResidualNewtons()) <= loads.balanceToleranceNewtons(), "Connector residual closes");
        }
        auto shifted = train; shifted.cars.resize(1); shifted.connections.clear();
        shifted.cars[0].car.bogies[0].referencePositionMeters.x = -1;
        shifted.cars[0].car.bogies[1].referencePositionMeters.x = -3;
        physics::TrainDynamicsState endpoint; endpoint.generalizedReferenceLocation = {physics::primaryTrackPathId, 199.99}; endpoint.signedVelocityMetersPerSecond = 10;
        const auto stopped = physics::stepTrain(rail, shifted, {}, endpoint);
        require(stopped.telemetry.boundaryIntervention && stopped.state.runState == physics::FollowerRunState::StoppedAtEnd
            && stopped.state.signedVelocityMetersPerSecond == 0, "Reference endpoint stops outgoing velocity even with inset bogies");
        const std::vector<physics::ExternalForceApplication> inward{{0, {0,0,0}, {-10000,0,0}}};
        const auto recovered = physics::stepTrain(rail, shifted, {}, stopped.state, {}, inward);
        require(recovered.state.signedVelocityMetersPerSecond < 0 && !recovered.telemetry.boundaryIntervention, "Endpoint permits inward force recovery");
    }

    void profilesAndMechanisms()
    {
        for (std::size_t count : {1, 2, 4, 8, 12})
        for (auto kind : {coaster::TrackDeviceKind::Launch, coaster::TrackDeviceKind::CableLift,
            coaster::TrackDeviceKind::ChainLift, coaster::TrackDeviceKind::Transport, coaster::TrackDeviceKind::Brake})
        {
            auto track = fixture(200, count, 0, 2);
            auto devices = track.trackDevices();
            auto& device = devices.devices.front(); const auto id = device.id;
            device = coaster::makeTrackDevice(kind); device.id = id;
            device.startStationMeters = 0; device.endStationMeters = 180;
            device.maximumForceNewtons = 90000; device.targetAccelerationMetersPerSecondSquared = 3;
            if (auto* transport = std::get_if<coaster::TransportConfiguration>(&device.configuration)) transport->speedMetersPerSecond = 6;
            if (kind == coaster::TrackDeviceKind::Launch || kind == coaster::TrackDeviceKind::CableLift)
            {
                device.accelerationProfile = coaster::ChannelProfile{};
                device.accelerationProfile->segments.push_back({1, math::ScalarTransition{
                    .domainBegin = 0, .domainEnd = 180, .valueBegin = 2, .valueEnd = 4,
                    .transitionType = math::TransitionType::Linear}});
                device.accelerationProfile->nextSegmentId = 2;
            }
            track.setTrackDevices(devices);
            editor::SimulationPreview enabled, disabled;
            require(enabled.rebuild(track), "Mechanism fixture initializes");
            device.enabled = false; track.setTrackDevices(devices);
            require(disabled.rebuild(track), "Disabled comparison initializes");
            enabled.play(); disabled.play();
            double peakSupply = 0;
            for (int tick = 0; tick < 60; ++tick)
            {
                enabled.update(1./240.); disabled.update(1./240.);
                require(enabled.isAvailable() && disabled.isAvailable(), "Mechanism production stepping remains valid");
                rigid(enabled); rigid(disabled);
                const auto& force = enabled.lastDeviceForces().devices.front();
                require(force.occupiedBogieCount == count * 2, "Mechanism occupies every car's bogies");
                peakSupply = std::max(peakSupply, std::abs(force.appliedForceNewtons));
                near(disabled.lastDeviceForces().devices.front().appliedForceNewtons, 0, 0, "Disabled mechanism supplies no force");
                require(std::abs(force.appliedForceNewtons) <= 90000 + 1e-7, "Mechanism cap is per train, not per bogie");
                near(*force.generalizedForceNewtons, enabled.dynamicsDiagnostics().externalForceNewtons, 1e-8, "Mechanism projection is actual integrator input");
            }
            require(peakSupply > 0, "Enabled mechanism physically engaged");
            const double change = enabled.dynamicsState()->signedVelocityMetersPerSecond - disabled.dynamicsState()->signedVelocityMetersPerSecond;
            require(kind == coaster::TrackDeviceKind::Brake ? change < -.1 : change > .1, "Enabled mechanism changes real train motion");
            ++scenarios;
        }
    }
}
int main(int argc, char** argv)
{
    try
    {
        if (argc == 3 && std::string(argv[1]) == "--write-fixtures")
        {
            const std::filesystem::path directory(argv[2]); std::filesystem::create_directories(directory);
            for (std::size_t count : {4, 8, 12}) { std::ofstream file(directory / ("launch-" + std::to_string(count) + ".quantum"));
                file << coaster::serializeCoasterDocument(fixture(200, count)); require(bool(file), "Write runtime fixture"); }
            for (const auto& [name, track] : {std::pair{"launch-8-short", fixture(60, 8)},
                std::pair{"launch-8-uphill", fixture(200, 8, 10)}, std::pair{"invalid-16-short", fixture(60, 16)}})
            { std::ofstream file(directory / (std::string(name) + ".quantum")); file << coaster::serializeCoasterDocument(track); require(bool(file), "Write acceptance fixture"); }
            return 0;
        }
        matrix(); spacingAndInitialization(); launchLongRunAndReset(); reverseAndConnectorBalance(); profilesAndMechanisms();
        std::cout << scenarios << " controlled scenarios / " << checks << " numerical and integration checks passed\n";
    }
    catch (const std::exception& e) { std::cerr << "MultiCarPropulsionTests: " << e.what() << '\n'; return 1; }
}
