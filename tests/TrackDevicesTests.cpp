#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/TrainConfiguration.hpp>
#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/TrackDeviceForces.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    template<typename Function>
    void rejects(Function&& function)
    {
        bool rejected = false;
        try { function(); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid device edit was accepted");
    }

    quantum::physics::CompiledPhysicsTrack straightPhysicsTrack()
    {
        const quantum::geometry::CurveFrame frame{
            {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
        const std::vector<quantum::coaster::TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}},
            {100.0, {100.0, 0.0, 0.0}, frame, {0.0, 0.0, 0.0}}
        };
        return {samples, 1.0, quantum::coaster::TopologyKind::OpenLinear};
    }

    void validationAndPersistence()
    {
        using namespace quantum::coaster;
        AuthoredTrack document = createNewDocument();
        document.setLayoutMode(LayoutMode::Shuttle);
        TrackDevice launch;
        launch.name = "LSM A";
        launch.startStationMeters = 0.0;
        launch.endStationMeters = 20.0;
        const TrackDeviceId launchId = document.addTrackDevice(launch);
        TrackDevice brake;
        brake.name = "Brake A";
        brake.kind = TrackDeviceKind::Brake;
        brake.startStationMeters = 20.0;
        brake.endStationMeters = document.trackLengthMeters();
        const TrackDeviceId brakeId = document.addTrackDevice(brake);
        require(brakeId > launchId, "device IDs must increase");
        const auto committed = document.trackDevices();
        auto invalid = brake;
        invalid.id = brakeId;
        invalid.endStationMeters = document.trackLengthMeters() + 1.0;
        rejects([&] { document.updateTrackDevice(invalid); });
        require(document.trackDevices() == committed,
            "rejected edit must preserve committed devices");
        invalid.endStationMeters = invalid.startStationMeters;
        rejects([&] { document.updateTrackDevice(invalid); });
        const std::string json = serializeCoasterDocument(document);
        const auto reopened = deserializeCoasterDocument(json);
        require(reopened.has_value(), "device document must reopen");
        require(reopened->trackDevices() == committed,
            "device collection must round trip");
        document.removeTrackDevice(launchId);
        require(document.trackDevices().nextId == committed.nextId,
            "deleted IDs must not be reused");
        rejects([&] { document.removeTrackDevice(launchId); });

        auto legacy = nlohmann::json::parse(json);
        legacy.erase("trackDevices");
        const auto oldDocument = deserializeCoasterDocument(legacy.dump());
        require(oldDocument.has_value()
                && oldDocument->trackDevices().devices.empty(),
            "legacy document must load with no devices");

        auto negativeId = nlohmann::json::parse(json);
        negativeId["trackDevices"]["devices"][0]["id"] = -1;
        require(!deserializeCoasterDocument(negativeId.dump()).has_value(),
            "negative device ID must be rejected");
        negativeId = nlohmann::json::parse(json);
        negativeId["trackDevices"]["nextId"] = -1;
        require(!deserializeCoasterDocument(negativeId.dump()).has_value(),
            "negative next device ID must be rejected");

        TrackDeviceCollection circuit;
        TrackDevice seam = launch;
        seam.id = 1;
        seam.startStationMeters = 55.0;
        seam.endStationMeters = 5.0;
        circuit.devices.push_back(seam);
        circuit.nextId = 2;
        validateTrackDevices(circuit, 60.0, true);
        require(trackDeviceContainsStation(seam, 59.0, 60.0, true)
                && trackDeviceContainsStation(seam, 0.0, 60.0, true)
                && !trackDeviceContainsStation(seam, 5.0, 60.0, true),
            "circuit interval must wrap with half-open endpoints");
        rejects([&] { validateTrackDevices(circuit, 60.0, false); });
        require(trackDeviceContainsStation(brake,
                document.trackLengthMeters(),
                document.trackLengthMeters(), false),
            "open physical endpoint belongs to an interval ending there");
    }

    void forceOccupancyAndDynamics()
    {
        using namespace quantum;
        using namespace quantum::physics;
        const CompiledPhysicsTrack track = straightPhysicsTrack();
        const TrainDefinition train = coaster::resolveTrainConfiguration(
            coaster::createDefaultTrainConfiguration());
        const TrackLocation location{primaryTrackPathId, 14.75,
            TravelDirection::IncreasingStation};
        const TrainPose pose = solveTrainPose(track, train, location);
        coaster::TrackDevice launch;
        launch.id = 1;
        launch.name = "Launch";
        launch.startStationMeters = 0.0;
        launch.endStationMeters = 10.0;
        launch.targetAccelerationMetersPerSecondSquared = 4.0;
        launch.maximumForceNewtons = 5000.0;
        coaster::TrackDevice brake = launch;
        brake.id = 2;
        brake.name = "Brake";
        brake.kind = coaster::TrackDeviceKind::Brake;
        brake.startStationMeters = 20.0;
        brake.endStationMeters = 50.0;
        const std::vector devices{launch, brake};
        const auto controls = initialTrackDeviceRuntimeStates(devices);
        const auto forces = evaluateTrackDeviceForces(devices, controls,
            track, train, pose, 5.0);
        require(forces.devices[0].occupiedBogieCount > 0
                && forces.devices[0].occupiedBogieCount < 2 * pose.carCount(),
            "partly entered train must have partial bogie occupancy");
        require(forces.devices[0].appliedForceNewtons <= 5000.0,
            "launch must obey whole-device force cap");
        require(forces.applications.size()
                == forces.devices[0].occupiedBogieCount,
            "each occupied bogie must produce one per-car application");
        const auto disabled = [&]
        {
            auto copy = launch;
            copy.enabled = false;
            const auto state = initialTrackDeviceRuntimeStates(
                std::span<const coaster::TrackDevice>(&copy, 1));
            return evaluateTrackDeviceForces(
                std::span<const coaster::TrackDevice>(&copy, 1),
                state,
                track, train, pose, 5.0);
        }();
        require(disabled.applications.empty(),
            "disabled launch must apply no force");

        TrainDynamicsState initial;
        initial.generalizedReferenceLocation = location;
        initial.signedVelocityMetersPerSecond = 5.0;
        initial.runState = FollowerRunState::Running;
        const auto baseline = stepTrain(track, train, {}, initial);
        const auto launched = stepTrain(track, train, {}, initial,
            {}, forces.applications);
        require(launched.state.signedVelocityMetersPerSecond
                > baseline.state.signedVelocityMetersPerSecond,
            "launch must accelerate through stepTrain");
        const auto repeated = stepTrain(track, train, {}, initial,
            {}, forces.applications);
        require(repeated.state.signedVelocityMetersPerSecond
                == launched.state.signedVelocityMetersPerSecond,
            "same fixed step must be deterministic");

        const TrackLocation brakeLocation{primaryTrackPathId, 35.0,
            TravelDirection::IncreasingStation};
        const TrainPose brakePose = solveTrainPose(track, train,
            brakeLocation);
        TrainDynamicsState moving = initial;
        moving.generalizedReferenceLocation = brakeLocation;
        moving.signedVelocityMetersPerSecond = 8.0;
        const auto braking = evaluateTrackDeviceForces(
            std::span<const coaster::TrackDevice>(&brake, 1),
            initialTrackDeviceRuntimeStates(
                std::span<const coaster::TrackDevice>(&brake, 1)),
            track, train, brakePose, moving.signedVelocityMetersPerSecond);
        require(braking.devices.front().appliedForceNewtons < 0.0,
            "forward brake must oppose travel");
        const auto coast = stepTrain(track, train, {}, moving);
        const auto slowed = stepTrain(track, train, {}, moving,
            {}, braking.applications);
        require(slowed.state.signedVelocityMetersPerSecond
                < coast.state.signedVelocityMetersPerSecond,
            "brake must decelerate through stepTrain");
        const auto reverse = evaluateTrackDeviceForces(
            std::span<const coaster::TrackDevice>(&brake, 1),
            initialTrackDeviceRuntimeStates(
                std::span<const coaster::TrackDevice>(&brake, 1)),
            track, train, brakePose, -8.0);
        require(reverse.devices.front().appliedForceNewtons > 0.0,
            "reverse brake must oppose reverse travel");
        const auto rest = evaluateTrackDeviceForces(
            std::span<const coaster::TrackDevice>(&brake, 1),
            initialTrackDeviceRuntimeStates(
                std::span<const coaster::TrackDevice>(&brake, 1)),
            track, train, brakePose, 0.0);
        require(rest.applications.empty(), "brake must be inactive at rest");
    }
}

int main()
{
    try
    {
        validationAndPersistence();
        forceOccupancyAndDynamics();
        std::cout << "TrackDevicesTests passed\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "TrackDevicesTests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
