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
 
    void accelerationProfileTests()
    {
        using namespace quantum;
        using namespace quantum::coaster;
        using namespace quantum::physics;
        using namespace quantum::math;
 
        // Use the same track as the original test to ensure compatibility.
        const CompiledPhysicsTrack track = straightPhysicsTrack();
        const TrainDefinition train = coaster::resolveTrainConfiguration(
            coaster::createDefaultTrainConfiguration());
 
        // All tests use the same train position as the original test (14.75)
        // to ensure the train fits within the track.
        const double testStation = 14.75;
 
        // Test 1: Constant acceleration (no profile) matches legacy behavior.
        {
            const TrackLocation location{primaryTrackPathId, testStation,
                TravelDirection::IncreasingStation};
            const TrainPose pose = solveTrainPose(track, train, location);
 
            TrackDevice launch;
            launch.id = 1;
            launch.name = "Launch";
            launch.startStationMeters = 0.0;
            launch.endStationMeters = 20.0;
            launch.targetAccelerationMetersPerSecondSquared = 4.0;
            launch.maximumForceNewtons = 5000.0;
            // No acceleration profile - uses constant acceleration.
 
            const auto controls = initialTrackDeviceRuntimeStates(
                std::span<const TrackDevice>(&launch, 1));
            const auto forces = evaluateTrackDeviceForces(
                std::span<const TrackDevice>(&launch, 1),
                controls, track, train, pose, 5.0);
 
            // Both bogies in the device should get the same commanded acceleration.
            require(forces.devices[0].commandedAccelerationMetersPerSecondSquared == 4.0,
                "constant acceleration matches target");
        }
 
        // Test 2: Custom acceleration profile with ramp-up.
        {
            const TrackLocation location{primaryTrackPathId, testStation,
                TravelDirection::IncreasingStation};
            const TrainPose pose = solveTrainPose(track, train, location);
 
            TrackDevice launch;
            launch.id = 1;
            launch.name = "Launch";
            launch.startStationMeters = 0.0;
            launch.endStationMeters = 20.0;
            launch.targetAccelerationMetersPerSecondSquared = 4.0; // Fallback
            launch.maximumForceNewtons = 5000.0;
 
            // Create a ramp-up profile: 0 -> 4 m/s^2 over [0, 20]
            launch.accelerationProfile = ChannelProfile{};
            launch.accelerationProfile->segments.push_back(ProfileSegment{
                1,
                ScalarTransition{
                    .domainBegin = 0.0,
                    .domainEnd = 20.0,
                    .valueBegin = 0.0,
                    .valueEnd = 4.0,
                    .transitionType = TransitionType::Linear
                }
            });
            launch.accelerationProfile->nextSegmentId = 2;
 
            const auto controls = initialTrackDeviceRuntimeStates(
                std::span<const TrackDevice>(&launch, 1));
            const auto forces = evaluateTrackDeviceForces(
                std::span<const TrackDevice>(&launch, 1),
                controls, track, train, pose, 5.0);
 
            // At testStation, acceleration should be > 0.
            require(forces.devices[0].commandedAccelerationMetersPerSecondSquared > 0.0,
                "profile evaluation gives positive acceleration");
        }
 
        // Test 3: Multi-segment profile (ramp-up, sustain, ramp-down).
        {
            const TrackLocation location{primaryTrackPathId, testStation,
                TravelDirection::IncreasingStation};
            const TrainPose pose = solveTrainPose(track, train, location);
 
            TrackDevice launch;
            launch.id = 1;
            launch.name = "Launch";
            launch.startStationMeters = 0.0;
            launch.endStationMeters = 30.0;
            launch.targetAccelerationMetersPerSecondSquared = 5.0;
            launch.maximumForceNewtons = 10000.0;
 
            // Three segments: ramp 0->5 over [0,10], sustain 5 over [10,20], ramp 5->0 over [20,30]
            launch.accelerationProfile = ChannelProfile{};
            launch.accelerationProfile->segments.push_back(ProfileSegment{
                1,
                ScalarTransition{0.0, 10.0, 0.0, 5.0, TransitionType::Linear}
            });
            launch.accelerationProfile->segments.push_back(ProfileSegment{
                2,
                ScalarTransition{10.0, 20.0, 5.0, 5.0, TransitionType::Linear}
            });
            launch.accelerationProfile->segments.push_back(ProfileSegment{
                3,
                ScalarTransition{20.0, 30.0, 5.0, 0.0, TransitionType::Linear}
            });
            launch.accelerationProfile->nextSegmentId = 4;
 
            const auto controls = initialTrackDeviceRuntimeStates(
                std::span<const TrackDevice>(&launch, 1));
 
            // At testStation (14.75): sustain phase (value = 5.0)
            {
                const auto forces = evaluateTrackDeviceForces(
                    std::span<const TrackDevice>(&launch, 1), controls,
                    track, train, pose, 5.0);
                require(forces.devices[0].commandedAccelerationMetersPerSecondSquared > 4.9
                        && forces.devices[0].commandedAccelerationMetersPerSecondSquared < 5.1,
                    "sustain at 14.75 gives ~5.0 m/s^2");
            }
        }
 
        // Test 4: Brake with profile - force opposes travel.
        {
            const TrackLocation location{primaryTrackPathId, testStation,
                TravelDirection::IncreasingStation};
            const TrainPose pose = solveTrainPose(track, train, location);
 
            TrackDevice brake;
            brake.id = 1;
            brake.kind = TrackDeviceKind::Brake;
            brake.name = "Brake";
            brake.startStationMeters = 0.0;
            brake.endStationMeters = 20.0;
            brake.maximumForceNewtons = 5000.0;
            brake.accelerationProfile = ChannelProfile{};
            brake.accelerationProfile->segments.push_back(ProfileSegment{
                1,
                ScalarTransition{0.0, 20.0, 2.0, 4.0, TransitionType::Linear}
            });
            brake.accelerationProfile->nextSegmentId = 2;
 
            const auto controls = initialTrackDeviceRuntimeStates(
                std::span<const TrackDevice>(&brake, 1));
            const auto forces = evaluateTrackDeviceForces(
                std::span<const TrackDevice>(&brake, 1), controls,
                track, train, pose, 8.0);
 
            // Brake force should be negative (opposing forward travel).
            require(forces.devices[0].appliedForceNewtons < 0.0,
                "brake with profile opposes forward travel");
            // At testStation, profile value should be between 2 and 4.
            require(forces.devices[0].commandedAccelerationMetersPerSecondSquared > 2.0
                    && forces.devices[0].commandedAccelerationMetersPerSecondSquared < 4.0,
                "brake profile evaluation gives value in (2, 4)");
        }
 
        // Test 5: Profile serialization round-trip.
        {
            AuthoredTrack document = createNewDocument();
            document.setLayoutMode(LayoutMode::Shuttle);
 
            TrackDevice launch;
            launch.name = "LSM Profile";
            launch.startStationMeters = 0.0;
            launch.endStationMeters = 20.0;
            launch.targetAccelerationMetersPerSecondSquared = 3.0;
            launch.maximumForceNewtons = 5000.0;
            launch.accelerationProfile = ChannelProfile{};
            launch.accelerationProfile->segments.push_back(ProfileSegment{
                1,
                ScalarTransition{0.0, 20.0, 1.0, 5.0, TransitionType::Smoothstep}
            });
            launch.accelerationProfile->nextSegmentId = 2;
 
            const TrackDeviceId launchId = document.addTrackDevice(launch);
 
            const std::string json = serializeCoasterDocument(document);
            const auto reopened = deserializeCoasterDocument(json);
            require(reopened.has_value(), "profile document must reopen");
 
            const auto& reopenedDevice = reopened->trackDevices().devices[0];
            require(reopenedDevice.accelerationProfile.has_value(),
                "profile must be present after round-trip");
            require(reopenedDevice.accelerationProfile->segments.size() == 1,
                "profile segment count preserved");
            const auto& seg = reopenedDevice.accelerationProfile->segments[0];
            require(seg.transition.valueBegin == 1.0 && seg.transition.valueEnd == 5.0,
                "profile values preserved");
            require(seg.transition.transitionType == TransitionType::Smoothstep,
                "transition type preserved");
        }
 
        // Test 6: Legacy document without profile still works.
        {
            const std::string legacyJson = R"({
                "formatVersion": 1,
                "layoutMode": "Shuttle",
                "startPose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}},
                "physicalSettings": {"initialSpeed": 0, "metersPerCoordinateUnit": 1, "gravityAcceleration": 9.81},
                "trackStyle": {"name": "Test", "geometryFamily": "DualRailTubular", "visible": true, "railsVisible": true, "railCount": 2, "railOffsets": [{"lateral": -1, "vertical": 0}, {"lateral": 1, "vertical": 0}], "railRadius": 0.1, "railRadialSegments": 8, "railMaterial": {"baseColor": {"r": 1, "g": 1, "b": 1, "a": 1}, "metallicFactor": 1, "roughnessFactor": 0.5}, "spine": {"enabled": true, "type": "Tubular", "offset": {"lateral": 0, "vertical": 0}, "dimensions": {"x": 0.2, "y": 0.2}, "radialSegments": 8, "material": {"baseColor": {"r": 0.5, "g": 0.5, "b": 0.5, "a": 1}, "metallicFactor": 1, "roughnessFactor": 0.5}}, "repeatingHardware": []},
                "coasterSetup": {"styleId": "custom", "carsPerTrain": 2, "options": [{"optionId": "train-layout", "choice": "layout-1"}, {"optionId": "launch", "boolean": false}, {"optionId": "restraint", "choice": "restraint-option-1"}], "heartline": {"enabled": false, "offsetMeters": 0}},
                "supports": {"nextStructureId": 1, "structures": []},
                "trackDevices": {"nextId": 2, "devices": [{"id": 1, "kind": "Launch", "name": "Launch", "enabled": true, "startStationMeters": 0, "endStationMeters": 10, "targetAccelerationMetersPerSecondSquared": 3, "maximumForceNewtons": 5000}]},
                "sections": [{"kind": "RateProfiles", "length": 100, "rateProfiles": {"pitch": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}, "yaw": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}, "roll": {"nextSegmentId": 2, "segments": [{"id": 1, "transition": {"domainBegin": 0, "domainEnd": 100, "valueBegin": 0, "valueEnd": 0, "type": "Linear"}}]}}}]
            })";
 
            const auto document = deserializeCoasterDocument(legacyJson);
            if (!document.has_value()) {
                std::cerr << "Deserialize error: " << document.error() << std::endl;
            }
            require(document.has_value(), "legacy document without profile loads");
            require(document->trackDevices().devices.size() == 1,
                "legacy device loads without profile");
            require(!document->trackDevices().devices[0].accelerationProfile.has_value(),
                "legacy device has no profile");
        }
    }
}
 
int main()
{
    try
    {
        validationAndPersistence();
        forceOccupancyAndDynamics();
        accelerationProfileTests();
        std::cout << "TrackDevicesTests passed\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "TrackDevicesTests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
