#include <quantum/physics/TrackDeviceForces.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quantum::physics
{
    std::vector<TrackDeviceRuntimeState> initialTrackDeviceRuntimeStates(
        const std::span<const coaster::TrackDevice> devices)
    {
        std::vector<TrackDeviceRuntimeState> result;
        result.reserve(devices.size());
        for (const auto& device : devices)
            result.push_back({device.id, device.enabled});
        return result;
    }

    // Computes the device-local distance of a station along the device's interval.
    // For circuit-wrapping devices (start > end), the distance wraps through zero.
    static double deviceLocalDistance(const coaster::TrackDevice& device,
        double stationMeters, double trackLengthMeters, bool circuit) noexcept
    {
        if (!circuit || device.startStationMeters <= device.endStationMeters)
        {
            return stationMeters - device.startStationMeters;
        }
        // Circuit-wrapping device: [start, length) U [0, end)
        if (stationMeters >= device.startStationMeters)
        {
            return stationMeters - device.startStationMeters;
        }
        return trackLengthMeters - device.startStationMeters + stationMeters;
    }

    TrackDeviceForceResult evaluateTrackDeviceForces(
        const std::span<const coaster::TrackDevice> devices,
        const std::span<const TrackDeviceRuntimeState> runtimeStates,
        const CompiledPhysicsTrack& track,
        const TrainDefinition& train,
        const TrainPose& pose,
        const double signedVelocityMetersPerSecond)
    {
        if (!std::isfinite(signedVelocityMetersPerSecond)
            || pose.carCount() != train.cars.size()
            || devices.size() != runtimeStates.size())
            throw std::invalid_argument("Device force evaluation requires a matching train pose and finite speed.");

        TrackDeviceForceResult result;
        const bool circuit = track.topology() == coaster::TopologyKind::ClosedCircuit;
        for (std::size_t deviceIndex = 0;
            deviceIndex < devices.size(); ++deviceIndex)
        {
            const coaster::TrackDevice& device = devices[deviceIndex];
            const TrackDeviceRuntimeState& control =
                runtimeStates[deviceIndex];
            if (control.id != device.id)
                throw std::invalid_argument(
                    "Device runtime control does not match authored IDs.");
            TrackDeviceForceTelemetry telemetry;
            telemetry.id = device.id;
            if (!control.armed
                || (device.kind == coaster::TrackDeviceKind::Brake
                && std::abs(signedVelocityMetersPerSecond)
                    <= followerRestSpeedToleranceMetersPerSecond))
            {
                result.devices.push_back(telemetry);
                continue;
            }

            struct OccupiedBogie
            {
                std::size_t carIndex;
                const BogiePose* pose;
                glm::dvec3 localPoint;
                double requestedForce;
            };
            std::vector<OccupiedBogie> occupied;
            double requestedTotal = 0.0;
            // A custom profile commands a different acceleration at every
            // bogie, so telemetry reports the mean command in effect across
            // the occupied bogies. Requested force is proportional to the sum
            // of those commands, so the mean is the value that explains the
            // applied force. For a constant command the mean is exactly the
            // authored value.
            double commandedAccelerationTotal = 0.0;
            for (std::size_t carIndex = 0; carIndex < pose.carCount(); ++carIndex)
            {
                const TrainCarPose& car = pose.cars()[carIndex];
                const CarDefinition& definition = train.cars[carIndex].car;
                for (const BogiePose* bogie : {
                    &car.carPose().frontBogie(),
                    &car.carPose().rearBogie()})
                {
                    if (!coaster::trackDeviceContainsStation(device,
                        bogie->location().stationMeters,
                        track.lengthMeters(), circuit))
                        continue;

                    double commandedAccel = device.targetAccelerationMetersPerSecondSquared;
                    if (device.accelerationProfile.has_value())
                    {
                        const double localDist = deviceLocalDistance(device,
                            bogie->location().stationMeters,
                            track.lengthMeters(), circuit);
                        // Validation guarantees the profile covers the device
                        // length exactly, so clamping keeps the query inside
                        // the authored domain.
                        const double deviceLength = coaster::trackDeviceLengthMeters(
                            device, track.lengthMeters(), circuit);
                        const double clampedDist = std::clamp(localDist, 0.0, deviceLength);
                        commandedAccel = coaster::evaluateChannelProfile(
                            device.accelerationProfile.value(), clampedDist);
                    }

                    const double requested = 0.5
                        * car.loadedMassKilograms()
                        * commandedAccel;
                    occupied.push_back({carIndex, bogie,
                        definition.bogies[bogie->definitionIndex()]
                            .referencePositionMeters,
                        requested});
                    requestedTotal += requested;
                    commandedAccelerationTotal += commandedAccel;
                }
            }
            telemetry.occupiedBogieCount = occupied.size();
            if (occupied.empty())
            {
                result.devices.push_back(telemetry);
                continue;
            }
            const double applied = std::min(requestedTotal,
                device.maximumForceNewtons);
            const double direction = device.kind == coaster::TrackDeviceKind::Launch
                ? 1.0
                : (signedVelocityMetersPerSecond > 0.0 ? -1.0 : 1.0);
            const double scale = applied / requestedTotal;
            for (const OccupiedBogie& bogie : occupied)
            {
                result.applications.push_back({bogie.carIndex,
                    bogie.localPoint,
                    direction * scale * bogie.requestedForce
                        * bogie.pose->trackFrame().tangent});
            }
            telemetry.commandedAccelerationMetersPerSecondSquared =
                commandedAccelerationTotal / static_cast<double>(occupied.size());
            telemetry.appliedForceNewtons = direction * applied;
            result.devices.push_back(telemetry);
        }
        return result;
    }
}
