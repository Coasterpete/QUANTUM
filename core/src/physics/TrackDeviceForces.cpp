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
        const double signedVelocityMetersPerSecond,
        const double gravityAccelerationMetersPerSecondSquared)
    {
        if (!std::isfinite(gravityAccelerationMetersPerSecondSquared)
            || gravityAccelerationMetersPerSecondSquared < 0
            || !std::isfinite(signedVelocityMetersPerSecond)
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
            telemetry.armed = control.armed;
            telemetry.applicationBegin = result.applications.size();
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
            double profileAccelerationTotal = 0.0;
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

                    ++telemetry.occupiedBogieCount;
                    const bool acceptsForceCommand = control.armed
                        && (control.signedAccelerationCommand || device.kind <= coaster::TrackDeviceKind::Brake)
                        && (control.signedAccelerationCommand || device.kind != coaster::TrackDeviceKind::Brake
                            || std::abs(signedVelocityMetersPerSecond) > followerRestSpeedToleranceMetersPerSecond);
                    if (!acceptsForceCommand && device.kind != coaster::TrackDeviceKind::Launch) continue;
                    double commandedAccel = device.targetAccelerationMetersPerSecondSquared;
                    if (!control.signedAccelerationCommand && device.accelerationProfile.has_value())
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

                    if (device.kind == coaster::TrackDeviceKind::Launch)
                        profileAccelerationTotal += commandedAccel;
                    if (!acceptsForceCommand) continue;

                    if (!control.signedAccelerationCommand && device.kind == coaster::TrackDeviceKind::Brake)
                    {
                        if (const auto* brake = std::get_if<coaster::BrakeConfiguration>(&device.configuration))
                            commandedAccel = std::min(commandedAccel, std::max(0.0,
                                std::abs(signedVelocityMetersPerSecond) - brake->targetSpeedMetersPerSecond) / defaultFixedTimeStepSeconds);
                    }
                    if (!control.signedAccelerationCommand && device.kind == coaster::TrackDeviceKind::Launch)
                    {
                        if (const auto* launch = std::get_if<coaster::LaunchConfiguration>(&device.configuration);
                            launch && launch->targetSpeedMetersPerSecond)
                        {
                            const double limitedAccel = std::min(commandedAccel, std::max(0.0,
                                *launch->targetSpeedMetersPerSecond - signedVelocityMetersPerSecond * static_cast<int>(launch->direction)) / defaultFixedTimeStepSeconds);
                            telemetry.targetSpeedLimited |= limitedAccel < commandedAccel;
                            commandedAccel = limitedAccel;
                        }
                    }
                    if (control.signedAccelerationCommand) commandedAccel = *control.signedAccelerationCommand;
                    else if (device.kind == coaster::TrackDeviceKind::Brake)
                        commandedAccel *= signedVelocityMetersPerSecond > 0 ? -1 : 1;
                    else if (const auto* launch = std::get_if<coaster::LaunchConfiguration>(&device.configuration))
                        commandedAccel *= static_cast<int>(launch->direction);
                    if (control.compensateGravity)
                        commandedAccel += gravityAccelerationMetersPerSecondSquared * bogie->trackFrame().tangent.z;
                    const double requested = 0.5
                        * car.loadedMassKilograms()
                        * commandedAccel;
                    occupied.push_back({carIndex, bogie,
                        definition.bogies[bogie->definitionIndex()]
                            .referencePositionMeters,
                        requested});
                    requestedTotal += std::abs(requested);
                    telemetry.requestedForceNewtons += requested;
                    commandedAccelerationTotal += commandedAccel;
                }
            }
            if (telemetry.occupiedBogieCount > 0)
                telemetry.profileAccelerationMetersPerSecondSquared =
                    profileAccelerationTotal / static_cast<double>(telemetry.occupiedBogieCount);
            if (occupied.empty())
            {
                result.devices.push_back(telemetry);
                continue;
            }
            const double applied = std::min(requestedTotal,
                device.maximumForceNewtons);
            telemetry.requestedAbsoluteForceNewtons = requestedTotal;
            telemetry.suppliedAbsoluteForceNewtons = applied;
            telemetry.forceLimitReached = requestedTotal > device.maximumForceNewtons;
            telemetry.applicationCount = occupied.size();
            const double scale = requestedTotal > 0 ? applied / requestedTotal : 0;
            double signedApplied = 0;
            for (const OccupiedBogie& bogie : occupied)
            {
                result.applications.push_back({bogie.carIndex,
                    bogie.localPoint,
                    scale * bogie.requestedForce
                        * bogie.pose->trackFrame().tangent});
                signedApplied += scale * bogie.requestedForce;
            }
            telemetry.commandedAccelerationMetersPerSecondSquared =
                std::abs(commandedAccelerationTotal) / static_cast<double>(occupied.size());
            telemetry.appliedForceNewtons = signedApplied;
            result.devices.push_back(telemetry);
        }
        return result;
    }
}
