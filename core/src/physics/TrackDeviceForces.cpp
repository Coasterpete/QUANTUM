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
                    const double requested = 0.5
                        * car.loadedMassKilograms()
                        * device.targetAccelerationMetersPerSecondSquared;
                    occupied.push_back({carIndex, bogie,
                        definition.bogies[bogie->definitionIndex()]
                            .referencePositionMeters,
                        requested});
                    requestedTotal += requested;
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
                device.targetAccelerationMetersPerSecondSquared;
            telemetry.appliedForceNewtons = direction * applied;
            result.devices.push_back(telemetry);
        }
        return result;
    }
}
