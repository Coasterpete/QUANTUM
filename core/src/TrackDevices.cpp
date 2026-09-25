#include <quantum/coaster/TrackDevices.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace quantum::coaster
{
    void validateTrackDevices(const TrackDeviceCollection& collection,
        const double trackLengthMeters, const bool circuit)
    {
        if (collection.nextId == 0)
            throw std::invalid_argument("Track device next ID must be nonzero.");
        // Legacy construction paths temporarily hold zero sections. An empty
        // collection imposes no station-domain constraint on that state.
        if (collection.devices.empty())
            return;
        if (!std::isfinite(trackLengthMeters) || trackLengthMeters <= 0.0)
            throw std::invalid_argument("Track devices require a positive finite track length.");

        std::unordered_set<TrackDeviceId> ids;
        TrackDeviceId greatestId = 0;
        for (const TrackDevice& device : collection.devices)
        {
            if (device.id == 0 || !ids.insert(device.id).second)
                throw std::invalid_argument("Track device IDs must be nonzero and unique.");
            greatestId = std::max(greatestId, device.id);
            if (device.kind != TrackDeviceKind::Launch
                && device.kind != TrackDeviceKind::Brake)
                throw std::invalid_argument("Unknown track device kind.");
            if (device.name.empty())
                throw std::invalid_argument("Track device name must not be empty.");
            if (!std::isfinite(device.startStationMeters)
                || !std::isfinite(device.endStationMeters)
                || device.startStationMeters < 0.0
                || device.endStationMeters < 0.0
                || device.startStationMeters >= trackLengthMeters
                || (circuit ? device.endStationMeters >= trackLengthMeters
                    : device.endStationMeters > trackLengthMeters)
                || device.startStationMeters == device.endStationMeters
                || (!circuit && device.startStationMeters > device.endStationMeters))
                throw std::invalid_argument("Track device station interval is outside the track or empty.");
            if (!std::isfinite(device.targetAccelerationMetersPerSecondSquared)
                || device.targetAccelerationMetersPerSecondSquared <= 0.0
                || !std::isfinite(device.maximumForceNewtons)
                || device.maximumForceNewtons <= 0.0)
                throw std::invalid_argument("Track device acceleration and force limit must be finite and positive.");

            // Validate optional acceleration profile.
            if (device.accelerationProfile.has_value())
            {
                const double deviceLength = device.endStationMeters - device.startStationMeters;
                if (!circuit && device.startStationMeters > device.endStationMeters)
                {
                    // Wrapping device on circuit track - length computed differently.
                    // For validation we just need a positive device length.
                }
                const double effectiveDeviceLength = circuit && device.startStationMeters > device.endStationMeters
                    ? trackLengthMeters - device.startStationMeters + device.endStationMeters
                    : device.endStationMeters - device.startStationMeters;
                if (effectiveDeviceLength <= 0.0)
                    throw std::invalid_argument("Track device with acceleration profile must have positive length.");
                validateChannelProfile(device.accelerationProfile.value(), effectiveDeviceLength);
                // Profile values must be non-negative (commanded acceleration magnitude).
                for (const ProfileSegment& segment : device.accelerationProfile->segments)
                {
                    if (segment.transition.valueBegin < 0.0
                        || segment.transition.valueEnd < 0.0)
                    {
                        throw std::invalid_argument("Acceleration profile values must be non-negative.");
                    }
                }
            }
        }
        if (collection.nextId <= greatestId)
            throw std::invalid_argument("Track device next ID must exceed every allocated ID.");
    }

    bool trackDeviceContainsStation(const TrackDevice& device,
        const double stationMeters, const double trackLengthMeters,
        const bool circuit) noexcept
    {
        if (!std::isfinite(stationMeters) || stationMeters < 0.0
            || stationMeters > trackLengthMeters)
            return false;
        const double station = circuit && stationMeters == trackLengthMeters
            ? 0.0 : stationMeters;
        if (circuit && device.startStationMeters > device.endStationMeters)
            return station >= device.startStationMeters
                || station < device.endStationMeters;
        return station >= device.startStationMeters
            && (station < device.endStationMeters
                || (!circuit && station == trackLengthMeters
                    && device.endStationMeters == trackLengthMeters));
    }
}
