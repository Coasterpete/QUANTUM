#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>

#include <quantum/coaster/GeometricSection.hpp>

namespace quantum::coaster
{
    using TrackDeviceId = std::uint64_t;

    enum class TrackDeviceKind : std::uint8_t
    {
        Launch,
        Brake
    };

    // Authored, renderer-neutral physical command. Stations are SI metres
    // along the complete track, independent of geometry-region boundaries.
    struct TrackDevice
    {
        TrackDeviceId id = 0;
        TrackDeviceKind kind = TrackDeviceKind::Launch;
        std::string name;
        bool enabled = true;
        double startStationMeters = 0.0;
        double endStationMeters = 0.0;
        double targetAccelerationMetersPerSecondSquared = 1.0;
        double maximumForceNewtons = 10000.0;

        // Optional custom acceleration profile over the device-local
        // distance domain [0, deviceLength]. When present, this profile
        // replaces the constant targetAccelerationMetersPerSecondSquared.
        // The profile values represent commanded acceleration magnitude
        // (always >= 0). For brakes, the physical force opposes travel.
        std::optional<ChannelProfile> accelerationProfile;

        [[nodiscard]] friend bool operator==(
            const TrackDevice& a, const TrackDevice& b)
        {
            return a.id == b.id
                && a.kind == b.kind
                && a.name == b.name
                && a.enabled == b.enabled
                && a.startStationMeters == b.startStationMeters
                && a.endStationMeters == b.endStationMeters
                && a.targetAccelerationMetersPerSecondSquared == b.targetAccelerationMetersPerSecondSquared
                && a.maximumForceNewtons == b.maximumForceNewtons
                && a.accelerationProfile == b.accelerationProfile;
        }
    };

    struct TrackDeviceCollection
    {
        TrackDeviceId nextId = 1;
        std::vector<TrackDevice> devices;

        [[nodiscard]] friend bool operator==(
            const TrackDeviceCollection&, const TrackDeviceCollection&) = default;
    };

    // Half-open [start,end) intervals. Circuit start > end wraps through
    // station zero; start == end is invalid. Open tracks require start < end
    // and permit end == length. A circuit endpoint must be < length.
    void validateTrackDevices(const TrackDeviceCollection& collection,
        double trackLengthMeters, bool circuit);

    [[nodiscard]] bool trackDeviceContainsStation(const TrackDevice& device,
        double stationMeters, double trackLengthMeters, bool circuit) noexcept;
}
