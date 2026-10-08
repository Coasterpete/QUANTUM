#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RideHardware.hpp>

namespace quantum::coaster
{
    enum class TrackDeviceKind : std::uint8_t
    {
        Launch,
        Brake,
        Transport,
        ChainLift,
        CableLift,
        Station,
        Block,
        Transfer,
        Access
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
        HardwareConfiguration configuration;

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
                && a.accelerationProfile == b.accelerationProfile
                && a.configuration == b.configuration;
        }
    };

    struct TrackDeviceCollection
    {
        TrackDeviceId nextId = 1;
        std::vector<TrackDevice> devices;
        std::uint64_t nextInfrastructureId = 1;
        std::vector<TrackInfrastructure> infrastructure;

        [[nodiscard]] friend bool operator==(
            const TrackDeviceCollection&, const TrackDeviceCollection&) = default;
    };

    [[nodiscard]] const char* trackDeviceKindName(TrackDeviceKind kind);
    [[nodiscard]] TrackDeviceKind trackDeviceKindFromName(const std::string& name);
    [[nodiscard]] TrackDevice makeTrackDevice(TrackDeviceKind kind);
    [[nodiscard]] bool supportsAccelerationProfile(TrackDeviceKind kind) noexcept;
    void validateRideHardware(const TrackDeviceCollection& collection,
        double trackLengthMeters, bool circuit);

    // Half-open [start,end) intervals. Circuit start > end wraps through
    // station zero; start == end is invalid. Open tracks require start < end
    // and permit end == length. A circuit endpoint must be < length.
    void validateTrackDevices(const TrackDeviceCollection& collection,
        double trackLengthMeters, bool circuit);

    [[nodiscard]] bool trackDeviceContainsStation(const TrackDevice& device,
        double stationMeters, double trackLengthMeters, bool circuit) noexcept;

    // Travel length of one device along its own interval. A circuit device
    // with start > end wraps through station zero, so its length is
    // trackLengthMeters - start + end rather than end - start. This is the
    // single authority for the device-local distance domain that an authored
    // acceleration profile is validated and evaluated against.
    [[nodiscard]] double trackDeviceLengthMeters(const TrackDevice& device,
        double trackLengthMeters, bool circuit) noexcept;
}
