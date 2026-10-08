#pragma once
#include <quantum/coaster/TrackDevices.hpp>
#include <string_view>
namespace quantum::coaster
{
    [[nodiscard]] std::string serializeRideHardwareConfiguration(const TrackDevice& device);
    [[nodiscard]] HardwareConfiguration deserializeRideHardwareConfiguration(TrackDeviceKind kind, std::string_view text);
    [[nodiscard]] std::string serializeTrackInfrastructure(const std::vector<TrackInfrastructure>& items);
    [[nodiscard]] std::vector<TrackInfrastructure> deserializeTrackInfrastructure(std::string_view text);
}
