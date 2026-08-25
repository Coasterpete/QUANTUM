#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/GeometricSection.hpp>

#include <cstddef>

namespace quantum::editor
{
    // Station positions of one region inside the authored track ordering.
    // Stations are prefix sums of authored section lengths and therefore
    // match the cumulative distances of whole-track integration.
    struct RegionStations
    {
        double startStation = 0.0;
        double endStation = 0.0;
        double length = 0.0;
        double totalLength = 0.0;
    };

    // Sums authored lengths up to and across the requested section. Throws
    // std::out_of_range for an invalid index, matching AuthoredTrack.
    [[nodiscard]] RegionStations computeRegionStations(
        const coaster::AuthoredTrack& track,
        std::size_t sectionIndex
    );

    // Net angular change of one channel in degrees: the sum of the analytic
    // scalar-transition integrals over its segments.
    [[nodiscard]] double computeChannelNetRotationDegrees(
        const coaster::ChannelProfile& profile
    );

    // Net angular change of each rider-local channel over the whole region,
    // integrated from the existing authored transition segments with the
    // analytic scalar-transition integral and converted to degrees for
    // presentation. Stored data stays in radians; nothing here mutates it.
    struct RegionNetRotation
    {
        double rollDegrees = 0.0;
        double pitchDegrees = 0.0;
        double yawDegrees = 0.0;
    };

    [[nodiscard]] RegionNetRotation computeNetRotationDegrees(
        const coaster::GeometricSection& section
    );
}
