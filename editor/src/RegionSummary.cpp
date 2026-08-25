#include <quantum/editor/RegionSummary.hpp>

#include <quantum/math/ScalarTransition.hpp>

namespace quantum::editor
{
    namespace
    {
        constexpr double piRadians = 3.14159265358979323846;
        constexpr double degreesPerRadian = 180.0 / piRadians;
    }

    RegionStations computeRegionStations(
        const coaster::AuthoredTrack& track,
        const std::size_t sectionIndex
    )
    {
        // Throws std::out_of_range for an invalid index, which also guards
        // the prefix sums below.
        const double selectedLength = coaster::sectionLength(
            track.section(sectionIndex)
        );

        RegionStations stations;
        stations.length = selectedLength;

        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            if (index == sectionIndex)
            {
                stations.startStation = stations.totalLength;
                stations.endStation = stations.startStation + selectedLength;
            }

            stations.totalLength += coaster::sectionLength(
                track.section(index)
            );
        }

        return stations;
    }

    double computeChannelNetRotationDegrees(
        const coaster::ChannelProfile& profile
    )
    {
        // Segments tile the channel domain contiguously, so summing the
        // per-segment integrals integrates the whole channel.
        double totalRadians = 0.0;

        for (const coaster::ProfileSegment& segment : profile.segments)
        {
            totalRadians += quantum::math::integrateScalarTransition(
                segment.transition,
                segment.transition.domainBegin,
                segment.transition.domainEnd
            );
        }

        return totalRadians * degreesPerRadian;
    }

    RegionNetRotation computeNetRotationDegrees(
        const coaster::GeometricSection& section
    )
    {
        RegionNetRotation rotation;
        rotation.rollDegrees = computeChannelNetRotationDegrees(section.roll);
        rotation.pitchDegrees =
            computeChannelNetRotationDegrees(section.pitch);
        rotation.yawDegrees = computeChannelNetRotationDegrees(section.yaw);

        return rotation;
    }
}
