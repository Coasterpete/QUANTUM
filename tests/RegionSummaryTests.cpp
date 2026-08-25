// Tests for the designer-readable region summary helpers: station ranges
// derived from authored lengths and net channel rotations integrated from
// the authored scalar-transition segments.

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/math/ScalarTransition.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
    void require(
        const bool condition,
        const std::string& context
    )
    {
        if (!condition)
        {
            throw std::runtime_error(context);
        }
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string& context
    )
    {
        if (!(std::abs(actual - expected) <= tolerance))
        {
            throw std::runtime_error(
                context + ": expected " + std::to_string(expected)
                + ", got " + std::to_string(actual)
            );
        }
    }

    void runStationTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.appendSection();
        track.appendSection();

        const quantum::editor::RegionStations first =
            quantum::editor::computeRegionStations(track, 0);
        requireNear(first.startStation, 0.0, 1.0e-12, "first start station");
        requireNear(first.endStation, 60.0, 1.0e-12, "first end station");
        requireNear(first.length, 60.0, 1.0e-12, "first length");
        requireNear(first.totalLength, 180.0, 1.0e-12, "three-section total");

        const quantum::editor::RegionStations last =
            quantum::editor::computeRegionStations(track, 2);
        requireNear(last.startStation, 120.0, 1.0e-12, "last start station");
        requireNear(last.endStation, 180.0, 1.0e-12, "last end station");

        bool threw = false;
        try
        {
            std::ignore = quantum::editor::computeRegionStations(track, 3);
        }
        catch (const std::out_of_range&)
        {
            threw = true;
        }
        require(threw, "invalid station index throws std::out_of_range");
    }

    void runStationTestsWithGeometryRegion()
    {
        // Geometry regions keep their authored length, so converting one
        // must leave every station quantity unchanged.
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.appendSection();
        quantum::coaster::convertSectionToPlanarArc(track.section(1));

        const quantum::editor::RegionStations second =
            quantum::editor::computeRegionStations(track, 1);
        requireNear(second.startStation, 60.0, 1.0e-12, "geometry start");
        requireNear(second.endStation, 120.0, 1.0e-12, "geometry end");
        requireNear(second.length, 60.0, 1.0e-12, "geometry length");
        requireNear(second.totalLength, 120.0, 1.0e-12, "mixed-kind total");
    }

    void runNetRotationTests()
    {
        constexpr double piRadians = 3.14159265358979323846;

        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        quantum::coaster::GeometricSection& rates =
            track.section(0).rateProfileRegion().rateProfiles;

        // A linear ramp integrates to the endpoint average times the span:
        // yaw ends at pi/30 over 60 units, so it accumulates exactly pi rad.
        quantum::coaster::setChannelSegmentValue(
            rates.yaw,
            rates.yaw.segments.front().id,
            quantum::coaster::ProfileBoundary::End,
            piRadians / 30.0
        );
        quantum::coaster::setChannelSegmentValue(
            rates.roll,
            rates.roll.segments.front().id,
            quantum::coaster::ProfileBoundary::End,
            piRadians / 60.0
        );

        const quantum::editor::RegionNetRotation netRotation =
            quantum::editor::computeNetRotationDegrees(rates);
        requireNear(netRotation.yawDegrees, 180.0, 1.0e-9, "yaw net rotation");
        requireNear(netRotation.rollDegrees, 90.0, 1.0e-9, "roll net rotation");
        requireNear(
            netRotation.pitchDegrees,
            0.0,
            1.0e-9,
            "untouched pitch stays zero"
        );

        // Splitting a segment conserves the channel's integrated angle: the
        // pieces tile the same domain with matching joint values.
        const double yawBeforeSplit =
            quantum::editor::computeChannelNetRotationDegrees(rates.yaw);
        std::ignore = quantum::coaster::splitChannelSegment(
            rates.yaw,
            rates.yaw.segments.front().id,
            25.0
        );
        require(
            rates.yaw.segments.size() == 2,
            "split produced two segments"
        );
        const double yawAfterSplit =
            quantum::editor::computeChannelNetRotationDegrees(rates.yaw);
        requireNear(
            yawAfterSplit,
            yawBeforeSplit,
            1.0e-9,
            "split conserves integrated angle"
        );
    }
}

int main()
{
    try
    {
        runStationTests();
        runStationTestsWithGeometryRegion();
        runNetRotationTests();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] region summary verification: "
                  << exception.what() << '\n';
        return 1;
    }

    std::cout << "Region summary tests passed.\n";
    return 0;
}
