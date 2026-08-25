// Tests for structural region editing on AuthoredTrack: inserting new
// regions after an existing one and duplicating regions with exact,
// independent profile data.

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>

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

    [[nodiscard]] quantum::coaster::ChannelProfile singleSegmentChannel(
        const double length,
        const double valueBegin,
        const double valueEnd,
        const quantum::math::TransitionType transitionType
    )
    {
        quantum::coaster::ChannelProfile channel;
        channel.segments.push_back(quantum::coaster::ProfileSegment{
            channel.nextSegmentId,
            quantum::math::ScalarTransition{
                .domainBegin = 0.0,
                .domainEnd = length,
                .valueBegin = valueBegin,
                .valueEnd = valueEnd,
                .transitionType = transitionType
            }
        });
        ++channel.nextSegmentId;
        return channel;
    }

    void requireSameChannel(
        const quantum::coaster::ChannelProfile& actual,
        const quantum::coaster::ChannelProfile& expected,
        const std::string& context
    )
    {
        require(
            actual.segments.size() == expected.segments.size(),
            context + ": segment count"
        );

        for (std::size_t index = 0;
            index < expected.segments.size();
            ++index)
        {
            const quantum::math::ScalarTransition& actualTransition =
                actual.segments[index].transition;
            const quantum::math::ScalarTransition& expectedTransition =
                expected.segments[index].transition;

            require(
                actual.segments[index].id == expected.segments[index].id,
                context + ": segment id"
            );
            requireNear(
                actualTransition.domainBegin,
                expectedTransition.domainBegin,
                0.0,
                context + ": domain begin"
            );
            requireNear(
                actualTransition.domainEnd,
                expectedTransition.domainEnd,
                0.0,
                context + ": domain end"
            );
            requireNear(
                actualTransition.valueBegin,
                expectedTransition.valueBegin,
                0.0,
                context + ": value begin"
            );
            requireNear(
                actualTransition.valueEnd,
                expectedTransition.valueEnd,
                0.0,
                context + ": value end"
            );
            require(
                actualTransition.transitionType
                    == expectedTransition.transitionType,
                context + ": transition type"
            );
        }

        require(
            actual.nextSegmentId == expected.nextSegmentId,
            context + ": next segment id"
        );
    }

    // Builds a rate-profile section with distinct per-channel content and
    // a split roll segment so exact-duplication checks exercise several
    // segments, ids, and transition types at once.
    [[nodiscard]] quantum::coaster::AuthoredTrackSection createRichSection(
        const double length)
    {
        quantum::coaster::AuthoredTrackSection section =
            quantum::coaster::createRateProfileSection(length);

        quantum::coaster::GeometricSection& profiles =
            section.rateProfileRegion().rateProfiles;
        profiles.roll = singleSegmentChannel(
            length, 0.01, -0.02, quantum::math::TransitionType::Linear);
        profiles.pitch = singleSegmentChannel(
            length,
            0.03,
            0.05,
            quantum::math::TransitionType::CosineEaseInOut);
        profiles.yaw = singleSegmentChannel(
            length, -0.04, 0.06, quantum::math::TransitionType::Smoothstep);

        // Two-segment roll chain with a fresh id on the right piece.
        std::ignore = quantum::coaster::splitChannelSegment(
            profiles.roll,
            profiles.roll.segments.front().id,
            30.0
        );

        return section;
    }

    void runInsertAfterTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.appendSection();
        track.appendSection();

        // Distinguish the three original regions by authored length so
        // ordering checks cannot pass by accident.
        quantum::coaster::setSectionLength(track.section(0), 50.0);
        quantum::coaster::setSectionLength(track.section(1), 70.0);
        quantum::coaster::setSectionLength(track.section(2), 90.0);

        quantum::coaster::AuthoredTrackSection inserted =
            createRichSection(45.0);

        track.insertSectionAfter(1, inserted);

        require(track.sectionCount() == 4, "insert produced four sections");
        requireNear(
            quantum::coaster::sectionLength(track.section(0)),
            50.0,
            1.0e-12,
            "region A preserved at front"
        );
        requireNear(
            quantum::coaster::sectionLength(track.section(1)),
            70.0,
            1.0e-12,
            "anchor region B unchanged"
        );
        requireNear(
            quantum::coaster::sectionLength(track.section(2)),
            45.0,
            1.0e-12,
            "inserted region sits after anchor"
        );
        requireNear(
            quantum::coaster::sectionLength(track.section(3)),
            90.0,
            1.0e-12,
            "region C shifted behind insertion"
        );

        requireSameChannel(
            track.section(2).rateProfileRegion().rateProfiles.yaw,
            inserted.rateProfileRegion().rateProfiles.yaw,
            "inserted yaw payload copied exactly"
        );

        // Removing the inserted region restores A B C ordering exactly.
        track.removeSection(2);
        require(track.sectionCount() == 3, "removal restored count");
        requireNear(
            quantum::coaster::sectionLength(track.section(2)),
            90.0,
            1.0e-12,
            "ordering restored after removal"
        );

        // Inserting after the final index appends.
        track.insertSectionAfter(2, inserted);
        require(track.sectionCount() == 4, "end insert appended");
        requireNear(
            quantum::coaster::sectionLength(track.section(3)),
            45.0,
            1.0e-12,
            "end-inserted region landed last"
        );
    }

    void runDuplicateRateProfileTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.section(0) = createRichSection(80.0);

        track.duplicateSection(0);

        require(track.sectionCount() == 2, "duplicate added one section");
        require(
            track.section(1).kind
                == quantum::coaster::RegionKind::RateProfiles,
            "duplicate kept rate-profile kind"
        );
        requireNear(
            quantum::coaster::sectionLength(track.section(1)),
            80.0,
            0.0,
            "duplicate preserved authored length"
        );

        // Read both after duplication so no reference is captured before
        // the vector may reallocate during insert.
        const quantum::coaster::GeometricSection& original =
            track.section(0).rateProfileRegion().rateProfiles;
        const quantum::coaster::GeometricSection& duplicate =
            track.section(1).rateProfileRegion().rateProfiles;
        requireSameChannel(duplicate.roll, original.roll, "roll duplicated");
        requireSameChannel(duplicate.pitch, original.pitch, "pitch duplicated");
        requireSameChannel(duplicate.yaw, original.yaw, "yaw duplicated");
        require(
            duplicate.roll.segments.size() == 2,
            "split segments survived duplication"
        );
    }

    void runDuplicatePlanarArcTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        quantum::coaster::convertSectionToPlanarArc(track.section(0));

        // Distinct authored geometry: radius edit keeps the stored length,
        // tilt/bank are free parameters.
        quantum::coaster::setPlanarArcRadius(track.section(0), 25.0);
        quantum::coaster::setPlanarArcPlaneTilt(track.section(0), 0.15);
        quantum::coaster::setPlanarArcBankChange(track.section(0), -0.2);

        track.duplicateSection(0);

        require(track.sectionCount() == 2, "arc duplicate added one section");
        require(
            track.section(1).kind == quantum::coaster::RegionKind::Geometry,
            "duplicate kept geometry kind"
        );

        // Read both after duplication so no reference is captured before
        // the vector may reallocate during insert.
        const quantum::coaster::PlanarArcRegion& sourceArc =
            std::get<quantum::coaster::PlanarArcRegion>(
                std::get<quantum::coaster::GeometryRegion>(
                    track.section(0).region).construction);
        const quantum::coaster::PlanarArcRegion& duplicateArc =
            std::get<quantum::coaster::PlanarArcRegion>(
                std::get<quantum::coaster::GeometryRegion>(
                    track.section(1).region).construction);

        requireNear(
            quantum::coaster::sectionLength(track.section(1)),
            quantum::coaster::sectionLength(track.section(0)),
            0.0,
            "duplicate arc preserved length"
        );
        requireNear(duplicateArc.radius, sourceArc.radius, 0.0, "radius");
        requireNear(
            duplicateArc.sweptAngle, sourceArc.sweptAngle, 0.0, "swept angle"
        );
        requireNear(
            duplicateArc.planeTilt, sourceArc.planeTilt, 0.0, "plane tilt"
        );
        requireNear(
            duplicateArc.bankChange, sourceArc.bankChange, 0.0, "bank change"
        );
    }

    void runDuplicateIndependenceTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.section(0) = createRichSection(80.0);
        track.duplicateSection(0);

        const quantum::coaster::AuthoredTrackSection originalSnapshot =
            track.section(0);

        // Mutate every part of the duplicate; none of it may leak back
        // into the original.
        quantum::coaster::setChannelSegmentValue(
            track.section(1).rateProfileRegion().rateProfiles.roll,
            track.section(1)
                .rateProfileRegion()
                .rateProfiles.roll.segments.front()
                .id,
            quantum::coaster::ProfileBoundary::End,
            9.0
        );
        quantum::coaster::setSectionLength(track.section(1), 40.0);
        quantum::coaster::convertSectionToPlanarArc(track.section(1));

        requireSameChannel(
            track.section(0).rateProfileRegion().rateProfiles.roll,
            originalSnapshot.rateProfileRegion().rateProfiles.roll,
            "original roll untouched by duplicate edits"
        );
        requireSameChannel(
            track.section(0).rateProfileRegion().rateProfiles.pitch,
            originalSnapshot.rateProfileRegion().rateProfiles.pitch,
            "original pitch untouched by duplicate edits"
        );
        requireNear(
            quantum::coaster::sectionLength(track.section(0)),
            80.0,
            0.0,
            "original length untouched by duplicate edits"
        );
        require(
            track.section(0).kind
                == quantum::coaster::RegionKind::RateProfiles,
            "original kind untouched by duplicate conversion"
        );

        // Geometry duplicates are equally independent.
        quantum::coaster::AuthoredTrack arcTrack;
        arcTrack.appendSection();
        quantum::coaster::convertSectionToPlanarArc(arcTrack.section(0));
        quantum::coaster::setPlanarArcRadius(arcTrack.section(0), 25.0);
        arcTrack.duplicateSection(0);
        quantum::coaster::setPlanarArcRadius(arcTrack.section(1), 999.0);

        const quantum::coaster::PlanarArcRegion& originalArc =
            std::get<quantum::coaster::PlanarArcRegion>(
                std::get<quantum::coaster::GeometryRegion>(
                    arcTrack.section(0).region).construction);
        requireNear(originalArc.radius, 25.0, 0.0, "arc radius independent");
    }

    void runInvalidIndexTests()
    {
        quantum::coaster::AuthoredTrack track;
        track.appendSection();
        track.appendSection();

        const quantum::coaster::AuthoredTrackSection valid =
            quantum::coaster::createRateProfileSection(30.0);

        bool threw = false;
        try
        {
            track.insertSectionAfter(2, valid);
        }
        catch (const std::out_of_range&)
        {
            threw = true;
        }
        require(threw, "insert past last index throws out_of_range");

        threw = false;
        try
        {
            track.duplicateSection(99);
        }
        catch (const std::out_of_range&)
        {
            threw = true;
        }
        require(threw, "duplicate with invalid index throws out_of_range");

        // A malformed section must be rejected before entering the
        // document: default channels do not cover the stored length.
        threw = false;
        try
        {
            track.insertSectionAfter(
                0,
                quantum::coaster::AuthoredTrackSection{});
        }
        catch (const std::invalid_argument&)
        {
            threw = true;
        }
        require(threw, "malformed insert payload throws invalid_argument");

        require(track.sectionCount() == 2, "failed inserts changed nothing");
    }
}

int main()
{
    try
    {
        runInsertAfterTests();
        runDuplicateRateProfileTests();
        runDuplicatePlanarArcTests();
        runDuplicateIndependenceTests();
        runInvalidIndexTests();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] authored track structure verification: "
                  << exception.what() << '\n';
        return 1;
    }

    std::cout << "Authored track structure tests passed.\n";
    return 0;
}
