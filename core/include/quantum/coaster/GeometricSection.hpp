#pragma once

#include <quantum/math/ScalarTransition.hpp>

#include <cstdint>
#include <vector>

namespace quantum::coaster
{
    // Stable identifier for one ProfileSegment inside a ChannelProfile.
    // Zero is reserved so default-created segments are distinguishable from
    // assigned ones; validation rejects it. Ids stay stable across edits of
    // other segments and are never reused within a channel.
    using SegmentId = std::uint32_t;

    inline constexpr SegmentId invalidSegmentId = 0;

    // One constant-transition piece of an authored channel rate profile. The
    // embedded transition's domain is an absolute position along the
    // section-local distance axis [0, sectionLength].
    struct ProfileSegment
    {
        SegmentId id = invalidSegmentId;
        math::ScalarTransition transition{};
    };

    // One authored channel (pitch, yaw, or roll) as an ordered chain of
    // ProfileSegments covering [0, sectionLength] without gaps. Adjacent
    // segments must join with identical endpoint values (C0 continuity);
    // slope discontinuities at segment boundaries are authored content and
    // represent curvature steps. nextSegmentId hands out ids for newly
    // created segments and is never decremented or reused.
    struct ChannelProfile
    {
        std::vector<ProfileSegment> segments;
        SegmentId nextSegmentId = 1;
    };

    // Authored geometric-section channels. Angles use radians. The channels
    // are rider-local pitch/yaw/roll angular-rate profiles measured in
    // radians per coordinate unit over the section-local distance domain.
    struct GeometricSection
    {
        ChannelProfile pitch;
        ChannelProfile yaw;
        ChannelProfile roll;
    };

    // Authored channel values evaluated at one location in the section
    // domain. Their solver interpretation lives in RiderLocalGeometry.
    struct GeometricSectionState
    {
        double pitch;
        double yaw;
        double roll;
    };

    // Throws std::invalid_argument when the profile is malformed: empty,
    // non-finite distances or values, unsupported transition types, invalid
    // or duplicate segment ids, reversed or empty segment domains,
    // non-contiguous domains between adjacent segments, value discontinuities
    // between adjacent segments (C0), or when the chain does not cover
    // [0, expectedLength] exactly.
    void validateChannelProfile(
        const ChannelProfile& profile,
        double expectedLength
    );

    // Evaluates the authored channel at one location of its distance domain.
    // Validation remains authoritative for profile well-formedness; queries
    // outside the covered domain throw std::out_of_range like scalar-
    // transition evaluation does.
    [[nodiscard]] double evaluateChannelProfile(
        const ChannelProfile& profile,
        double independentValue
    );

    // Validates all three channels over one shared section-local distance
    // domain [0, sectionLength]. Throws std::invalid_argument for a
    // non-positive non-finite length or any per-channel defect above.
    void validateGeometricSection(
        const GeometricSection& section,
        double sectionLength
    );

    // Evaluates all authored channels at the same independent value of
    // [0, sectionLength]. Section validation and channel evaluation behavior
    // remain authoritative.
    [[nodiscard]] GeometricSectionState evaluateGeometricSection(
        const GeometricSection& section,
        double sectionLength,
        double independentValue
    );
}
