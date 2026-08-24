#pragma once

#include <quantum/coaster/GeometricSection.hpp>

namespace quantum::coaster
{
    // Which domain boundary of one profile segment an editing operation
    // addresses. Interior boundaries are shared with the adjoining
    // segment; the operations below keep both sides consistent.
    enum class ProfileBoundary
    {
        Begin,
        End
    };

    // Returns the id of the segment whose domain contains distance. A
    // distance exactly on a joint resolves to the left segment. Returns
    // invalidSegmentId when no segment contains it.
    [[nodiscard]] SegmentId findChannelSegmentAtDistance(
        const ChannelProfile& profile,
        double distance
    );

    // Returns the transition of the segment with the given id, or nullptr
    // when the profile has no such segment.
    [[nodiscard]] math::ScalarTransition* findChannelSegmentTransition(
        ChannelProfile& profile,
        SegmentId id
    );

    // Splits the segment at a strictly interior distance. The left piece
    // keeps the original segment id and transition type; the right piece
    // receives a fresh id from nextSegmentId and carries the interpolated
    // split value so the channel stays covered and C0-continuous.
    // Returns the right piece's id.
    [[nodiscard]] SegmentId splitChannelSegment(
        ChannelProfile& profile,
        SegmentId id,
        double distance
    );

    // Removes a segment by merging it into its previous neighbor, or into
    // its next one when it is the first. The survivor keeps its own id,
    // transition type, and leading values, and extends over the removed
    // span. Refuses to remove the last remaining segment. Returns the
    // survivor's id.
    [[nodiscard]] SegmentId removeChannelSegment(
        ChannelProfile& profile,
        SegmentId id
    );

    // Moves an interior boundary horizontally. The adjoining segment's
    // matching boundary follows so contiguity is preserved exactly;
    // endpoint values are untouched. The first segment's Begin boundary
    // (section start) and last segment's End boundary (section end) are
    // pinned to the section coverage and refuse moves. newDistance must
    // stay strictly between the neighbouring outer boundaries of the
    // moved joint.
    void moveChannelSegmentBoundary(
        ChannelProfile& profile,
        SegmentId id,
        ProfileBoundary boundary,
        double newDistance
    );

    // Sets one endpoint value of a segment. Because valid channels are
    // C0-continuous, an interior boundary value is shared with the
    // adjoining segment and that segment's matching endpoint follows
    // automatically.
    void setChannelSegmentValue(
        ChannelProfile& profile,
        SegmentId id,
        ProfileBoundary boundary,
        double value
    );
}
