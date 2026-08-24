#include <quantum/coaster/ChannelProfileEditing.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] std::size_t requireSegmentIndex(
            const ChannelProfile& profile,
            const SegmentId id)
        {
            for (std::size_t index = 0;
                index < profile.segments.size();
                ++index)
            {
                if (profile.segments[index].id == id)
                {
                    return index;
                }
            }

            throw std::invalid_argument(
                "the channel has no segment with the requested id"
            );
        }
    }

    SegmentId findChannelSegmentAtDistance(
        const ChannelProfile& profile,
        const double distance)
    {
        if (!std::isfinite(distance))
        {
            return invalidSegmentId;
        }

        // Ascending scan makes an exact joint resolve to the left segment.
        for (const ProfileSegment& segment : profile.segments)
        {
            if (distance >= segment.transition.domainBegin
                && distance <= segment.transition.domainEnd)
            {
                return segment.id;
            }
        }

        return invalidSegmentId;
    }

    math::ScalarTransition* findChannelSegmentTransition(
        ChannelProfile& profile,
        const SegmentId id)
    {
        if (id == invalidSegmentId)
        {
            return nullptr;
        }

        for (ProfileSegment& segment : profile.segments)
        {
            if (segment.id == id)
            {
                return &segment.transition;
            }
        }

        return nullptr;
    }

    SegmentId splitChannelSegment(
        ChannelProfile& profile,
        const SegmentId id,
        const double distance)
    {
        if (!std::isfinite(distance))
        {
            throw std::invalid_argument(
                "the split distance must be finite"
            );
        }

        const std::size_t index = requireSegmentIndex(profile, id);
        ProfileSegment& segment = profile.segments[index];

        if (!(segment.transition.domainBegin < distance
            && distance < segment.transition.domainEnd))
        {
            throw std::invalid_argument(
                "the split distance must lie strictly inside the segment "
                "domain"
            );
        }

        const double splitValue = math::evaluateScalarTransition(
            segment.transition,
            distance
        );

        ProfileSegment rightPiece;
        rightPiece.id = profile.nextSegmentId++;
        rightPiece.transition = segment.transition;
        rightPiece.transition.domainBegin = distance;
        rightPiece.transition.valueBegin = splitValue;

        segment.transition.domainEnd = distance;
        segment.transition.valueEnd = splitValue;

        profile.segments.insert(
            profile.segments.begin()
                + static_cast<std::ptrdiff_t>(index) + 1,
            rightPiece
        );

        return rightPiece.id;
    }

    SegmentId removeChannelSegment(
        ChannelProfile& profile,
        const SegmentId id)
    {
        if (profile.segments.size() < 2)
        {
            throw std::invalid_argument(
                "the last remaining profile segment cannot be removed"
            );
        }

        const std::size_t index = requireSegmentIndex(profile, id);
        const std::size_t survivorIndex = index > 0 ? index - 1 : index + 1;
        ProfileSegment& survivor = profile.segments[survivorIndex];
        const ProfileSegment& removed = profile.segments[index];

        if (survivorIndex < index)
        {
            // The previous neighbor absorbs the removed span and inherits
            // its end value so the joint with the next segment stays C0.
            survivor.transition.domainEnd =
                removed.transition.domainEnd;
            survivor.transition.valueEnd =
                removed.transition.valueEnd;
        }
        else
        {
            // The first segment is removed; the next one absorbs backwards.
            survivor.transition.domainBegin =
                removed.transition.domainBegin;
            survivor.transition.valueBegin =
                removed.transition.valueBegin;
        }

        const SegmentId survivorId = survivor.id;
        profile.segments.erase(
            profile.segments.begin() + static_cast<std::ptrdiff_t>(index)
        );
        return survivorId;
    }

    void moveChannelSegmentBoundary(
        ChannelProfile& profile,
        const SegmentId id,
        const ProfileBoundary boundary,
        const double newDistance)
    {
        if (!std::isfinite(newDistance))
        {
            throw std::invalid_argument(
                "the moved boundary distance must be finite"
            );
        }

        const std::size_t index = requireSegmentIndex(profile, id);

        switch (boundary)
        {
        case ProfileBoundary::Begin:
        {
            if (index == 0)
            {
                throw std::invalid_argument(
                    "the first segment's begin boundary is pinned to the "
                    "section start"
                );
            }

            const double lowerBound = index >= 2
                ? profile.segments[index - 2].transition.domainEnd
                : profile.segments.front().transition.domainBegin;
            const double upperBound =
                profile.segments[index].transition.domainEnd;

            if (!(lowerBound < newDistance && newDistance < upperBound))
            {
                throw std::invalid_argument(
                    "the moved boundary must stay strictly between its "
                    "neighbouring boundaries"
                );
            }

            profile.segments[index - 1].transition.domainEnd = newDistance;
            profile.segments[index].transition.domainBegin = newDistance;
            break;
        }
        case ProfileBoundary::End:
        {
            if (index + 1 >= profile.segments.size())
            {
                throw std::invalid_argument(
                    "the last segment's end boundary is pinned to the "
                    "section end"
                );
            }

            const double lowerBound =
                profile.segments[index].transition.domainBegin;
            const double upperBound = index + 2 < profile.segments.size()
                ? profile.segments[index + 2].transition.domainBegin
                : profile.segments.back().transition.domainEnd;

            if (!(lowerBound < newDistance && newDistance < upperBound))
            {
                throw std::invalid_argument(
                    "the moved boundary must stay strictly between its "
                    "neighbouring boundaries"
                );
            }

            profile.segments[index].transition.domainEnd = newDistance;
            profile.segments[index + 1].transition.domainBegin =
                newDistance;
            break;
        }
        }
    }

    void setChannelSegmentValue(
        ChannelProfile& profile,
        const SegmentId id,
        const ProfileBoundary boundary,
        const double value)
    {
        if (!std::isfinite(value))
        {
            throw std::invalid_argument(
                "the profile rate value must be finite"
            );
        }

        const std::size_t index = requireSegmentIndex(profile, id);

        switch (boundary)
        {
        case ProfileBoundary::Begin:
            profile.segments[index].transition.valueBegin = value;
            if (index > 0)
            {
                // Valid channels are C0-continuous, so the adjoining
                // segment shares this boundary value.
                profile.segments[index - 1].transition.valueEnd = value;
            }
            break;
        case ProfileBoundary::End:
            profile.segments[index].transition.valueEnd = value;
            if (index + 1 < profile.segments.size())
            {
                profile.segments[index + 1].transition.valueBegin = value;
            }
            break;
        }
    }
}
