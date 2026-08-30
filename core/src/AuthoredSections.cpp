#include <quantum/coaster/GeometricSection.hpp>

#include <cmath>
#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        void validateSegmentTransition(
            const math::ScalarTransition& transition
        )
        {
            if (!std::isfinite(transition.domainBegin)
                || !std::isfinite(transition.domainEnd)
                || !std::isfinite(transition.valueBegin)
                || !std::isfinite(transition.valueEnd))
            {
                throw std::invalid_argument(
                    "A channel profile segment requires finite distances "
                    "and values."
                );
            }

            if (!(transition.domainEnd > transition.domainBegin))
            {
                throw std::invalid_argument(
                    "A channel profile segment must span a non-empty "
                    "distance range."
                );
            }

            // The scalar evaluator remains authoritative for value and
            // transition-type validity.
            static_cast<void>(math::evaluateScalarTransition(
                transition,
                transition.domainBegin
            ));
            static_cast<void>(math::evaluateTransition(
                transition.transitionType,
                0.5
            ));
        }

        void requireUniqueSegmentIds(const ChannelProfile& profile)
        {
            for (std::size_t i = 0; i < profile.segments.size(); ++i)
            {
                for (std::size_t j = i + 1; j < profile.segments.size(); ++j)
                {
                    if (profile.segments[i].id == profile.segments[j].id)
                    {
                        throw std::invalid_argument(
                            "Channel profile segment ids must be unique "
                            "within a channel."
                        );
                    }
                }
            }
        }
    }

    void validateChannelProfile(
        const ChannelProfile& profile,
        const double expectedLength
    )
    {
        if (!std::isfinite(expectedLength) || expectedLength <= 0.0)
        {
            throw std::invalid_argument(
                "An authored channel profile requires a positive finite "
                "length."
            );
        }

        if (profile.segments.empty())
        {
            throw std::invalid_argument(
                "An authored channel profile requires at least one segment."
            );
        }

        requireUniqueSegmentIds(profile);

        const math::ScalarTransition* previous = nullptr;
        for (const ProfileSegment& segment : profile.segments)
        {
            if (segment.id == invalidSegmentId)
            {
                throw std::invalid_argument(
                    "A channel profile segment must carry a valid id."
                );
            }

            validateSegmentTransition(segment.transition);

            if (previous == nullptr)
            {
                // Sections author their channels over the canonical local
                // domain starting at exactly zero so that chaining and
                // rescaling stay unambiguous.
                if (segment.transition.domainBegin != 0.0)
                {
                    throw std::invalid_argument(
                        "A channel profile must start its domain at zero."
                    );
                }
            }
            else
            {
                // Exact comparisons keep the chain gap-free: adjacent
                // segments share one boundary distance and join with one
                // shared endpoint value (C0 continuity).
                if (segment.transition.domainBegin != previous->domainEnd)
                {
                    throw std::invalid_argument(
                        "Adjacent channel profile segments must be contiguous."
                    );
                }

                if (segment.transition.valueBegin != previous->valueEnd)
                {
                    throw std::invalid_argument(
                        "Adjacent channel profile segments must join with "
                        "identical values (C0 continuity)."
                    );
                }
            }

            previous = &segment.transition;
        }

        if (previous->domainEnd != expectedLength)
        {
            throw std::invalid_argument(
                "A channel profile must cover [0, length] exactly."
            );
        }
    }

    double evaluateChannelProfile(
        const ChannelProfile& profile,
        const double independentValue
    )
    {
        if (!std::isfinite(independentValue))
        {
            throw std::invalid_argument(
                "The queried distance must be finite."
            );
        }

        // Boundary queries may match two adjacent segments; this is safe
        // because validation guarantees identical values on both sides of
        // every interior boundary (C0 continuity).
        for (const ProfileSegment& segment : profile.segments)
        {
            if (independentValue >= segment.transition.domainBegin
                && independentValue <= segment.transition.domainEnd)
            {
                return math::evaluateScalarTransition(
                    segment.transition,
                    independentValue
                );
            }
        }

        throw std::out_of_range(
            "The queried distance lies outside the authored channel "
            "profile domain."
        );
    }

    void validateGeometricSection(
        const GeometricSection& section,
        const double sectionLength
    )
    {
        validateChannelProfile(section.pitch, sectionLength);
        validateChannelProfile(section.yaw, sectionLength);
        validateChannelProfile(section.roll, sectionLength);
    }

    GeometricSectionState evaluateGeometricSection(
        const GeometricSection& section,
        const double sectionLength,
        const double independentValue
    )
    {
        validateGeometricSection(section, sectionLength);

        return {
            evaluateChannelProfile(section.pitch, independentValue),
            evaluateChannelProfile(section.yaw, independentValue),
            evaluateChannelProfile(section.roll, independentValue)
        };
    }
}
