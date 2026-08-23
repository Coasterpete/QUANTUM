#include <quantum/coaster/AuthoredTrack.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace quantum::coaster
{
    namespace
    {
        void validateSectionLocalDomain(
            const AuthoredTrackSection& section)
        {
            // Track sections must author their rate profiles over the
            // canonical section-local domain [0, length] so that chaining is
            // unambiguous. The scalar-transition evaluator remains
            // authoritative for value and transition-type validity.
            validateGeometricSection(section.rateProfiles);

            const double domainBegin =
                section.rateProfiles.pitch.domainBegin;
            const double length = section.rateProfiles.pitch.domainEnd;

            if (domainBegin != 0.0
                || !std::isfinite(length)
                || length <= 0.0)
            {
                throw std::invalid_argument(
                    "An authored track section must use the canonical "
                    "section-local domain [0, length] with a positive "
                    "finite length."
                );
            }
        }

        [[nodiscard]] AuthoredTrackSection createDefaultSection()
        {
            constexpr math::TransitionType straightType =
                math::TransitionType::Linear;

            const math::ScalarTransition zeroRate{
                .domainBegin = 0.0,
                .domainEnd = defaultNewSectionLength,
                .valueBegin = 0.0,
                .valueEnd = 0.0,
                .transitionType = straightType
            };

            AuthoredTrackSection section;
            section.rateProfiles.pitch = zeroRate;
            section.rateProfiles.yaw = zeroRate;
            section.rateProfiles.roll = zeroRate;
            return section;
        }
    }

    double sectionLength(const AuthoredTrackSection& section)
    {
        validateGeometricSection(section.rateProfiles);
        return section.rateProfiles.pitch.domainEnd;
    }

    void setSectionLength(AuthoredTrackSection& section, double newLength)
    {
        if (!std::isfinite(newLength) || newLength <= 0.0)
        {
            throw std::invalid_argument(
                "An authored track section requires a positive finite "
                "length."
            );
        }

        for (math::ScalarTransition* channel :
            {&section.rateProfiles.pitch,
             &section.rateProfiles.yaw,
             &section.rateProfiles.roll})
        {
            channel->domainBegin = 0.0;
            channel->domainEnd = newLength;
        }
    }

    std::size_t AuthoredTrack::sectionCount() const noexcept
    {
        return sections_.size();
    }

    const AuthoredTrackSection& AuthoredTrack::section(
        const std::size_t index) const
    {
        return sections_.at(index);
    }

    AuthoredTrackSection& AuthoredTrack::section(const std::size_t index)
    {
        return sections_.at(index);
    }

    void AuthoredTrack::appendSection()
    {
        sections_.push_back(createDefaultSection());
    }

    void AuthoredTrack::prependSection()
    {
        sections_.insert(sections_.begin(), createDefaultSection());
    }

    void AuthoredTrack::removeSection(const std::size_t index)
    {
        if (index >= sections_.size())
        {
            throw std::out_of_range(
                "Authored track section index is out of range."
            );
        }

        if (sections_.size() == 1)
        {
            throw std::invalid_argument(
                "An authored track always keeps at least one section."
            );
        }

        sections_.erase(sections_.begin()
                        + static_cast<std::ptrdiff_t>(index));
    }

    void AuthoredTrack::moveSection(
        const std::size_t fromIndex,
        const std::size_t toIndex)
    {
        if (fromIndex >= sections_.size() || toIndex >= sections_.size())
        {
            throw std::out_of_range(
                "Authored track section index is out of range."
            );
        }

        if (fromIndex == toIndex)
        {
            return;
        }

        AuthoredTrackSection moved = std::move(sections_[fromIndex]);
        sections_.erase(
            sections_.begin() + static_cast<std::ptrdiff_t>(fromIndex));

        // After the erase, toIndex already addresses the final position:
        // sections before fromIndex are untouched and the sections between
        // fromIndex and toIndex have shifted down by one.
        sections_.insert(
            sections_.begin() + static_cast<std::ptrdiff_t>(toIndex),
            std::move(moved));
    }

    AuthoredTrack createDefaultAuthoredTrack()
    {
        // Reproduces the original demonstration profile behavior: one curved
        // section whose rider-local roll/pitch/yaw rate profiles span
        // [0, 180].
        AuthoredTrack track;

        track.appendSection();
        setSectionLength(track.section(0), 180.0);

        AuthoredTrackSection& section = track.section(0);
        section.rateProfiles.roll = {
            .domainBegin = 0.0,
            .domainEnd = 180.0,
            .valueBegin = 0.0,
            .valueEnd = 0.024,
            .transitionType = math::TransitionType::Smootherstep
        };
        section.rateProfiles.pitch = {
            .domainBegin = 0.0,
            .domainEnd = 180.0,
            .valueBegin = 0.018,
            .valueEnd = -0.010,
            .transitionType = math::TransitionType::CosineEaseInOut
        };
        section.rateProfiles.yaw = {
            .domainBegin = 0.0,
            .domainEnd = 180.0,
            .valueBegin = 0.004,
            .valueEnd = 0.022,
            .transitionType = math::TransitionType::Smoothstep
        };

        return track;
    }

    std::vector<RiderLocalGeometryState> integrateAuthoredTrack(
        const AuthoredTrack& track,
        const double integrationSpacing)
    {
        if (!std::isfinite(integrationSpacing) || integrationSpacing <= 0.0)
        {
            throw std::invalid_argument(
                "Centerline integration spacing must be positive and "
                "finite."
            );
        }

        if (track.sectionCount() == 0)
        {
            throw std::invalid_argument(
                "An authored track requires at least one section to "
                "generate a centerline."
            );
        }

        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            validateSectionLocalDomain(track.section(index));
        }

        constexpr geometry::CurveFrame startingFrame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };

        glm::dvec3 position{0.0, 0.0, 0.0};
        geometry::CurveFrame frame = startingFrame;
        double distanceOffset = 0.0;

        std::vector<RiderLocalGeometryState> states;

        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            const AuthoredTrackSection& section = track.section(index);

            const std::vector<RiderLocalGeometryState> localStates =
                integrateLocalRollPitchYawRateProfiles(
                    position,
                    frame,
                    section.rateProfiles.roll,
                    section.rateProfiles.pitch,
                    section.rateProfiles.yaw,
                    integrationSpacing
                );

            // The first state of every section repeats the previous
            // section's final joint state; skip it so the concatenation
            // contains each joint exactly once. The track's very first
            // state is kept so a single-section track matches the
            // direct integration output.
            const bool isFirstSection = states.empty();
            for (std::size_t stateIndex = isFirstSection ? 0u : 1u;
                stateIndex < localStates.size();
                ++stateIndex)
            {
                RiderLocalGeometryState state = localStates[stateIndex];
                state.distance += distanceOffset;
                states.push_back(std::move(state));
            }

            const RiderLocalGeometryState& finalState = localStates.back();
            position = finalState.position;
            frame = finalState.frame;
            distanceOffset += sectionLength(section);
        }

        return states;
    }
}

