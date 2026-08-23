#pragma once

#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <cstddef>
#include <vector>

namespace quantum::coaster
{
    // One authored interval along the track's distance domain. The section's
    // channels are rider-local pitch/yaw/roll angular-rate profiles measured
    // in radians per coordinate unit over the section-local domain
    // [0, sectionLength()], matching the RiderLocalGeometry integration
    // conventions. A section's place along the track is its position in the
    // owning AuthoredTrack ordering.
    struct AuthoredTrackSection
    {
        GeometricSection rateProfiles;
    };

    // Returns the authored distance-domain length of a section. Throws
    // std::invalid_argument when the section's channels are malformed or do
    // not share one domain.
    [[nodiscard]] double sectionLength(
        const AuthoredTrackSection& section
    );

    // Rebases every channel domain to [0, newLength] while preserving each
    // channel's authored endpoint values and transition type. Throws
    // std::invalid_argument for non-finite or non-positive lengths.
    void setSectionLength(
        AuthoredTrackSection& section,
        double newLength
    );

    // An ordered document of authored track sections. The track always keeps
    // at least one section so that a generated centerline is well-defined;
    // removeSection refuses to delete the final remaining section.
    class AuthoredTrack
    {
    public:
        AuthoredTrack() = default;

        [[nodiscard]] std::size_t sectionCount() const noexcept;

        // Throws std::out_of_range for an invalid index.
        [[nodiscard]] const AuthoredTrackSection& section(
            std::size_t index
        ) const;

        [[nodiscard]] AuthoredTrackSection& section(std::size_t index);

        // Appends/prepends one straight default section (zero rate profiles,
        // Linear transitions, defaultNewSectionLength units long).
        void appendSection();
        void prependSection();

        // Throws std::out_of_range for an invalid index and
        // std::invalid_argument when removing the last remaining section.
        void removeSection(std::size_t index);

        // Moves a section so that it ends up at toIndex in the final
        // ordering. Both indices must address existing sections; moving a
        // section onto its own index does nothing.
        void moveSection(std::size_t fromIndex, std::size_t toIndex);

    private:
        std::vector<AuthoredTrackSection> sections_;
    };

    // Length used by appendSection/prependSection for new sections.
    inline constexpr double defaultNewSectionLength = 60.0;

    // Single-section document whose authored rate profiles reproduce the
    // original demonstration centerline behavior.
    [[nodiscard]] AuthoredTrack createDefaultAuthoredTrack();

    // Chains every authored section through the rider-local roll/pitch/yaw
    // rate integration into one continuous centerline. Each section starts
    // where the previous one ended; returned distances are cumulative from
    // the start of the track and strictly increase. The joint sample between
    // consecutive sections appears exactly once.
    //
    // Throws std::invalid_argument when the track is empty, a section is
    // malformed (including a non-canonical [0, length] local domain), or the
    // integration spacing is not finite and positive.
    [[nodiscard]] std::vector<RiderLocalGeometryState> integrateAuthoredTrack(
        const AuthoredTrack& track,
        double integrationSpacing
    );
}

