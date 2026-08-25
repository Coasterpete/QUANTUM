#pragma once

#include <quantum/coaster/GeometricSection.hpp>
#include <quantum/coaster/PlanarArcRegion.hpp>
#include <quantum/coaster/RiderLocalGeometry.hpp>

#include <cstddef>
#include <stdexcept>
#include <variant>
#include <vector>

namespace quantum::coaster
{
    // Length used by appendSection/prependSection for new sections.
    inline constexpr double defaultNewSectionLength = 60.0;

    // Authoring approach used by one track interval. The taxonomy is
    // deliberately open: geometry-driven constructions join behind a single
    // Geometry kind while rate-profile authoring remains the established
    // system. Region terminology is internal until the architecture is
    // proven; public Section names are unchanged.
    enum class RegionKind
    {
        RateProfiles,
        Geometry
    };

    // Rate-profile authoring payload: three rider-local angular-rate channel
    // profiles over the section-local distance domain.
    struct RateProfileRegion
    {
        GeometricSection rateProfiles;
    };

    // Geometry-driven authoring payload. Constructions are expressed in
    // designer-facing geometry terms: geometry parameters own the
    // centerline, and orientation-only parameters such as bank never move
    // it. Constructions compile to rider-local rates; the compiled
    // representation is deliberately not part of this API.
    struct GeometryRegion
    {
        std::variant<PlanarArcRegion> construction;
    };

    // One authored interval along the track's distance domain. The section's
    // length is authoritative: the authored region must cover the
    // section-local domain [0, length] exactly, matching the
    // RiderLocalGeometry integration conventions. A section's place along
    // the track is its position in the owning AuthoredTrack ordering.
    struct AuthoredTrackSection
    {
        RegionKind kind = RegionKind::RateProfiles;
        double length = defaultNewSectionLength;
        std::variant<RateProfileRegion, GeometryRegion> region;

        // Rate-profile access to the authored region. Throws
        // std::logic_error when the section authors a different region kind;
        // callers that handle multiple kinds must dispatch on `kind`.
        [[nodiscard]] RateProfileRegion& rateProfileRegion()
        {
            if (kind != RegionKind::RateProfiles)
            {
                throw std::logic_error(
                    "The section does not author rate profiles."
                );
            }

            return std::get<RateProfileRegion>(region);
        }

        [[nodiscard]] const RateProfileRegion& rateProfileRegion() const
        {
            if (kind != RegionKind::RateProfiles)
            {
                throw std::logic_error(
                    "The section does not author rate profiles."
                );
            }

            return std::get<RateProfileRegion>(region);
        }
    };

    // Returns the authored distance-domain length of a section. Throws
    // std::invalid_argument when the channels do not cover [0, length]
    // exactly or are otherwise malformed.
    [[nodiscard]] double sectionLength(
        const AuthoredTrackSection& section
    );

    // Rescales a section to newLength. Every segment boundary distance is
    // scaled proportionally so authored rate-profile shapes are preserved,
    // while endpoint values and transition types stay untouched; the first
    // boundary stays exactly zero and the last lands exactly on newLength.
    // The section must currently be valid over [0, sectionLength(section)].
    // Throws std::invalid_argument for non-finite or non-positive lengths
    // or a malformed current state.
    void setSectionLength(
        AuthoredTrackSection& section,
        double newLength
    );

    // Creates one rate-profile section whose zero-rate channels cover
    // [0, length] exactly. Used for new sections and for converting a
    // geometry section back to rate-profile authoring at its current
    // length. Throws std::invalid_argument for a non-positive or
    // non-finite length.
    [[nodiscard]] AuthoredTrackSection createRateProfileSection(
        double length
    );

    // Region-kind mutations used by the authoring workflow. All of them
    // leave the section valid over its stored length and throw
    // std::invalid_argument for malformed input.
    //
    // Conversions preserve the section's current authored length; they are
    // no-ops when the section already has the target kind.
    void convertSectionToRateProfiles(AuthoredTrackSection& section);
    void convertSectionToPlanarArc(AuthoredTrackSection& section);

    // Planar-arc parameter edits enforcing the authoring length policy:
    // the stored length stays authoritative for radius edits (the swept
    // angle absorbs the change, preserving its sign), a swept-angle edit
    // moves the stored length to |angle| * radius, and tilt/bank edits
    // never touch the length.
    void setPlanarArcRadius(AuthoredTrackSection& section, double radius);
    void setPlanarArcSweptAngle(
        AuthoredTrackSection& section,
        double sweptAngle
    );
    void setPlanarArcPlaneTilt(
        AuthoredTrackSection& section,
        double planeTilt
    );
    void setPlanarArcBankChange(
        AuthoredTrackSection& section,
        double bankChange
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

        // Inserts one copy of section immediately after the existing section
        // at index; later sections shift back by one. Throws
        // std::out_of_range for an invalid index and std::invalid_argument
        // when the inserted section fails its local-domain validation, so
        // malformed regions cannot enter the document.
        void insertSectionAfter(
            std::size_t index,
            const AuthoredTrackSection& section);

        // Inserts an exact independent copy of the section at index
        // immediately after it. Every profile is owned by value, so the
        // duplicate shares no mutable state with the original. Throws
        // std::out_of_range for an invalid index.
        void duplicateSection(std::size_t index);

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

    // Single-section document whose authored rate profiles reproduce the
    // original demonstration centerline behavior.
    [[nodiscard]] AuthoredTrack createDefaultAuthoredTrack();

    // Clean default document: exactly one Rate/Profile Region, length 60,
    // straight/neutral zero-rate channels, no demo layout.
    [[nodiscard]] AuthoredTrack createNewDocument();

    // Chains every authored section through the rider-local roll/pitch/yaw
    // rate integration into one continuous centerline. Each section starts
    // where the previous one ended; returned distances are cumulative from
    // the start of the track and strictly increase. The joint sample between
    // consecutive sections appears exactly once.
    //
    // Throws std::invalid_argument when the track is empty, a section's
    // channels do not cover [0, sectionLength] exactly (or are otherwise
    // malformed), or the integration spacing is not finite and positive.
    [[nodiscard]] std::vector<RiderLocalGeometryState> integrateAuthoredTrack(
        const AuthoredTrack& track,
        double integrationSpacing
    );
}
