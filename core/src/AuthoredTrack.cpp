#include <quantum/coaster/AuthoredTrack.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace quantum::coaster
{
    namespace
    {
        // Single-construction access into GeometryRegion until more
        // constructions exist; keeps dispatch sites uniform.
        [[nodiscard]] const PlanarArcRegion& planarArcConstruction(
            const GeometryRegion& region)
        {
            const PlanarArcRegion* construction =
                std::get_if<PlanarArcRegion>(&region.construction);

            if (construction == nullptr)
            {
                throw std::logic_error(
                    "Unsupported geometry construction kind."
                );
            }

            return *construction;
        }

        [[nodiscard]] PlanarArcRegion& planarArcConstruction(
            GeometryRegion& region)
        {
            return const_cast<PlanarArcRegion&>(
                planarArcConstruction(
                    static_cast<const GeometryRegion&>(region)));
        }

        void validateSectionLocalDomain(
            const AuthoredTrackSection& section)
        {
            // Track sections must author their content over the canonical
            // section-local domain [0, section.length] so that chaining and
            // rescaling stay unambiguous. Per-kind validation remains
            // authoritative for finer defects.
            std::visit(
                [&section](const auto& authored)
                {
                    using AuthoredRegion =
                        std::decay_t<decltype(authored)>;

                    if constexpr (std::is_same_v<
                        AuthoredRegion, RateProfileRegion>)
                    {
                        validateGeometricSection(
                            authored.rateProfiles,
                            section.length
                        );
                    }
                    else
                    {
                        validatePlanarArcRegion(
                            planarArcConstruction(authored),
                            section.length
                        );
                    }
                },
                section.region
            );
        }

        // Single zero-rate Linear segment covering [0, length].
        [[nodiscard]] ChannelProfile createZeroRateChannel(
            const double length)
        {
            ChannelProfile channel;
            channel.segments.push_back(ProfileSegment{
                channel.nextSegmentId,
                math::ScalarTransition{
                    .domainBegin = 0.0,
                    .domainEnd = length,
                    .valueBegin = 0.0,
                    .valueEnd = 0.0,
                    .transitionType = math::TransitionType::Linear
                }
            });
            ++channel.nextSegmentId;
            return channel;
        }

        [[nodiscard]] AuthoredTrackSection createDefaultSection()
        {
            return createRateProfileSection(defaultNewSectionLength);
        }

        // Builds one single-segment channel covering [0, length]. Used for
        // documents whose channels are authored wholesale.
        [[nodiscard]] ChannelProfile createSingleSegmentChannel(
            const double length,
            const double valueBegin,
            const double valueEnd,
            const math::TransitionType transitionType)
        {
            ChannelProfile channel;
            channel.segments.push_back(ProfileSegment{
                channel.nextSegmentId,
                math::ScalarTransition{
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
    }

    double sectionLength(const AuthoredTrackSection& section)
    {
        validateSectionLocalDomain(section);
        return section.length;
    }

    AuthoredTrackSection createRateProfileSection(const double length)
    {
        if (!std::isfinite(length) || length <= 0.0)
        {
            throw std::invalid_argument(
                "An authored track section requires a positive finite "
                "length."
            );
        }

        AuthoredTrackSection section;
        section.length = length;
        section.rateProfileRegion().rateProfiles.pitch =
            createZeroRateChannel(length);
        section.rateProfileRegion().rateProfiles.yaw =
            createZeroRateChannel(length);
        section.rateProfileRegion().rateProfiles.roll =
            createZeroRateChannel(length);
        return section;
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

        // Rescaling requires a well-formed current state so the applied
        // policy is derived from a real [0, oldLength] coverage.
        const double oldLength = sectionLength(section);

        if (newLength == oldLength)
        {
            return;
        }

        const double scale = newLength / oldLength;

        std::visit(
            [&scale, newLength](auto& authored)
            {
                using AuthoredRegion = std::decay_t<decltype(authored)>;

                if constexpr (std::is_same_v<
                    AuthoredRegion, RateProfileRegion>)
                {
                    for (ChannelProfile* channel :
                        {&authored.rateProfiles.pitch,
                         &authored.rateProfiles.yaw,
                         &authored.rateProfiles.roll})
                    {
                        // Multiplying identical boundary doubles by one
                        // scale factor is deterministic, so shared
                        // boundaries stay contiguous on both sides; the
                        // outer endpoints are pinned exactly.
                        for (ProfileSegment& segment : channel->segments)
                        {
                            segment.transition.domainBegin *= scale;
                            segment.transition.domainEnd *= scale;
                        }

                        channel->segments.front().transition.domainBegin = 0.0;
                        channel->segments.back().transition.domainEnd = newLength;
                    }
                }
                else
                {
                    std::visit(
                        [&scale, newLength](auto& construction)
                        {
                            using Construction =
                                std::decay_t<decltype(construction)>;

                            if constexpr (std::is_same_v<
                                Construction, PlanarArcRegion>)
                            {
                                // Length edits keep the authored radius and
                                // sweep sign designer-authoritative; the
                                // swept angle absorbs the new length.
                                construction.sweptAngle =
                                    (construction.sweptAngle < 0.0
                                        ? -1.0
                                        : 1.0)
                                    * newLength
                                    / construction.radius;
                            }
                            else
                            {
                                static_assert(
                                    sizeof(Construction) == 0,
                                    "Unhandled geometry construction.");
                            }
                        },
                        authored.construction
                    );
                }
            },
            section.region
        );

        section.length = newLength;
    }

    void convertSectionToRateProfiles(AuthoredTrackSection& section)
    {
        if (section.kind == RegionKind::RateProfiles)
        {
            return;
        }

        // Conversions preserve the currently authored length so the track
        // shape at the boundary is unaffected by the kind switch itself.
        const double length = sectionLength(section);
        section = createRateProfileSection(length);
    }

    void convertSectionToPlanarArc(AuthoredTrackSection& section)
    {
        if (section.kind == RegionKind::Geometry)
        {
            return;
        }

        const double length = sectionLength(section);

        AuthoredTrackSection converted;
        converted.kind = RegionKind::Geometry;
        converted.length = length;

        // The default radius is designer-authoritative for the conversion;
        // the sweep follows from preserving the current length.
        PlanarArcRegion arc;
        arc.sweptAngle = length / arc.radius;
        converted.region = GeometryRegion{std::move(arc)};

        section = std::move(converted);
    }

    void setPlanarArcRadius(AuthoredTrackSection& section, const double radius)
    {
        if (section.kind != RegionKind::Geometry)
        {
            throw std::invalid_argument(
                "Planar-arc parameter edits require a geometry region."
            );
        }

        if (!std::isfinite(radius) || radius <= 0.0)
        {
            throw std::invalid_argument(
                "A planar-arc radius must be positive and finite."
            );
        }

        // The stored length stays authoritative; the swept angle absorbs
        // the radius change while preserving its turn direction.
        PlanarArcRegion& arc =
            planarArcConstruction(std::get<GeometryRegion>(section.region));
        const double length = sectionLength(section);

        arc.radius = radius;
        arc.sweptAngle =
            (arc.sweptAngle < 0.0 ? -1.0 : 1.0) * length / radius;

        validatePlanarArcRegion(arc, section.length);
    }

    void setPlanarArcSweptAngle(
        AuthoredTrackSection& section,
        const double sweptAngle)
    {
        if (section.kind != RegionKind::Geometry)
        {
            throw std::invalid_argument(
                "Planar-arc parameter edits require a geometry region."
            );
        }

        if (!std::isfinite(sweptAngle) || sweptAngle == 0.0)
        {
            throw std::invalid_argument(
                "A planar-arc swept angle must be finite and nonzero."
            );
        }

        PlanarArcRegion& arc =
            planarArcConstruction(std::get<GeometryRegion>(section.region));

        // A designer-authored sweep defines the resulting length.
        arc.sweptAngle = sweptAngle;
        section.length = planarArcLength(arc);

        validatePlanarArcRegion(arc, section.length);
    }

    void setPlanarArcPlaneTilt(
        AuthoredTrackSection& section,
        const double planeTilt)
    {
        if (section.kind != RegionKind::Geometry)
        {
            throw std::invalid_argument(
                "Planar-arc parameter edits require a geometry region."
            );
        }

        if (!std::isfinite(planeTilt))
        {
            throw std::invalid_argument(
                "A planar-arc plane tilt must be finite."
            );
        }

        PlanarArcRegion& arc =
            planarArcConstruction(std::get<GeometryRegion>(section.region));
        arc.planeTilt = planeTilt;

        validatePlanarArcRegion(arc, section.length);
    }

    void setPlanarArcBankChange(
        AuthoredTrackSection& section,
        const double bankChange)
    {
        if (section.kind != RegionKind::Geometry)
        {
            throw std::invalid_argument(
                "Planar-arc parameter edits require a geometry region."
            );
        }

        if (!std::isfinite(bankChange))
        {
            throw std::invalid_argument(
                "A planar-arc bank change must be finite."
            );
        }

        PlanarArcRegion& arc =
            planarArcConstruction(std::get<GeometryRegion>(section.region));
        arc.bankChange = bankChange;

        validatePlanarArcRegion(arc, section.length);
    }

    // ----------------------------------------------------------------
    // LayoutMode
    // ----------------------------------------------------------------

    const char* layoutModeToString(const LayoutMode mode) noexcept
    {
        switch (mode)
        {
        case LayoutMode::Circuit: return "Circuit";
        case LayoutMode::Shuttle: return "Shuttle";
        }
        return "Circuit";
    }

    LayoutMode layoutModeFromString(const std::string_view name)
    {
        if (name == "Circuit") return LayoutMode::Circuit;
        if (name == "Shuttle") return LayoutMode::Shuttle;
        throw std::invalid_argument(
            std::string("Unknown layout mode '") + std::string(name)
            + "' (expected 'Circuit' or 'Shuttle')");
    }

    LayoutMode AuthoredTrack::layoutMode() const noexcept
    {
        return layoutMode_;
    }

    void AuthoredTrack::setLayoutMode(const LayoutMode mode) noexcept
    {
        layoutMode_ = mode;
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

    void AuthoredTrack::insertSectionAfter(
        const std::size_t index,
        const AuthoredTrackSection& section)
    {
        if (index >= sections_.size())
        {
            throw std::out_of_range(
                "Authored track section index is out of range."
            );
        }

        // Reject malformed input before it can enter the document; the
        // candidate/commit gate would also catch it, but a rejected insert
        // should be identifiable at the Core operation itself.
        validateSectionLocalDomain(section);

        sections_.insert(
            sections_.begin()
                + static_cast<std::ptrdiff_t>(index) + 1,
            section);
    }

    void AuthoredTrack::duplicateSection(const std::size_t index)
    {
        if (index >= sections_.size())
        {
            throw std::out_of_range(
                "Authored track section index is out of range."
            );
        }

        // Copy explicitly rather than inserting from a reference into the
        // same vector so reallocation during the insert can never alias.
        AuthoredTrackSection duplicate = sections_.at(index);

        sections_.insert(
            sections_.begin()
                + static_cast<std::ptrdiff_t>(index) + 1,
            std::move(duplicate));
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
        constexpr double defaultDemoLength = 180.0;

        AuthoredTrack track;

        track.appendSection();
        setSectionLength(track.section(0), defaultDemoLength);

        AuthoredTrackSection& section = track.section(0);
        section.rateProfileRegion().rateProfiles.roll = createSingleSegmentChannel(
            defaultDemoLength,
            0.0,
            0.024,
            math::TransitionType::Smootherstep
        );
        section.rateProfileRegion().rateProfiles.pitch = createSingleSegmentChannel(
            defaultDemoLength,
            0.018,
            -0.010,
            math::TransitionType::CosineEaseInOut
        );
        section.rateProfileRegion().rateProfiles.yaw = createSingleSegmentChannel(
            defaultDemoLength,
            0.004,
            0.022,
            math::TransitionType::Smoothstep
        );

        return track;
    }

    AuthoredTrack createNewDocument()
    {
        AuthoredTrack track;
        track.appendSection();
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
                std::visit(
                    [&](const auto& authored)
                        -> std::vector<RiderLocalGeometryState>
                    {
                        using AuthoredRegion =
                            std::decay_t<decltype(authored)>;

                        if constexpr (std::is_same_v<
                            AuthoredRegion, RateProfileRegion>)
                        {
                            return integrateLocalRollPitchYawRateProfiles(
                                position,
                                frame,
                                authored.rateProfiles.roll,
                                authored.rateProfiles.pitch,
                                authored.rateProfiles.yaw,
                                section.length,
                                integrationSpacing
                            );
                        }
                        else
                        {
                            return integratePlanarArcRegion(
                                position,
                                frame,
                                planarArcConstruction(authored),
                                integrationSpacing
                            );
                        }
                    },
                    section.region
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
