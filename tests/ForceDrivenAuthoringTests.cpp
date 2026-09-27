// Geometry Authoring M0: editable force-driven region authoring.
//
// These tests exercise the authoring pipeline a user drives when they create
// a Force-Based region and edit its three target profiles: Core region
// creation, the editor's row routing, profile editing operations, region
// length changes, force-driven regeneration and its failures, serialization,
// and the document history. No ImGui, renderer, or windowing code is
// involved; the editor pieces used here are the same pure sources the
// application links.

#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>
#include <quantum/editor/TransitionEditorModel.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using namespace quantum::coaster;
    using namespace quantum::editor;

    constexpr double piRadians = 3.14159265358979323846;

    class TestFailure : public std::runtime_error
    {
    public:
        explicit TestFailure(const std::string& message)
            : std::runtime_error(message)
        {
        }
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition) throw TestFailure(std::string(message));
    }

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view message)
    {
        if (!(std::abs(actual - expected) <= tolerance))
        {
            throw TestFailure(
                std::string(message) + " (expected " + std::to_string(expected)
                + ", got " + std::to_string(actual) + ")");
        }
    }

    template <typename ExpectedException, typename Callable>
    void requireThrows(Callable&& callable, const std::string_view message)
    {
        try
        {
            callable();
        }
        catch (const ExpectedException&)
        {
            return;
        }
        catch (const std::exception& error)
        {
            throw TestFailure(
                std::string(message) + " (threw the wrong exception: "
                + error.what() + ")");
        }
        throw TestFailure(std::string(message) + " (nothing was thrown)");
    }

    ForceDrivenRegion& forceRegion(AuthoredTrackSection& section)
    {
        return std::get<ForceDrivenRegion>(
            std::get<GeometryRegion>(section.region).construction);
    }

    const ForceDrivenRegion& forceRegion(const AuthoredTrackSection& section)
    {
        return std::get<ForceDrivenRegion>(
            std::get<GeometryRegion>(section.region).construction);
    }

    // A straight lead-in, a force-driven middle, and a trailing rate-profile
    // region. That is the smallest document that exercises a force-driven
    // region between authored neighbours.
    AuthoredTrack authoredTrackWithForceRegion()
    {
        auto track = createNewDocument();
        auto setup = track.coasterSetup();
        setup.heartline.offsetMeters = 0.0;
        track.setCoasterSetup(setup);
        setSectionLength(track.section(0), 20.0);
        track.insertSectionAfter(0, createForceDrivenSection(40.0));
        track.appendSection();
        setSectionLength(track.section(2), 20.0);
        return track;
    }

    // Index of the generated sample closest to a whole-track distance. The
    // tests assert on a sample inside a specific region, not on a fixed
    // index that a different region length would move.
    std::size_t sampleNearestDistance(
        const std::vector<TrackKinematicState>& states,
        const double distance)
    {
        std::size_t best = 0;
        double bestDistance = 0.0;
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            const double candidate = std::abs(states[i].distance - distance);
            if (i == 0 || candidate < bestDistance)
            {
                best = i;
                bestDistance = candidate;
            }
        }
        return best;
    }

    // Every one of the three authored channels, in slot order.
    std::vector<const ChannelProfile*> forceChannels(
        const AuthoredTrackSection& section)
    {
        const ForceDrivenRegion& force = forceRegion(section);
        return {&force.rollRate, &force.targetNormalG, &force.targetLateralG};
    }

    void requireCoversExactly(
        const ChannelProfile& profile,
        const double length,
        const std::string_view message)
    {
        // The document validator is authoritative for coverage, C0
        // continuity, and segment id uniqueness.
        validateChannelProfile(profile, length);
        require(!profile.segments.empty(), message);
        require(profile.segments.front().transition.domainBegin == 0.0, message);
        require(profile.segments.back().transition.domainEnd == length, message);
        for (std::size_t i = 1; i < profile.segments.size(); ++i)
        {
            require(
                profile.segments[i].transition.domainBegin
                    == profile.segments[i - 1].transition.domainEnd,
                message);
            require(
                profile.segments[i].transition.valueBegin
                    == profile.segments[i - 1].transition.valueEnd,
                message);
        }
        SegmentId highest = invalidSegmentId;
        for (const ProfileSegment& segment : profile.segments)
        {
            require(segment.id != invalidSegmentId, message);
            require(segment.id > highest, message);
            highest = segment.id;
        }
        require(profile.nextSegmentId > highest, message);
    }

    // ---------------------------------------------------------------------
    // 1. Region creation
    // ---------------------------------------------------------------------

    void forceDrivenCreationProducesValidCoveringProfiles()
    {
        for (const double length : {5.0, 40.0, 137.5})
        {
            const AuthoredTrackSection section =
                createForceDrivenSection(length);
            require(section.kind == RegionKind::Geometry, "created region kind");
            require(isForceDrivenSection(section), "created region construction");
            require(section.length == length, "created region length");
            require(sectionLength(section) == length, "created region validates");

            const ForceDrivenRegion& force = forceRegion(section);
            // A fresh region is editable immediately: each channel already
            // covers its whole authored length.
            requireCoversExactly(force.rollRate, length, "roll rate coverage");
            requireCoversExactly(force.targetNormalG, length, "normal G coverage");
            requireCoversExactly(force.targetLateralG, length, "lateral G coverage");

            require(force.targetNormalG.segments.size() == 1, "single normal segment");
            requireNear(
                force.targetNormalG.segments.front().transition.valueBegin,
                1.0, 0.0, "initial normal G target");
            requireNear(
                force.targetNormalG.segments.front().transition.valueEnd,
                1.0, 0.0, "initial normal G target");
            requireNear(
                force.targetLateralG.segments.front().transition.valueBegin,
                0.0, 0.0, "initial lateral G target");
            requireNear(
                force.rollRate.segments.front().transition.valueBegin,
                0.0, 0.0, "initial roll rate");
        }

        // The editor's typed-create path mirrors the Circular Arc path: take
        // the default section and let Core own the construction.
        auto appended = createNewDocument();
        appended.appendSection();
        convertSectionToForceDriven(appended.section(appended.sectionCount() - 1));
        require(isForceDrivenSection(appended.section(appended.sectionCount() - 1)),
            "appended force region");
        require(appended.section(appended.sectionCount() - 1).length
                == defaultNewSectionLength,
            "appended force region keeps the default length");

        auto prepended = createNewDocument();
        prepended.prependSection();
        convertSectionToForceDriven(prepended.section(0));
        require(isForceDrivenSection(prepended.section(0)),
            "prepended force region");
        require(computeRegionStations(prepended, 0).startStation == 0.0,
            "prepended force region leads the ordering");

        auto inserted = createNewDocument();
        inserted.insertSectionAfter(0, createForceDrivenSection(defaultNewSectionLength));
        require(isForceDrivenSection(inserted.section(1)), "inserted force region");
        require(inserted.sectionCount() == 2, "inserted region shifts the ordering");

        // Converting an existing region keeps its authored length and its
        // region track style, and is a no-op once already force-driven.
        auto converted = createNewDocument();
        setSectionLength(converted.section(0), 33.0);
        auto& style = converted.section(0).trackStyleOverrides;
        style.enabled = true;
        style.spineRadius = std::optional<double>{0.5};
        convertSectionToForceDriven(converted.section(0));
        require(converted.section(0).length == 33.0, "conversion preserves length");
        require(converted.section(0).trackStyleOverrides.enabled
                && converted.section(0).trackStyleOverrides.spineRadius
                    == style.spineRadius,
            "conversion preserves region track style");
        const ChannelProfile beforeConvert =
            forceRegion(converted.section(0)).targetNormalG;
        convertSectionToForceDriven(converted.section(0));
        require(forceRegion(converted.section(0)).targetNormalG == beforeConvert,
            "converting an already force-driven region is a no-op");

        requireThrows<std::invalid_argument>(
            [] { createForceDrivenSection(0.0); },
            "zero-length force region is rejected");
        requireThrows<std::invalid_argument>(
            [] { createForceDrivenSection(std::nan("")); },
            "non-finite force region length is rejected");
    }

    // ---------------------------------------------------------------------
    // 2. Row routing and display units
    // ---------------------------------------------------------------------

    void sectionProfileChannelRoutesTheRegionAuthoringModel()
    {
        auto rates = createNewDocument();
        for (const auto* channel :
            {&sectionProfileChannel(rates.section(0), ProfileChannel::Roll),
             &sectionProfileChannel(rates.section(0), ProfileChannel::Pitch),
             &sectionProfileChannel(rates.section(0), ProfileChannel::Yaw)})
        {
            require(channel != nullptr, "rate rows resolve");
        }
        sectionProfileChannel(
            rates.section(0), ProfileChannel::Roll).segments.front()
            .transition.valueEnd = 0.25;
        requireNear(
            rates.section(0).rateProfileRegion().rateProfiles.roll.segments
                .front().transition.valueEnd,
            0.25, 0.0, "rate roll row edits the authored roll channel");

        // A force-driven region authors the rider-force target on the same
        // rider-local axes, so the Pitch row is the normal G target, the Yaw
        // row the lateral G target, and the Roll row the authored roll rate.
        const AuthoredTrackSection force = createForceDrivenSection(30.0);
        require(
            &sectionProfileChannel(force, ProfileChannel::Pitch)
                == &forceRegion(force).targetNormalG,
            "Pitch row is the normal G target");
        require(
            &sectionProfileChannel(force, ProfileChannel::Yaw)
                == &forceRegion(force).targetLateralG,
            "Yaw row is the lateral G target");
        require(
            &sectionProfileChannel(force, ProfileChannel::Roll)
                == &forceRegion(force).rollRate,
            "Roll row is the authored roll rate");

        auto& forceCopy = const_cast<AuthoredTrackSection&>(force);
        sectionProfileChannel(
            forceCopy, ProfileChannel::Pitch).segments.front()
            .transition.valueEnd = 2.5;
        requireNear(
            forceRegion(force).targetNormalG.segments.front()
                .transition.valueEnd,
            2.5, 0.0, "editing the Pitch row only moves the normal G target");
        requireNear(
            forceRegion(force).targetLateralG.segments.front()
                .transition.valueEnd,
            0.0, 0.0, "lateral G target untouched by a Pitch row edit");
    }

    void rowStyleUnitsAccountForTheCoordinateScale()
    {
        for (const double scale : {1.0, 0.5, 2.0})
        {
            const std::array<ProfileRowStyle, profileChannelCount> rates =
                rateProfileRowStyles(scale);
            const std::array<ProfileRowStyle, profileChannelCount> force =
                forceDrivenRowStyles(scale);

            // Degrees per meter is an authored per-coordinate-unit rate
            // divided by the document's coordinate-unit scale.
            for (std::size_t i = 0; i < profileChannelCount; ++i)
            {
                requireNear(
                    rates[i].authoredToDisplay,
                    degreesPerRadian / scale, 1.0e-15,
                    "rate row degrees-per-meter conversion");
                requireNear(
                    rates[i].authoredToDisplay * rates[i].displayToAuthored,
                    1.0, 1.0e-15, "rate row conversion round trip");
                requireNear(
                    force[static_cast<std::size_t>(ProfileChannel::Roll)]
                        .authoredToDisplay,
                    degreesPerRadian / scale, 1.0e-15,
                    "force roll rate degrees-per-meter conversion");
            }

            // The dimensionless G rows are scale independent.
            for (const ProfileChannel channel :
                {ProfileChannel::Pitch, ProfileChannel::Yaw})
            {
                const ProfileRowStyle& style =
                    force[static_cast<std::size_t>(channel)];
                requireNear(style.authoredToDisplay, 1.0, 0.0,
                    "G target display is the authored target");
                requireNear(style.displayToAuthored, 1.0, 0.0,
                    "G target edit reaches the authored target");
                require(std::string_view(style.valueUnitLabel) == "G",
                    "G target unit label");
                require(style.diagnostic == ProfileRowDiagnostic::ForceTarget,
                    "G target diagnostic");
            }
            require(force[static_cast<std::size_t>(ProfileChannel::Roll)]
                        .diagnostic
                    == ProfileRowDiagnostic::IntegratedRotation,
                "roll rate still reports integrated rotation");
            require(rates[static_cast<std::size_t>(ProfileChannel::Pitch)]
                        .diagnostic
                    == ProfileRowDiagnostic::Curvature,
                "rate pitch row still reports curvature");

            // A flat roll profile fits to the same displayed degree range on
            // every row of the same family.
            const std::array<double, 1> flat{0.0};
            const GraphValueRange rollRange = fitSymmetricGraphRange(
                flat,
                force[static_cast<std::size_t>(ProfileChannel::Roll)]
                    .minimumGraphMagnitude);
            requireNear(
                rollRange.magnitude()
                    * force[static_cast<std::size_t>(ProfileChannel::Roll)]
                        .authoredToDisplay,
                defaultGraphMagnitude(ProfileChannel::Roll)
                    * degreesPerRadian,
                1.0e-9,
                "flat roll profile fits the same displayed degrees per meter");
        }

        for (const double invalid : {0.0, -1.0, std::nan("")})
        {
            requireThrows<std::invalid_argument>(
                [&] { rateProfileRowStyles(invalid); },
                "invalid coordinate scale is rejected");
            requireThrows<std::invalid_argument>(
                [&] { forceDrivenRowStyles(invalid); },
                "invalid coordinate scale is rejected");
        }
    }

    void viewOnlyRowOperationsDoNotMutateAuthoredData()
    {
        const auto track = authoredTrackWithForceRegion();
        const std::string before = serializeCoasterDocument(track);

        for (const double scale : {1.0, 0.25})
        {
            const auto styles = forceDrivenRowStyles(scale);
            for (const ProfileRowStyle& style : styles)
            {
                const std::array<double, 2> values{0.5, -1.25};
                (void)fitSymmetricGraphRange(
                    values, style.minimumGraphMagnitude);
                (void)scaleGraphRange({-1.0, 1.0}, 0.8);
                (void)expandGraphRangeToInclude({-1.0, 1.0}, 7.5);
                (void)graphValueToNormalized(0.25, {-1.0, 1.0});
                (void)normalizedToGraphValue(0.25, {-1.0, 1.0});
                (void)graphValueUnitsPerPixel({-1.0, 1.0}, 400.0);
                (void)style.authoredToDisplay;
                (void)style.displayToAuthored;
            }
            const auto& profiles = forceChannels(track.section(1));
            for (const ChannelProfile* profile : profiles)
            {
                (void)extractSemanticProfileMarkers(*profile);
                (void)profileBoundaryMoveBounds(
                    *profile, profile->segments.front().id,
                    ScalarProfileEndpoint::End);
                (void)proposeMarkerValueDrag(1.0, -3.0, 0.01, 1.0);
                (void)proposeBoundaryDistanceDrag(
                    5.0, 4.0, 0.1, 1.0, ProfileBoundaryMoveBounds{1.0, 30.0});
            }
        }
        require(serializeCoasterDocument(track) == before,
            "view-only row operations never touch authored data");
    }

    // ---------------------------------------------------------------------
    // 3. Profile editing through the shared Core operations
    // ---------------------------------------------------------------------

    void forceProfileEditingPreservesCoverageAndIdentity()
    {
        auto track = authoredTrackWithForceRegion();
        const double length = sectionLength(track.section(1));
        auto& section = track.section(1);
        const SegmentId lateralTailBefore =
            forceRegion(section).targetLateralG.segments.back().id;

        // The same operations the graph's drag, numeric, and context-menu
        // paths issue.
        ChannelProfile& normal = forceRegion(section).targetNormalG;
        const SegmentId right = splitChannelSegment(normal, normal.segments.front().id, 12.0);
        require(right != invalidSegmentId, "split allocates a fresh id");
        require(normal.segments.size() == 2, "split adds a segment");
        requireCoversExactly(normal, length, "coverage survives a split");

        setChannelSegmentValue(normal, normal.segments.front().id, ProfileBoundary::End, 0.4);
        requireCoversExactly(normal, length, "coverage survives a value edit");
        requireNear(
            normal.segments[1].transition.valueBegin, 0.4, 0.0,
            "an interior value edit is shared by the adjoining segment");

        moveChannelSegmentBoundary(
            normal, normal.segments.front().id, ProfileBoundary::End, 18.0);
        requireCoversExactly(normal, length, "coverage survives a boundary move");
        requireNear(
            normal.segments[0].transition.domainEnd, 18.0, 0.0,
            "the moved boundary lands on the requested distance");
        requireNear(
            normal.segments[0].transition.valueEnd, 0.4, 0.0,
            "a horizontal boundary move does not change values");

        requireThrows<std::invalid_argument>(
            [&]
            {
                moveChannelSegmentBoundary(
                    normal, normal.segments.front().id,
                    ProfileBoundary::Begin, 0.0);
            },
            "the region start boundary stays pinned");
        requireThrows<std::invalid_argument>(
            [&]
            {
                moveChannelSegmentBoundary(
                    normal, normal.segments.back().id,
                    ProfileBoundary::End, length);
            },
            "the region end boundary stays pinned");

        // All existing transition shapes remain available per segment.
        for (const quantum::math::TransitionType type : {
            quantum::math::TransitionType::Linear,
            quantum::math::TransitionType::Smoothstep,
            quantum::math::TransitionType::Smootherstep,
            quantum::math::TransitionType::SeventhOrderSmoothstep,
            quantum::math::TransitionType::CosineEaseInOut,
            quantum::math::TransitionType::SineEaseIn,
            quantum::math::TransitionType::SineEaseOut,
            quantum::math::TransitionType::CubicEaseInOut})
        {
            auto* transition = findChannelSegmentTransition(normal, right);
            require(transition != nullptr, "segment is addressable by id");
            transition->transitionType = type;
            requireCoversExactly(normal, length, "every transition shape validates");
            require(
                quantum::math::evaluateScalarTransition(
                    *transition, transition->domainBegin)
                    == transition->valueBegin,
                "the chosen shape evaluates at its authored begin value");
        }

        const SegmentId survivor = removeChannelSegment(normal, right);
        require(survivor != invalidSegmentId, "remove returns the survivor");
        require(normal.segments.size() == 1, "remove drops one segment");
        requireCoversExactly(normal, length, "coverage survives a removal");
        requireThrows<std::invalid_argument>(
            [&]
            {
                ChannelProfile single = normal;
                removeChannelSegment(single, single.segments.front().id);
            },
            "the last remaining segment cannot be removed");

        // The three channels are independent authoring state.
        for (const ChannelProfile* profile : forceChannels(section))
        {
            requireCoversExactly(*profile, length, "every channel stays valid");
        }
        require(
            forceRegion(section).targetLateralG.segments.back().id
                == lateralTailBefore,
            "editing the normal G target leaves the other channels alone");
        requireNear(
            forceRegion(section).rollRate.segments.front()
                .transition.valueEnd,
            0.0, 0.0, "roll rate untouched by normal G edits");
    }

    void regionLengthChangesPreserveProfileCoverage()
    {
        auto track = authoredTrackWithForceRegion();
        auto& section = track.section(1);
        forceRegion(section).targetNormalG.segments.front().transition.valueEnd = 0.35;
        forceRegion(section).rollRate.segments.front().transition.valueEnd = 0.1;
        splitChannelSegment(
            forceRegion(section).rollRate,
            forceRegion(section).rollRate.segments.front().id, 9.0);
        const SegmentId rollTailBefore =
            forceRegion(section).rollRate.segments.back().id;

        setSectionLength(section, 95.0);
        require(sectionLength(section) == 95.0, "new length is accepted");
        for (const ChannelProfile* profile : forceChannels(section))
        {
            requireCoversExactly(*profile, 95.0, "coverage follows the new length");
        }
        requireNear(
            forceRegion(section).targetNormalG.segments.front()
                .transition.valueEnd,
            0.35, 0.0, "values are preserved by a length change");
        requireNear(
            forceRegion(section).rollRate.segments.back()
                .transition.valueEnd,
            0.1, 0.0, "roll rate values are preserved");
        require(
            forceRegion(section).rollRate.segments.front().id
                != forceRegion(section).rollRate.segments.back().id,
            "the split survived the length change");
        require(
            forceRegion(section).rollRate.segments.front().transition
                    .domainEnd
                == 9.0 * (95.0 / 40.0),
            "the interior boundary is rescaled with the region");
        require(
            forceRegion(section).rollRate.segments.back().id == rollTailBefore,
            "segment ids are preserved by a length change");
        require(
            forceRegion(section).rollRate.segments.back().transition.domainEnd
                == 95.0,
            "the last boundary is pinned to the new length");

        // The generated geometry follows the new authored length.
        const auto centerline = createCenterlineVisualization(track);
        require(centerline.samples.size() > 2, "centerline is generated");
        const double trackLength = centerline.samples.back().distance
            - centerline.samples.front().distance;
        requireNear(trackLength, 20.0 + 95.0 + 20.0, 1.0e-6,
            "the whole track covers the new region length");

        // An impossible length is rejected without mutating the region.
        const ChannelProfile before =
            forceRegion(section).targetLateralG;
        requireThrows<std::invalid_argument>(
            [&] { setSectionLength(section, -5.0); },
            "a non-positive region length is rejected");
        require(sectionLength(section) == 95.0,
            "a rejected length leaves the region untouched");
        require(forceRegion(section).targetLateralG == before,
            "a rejected length leaves authored targets untouched");
    }

    // ---------------------------------------------------------------------
    // 4. Force-driven geometry generation
    // ---------------------------------------------------------------------

    void editingProfilesRegeneratesTrackGeometry()
    {
        auto track = authoredTrackWithForceRegion();
        auto physical = track.physicalSettings();
        physical.initialSpeed = 30.0;
        track.setPhysicalSettings(physical);
        const double mu = track.physicalSettings().metersPerCoordinateUnit;
        const double speed = track.physicalSettings().initialSpeed;

        // A constant +1.0 G normal target on a level start is free riding:
        // it is exactly the specific force gravity already applies, so the
        // region generates straight. That is what makes it a safe default.
        const auto baselineSamples =
            createCenterlineVisualization(track).samples;
        const auto baselineKinematics = integrateAuthoredTrackKinematics(
            track, centerlineVisualizationSampleSpacing);
        require(baselineSamples.size() > 2, "baseline centerline is generated");
        requireNear(
            glm::length(baselineKinematics[3].centerlineCurvature),
            0.0, 0.0,
            "a level +1.0 G normal target generates a straight track");
        const glm::dvec3 baselineEnd = baselineSamples.back().position;

        AuthoredTrackEditTransaction transaction{track};
        ChannelProfile& normal =
            forceRegion(transaction.candidate().section(1)).targetNormalG;
        setChannelSegmentValue(
            normal, normal.segments.front().id, ProfileBoundary::Begin, 0.4);
        setChannelSegmentValue(
            normal, normal.segments.front().id, ProfileBoundary::End, 0.4);
        const auto loads = evaluateRiderLoadDiagnostics(transaction.candidate());
        transaction.requireAcceptableRiderLoads(loads);
        const auto editedSamples = createCenterlineVisualization(
            transaction.candidate()).samples;
        const auto editedKinematics = integrateAuthoredTrackKinematics(
            transaction.candidate(), centerlineVisualizationSampleSpacing);
        transaction.commit(track);

        require(editedSamples.size() == baselineSamples.size(),
            "regeneration keeps the authored sampling grid");
        require(
            editedSamples.back().position != baselineEnd,
            "editing the normal G target moves the generated track");
        // The world is Z-up and a level frame already carries 1.0 G, so a
        // smaller target presses the track downward.
        require(
            editedSamples.back().position.z < baselineEnd.z,
            "a sub-1.0 G normal target presses the track downward");
        // The world is Z-up and the level entry frame already carries 1.0 G,
        // so a 0.6 G shortfall is exactly the curvature of the dive at the
        // region entry. This is the speed-dependent behaviour of the
        // integrator: the same target gives a different shape at a different
        // entry speed.
        const std::size_t regionEntry = sampleNearestDistance(
            editedKinematics, 20.0);
        requireNear(
            glm::length(editedKinematics[regionEntry].centerlineCurvature),
            0.6 * standardGravityAcceleration * mu / (speed * speed),
            1.0e-12,
            "the edited region uses the edited target");

        // Easing between the same endpoints regenerates a different shape
        // from the same targets, which is what a transition-shape edit does.
        const auto rampedTrack = [](const double length,
            const double endValue,
            const quantum::math::TransitionType type)
        {
            auto ramped = createNewDocument();
            setSectionLength(ramped.section(0), length);
            ramped.insertSectionAfter(0, createForceDrivenSection(length));
            ChannelProfile& profile =
                forceRegion(ramped.section(1)).targetNormalG;
            setChannelSegmentValue(
                profile, profile.segments.front().id,
                ProfileBoundary::Begin, 0.4);
            setChannelSegmentValue(
                profile, profile.segments.front().id,
                ProfileBoundary::End, endValue);
            findChannelSegmentTransition(
                profile, profile.segments.front().id)->transitionType = type;
            return ramped;
        };
        const auto linearTrack = rampedTrack(
            20.0, 1.0, quantum::math::TransitionType::Linear);
        const auto easedTrack = rampedTrack(
            20.0, 1.0,
            quantum::math::TransitionType::SeventhOrderSmoothstep);
        const auto linearSamples =
            createCenterlineVisualization(linearTrack).samples;
        const auto easedSamples =
            createCenterlineVisualization(easedTrack).samples;
        require(linearSamples.size() == easedSamples.size(),
            "both transition shapes use the same sampling grid");
        require(
            linearSamples.back().position != easedSamples.back().position,
            "a transition shape change regenerates a different track");

        // A constant lateral target on a level region turns it sideways with
        // the analytic circular-arc curvature.
        AuthoredTrackEditTransaction turn{track};
        auto& force = forceRegion(turn.candidate().section(1));
        setChannelSegmentValue(
            force.targetNormalG, force.targetNormalG.segments.front().id,
            ProfileBoundary::Begin, 1.0);
        setChannelSegmentValue(
            force.targetNormalG, force.targetNormalG.segments.front().id,
            ProfileBoundary::End, 1.0);
        setChannelSegmentValue(
            force.targetLateralG, force.targetLateralG.segments.front().id,
            ProfileBoundary::Begin, 0.7);
        setChannelSegmentValue(
            force.targetLateralG, force.targetLateralG.segments.front().id,
            ProfileBoundary::End, 0.7);
        const auto turnedSamples = createCenterlineVisualization(
            turn.candidate()).samples;
        const auto turnedKinematics = integrateAuthoredTrackKinematics(
            turn.candidate(), centerlineVisualizationSampleSpacing);
        const std::size_t turnEntry =
            sampleNearestDistance(turnedKinematics, 20.0);
        requireNear(
            glm::length(turnedKinematics[turnEntry].centerlineCurvature),
            standardGravityAcceleration * 0.7 * mu / (speed * speed),
            1.0e-12,
            "a constant lateral G target generates the analytic curvature");
        const std::size_t middle = turnedSamples.size() / 2;
        require(
            std::abs(turnedSamples[middle].position.z
                     - turnedSamples.front().position.z) < 1.0e-9,
            "a horizontal target keeps the track at one height");
        require(
            std::abs(turnedSamples[middle].position.y
                     - turnedSamples.front().position.y) > 1.0e-3,
            "a lateral target turns the track sideways");

        // Roll rate is authored directly, so it banks the generated frame
        // without needing a force target to change.
        AuthoredTrackEditTransaction banked{turn.candidate()};
        setChannelSegmentValue(
            forceRegion(banked.candidate().section(1)).rollRate,
            forceRegion(banked.candidate().section(1)).rollRate.segments
                .front().id,
            ProfileBoundary::Begin, 0.05);
        setChannelSegmentValue(
            forceRegion(banked.candidate().section(1)).rollRate,
            forceRegion(banked.candidate().section(1)).rollRate.segments
                .front().id,
            ProfileBoundary::End, 0.05);
        const auto bankedSamples = createCenterlineVisualization(
            banked.candidate()).samples;
        require(bankedSamples.size() == turnedSamples.size(),
            "banking does not change the sampling grid");
        require(
            bankedSamples[middle].frame.up != turnedSamples[middle].frame.up,
            "an authored roll rate banks the generated frame");
    }

    void infeasibleTargetsReportTheRealFailureAndKeepTheLastValidState()
    {
        // A committed document whose force-driven region is free riding, then
        // an edit that asks for a vertical circle the entry speed cannot
        // reach. A vertical circle's signed normal target is
        // n(s) = v0^2/(gR) - 2 + 3 cos(s/R), which is exactly what a
        // designer would author to pull out of a dive into a loop.
        const double radius = 10.0;
        const double half = piRadians * radius;
        const double length = 2.0 * half;
        auto track = createNewDocument();
        auto setup = track.coasterSetup();
        setup.heartline.offsetMeters = 0.0;
        track.setCoasterSetup(setup);
        setSectionLength(track.section(0), 20.0);
        track.insertSectionAfter(0, createForceDrivenSection(length));
        track.appendSection();
        setSectionLength(track.section(2), 20.0);
        track.setPhysicalSettings({std::sqrt(
            3.0 * standardGravityAcceleration * radius), 1.0,
            standardGravityAcceleration});

        const auto committed = createCenterlineVisualization(track);
        const std::string committedDocument = serializeCoasterDocument(track);
        require(committed.samples.size() > 2, "the committed region generates");

        // The infeasible edit, authored only through the shared profile
        // operations the graph issues.
        auto loop = track;
        ChannelProfile& normal = forceRegion(loop.section(1)).targetNormalG;
        splitChannelSegment(normal, normal.segments.front().id, half);
        normal.segments.front().transition.valueBegin = 4.0;
        normal.segments.front().transition.valueEnd = -2.0;
        normal.segments.front().transition.transitionType =
            quantum::math::TransitionType::CosineEaseInOut;
        normal.segments[1].transition.valueBegin = -2.0;
        normal.segments[1].transition.valueEnd = 4.0;
        normal.segments[1].transition.transitionType =
            quantum::math::TransitionType::CosineEaseInOut;

        const double barrier = radius * std::acos(-0.5);

        // The editor's own pipeline refuses the candidate and keeps the last
        // valid committed state.
        AuthoredTrackEditTransaction transaction{track};
        transaction.candidate() = loop;
        bool rejected = false;
        TrackGenerationFailure reported{};
        try
        {
            const auto candidateLoads =
                evaluateRiderLoadDiagnostics(transaction.candidate());
            transaction.requireAcceptableRiderLoads(candidateLoads);
        }
        catch (const TrackGenerationError& error)
        {
            rejected = true;
            reported = error.failure();
        }
        require(rejected, "an unreachable vertical circle is rejected");
        // The circle ODE turns singular at the summit, so error control may
        // run out of refinement just before a negative stage is observed.
        // Both are real, specific generation failures; neither is a generic
        // error standing in for the solver.
        require(
            reported.reason == TrackGenerationFailureReason::EnergeticallyUnreachable
                || reported.reason == TrackGenerationFailureReason::IntegrationFailure,
            "the reported reason is a specific generation failure");
        require(
            !std::string_view(trackGenerationFailureReasonToString(
                reported.reason)).empty(),
            "the reason has a stable label for the editor surface");
        require(reported.sectionIndex.has_value() && *reported.sectionIndex == 1,
            "the failure names the region that could not be generated");
        require(
            reported.localDistance.has_value()
                && *reported.localDistance > 0.0
                && *reported.localDistance < half,
            "the failure happens inside the region, before its summit");
        requireNear(
            *reported.localDistance, barrier, 0.01,
            "the failure locates the actual energy barrier");
        require(
            reported.speedSquared.has_value()
                && std::isfinite(*reported.speedSquared),
            "the failure reports the available speed squared");
        require(!reported.message.empty(),
            "the failure keeps Core's own explanation");
        require(!transaction.committed()
                && !transaction.selectionAfterCommit(),
            "a rejected candidate cannot publish a selection");
        require(serializeCoasterDocument(track) == committedDocument,
            "the committed document is unchanged by a rejected candidate");

        // With no refinement headroom the same intent reports the physical
        // reason directly, so the infeasibility is never disguised.
        const auto unrefined = generateAuthoredTrackKinematics(
            loop, length, {1.0e-10, 0});
        require(!unrefined, "the unrefined solve also stops");
        require(
            unrefined.error().reason
                == TrackGenerationFailureReason::EnergeticallyUnreachable,
            "without refinement the barrier reports unreachable energy");
        require(
            unrefined.error().speedSquared.has_value()
                && *unrefined.error().speedSquared < 0.0,
            "the barrier reports the negative energy squared it hit");
        require(
            unrefined.error().localDistance.has_value()
                && *unrefined.error().localDistance < length,
            "the barrier is inside the region");

        // Insufficient speed: no resolved forward speed squared at all.
        auto noSpeed = track;
        auto stoppedPhysical = noSpeed.physicalSettings();
        stoppedPhysical.initialSpeed = 0.0;
        noSpeed.setPhysicalSettings(stoppedPhysical);
        AuthoredTrackEditTransaction stopped{track};
        stopped.candidate() = noSpeed;
        bool stoppedRejected = false;
        TrackGenerationFailure stoppedFailure{};
        try
        {
            const auto candidateLoads =
                evaluateRiderLoadDiagnostics(stopped.candidate());
            stopped.requireAcceptableRiderLoads(candidateLoads);
        }
        catch (const TrackGenerationError& error)
        {
            stoppedRejected = true;
            stoppedFailure = error.failure();
        }
        require(stoppedRejected, "a target with no entry speed is rejected");
        require(
            stoppedFailure.reason
                == TrackGenerationFailureReason::InsufficientSpeed,
            "a stationary entry reports insufficient speed, not a substitute");
        require(
            stoppedFailure.sectionIndex.has_value()
                && *stoppedFailure.sectionIndex == 1,
            "the stationary entry names its region");
        require(serializeCoasterDocument(track) == committedDocument,
            "the committed document is still unchanged");

        // The last valid committed state is still what the editor shows.
        const auto after = createCenterlineVisualization(track);
        require(after.samples.size() == committed.samples.size(),
            "the last valid geometry survives a rejected edit");
        for (std::size_t i = 0; i < after.samples.size(); ++i)
        {
            require(after.samples[i].position == committed.samples[i].position,
                "every committed sample is unchanged");
        }

        // A gentler pull-up that the same entry speed can sustain commits.
        AuthoredTrackEditTransaction accepted{track};
        ChannelProfile& acceptedNormal =
            forceRegion(accepted.candidate().section(1)).targetNormalG;
        splitChannelSegment(
            acceptedNormal, acceptedNormal.segments.front().id, half);
        acceptedNormal.segments.front().transition.valueBegin = 2.0;
        acceptedNormal.segments.front().transition.valueEnd = 1.0;
        acceptedNormal.segments[1].transition.valueBegin = 1.0;
        acceptedNormal.segments[1].transition.valueEnd = 2.0;
        const auto acceptedLoads =
            evaluateRiderLoadDiagnostics(accepted.candidate());
        accepted.requireAcceptableRiderLoads(acceptedLoads);
        accepted.commit(track);
        require(accepted.committed(), "a feasible target commits");
        require(
            serializeCoasterDocument(track) != committedDocument,
            "the accepted edit is the new committed state");
    }

    // ---------------------------------------------------------------------
    // 5. Transactions, history, duplication, and reordering
    // ---------------------------------------------------------------------

    void rejectedTransactionsNeverPartiallyPublish()
    {
        auto track = authoredTrackWithForceRegion();
        const std::string before = serializeCoasterDocument(track);

        // A malformed profile must not reach the committed document, and a
        // rejected length edit must not leave a partially rescaled region.
        AuthoredTrackEditTransaction malformed{track};
        forceRegion(malformed.candidate().section(1))
            .targetNormalG.segments.front().transition.domainEnd = 30.0;
        requireThrows<std::invalid_argument>(
            [&] { sectionLength(malformed.candidate().section(1)); },
            "a gap in the authored coverage is rejected");
        require(!malformed.committed(), "a malformed candidate never commits");
        require(serializeCoasterDocument(track) == before,
            "a malformed candidate leaves the document untouched");

        AuthoredTrackEditTransaction badLength{track};
        requireThrows<std::invalid_argument>(
            [&] { setSectionLength(badLength.candidate().section(1), 0.0); },
            "a zero region length is rejected");
        require(!badLength.committed(), "a rejected length never commits");
        require(serializeCoasterDocument(track) == before,
            "a rejected length leaves the document untouched");

        // A force-driven candidate without a completed evaluation is refused
        // even though its geometry generates.
        AuthoredTrackEditTransaction unevaluated{track};
        setChannelSegmentValue(
            forceRegion(unevaluated.candidate().section(1)).targetNormalG,
            forceRegion(unevaluated.candidate().section(1))
                .targetNormalG.segments.front().id,
            ProfileBoundary::End, 0.8);
        RiderLoadHistory incomplete;
        requireThrows<std::invalid_argument>(
            [&] { unevaluated.requireAcceptableRiderLoads(incomplete); },
            "a force candidate without evaluation is refused");
        require(!unevaluated.committed(), "an unevaluated candidate never commits");
        require(serializeCoasterDocument(track) == before,
            "an unevaluated candidate leaves the document untouched");
    }

    void saveLoadRoundTripsForceDrivenRegions()
    {
        auto track = authoredTrackWithForceRegion();
        auto& force = forceRegion(track.section(1));
        splitChannelSegment(
            force.targetNormalG, force.targetNormalG.segments.front().id, 14.0);
        setChannelSegmentValue(
            force.targetNormalG, force.targetNormalG.segments.front().id,
            ProfileBoundary::End, 0.15);
        setChannelSegmentValue(
            force.targetLateralG, force.targetLateralG.segments.front().id,
            ProfileBoundary::Begin, -0.35);
        setChannelSegmentValue(
            force.targetLateralG, force.targetLateralG.segments.front().id,
            ProfileBoundary::End, 0.45);
        setChannelSegmentValue(
            force.rollRate, force.rollRate.segments.front().id,
            ProfileBoundary::Begin, 0.02);
        findChannelSegmentTransition(
            force.rollRate, force.rollRate.segments.front().id)
            ->transitionType = quantum::math::TransitionType::SeventhOrderSmoothstep;
        setSectionLength(track.section(1), 55.0);

        const std::string serialized = serializeCoasterDocument(track);
        auto reloaded = deserializeCoasterDocument(serialized);
        require(reloaded.has_value(),
            "a force-driven document reloads: " + reloaded.error_or(""));
        require(serializeCoasterDocument(*reloaded) == serialized,
            "the round trip is byte identical");
        for (const ChannelProfile* profile : forceChannels((*reloaded).section(1)))
        {
            requireCoversExactly(*profile, 55.0, "reloaded coverage");
        }
        require(
            forceRegion((*reloaded).section(1)).targetNormalG
                == force.targetNormalG,
            "reloaded normal G target");
        require(
            forceRegion((*reloaded).section(1)).targetLateralG
                == force.targetLateralG,
            "reloaded lateral G target");
        require(
            forceRegion((*reloaded).section(1)).rollRate == force.rollRate,
            "reloaded roll rate");
        require(
            createCenterlineVisualization(*reloaded).samples.size()
                == createCenterlineVisualization(track).samples.size(),
            "a reloaded force document generates the same geometry");
    }

    void undoRedoRestoresForceDrivenProfiles()
    {
        auto track = authoredTrackWithForceRegion();
        DocumentHistory history;
        history.reset(track);
        require(!history.canUndo() && !history.canRedo(),
            "a reset history has nothing to move through");
        const std::string original = serializeCoasterDocument(track);

        // One continuous numeric edit, then one structural edit: the same two
        // shapes the editor produces.
        history.endContinuousEdit();
        AuthoredTrackEditTransaction valueEdit{track};
        setChannelSegmentValue(
            forceRegion(valueEdit.candidate().section(1)).targetNormalG,
            forceRegion(valueEdit.candidate().section(1))
                .targetNormalG.segments.front().id,
            ProfileBoundary::End, 0.25);
        valueEdit.commit(track);
        history.record(track, true);
        const std::string afterValue = serializeCoasterDocument(track);
        require(afterValue != original, "the value edit changed the document");

        history.endContinuousEdit();
        AuthoredTrackEditTransaction split{track};
        splitChannelSegment(
            forceRegion(split.candidate().section(1)).rollRate,
            forceRegion(split.candidate().section(1)).rollRate.segments
                .front().id,
            11.0);
        split.commit(track);
        history.record(track);
        const std::string afterSplit = serializeCoasterDocument(track);
        require(afterSplit != afterValue, "the split changed the document");
        require(history.canUndo() && !history.canRedo(),
            "two accepted edits leave one redo path");

        const auto undoneSplit = history.undo();
        require(undoneSplit.has_value(), "undo returns a document");
        require(serializeCoasterDocument(*undoneSplit) == afterValue,
            "undo removes the split and keeps the value edit");
        require(history.canUndo() && history.canRedo(), "the gesture is one step");

        const auto undoneValue = history.undo();
        require(undoneValue.has_value(), "undo returns a document");
        require(serializeCoasterDocument(*undoneValue) == original,
            "undo restores the original document exactly");
        require(!history.canUndo() && history.canRedo(),
            "the history is back at the baseline");

        const auto redoneValue = history.redo();
        require(redoneValue.has_value(), "redo returns a document");
        require(serializeCoasterDocument(*redoneValue) == afterValue,
            "redo restores the value edit exactly");
        const auto redoneSplit = history.redo();
        require(redoneSplit.has_value(), "redo returns a document");
        require(serializeCoasterDocument(*redoneSplit) == afterSplit,
            "redo restores the split exactly");
        for (const ChannelProfile* profile : forceChannels((*redoneSplit).section(1)))
        {
            requireCoversExactly(*profile, 40.0, "redone coverage");
        }
        require(!history.canRedo(), "the history is at the newest state");
    }

    void duplicationAndReorderingPreserveForceDrivenProfiles()
    {
        auto track = authoredTrackWithForceRegion();
        auto& force = forceRegion(track.section(1));
        splitChannelSegment(
            force.targetNormalG, force.targetNormalG.segments.front().id, 18.0);
        setChannelSegmentValue(
            force.targetNormalG, force.targetNormalG.segments.front().id,
            ProfileBoundary::End, 0.3);
        setSectionLength(track.section(1), 44.0);
        const ChannelProfile expectedNormal = force.targetNormalG;
        const ChannelProfile expectedRoll = force.rollRate;
        const std::string serialized = serializeCoasterDocument(track);

        // Duplication must produce an independent copy, not shared state.
        track.duplicateSection(1);
        require(track.sectionCount() == 4, "duplication adds a region");
        require(forceRegion(track.section(2)).targetNormalG == expectedNormal,
            "the duplicate keeps the authored targets");
        require(forceRegion(track.section(2)).rollRate == expectedRoll,
            "the duplicate keeps the authored roll rate");
        require(track.section(2).length == 44.0, "the duplicate keeps the length");
        forceRegion(track.section(2)).targetNormalG.segments.front()
            .transition.valueEnd = 2.9;
        require(
            forceRegion(track.section(1)).targetNormalG.segments.front()
                .transition.valueEnd
                == expectedNormal.segments.front().transition.valueEnd,
            "the duplicate shares no mutable state with the original");

        // Reordering must not disturb the authored content of a region.
        track.moveSection(2, 3);
        require(track.sectionCount() == 4, "reordering keeps the region count");
        require(forceRegion(track.section(3)).targetNormalG.segments.front()
                    .transition.valueEnd
                == 2.9,
            "reordering preserves the duplicate's edited target");
        for (const ChannelProfile* profile : forceChannels(track.section(1)))
        {
            requireCoversExactly(*profile, 44.0, "original coverage after reorder");
        }
        require(forceRegion(track.section(1)).targetNormalG == expectedNormal,
            "reordering leaves the original region untouched");

        // Converting a force-driven region to a planar arc preserves its
        // length and stops it being force driven. The reverse is not offered:
        // a generated region has no authored geometry to recover, and Core
        // refuses to invent state-independent rates from force targets.
        auto converted = deserializeCoasterDocument(serialized);
        require(converted.has_value(), "the authored document is reloadable");
        const std::string beforeArcConversion =
            serializeCoasterDocument(*converted);
        requireThrows<std::invalid_argument>(
            [&] { convertSectionToRateProfiles((*converted).section(1)); },
            "a force-driven region cannot become rate profiles");
        require(serializeCoasterDocument(*converted) == beforeArcConversion,
            "a rejected conversion leaves the document untouched");
        convertSectionToPlanarArc((*converted).section(1));
        require(!isForceDrivenSection((*converted).section(1)),
            "a force region converts to a planar arc");
        require((*converted).section(1).length == 44.0,
            "the conversion preserves the authored length");
    }

    void regionKindSwitchingKeepsBothEditorsStable()
    {
        // A document with one of every authoring model, which is the state a
        // user reaches by switching between regions while editing.
        auto track = createNewDocument();
        setSectionLength(track.section(0), 20.0);
        track.insertSectionAfter(0, createForceDrivenSection(40.0));
        track.appendSection();
        convertSectionToPlanarArc(track.section(2));
        setSectionLength(track.section(2), 15.0);
        setSectionLength(track.section(1), 30.0);

        // Visiting every region in both directions is what a user does when
        // they compare authoring models. Each profile-bearing visit resolves
        // the same three rows and each row must still cover its region.
        for (const std::size_t index : {0u, 1u, 0u, 1u})
        {
            AuthoredTrackSection& section = track.section(index);
            const std::array<ProfileRowStyle, profileChannelCount> styles =
                isForceDrivenSection(section)
                ? forceDrivenRowStyles(
                    track.physicalSettings().metersPerCoordinateUnit)
                : rateProfileRowStyles(
                    track.physicalSettings().metersPerCoordinateUnit);
            for (const ProfileChannel channel : {
                ProfileChannel::Roll,
                ProfileChannel::Pitch,
                ProfileChannel::Yaw})
            {
                requireCoversExactly(
                    sectionProfileChannel(section, channel),
                    section.length,
                    "every row of every region kind covers its region");
            }
            for (const ProfileRowStyle& style : styles)
            {
                require(
                    style.authoredToDisplay * style.displayToAuthored == 1.0,
                    "every row style round trips in both authoring models");
            }
        }

        // A planar arc owns scalar parameters, not authored curves, so it
        // resolves no row. That refusal is what keeps the Circular Arc
        // editor from silently editing a force target.
        AuthoredTrackSection& arc = track.section(2);
        require(!isForceDrivenSection(arc), "the third region is an arc");
        requireThrows<std::logic_error>(
            [&] { sectionProfileChannel(arc, ProfileChannel::Roll); },
            "a planar arc resolves no profile row");

        // A rate-profile region and a force-driven neighbour coexist, and the
        // whole document keeps generating.
        const auto centerline = createCenterlineVisualization(track);
        require(centerline.samples.size() > 2,
            "a mixed document generates geometry");
        require(centerline.sectionSlices.size() == 3,
            "a mixed document keeps its section boundaries");
    }

    // The document used for this milestone's real Editor captures. It is
    // built here through the same profile operations the graph issues so the
    // committed smoke-test fixture can never contain a profile Core would
    // reject.
    AuthoredTrack captureFixture(bool authored)
    {
        auto track = createNewDocument();
        track.setPhysicalSettings(
            {25.0, 1.0, standardGravityAcceleration});
        setSectionLength(track.section(0), 45.0);
        // A varying rate profile before the Force-Based region, so the same
        // document also exercises the unchanged Transition Editor.
        setChannelSegmentValue(
            track.section(0).rateProfileRegion().rateProfiles.pitch,
            track.section(0).rateProfileRegion().rateProfiles.pitch.segments
                .front().id,
            ProfileBoundary::End, 0.02);
        track.insertSectionAfter(0, createForceDrivenSection(80.0));
        track.appendSection();
        convertSectionToPlanarArc(track.section(2));
        setSectionLength(track.section(2), 40.0);
        if (!authored)
        {
            return track;
        }

        ForceDrivenRegion& force = forceRegion(track.section(1));

        // Normal G: dive, pull, then level out. Every endpoint value goes
        // through setChannelSegmentValue so the shared interior value stays
        // identical on both sides of each joint.
        ChannelProfile& normal = force.targetNormalG;
        setChannelSegmentValue(
            normal, normal.segments.front().id, ProfileBoundary::Begin, 0.35);
        setChannelSegmentValue(
            normal, normal.segments.front().id, ProfileBoundary::End, 0.15);
        const SegmentId normalPull = splitChannelSegment(
            normal, normal.segments.front().id, 30.0);
        setChannelSegmentValue(
            normal, normal.segments.front().id, ProfileBoundary::End, 1.55);
        setChannelSegmentValue(
            normal, normalPull, ProfileBoundary::End, 0.85);
        normal.segments[0].transition.transitionType =
            quantum::math::TransitionType::CubicEaseInOut;
        normal.segments[1].transition.transitionType =
            quantum::math::TransitionType::Smootherstep;

        // Lateral G: a turn that eases in and back out.
        ChannelProfile& lateral = force.targetLateralG;
        const SegmentId lateralIn = splitChannelSegment(
            lateral, lateral.segments.front().id, 18.0);
        setChannelSegmentValue(
            lateral, lateral.segments.front().id, ProfileBoundary::End, 0.6);
        setChannelSegmentValue(
            lateral, lateralIn, ProfileBoundary::End, 0.15);
        lateral.segments[0].transition.transitionType =
            quantum::math::TransitionType::SineEaseIn;
        lateral.segments[1].transition.transitionType =
            quantum::math::TransitionType::Smootherstep;

        // Roll rate: a bank that eases in and back out, in radians per Core
        // coordinate unit.
        ChannelProfile& roll = force.rollRate;
        const SegmentId rollIn = splitChannelSegment(
            roll, roll.segments.front().id, 20.0);
        setChannelSegmentValue(
            roll, roll.segments.front().id, ProfileBoundary::End, 0.022);
        setChannelSegmentValue(
            roll, rollIn, ProfileBoundary::End, 0.0);
        roll.segments[0].transition.transitionType =
            quantum::math::TransitionType::Smootherstep;
        roll.segments[1].transition.transitionType =
            quantum::math::TransitionType::CosineEaseInOut;

        return track;
    }

    void captureFixtureIsAValidAuthoredDocument()
    {
        for (const bool authored : {false, true})
        {
            auto track = captureFixture(authored);
            require(track.sectionCount() == 3, "fixture has three regions");
            require(!isForceDrivenSection(track.section(0))
                    && isForceDrivenSection(track.section(1))
                    && !isForceDrivenSection(track.section(2)),
                "the middle region is the Force-Based one");
            for (const ChannelProfile* profile : forceChannels(track.section(1)))
            {
                requireCoversExactly(*profile, 80.0,
                    "every fixture channel covers its region");
            }
            // The authored fixture must have real multi-segment curves, or
            // the capture would show only the flat creation defaults.
            if (authored)
            {
                for (const ChannelProfile* profile :
                    forceChannels(track.section(1)))
                {
                    require(profile->segments.size() == 2,
                        "each authored channel has two segments");
                }
            }

            const std::string serialized = serializeCoasterDocument(track);
            auto reloaded = deserializeCoasterDocument(serialized);
            require(reloaded.has_value(),
                "the fixture reloads: " + reloaded.error_or(""));
            require(serializeCoasterDocument(*reloaded) == serialized,
                "the fixture round trips byte identically");
            const auto centerline = createCenterlineVisualization(*reloaded);
            require(centerline.samples.size() > 2,
                "the fixture generates track geometry");
        }

        // The authored fixture really does reshape the region relative to the
        // freshly created one, so the capture shows an edited track.
        const auto fresh = captureFixture(false);
        const auto authored = captureFixture(true);
        require(
            serializeCoasterDocument(fresh)
                != serializeCoasterDocument(authored),
            "the authored fixture differs from the creation defaults");
        const auto freshEnd =
            createCenterlineVisualization(fresh).samples.back().position;
        const auto authoredEnd =
            createCenterlineVisualization(authored).samples.back().position;
        require(freshEnd != authoredEnd,
            "the authored targets move the generated track");
    }

    const std::pair<const char*, void (*)()> tests[]{
        {"capture fixture is a valid authored document",
            captureFixtureIsAValidAuthoredDocument},
        {"force driven creation produces valid covering profiles",
            forceDrivenCreationProducesValidCoveringProfiles},
        {"row routing follows the region authoring model",
            sectionProfileChannelRoutesTheRegionAuthoringModel},
        {"row display units account for the coordinate scale",
            rowStyleUnitsAccountForTheCoordinateScale},
        {"view only row operations do not mutate authored data",
            viewOnlyRowOperationsDoNotMutateAuthoredData},
        {"force profile editing preserves coverage and identity",
            forceProfileEditingPreservesCoverageAndIdentity},
        {"region length changes preserve profile coverage",
            regionLengthChangesPreserveProfileCoverage},
        {"editing profiles regenerates track geometry",
            editingProfilesRegeneratesTrackGeometry},
        {"infeasible targets report the real failure",
            infeasibleTargetsReportTheRealFailureAndKeepTheLastValidState},
        {"rejected transactions never partially publish",
            rejectedTransactionsNeverPartiallyPublish},
        {"save and load round trips force driven regions",
            saveLoadRoundTripsForceDrivenRegions},
        {"undo and redo restore force driven profiles",
            undoRedoRestoresForceDrivenProfiles},
        {"duplication and reordering preserve force driven profiles",
            duplicationAndReorderingPreserveForceDrivenProfiles},
        {"region kind switching keeps both editors stable",
            regionKindSwitchingKeepsBothEditorsStable},
    };
}

int main()
{
    int failed = 0;
    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "PASS " << name << '\n';
        }
        catch (const std::exception& error)
        {
            std::cout << "FAIL " << name << ": " << error.what() << '\n';
            ++failed;
        }
    }
    if (failed != 0)
    {
        std::cout << failed << " test(s) failed.\n";
        return 1;
    }
    std::cout << sizeof(tests) / sizeof(tests[0])
              << " force-driven authoring tests passed.\n";
    return 0;
}
