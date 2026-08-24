#include <quantum/coaster/AuthoredTrack.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::convertSectionToPlanarArc;
    using quantum::coaster::convertSectionToRateProfiles;
    using quantum::coaster::createRateProfileSection;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::GeometryRegion;
    using quantum::coaster::integrateAuthoredTrack;
    using quantum::coaster::planarArcLength;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::RateProfileRegion;
    using quantum::coaster::RegionKind;
    using quantum::coaster::sectionLength;
    using quantum::coaster::setPlanarArcBankChange;
    using quantum::coaster::setPlanarArcPlaneTilt;
    using quantum::coaster::setPlanarArcRadius;
    using quantum::coaster::setPlanarArcSweptAngle;
    using quantum::geometry::CurveFrame;

    constexpr double pi = 3.141592653589793;

    class TestFailure final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename ExpectedException, typename Function>
    void requireThrows(Function&& function, const std::string_view context)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const ExpectedException&)
        {
            return;
        }
        catch (const std::exception& exception)
        {
            throw TestFailure(
                std::string(context) + ": unexpected exception: "
                + exception.what()
            );
        }

        throw TestFailure(
            std::string(context) + ": expected exception was not thrown"
        );
    }

    [[nodiscard]] const PlanarArcRegion& constructionOf(
        const AuthoredTrackSection& section)
    {
        return std::get<PlanarArcRegion>(
            std::get<GeometryRegion>(section.region).construction);
    }

    // Exact field comparison; the authoring structs intentionally do not
    // define operator==.
    [[nodiscard]] bool sameRateProfileContent(
        const AuthoredTrackSection& section,
        const AuthoredTrackSection& snapshot)
    {
        if (section.kind != RegionKind::RateProfiles
            || section.length != snapshot.length)
        {
            return false;
        }

        const RateProfileRegion& authored =
            std::get<RateProfileRegion>(section.region);
        const RateProfileRegion& stored =
            std::get<RateProfileRegion>(snapshot.region);

        const ChannelProfile* current[] =
            {&authored.rateProfiles.pitch,
             &authored.rateProfiles.yaw,
             &authored.rateProfiles.roll};
        const ChannelProfile* saved[] =
            {&stored.rateProfiles.pitch,
             &stored.rateProfiles.yaw,
             &stored.rateProfiles.roll};

        for (std::size_t channelIndex = 0; channelIndex < 3; ++channelIndex)
        {
            const ChannelProfile& a = *current[channelIndex];
            const ChannelProfile& b = *saved[channelIndex];

            if (a.segments.size() != b.segments.size()
                || a.nextSegmentId != b.nextSegmentId)
            {
                return false;
            }

            for (std::size_t index = 0; index < a.segments.size(); ++index)
            {
                const quantum::math::ScalarTransition& left =
                    a.segments[index].transition;
                const quantum::math::ScalarTransition& right =
                    b.segments[index].transition;

                if (a.segments[index].id != b.segments[index].id
                    || left.domainBegin != right.domainBegin
                    || left.domainEnd != right.domainEnd
                    || left.valueBegin != right.valueBegin
                    || left.valueEnd != right.valueEnd
                    || left.transitionType != right.transitionType)
                {
                    return false;
                }
            }
        }

        return true;
    }

    // Replicates the editor's typed-create flow for one new region:
    // structural insert followed by a conversion when geometry was chosen.
    void appendTypedRegion(
        AuthoredTrack& track,
        const RegionKind kind)
    {
        track.appendSection();

        if (kind == RegionKind::Geometry)
        {
            convertSectionToPlanarArc(track.section(track.sectionCount() - 1));
        }
    }

    void prependTypedRegion(
        AuthoredTrack& track,
        const RegionKind kind)
    {
        track.prependSection();

        if (kind == RegionKind::Geometry)
        {
            convertSectionToPlanarArc(track.section(0));
        }
    }

    void testTypedCreationAppendsBothKinds()
    {
        // The editor document always starts from the default track, which
        // holds one rate-profile region.
        AuthoredTrack track = quantum::coaster::createDefaultAuthoredTrack();
        appendTypedRegion(track, RegionKind::RateProfiles);
        appendTypedRegion(track, RegionKind::Geometry);

        require(track.sectionCount() == 3,
            "appendTypedRegion should grow the track by one region each");
        require(track.section(0).kind == RegionKind::RateProfiles,
            "the initial region stays RateProfiles");
        require(track.section(1).kind == RegionKind::RateProfiles,
            "an appended rate-profile region is created as RateProfiles");
        require(track.section(2).kind == RegionKind::Geometry,
            "an appended geometry region is created as Geometry");

        const AuthoredTrackSection& created =
            track.section(track.sectionCount() - 1);
        const PlanarArcRegion& arc = constructionOf(created);

        constexpr double defaultPlanarArcRadius = 25.0;
        const double expectedSweep = defaultNewSectionLength
            / defaultPlanarArcRadius;
        require(created.length == defaultNewSectionLength,
            "created geometry region preserves its authored length");
        require(arc.sweptAngle == expectedSweep,
            "created geometry region derives its sweep from the length");

        // Every created region must be immediately valid.
        (void)sectionLength(track.section(0));
        (void)sectionLength(track.section(1));
        (void)sectionLength(track.section(2));

        const auto states = integrateAuthoredTrack(track, 0.5);
        require(states.size() >= 2, "created regions integrate");
    }

    void testTypedCreationPrependsBothKinds()
    {
        AuthoredTrack track = quantum::coaster::createDefaultAuthoredTrack();
        prependTypedRegion(track, RegionKind::Geometry);
        prependTypedRegion(track, RegionKind::RateProfiles);

        require(track.sectionCount() == 3,
            "prependTypedRegion should grow the track by one region each");
        require(track.section(0).kind == RegionKind::RateProfiles,
            "a prepended rate-profile region lands at index zero");
        require(track.section(1).kind == RegionKind::Geometry,
            "the earlier prepended geometry region shifts to index one");
        require(track.section(2).kind == RegionKind::RateProfiles,
            "the original region shifts to the end");

        const auto states = integrateAuthoredTrack(track, 0.5);
        require(states.size() >= 2, "prepended regions integrate");
    }

    void testKindConversionsPreserveAuthoredLength()
    {
        AuthoredTrackSection section = createRateProfileSection(37.5);

        convertSectionToRateProfiles(section);
        require(section.kind == RegionKind::RateProfiles,
            "converting a rate-profile section to itself is a no-op");

        convertSectionToPlanarArc(section);
        require(section.kind == RegionKind::Geometry,
            "conversion switches the region kind");
        require(section.length == 37.5,
            "conversion preserves the authored length");
        require(constructionOf(section).sweptAngle == 37.5
            / constructionOf(section).radius,
            "conversion derives the sweep from the preserved length");
        (void)sectionLength(section);

        convertSectionToPlanarArc(section);
        require(constructionOf(section).planeTilt == 0.0
            && constructionOf(section).bankChange == 0.0,
            "re-converting an existing geometry region is a no-op");

        convertSectionToRateProfiles(section);
        require(section.kind == RegionKind::RateProfiles,
            "converting back restores rate-profile authoring");
        require(section.length == 37.5,
            "converting back preserves the authored length");

        const RateProfileRegion& restored = std::get<RateProfileRegion>(
            section.region);
        for (const ChannelProfile* channel :
            {&restored.rateProfiles.pitch,
             &restored.rateProfiles.yaw,
             &restored.rateProfiles.roll})
        {
            require(channel->segments.size() == 1,
                "converted-back channels hold one covering segment");
            require(channel->segments.front().transition.domainBegin == 0.0
                && channel->segments.front().transition.domainEnd == 37.5,
                "converted-back channels cover the preserved length");
            require(channel->segments.front().transition.valueBegin == 0.0
                && channel->segments.front().transition.valueEnd == 0.0,
                "converted-back channels start as zero rates");
        }
    }

    void testRadiusEditKeepsStoredLengthAuthoritative()
    {
        AuthoredTrackSection section = createRateProfileSection(40.0);
        convertSectionToPlanarArc(section);

        PlanarArcRegion& arc =
            std::get<PlanarArcRegion>(
                std::get<GeometryRegion>(section.region).construction);

        setPlanarArcRadius(section, 50.0);
        require(arc.radius == 50.0, "radius edit applies the new radius");
        require(arc.sweptAngle == 40.0 / 50.0,
            "radius edit rescales the sweep from the stored length");
        require(section.length == 40.0,
            "radius edit leaves the stored length authoritative");
        (void)sectionLength(section);

        setPlanarArcSweptAngle(section, -pi / 2.0);
        const double lengthAfterSweepEdit = section.length;
        require(lengthAfterSweepEdit == (pi / 2.0) * 50.0,
            "sweep edit redefines the authoritative length");

        setPlanarArcRadius(section, 25.0);
        require(arc.sweptAngle == -lengthAfterSweepEdit / 25.0,
            "radius edit preserves a negative turn direction");
        require(section.length == lengthAfterSweepEdit,
            "negative-sweep radius edit also keeps the length fixed");
        (void)sectionLength(section);
    }

    void testSweptAngleEditDefinesResultingLength()
    {
        AuthoredTrackSection section = createRateProfileSection(40.0);
        convertSectionToPlanarArc(section);

        PlanarArcRegion& arc =
            std::get<PlanarArcRegion>(
                std::get<GeometryRegion>(section.region).construction);

        setPlanarArcSweptAngle(section, pi / 3.0);
        require(arc.sweptAngle == pi / 3.0,
            "swept-angle edit applies the authored angle");
        require(section.length == planarArcLength(arc),
            "swept-angle edit moves the stored length to the arc length");
        require(section.length == (pi / 3.0) * arc.radius,
            "resulting length matches radius times sweep magnitude");
        (void)sectionLength(section);

        setPlanarArcSweptAngle(section, -pi / 4.0);
        require(section.length == planarArcLength(arc),
            "negative sweeps follow the same length rule");
        (void)sectionLength(section);
    }

    void testTiltAndBankEditsPreserveLength()
    {
        AuthoredTrackSection section = createRateProfileSection(40.0);
        convertSectionToPlanarArc(section);

        PlanarArcRegion& arc =
            std::get<PlanarArcRegion>(
                std::get<GeometryRegion>(section.region).construction);

        const double lengthBefore = section.length;

        setPlanarArcPlaneTilt(section, pi / 6.0);
        require(arc.planeTilt == pi / 6.0, "tilt edit applies the tilt");
        require(section.length == lengthBefore,
            "tilt edit never touches the stored length");

        setPlanarArcBankChange(section, pi / 8.0);
        require(arc.bankChange == pi / 8.0, "bank edit applies the bank");
        require(section.length == lengthBefore,
            "bank edit never touches the stored length");
        (void)sectionLength(section);

        // Orientation-only edits must not disturb the centerline length
        // contract of any other parameter.
        require(arc.radius > 0.0 && arc.sweptAngle != 0.0,
            "orientation edits leave the shape parameters intact");
    }

    void testMalformedParameterEditsAreRejected()
    {
        AuthoredTrackSection rates = createRateProfileSection(30.0);
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcRadius(rates, 10.0); },
            "radius edit on a rate-profile region");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcSweptAngle(rates, 1.0); },
            "sweep edit on a rate-profile region");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcPlaneTilt(rates, 1.0); },
            "tilt edit on a rate-profile region");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcBankChange(rates, 1.0); },
            "bank edit on a rate-profile region");

        AuthoredTrackSection arcSection = createRateProfileSection(30.0);
        convertSectionToPlanarArc(arcSection);

        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcRadius(arcSection, 0.0); },
            "zero radius");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcRadius(arcSection, -5.0); },
            "negative radius");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcRadius(arcSection,
                std::numeric_limits<double>::quiet_NaN()); },
            "NaN radius");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcRadius(arcSection,
                std::numeric_limits<double>::infinity()); },
            "infinite radius");

        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcSweptAngle(arcSection, 0.0); },
            "zero sweep");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcSweptAngle(arcSection,
                std::numeric_limits<double>::quiet_NaN()); },
            "NaN sweep");

        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcPlaneTilt(arcSection,
                std::numeric_limits<double>::quiet_NaN()); },
            "NaN tilt");
        requireThrows<std::invalid_argument>(
            [&] { setPlanarArcBankChange(arcSection,
                std::numeric_limits<double>::quiet_NaN()); },
            "NaN bank change");

        // Rejected edits must not corrupt the committed state.
        (void)sectionLength(arcSection);
    }

    void testAuthoringWorkflowPreservesRateProfileContent()
    {
        AuthoredTrack track;
        appendTypedRegion(track, RegionKind::RateProfiles);
        appendTypedRegion(track, RegionKind::Geometry);
        appendTypedRegion(track, RegionKind::RateProfiles);

        // Author distinct content in the two rate-profile regions by
        // editing the values of their existing covering segments.
        AuthoredTrackSection& first = track.section(0);
        first.rateProfileRegion().rateProfiles.pitch.segments.front()
            .transition.valueBegin = 0.004;
        first.rateProfileRegion().rateProfiles.pitch.segments.front()
            .transition.valueEnd = -0.002;

        AuthoredTrackSection& last = track.section(2);
        last.rateProfileRegion().rateProfiles.yaw.segments.front()
            .transition.valueBegin = 0.010;
        last.rateProfileRegion().rateProfiles.yaw.segments.front()
            .transition.valueEnd = 0.010;

        // Snapshot everything, then run geometry-region parameter edits of
        // every kind. Selection switching between editors must not disturb
        // unrelated regions.
        const AuthoredTrackSection firstSnapshot = first;
        const AuthoredTrackSection lastSnapshot = last;
        const AuthoredTrackSection geometrySnapshot = track.section(1);

        setPlanarArcRadius(track.section(1), 45.0);
        setPlanarArcSweptAngle(track.section(1), pi / 2.0);
        setPlanarArcPlaneTilt(track.section(1), pi / 8.0);
        setPlanarArcBankChange(track.section(1), pi / 6.0);

        require(sameRateProfileContent(track.section(0), firstSnapshot),
            "rate-profile region is untouched by geometry edits");
        require(sameRateProfileContent(track.section(2), lastSnapshot),
            "trailing rate-profile region is untouched by geometry edits");

        const PlanarArcRegion& edited = constructionOf(track.section(1));
        require(edited.radius == 45.0 && edited.planeTilt == pi / 8.0
            && edited.bankChange == pi / 6.0,
            "edited geometry region carries all applied parameters");
        require(geometrySnapshot.kind == track.section(1).kind,
            "parameter edits do not switch the region kind");

        const auto states = integrateAuthoredTrack(track, 0.25);
        require(states.size() >= 4, "mixed workflow track integrates");

        for (std::size_t index = 1; index < states.size(); ++index)
        {
            require(states[index].distance > states[index - 1].distance,
                "integrated distances increase monotonically");
        }

        // Joint continuity across every region boundary.
        for (std::size_t index = 1; index < states.size(); ++index)
        {
            const glm::dvec3 jump = states[index].position
                - states[index - 1].position;
            require(glm::length(jump) <= 0.25 * (1.0 + 1e-9),
                "consecutive samples stay within one spacing step");
            require(std::fabs(
                    glm::dot(states[index].frame.tangent,
                        states[index].frame.tangent) - 1.0) <= 1e-9,
                "tangents stay unit length through the workflow track");
        }

        constexpr double expectedTotal = 2.0 * defaultNewSectionLength
            + (pi / 2.0) * 45.0;
        require(std::fabs(states.back().distance - expectedTotal) <= 1e-9,
            "total distance equals the sum of authored region lengths");
    }
}

int main()
{
    const std::pair<std::string_view, void(*)()> tests[] = {
        {"typed creation appends both region kinds",
            testTypedCreationAppendsBothKinds},
        {"typed creation prepends both region kinds",
            testTypedCreationPrependsBothKinds},
        {"kind conversions preserve authored length",
            testKindConversionsPreserveAuthoredLength},
        {"radius edit keeps stored length authoritative",
            testRadiusEditKeepsStoredLengthAuthoritative},
        {"swept angle edit defines resulting length",
            testSweptAngleEditDefinesResultingLength},
        {"tilt and bank edits preserve length",
            testTiltAndBankEditsPreserveLength},
        {"malformed parameter edits are rejected",
            testMalformedParameterEditsAreRejected},
        {"authoring workflow preserves rate profile content",
            testAuthoringWorkflowPreservesRateProfileContent}
    };

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[FAIL] " << name << ": "
                << exception.what() << '\n';
            return 1;
        }
    }

    std::cout << "All region authoring workflow tests passed.\n";
    return 0;
}
