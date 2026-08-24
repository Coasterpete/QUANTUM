#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/PlanarArcRegion.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::AuthoredTrackSection;
    using quantum::coaster::ChannelProfile;
    using quantum::coaster::compilePlanarArcRates;
    using quantum::coaster::GeometryRegion;
    using quantum::coaster::integrateAuthoredTrack;
    using quantum::coaster::integrateLocalRollPitchYawRateProfiles;
    using quantum::coaster::integratePlanarArcRegion;
    using quantum::coaster::planarArcLength;
    using quantum::coaster::PlanarArcCompiledRates;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RegionKind;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::coaster::sectionLength;
    using quantum::coaster::setSectionLength;
    using quantum::coaster::validatePlanarArcRegion;
    using quantum::geometry::CurveFrame;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

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

    // Matches the frame integrateAuthoredTrack seeds tracks with.
    [[nodiscard]] CurveFrame identityFrame()
    {
        return CurveFrame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
    }

    // Rodrigues rotation of v about a unit axis.
    [[nodiscard]] glm::dvec3 rotateAbout(
        const glm::dvec3& v,
        const glm::dvec3& unitAxis,
        const double angle)
    {
        return v * std::cos(angle)
            + glm::cross(unitAxis, v) * std::sin(angle)
            + unitAxis * glm::dot(unitAxis, v) * (1.0 - std::cos(angle));
    }

    // The authored plane normal expressed in world coordinates for an entry
    // frame, per the documented compilation rule
    // n = cos(planeTilt) * U0 - sin(planeTilt) * L0.
    [[nodiscard]] glm::dvec3 authoredPlaneNormal(
        const PlanarArcRegion& region,
        const CurveFrame& entryFrame)
    {
        return entryFrame.up * std::cos(region.planeTilt)
            - entryFrame.lateral * std::sin(region.planeTilt);
    }

    // Verifies every solved state against the closed-form circular motion
    // implied by the construction: constant rotation about the world-fixed
    // axis n with signed angular speed kappa = sign(sweptAngle)/radius.
    // This simultaneously pins planarity, radius, sweep angle, and turn
    // handedness against QUANTUM's actual CurveFrame conventions.
    void checkCircularArc(
        const std::vector<RiderLocalGeometryState>& states,
        const PlanarArcRegion& region,
        const glm::dvec3& startPosition,
        const CurveFrame& entryFrame,
        const std::string_view context)
    {
        const double sweepSign =
            region.sweptAngle < 0.0 ? -1.0 : 1.0;
        const double curvature = sweepSign / region.radius;

        const glm::dvec3 normal =
            glm::normalize(authoredPlaneNormal(region, entryFrame));
        const glm::dvec3 centerDirection =
            sweepSign * glm::cross(normal, entryFrame.tangent);
        const glm::dvec3 center =
            startPosition + centerDirection * region.radius;

        require(!states.empty(), std::string(context) + ": produced states");
        require(states.front().distance == 0.0,
            std::string(context) + ": first distance starts at zero");
        require(states.front().position == startPosition,
            std::string(context) + ": entry position inherited exactly");
        require(states.front().frame == entryFrame,
            std::string(context) + ": entry frame inherited exactly");

        double previousDistance = -std::numeric_limits<double>::infinity();
        for (const RiderLocalGeometryState& state : states)
        {
            require(state.distance > previousDistance,
                std::string(context) + ": distances strictly increase");
            previousDistance = state.distance;

            const double angle = curvature * state.distance;

            const glm::dvec3 expectedPosition =
                center + rotateAbout(startPosition - center, normal, angle);
            require(glm::length(state.position - expectedPosition) <= 1e-5,
                std::string(context)
                    + ": position follows the authored circle");

            const glm::dvec3 expectedTangent =
                rotateAbout(entryFrame.tangent, normal, angle);
            require(glm::length(state.frame.tangent - expectedTangent)
                    <= 1e-8,
                std::string(context)
                    + ": tangent sweeps the authored angle");
        }
    }

    [[nodiscard]] ChannelProfile makeConstantChannel(
        const double length,
        const double value)
    {
        ChannelProfile channel;
        channel.segments.push_back(ProfileSegment{
            channel.nextSegmentId,
            ScalarTransition{
                .domainBegin = 0.0,
                .domainEnd = length,
                .valueBegin = value,
                .valueEnd = value,
                .transitionType = TransitionType::Linear
            }
        });
        ++channel.nextSegmentId;
        return channel;
    }

    [[nodiscard]] ChannelProfile wrapTransition(
        const ScalarTransition& transition)
    {
        ChannelProfile channel;
        channel.segments.push_back(ProfileSegment{
            channel.nextSegmentId,
            transition
        });
        ++channel.nextSegmentId;
        return channel;
    }

    [[nodiscard]] ScalarTransition createZeroRollChannel(
        const double length)
    {
        return ScalarTransition{
            .domainBegin = 0.0,
            .domainEnd = length,
            .valueBegin = 0.0,
            .valueEnd = 0.0,
            .transitionType = TransitionType::Linear
        };
    }

    void requireSameStates(
        const std::vector<RiderLocalGeometryState>& expected,
        const std::vector<RiderLocalGeometryState>& actual,
        const double tolerance,
        const std::string_view context)
    {
        require(expected.size() == actual.size(),
            std::string(context) + ": identical sample count");

        for (std::size_t index = 0; index < expected.size(); ++index)
        {
            require(std::abs(expected[index].distance
                        - actual[index].distance) <= tolerance,
                std::string(context) + ": distance at sample "
                    + std::to_string(index));
            require(glm::length(expected[index].position
                        - actual[index].position) <= tolerance,
                std::string(context) + ": position at sample "
                    + std::to_string(index));
            require(glm::length(expected[index].frame.tangent
                        - actual[index].frame.tangent) <= tolerance,
                std::string(context) + ": tangent at sample "
                    + std::to_string(index));
            require(glm::length(expected[index].frame.lateral
                        - actual[index].frame.lateral) <= tolerance,
                std::string(context) + ": lateral at sample "
                    + std::to_string(index));
            require(glm::length(expected[index].frame.up
                        - actual[index].frame.up) <= tolerance,
                std::string(context) + ": up at sample "
                    + std::to_string(index));
        }
    }

    void testValidationRefusals()
    {
        constexpr double nanValue =
            std::numeric_limits<double>::quiet_NaN();
        constexpr double infValue =
            std::numeric_limits<double>::infinity();

        require(planarArcLength(PlanarArcRegion{25.0, pi, 0.0, 0.0})
                == 25.0 * pi,
            "constructed length equals |sweep| * radius");

        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{0.0, pi, 0.0, 0.0}, 0.0); },
            "zero radius rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{-5.0, pi, 0.0, 0.0}, 0.0); },
            "negative radius rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{25.0, 0.0, 0.0, 0.0}, 0.0); },
            "zero swept angle rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{nanValue, pi, 0.0, 0.0}, 0.0); },
            "NaN radius rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{25.0, nanValue, 0.0, 0.0}, 0.0); },
            "NaN swept angle rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{25.0, pi, nanValue, 0.0}, 0.0); },
            "NaN plane tilt rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{25.0, pi, 0.0, infValue}, 0.0); },
            "infinite bank change rejected");
        requireThrows<std::invalid_argument>(
            [stored = 25.0 * pi + 1.0]
            { validatePlanarArcRegion(PlanarArcRegion{25.0, pi, 0.0, 0.0}, stored); },
            "stored length mismatch rejected");
        requireThrows<std::invalid_argument>(
            [] { validatePlanarArcRegion(PlanarArcRegion{25.0, pi, 0.0, 0.0}, infValue); },
            "non-finite stored length rejected");

        // The consistent combination validates cleanly.
        validatePlanarArcRegion(PlanarArcRegion{25.0, pi, 0.35, 0.4},
            25.0 * pi);
    }

    // planeTilt = 0 authors a horizontal turn: the whole solution stays in
    // the horizontal plane through the entry position, follows the authored
    // circle, and positive sweep turns toward the entry lateral direction.
    void testFlatArcGeometry()
    {
        const PlanarArcRegion region{25.0, pi / 2.0, 0.0, 0.0};
        const glm::dvec3 start{0.0, 0.0, 0.0};

        const auto states = integratePlanarArcRegion(
            start, identityFrame(), region, 1.0);

        checkCircularArc(states, region, start, identityFrame(),
            "flat arc");

        for (const RiderLocalGeometryState& state : states)
        {
            require(std::abs(state.position.z) <= 1e-7,
                "flat arc stays at entry height");
        }

        const RiderLocalGeometryState finalState = states.back();
        const glm::dvec3 expectedExit{
            25.0 * std::sin(pi / 2.0),
            25.0 * (1.0 - std::cos(pi / 2.0)),
            0.0
        };
        require(glm::length(finalState.position - expectedExit) <= 1e-5,
            "flat arc exits on the authored quarter circle");
        require(glm::length(finalState.frame.tangent
                    - glm::dvec3{0.0, 1.0, 0.0}) <= 1e-8,
            "positive sweep turns toward entry lateral (left)");
    }

    // planeTilt = +pi/2 compiles to pitchRate = -curvature, which turns the
    // tangent toward +up per the documented RiderLocalGeometry convention,
    // so a positive sweep authors a vertical arch rising above the entry
    // height. -pi/2 mirrors it into a dip below the entry height.
    void testVerticalArcOrientations()
    {
        const PlanarArcRegion arch{25.0, pi, pi / 2.0, 0.0};
        const auto archStates = integratePlanarArcRegion(
            glm::dvec3{0.0, 0.0, 0.0}, identityFrame(), arch, 1.0);

        checkCircularArc(archStates, arch, glm::dvec3{0.0, 0.0, 0.0},
            identityFrame(), "+90 degree arch");

        double maxHeight = -std::numeric_limits<double>::infinity();
        for (const RiderLocalGeometryState& state : archStates)
        {
            require(std::abs(state.position.y) <= 1e-6,
                "vertical arch stays in the vertical entry plane");
            maxHeight = std::max(maxHeight, state.position.z);
        }

        require(std::abs(maxHeight - 2.0 * arch.radius) <= 1e-4,
            "+90 degree tilt arches 2*radius above the entry height");
        require(glm::length(archStates.back().position
                    - glm::dvec3{0.0, 0.0, 2.0 * arch.radius}) <= 1e-4,
            "+90 degree half arch ends one radius past the apex");
        require(glm::length(archStates.back().frame.tangent
                    - glm::dvec3{-1.0, 0.0, 0.0}) <= 1e-8,
            "+90 degree half arch reverses the entry tangent");

        const PlanarArcRegion dip{25.0, pi, -pi / 2.0, 0.0};
        const auto dipStates = integratePlanarArcRegion(
            glm::dvec3{0.0, 0.0, 0.0}, identityFrame(), dip, 1.0);

        checkCircularArc(dipStates, dip, glm::dvec3{0.0, 0.0, 0.0},
            identityFrame(), "-90 degree dip");

        double minHeight = std::numeric_limits<double>::infinity();
        for (const RiderLocalGeometryState& state : dipStates)
        {
            require(std::abs(state.position.y) <= 1e-6,
                "vertical dip stays in the vertical entry plane");
            minHeight = std::min(minHeight, state.position.z);
        }

        require(std::abs(minHeight + 2.0 * dip.radius) <= 1e-4,
            "-90 degree tilt dips 2*radius below the entry height");
    }

    // An intermediate tilt must produce a genuinely tilted planar arc: the
    // solution leaves the horizontal plane yet stays inside the authored
    // inclined plane, with the authored radius and sweep preserved.
    void testIntermediateTiltArc()
    {
        const PlanarArcRegion region{20.0, 3.0 * pi / 4.0, pi / 6.0, 0.0};
        const glm::dvec3 start{0.0, 0.0, 0.0};

        const auto states = integratePlanarArcRegion(
            start, identityFrame(), region, 1.0);

        checkCircularArc(states, region, start, identityFrame(),
            "tilted arc");

        const glm::dvec3 normal = glm::normalize(
            authoredPlaneNormal(region, identityFrame()));

        bool climbsAboveEntry = false;
        for (const RiderLocalGeometryState& state : states)
        {
            require(std::abs(glm::dot(normal, state.position)) <= 1e-5,
                "tilted arc stays inside the authored inclined plane");
            climbsAboveEntry =
                climbsAboveEntry || state.position.z > 1e-3;
        }

        require(climbsAboveEntry,
            "positive sweep with positive tilt climbs above entry height");
    }

    // BankChange rolls lateral/up about the evolving tangent without ever
    // entering the tangent derivative, so it must not move the centerline.
    void testBankChangeDoesNotMoveCenterline()
    {
        const PlanarArcRegion unbanked{15.0, 2.0 * pi / 3.0, pi / 4.0, 0.0};
        const PlanarArcRegion banked{15.0, 2.0 * pi / 3.0, pi / 4.0, 0.65};
        const glm::dvec3 start{0.0, 0.0, 0.0};
        const CurveFrame entryFrame = identityFrame();

        const auto unbankedStates = integratePlanarArcRegion(
            start, entryFrame, unbanked, 1.0);
        const auto bankedStates = integratePlanarArcRegion(
            start, entryFrame, banked, 1.0);

        require(unbankedStates.size() == bankedStates.size(),
            "banking preserves the sample grid");
        require(unbankedStates.back().frame.lateral
                != unbankedStates.front().frame.lateral,
            "banking actually rotates the frame");

        const double length = planarArcLength(banked);
        for (std::size_t index = 0; index < bankedStates.size(); ++index)
        {
            const double distance = bankedStates[index].distance;
            require(std::abs(unbankedStates[index].distance - distance)
                    == 0.0,
                "banking preserves traveled distances");
            require(glm::length(
                        unbankedStates[index].position
                        - bankedStates[index].position) <= 1e-10,
                "banking preserves the centerline");
            require(glm::length(
                        unbankedStates[index].frame.tangent
                        - bankedStates[index].frame.tangent) <= 1e-12,
                "banking preserves the tangent field");

            // Roll accumulates linearly: bankChange * distance / length.
            const double accumulatedRoll =
                banked.bankChange * distance / length;
            const glm::dvec3 expectedLateral = rotateAbout(
                unbankedStates[index].frame.lateral,
                unbankedStates[index].frame.tangent,
                accumulatedRoll);
            const glm::dvec3 expectedUp = rotateAbout(
                unbankedStates[index].frame.up,
                unbankedStates[index].frame.tangent,
                accumulatedRoll);
            require(glm::length(bankedStates[index].frame.lateral
                        - expectedLateral) <= 1e-9,
                "banked lateral matches accumulated roll");
            require(glm::length(bankedStates[index].frame.up
                        - expectedUp) <= 1e-9,
                "banked up matches accumulated roll");
        }
    }

    // The compiled centerline-driving rates and the construction must drive
    // the shared rate-profile solver to identical solutions, both through
    // the single-transition entry point and through the breakpoint-aware
    // profile entry point used by track integration. Banking composes on
    // top of this solution and is covered by the dedicated bank group.
    void testEquivalenceWithRateSolverRepresentation()
    {
        const PlanarArcRegion region{18.0, 5.0 * pi / 6.0, 0.35, 0.0};
        const double length = planarArcLength(region);
        const glm::dvec3 start{2.0, -3.0, 5.0};
        const CurveFrame entryFrame = identityFrame();

        const PlanarArcCompiledRates compiled =
            compilePlanarArcRates(region, length);

        // Independent restatement of the documented compilation rule.
        const double curvature = 1.0 / region.radius;
        require(std::abs(compiled.pitchRate.valueBegin
                    - (-curvature * std::sin(region.planeTilt))) <= 1e-15,
            "compiled pitch rate matches the compilation rule");
        require(std::abs(compiled.yawRate.valueBegin
                    - (curvature * std::cos(region.planeTilt))) <= 1e-15,
            "compiled yaw rate matches the compilation rule");
        require(compiled.pitchRate.domainBegin == 0.0
                && compiled.yawRate.domainEnd == length,
            "compiled rates span the constructed domain");
        require(compiled.pitchRate.valueBegin == compiled.pitchRate.valueEnd
                && compiled.yawRate.valueBegin == compiled.yawRate.valueEnd,
            "compiled rates are constant");
        require(compiled.pitchRate.transitionType == TransitionType::Linear,
            "compiled rates use Linear transitions");

        const ScalarTransition zeroRoll =
            createZeroRollChannel(length);
        const auto directStates = integratePlanarArcRegion(
            start, entryFrame, region, 1.0);
        const auto manualStates = integrateLocalRollPitchYawRateProfiles(
            start,
            entryFrame,
            zeroRoll,
            compiled.pitchRate,
            compiled.yawRate,
            1.0);

        requireSameStates(manualStates, directStates, 0.0,
            "compiled rates reproduce the construction exactly");

        const auto profileStates = integrateLocalRollPitchYawRateProfiles(
            start,
            entryFrame,
            wrapTransition(zeroRoll),
            wrapTransition(compiled.pitchRate),
            wrapTransition(compiled.yawRate),
            length,
            1.0);

        requireSameStates(profileStates, directStates, 0.0,
            "profile-wrapped compiled rates reproduce the construction "
            "exactly");
    }

    // RateProfile -> Geometry -> RateProfile must chain continuously: the
    // track-level integration equals manually feeding each section's final
    // joint state into the next section's solve.
    void testMixedTrackChainsContinuously()
    {
        AuthoredTrack track;
        track.appendSection();

        // Section 0: rate profiles, quarter turn left over 60 units.
        {
            AuthoredTrackSection& section = track.section(0);
            section.length = 60.0;
            section.rateProfileRegion().rateProfiles.yaw =
                makeConstantChannel(60.0, (pi / 2.0) / 60.0);
        }

        // Section 1: geometry construction.
        {
            track.appendSection();
            AuthoredTrackSection& section = track.section(1);
            section.kind = RegionKind::Geometry;
            const PlanarArcRegion arc{25.0, pi / 2.0, 0.3, 0.5};
            section.region = GeometryRegion{arc};
            section.length = planarArcLength(arc);

            // Length queries validate through the geometry dispatch path.
            require(sectionLength(section) == planarArcLength(arc),
                "geometry section length validates via its construction");
        }

        // Section 2: back to rate profiles, diving pitch over 40 units.
        {
            track.appendSection();
            AuthoredTrackSection& section = track.section(2);
            setSectionLength(section, 40.0);
            section.rateProfileRegion().rateProfiles.pitch =
                makeConstantChannel(40.0, (-pi / 6.0) / 40.0);
        }

        const auto trackStates = integrateAuthoredTrack(track, 2.0);
        require(!trackStates.empty(), "mixed track integrates");

        // Manual reference chain through the public per-kind solvers,
        // accumulating each section's length into the distance domain the
        // same way integrateAuthoredTrack does.
        const CurveFrame entryFrame = identityFrame();
        const double firstOffset = sectionLength(track.section(0));
        const double secondOffset =
            firstOffset + sectionLength(track.section(1));

        const auto firstStates = integrateLocalRollPitchYawRateProfiles(
            glm::dvec3{0.0, 0.0, 0.0},
            entryFrame,
            track.section(0).rateProfileRegion().rateProfiles.roll,
            track.section(0).rateProfileRegion().rateProfiles.pitch,
            track.section(0).rateProfileRegion().rateProfiles.yaw,
            60.0,
            2.0);

        const auto secondStates = integratePlanarArcRegion(
            firstStates.back().position,
            firstStates.back().frame,
            std::get<PlanarArcRegion>(
                std::get<GeometryRegion>(
                    track.section(1).region).construction),
            2.0);

        const auto thirdStates = integrateLocalRollPitchYawRateProfiles(
            secondStates.back().position,
            secondStates.back().frame,
            track.section(2).rateProfileRegion().rateProfiles.roll,
            track.section(2).rateProfileRegion().rateProfiles.pitch,
            track.section(2).rateProfileRegion().rateProfiles.yaw,
            40.0,
            2.0);

        std::vector<RiderLocalGeometryState> reference = firstStates;
        for (std::size_t index = 1; index < secondStates.size(); ++index)
        {
            RiderLocalGeometryState state = secondStates[index];
            state.distance += firstOffset;
            reference.push_back(std::move(state));
        }
        for (std::size_t index = 1; index < thirdStates.size(); ++index)
        {
            RiderLocalGeometryState state = thirdStates[index];
            state.distance += secondOffset;
            reference.push_back(std::move(state));
        }

        requireSameStates(reference, trackStates, 1e-9,
            "mixed track matches the manually chained solves");

        // Distances accumulate monotonically across kinds.
        for (std::size_t index = 1; index < trackStates.size(); ++index)
        {
            require(trackStates[index].distance
                    > trackStates[index - 1].distance,
                "mixed track distances strictly increase");
        }
    }

    // Length rescaling keeps radius, tilt, and bank designer-authoritative
    // and absorbs the change into the signed swept angle.
    void testRescaleAdjustsSweptAnglePreservingRadius()
    {
        AuthoredTrackSection section;
        section.kind = RegionKind::Geometry;
        const PlanarArcRegion original{20.0, pi / 2.0, 0.2, 0.3};
        section.region = GeometryRegion{original};
        section.length = planarArcLength(original);

        const double newLength = 50.0;
        setSectionLength(section, newLength);

        const PlanarArcRegion& adjusted =
            std::get<PlanarArcRegion>(
                std::get<GeometryRegion>(section.region).construction);

        require(adjusted.radius == original.radius,
            "rescale preserves the authored radius");
        require(adjusted.planeTilt == original.planeTilt,
            "rescale preserves the authored plane tilt");
        require(adjusted.bankChange == original.bankChange,
            "rescale preserves the authored bank change");
        require(std::abs(adjusted.sweptAngle
                    - (newLength / adjusted.radius)) <= 1e-12,
            "rescale absorbs the new length into the swept angle");
        require(sectionLength(section) == newLength,
            "rescaled section reports the new length");
        validatePlanarArcRegion(adjusted, newLength);

        const auto states = integratePlanarArcRegion(
            glm::dvec3{0.0, 0.0, 0.0},
            identityFrame(),
            adjusted,
            1.0);

        const glm::dvec3 normal = glm::normalize(
            authoredPlaneNormal(adjusted, identityFrame()));
        const glm::dvec3 expectedExitTangent = rotateAbout(
            glm::dvec3{1.0, 0.0, 0.0},
            normal,
            adjusted.sweptAngle);
        require(glm::length(states.back().frame.tangent
                    - expectedExitTangent) <= 1e-8,
            "rescaled construction sweeps its new angle");
    }

    // Signed angle that rotates `from` onto `to` around a unit axis; used
    // to pin accumulated bank amounts against expectations.
    [[nodiscard]] double signedAngleAbout(
        const glm::dvec3& unitAxis,
        const glm::dvec3& from,
        const glm::dvec3& to)
    {
        return std::atan2(
            glm::dot(unitAxis, glm::cross(from, to)),
            glm::dot(from, to));
    }

    // bankChange = 0 must be an exact no-op on the frame orientation: the
    // construction path and the plain rate-solver path produce identical
    // solutions even from an entry frame that already carries roll.
    void testZeroBankPreservesInheritedOrientationExactly()
    {
        CurveFrame rolledEntry = identityFrame();
        rolledEntry.lateral =
            rotateAbout(rolledEntry.lateral, rolledEntry.tangent, 0.9);
        rolledEntry.up =
            rotateAbout(rolledEntry.up, rolledEntry.tangent, 0.9);

        const PlanarArcRegion region{20.0, pi / 2.0, 0.25, 0.0};
        const double length = planarArcLength(region);
        const PlanarArcCompiledRates compiled =
            compilePlanarArcRates(region, length);
        const glm::dvec3 start{0.0, 0.0, 0.0};

        const auto viaConstruction = integratePlanarArcRegion(
            start, rolledEntry, region, 1.5);
        const auto viaPlainRates = integrateLocalRollPitchYawRateProfiles(
            start,
            rolledEntry,
            createZeroRollChannel(length),
            compiled.pitchRate,
            compiled.yawRate,
            1.5);

        requireSameStates(viaPlainRates, viaConstruction, 0.0,
            "zero bank matches the plain rate-solver solution exactly");
    }

    // Positive and negative bankChange are mirror images about the unbanked
    // solve: both preserve its centerline and tangent field exactly, and
    // their lateral/up orientations roll by +/-psi(d).
    void testPositiveAndNegativeBankBehaveSymmetrically()
    {
        const PlanarArcRegion unbanked{15.0, 2.0 * pi / 3.0, pi / 4.0, 0.0};
        const PlanarArcRegion positive{15.0, 2.0 * pi / 3.0, pi / 4.0, 0.8};
        const PlanarArcRegion negative{15.0, 2.0 * pi / 3.0, pi / 4.0, -0.8};
        const glm::dvec3 start{0.0, 0.0, 0.0};
        const CurveFrame entryFrame = identityFrame();

        const auto unbankedStates = integratePlanarArcRegion(
            start, entryFrame, unbanked, 1.0);
        const auto positiveStates = integratePlanarArcRegion(
            start, entryFrame, positive, 1.0);
        const auto negativeStates = integratePlanarArcRegion(
            start, entryFrame, negative, 1.0);

        require(unbankedStates.size() == positiveStates.size()
                && positiveStates.size() == negativeStates.size(),
            "bank sign preserves the sample grid");

        const double length = planarArcLength(positive);
        for (std::size_t index = 0; index < positiveStates.size(); ++index)
        {
            const double distance = positiveStates[index].distance;
            const double psi = positive.bankChange * distance / length;
            const glm::dvec3 axis = unbankedStates[index].frame.tangent;

            require(glm::length(unbankedStates[index].position
                        - positiveStates[index].position) <= 1e-10
                    && glm::length(unbankedStates[index].position
                        - negativeStates[index].position) <= 1e-10,
                "both bank signs preserve the centerline");
            require(glm::length(unbankedStates[index].frame.tangent
                        - positiveStates[index].frame.tangent) <= 1e-12
                    && glm::length(unbankedStates[index].frame.tangent
                        - negativeStates[index].frame.tangent) <= 1e-12,
                "both bank signs preserve the tangent field");
            require(glm::length(
                        rotateAbout(unbankedStates[index].frame.lateral,
                            axis, psi)
                        - positiveStates[index].frame.lateral) <= 1e-9,
                "positive bank rolls lateral by +psi");
            require(glm::length(
                        rotateAbout(unbankedStates[index].frame.up,
                            axis, psi)
                        - positiveStates[index].frame.up) <= 1e-9,
                "positive bank rolls up by +psi");
            require(glm::length(
                        rotateAbout(unbankedStates[index].frame.lateral,
                            axis, -psi)
                        - negativeStates[index].frame.lateral) <= 1e-9,
                "negative bank mirrors lateral by -psi");
            require(glm::length(
                        rotateAbout(unbankedStates[index].frame.up,
                            axis, -psi)
                        - negativeStates[index].frame.up) <= 1e-9,
                "negative bank mirrors up by -psi");
        }
    }

    // Banking accumulates continuously: the signed roll between the
    // unbanked and banked solves grows linearly with distance instead of
    // jumping at the ends.
    void testBankingAccumulatesContinuously()
    {
        const PlanarArcRegion unbanked{18.0, pi, pi / 6.0, 0.0};
        const PlanarArcRegion banked{18.0, pi, pi / 6.0, 1.1};
        const double length = planarArcLength(banked);
        const glm::dvec3 start{0.0, 0.0, 0.0};

        const auto unbankedStates = integratePlanarArcRegion(
            start, identityFrame(), unbanked, 0.5);
        const auto bankedStates = integratePlanarArcRegion(
            start, identityFrame(), banked, 0.5);

        require(unbankedStates.size() == bankedStates.size(),
            "continuous accumulation preserves the sample grid");

        double previousMeasured = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < bankedStates.size();
            index += 17)
        {
            const double expected =
                banked.bankChange * bankedStates[index].distance / length;
            const double measured = signedAngleAbout(
                bankedStates[index].frame.tangent,
                unbankedStates[index].frame.lateral,
                bankedStates[index].frame.lateral);

            require(std::abs(measured - expected) <= 1e-9,
                "accumulated bank matches the linear rule");
            require(measured > previousMeasured,
                "accumulated bank increases along the region");
            previousMeasured = measured;
        }

        const double exitMeasured = signedAngleAbout(
            bankedStates.back().frame.tangent,
            unbankedStates.back().frame.lateral,
            bankedStates.back().frame.lateral);
        require(std::abs(exitMeasured - banked.bankChange) <= 1e-9,
            "accumulated bank reaches the authored exit amount");
    }

    // A banked construction's exit frame is the entry frame the next region
    // inherits: integrating a following geometry construction from that
    // exit state reproduces the track-level continuation exactly.
    void testBankedExitFrameIsInheritedByFollowingRegion()
    {
        AuthoredTrack track;
        track.appendSection();

        AuthoredTrackSection& first = track.section(0);
        first.kind = RegionKind::Geometry;
        const PlanarArcRegion leadArc{22.0, pi / 3.0, 0.15, 0.7};
        first.region = GeometryRegion{leadArc};
        first.length = planarArcLength(leadArc);

        track.appendSection();
        AuthoredTrackSection& second = track.section(1);
        second.kind = RegionKind::Geometry;
        const PlanarArcRegion trailArc{30.0, pi / 2.0, 0.0, 0.0};
        second.region = GeometryRegion{trailArc};
        second.length = planarArcLength(trailArc);

        const auto trackStates = integrateAuthoredTrack(track, 1.0);

        // Standalone reference: solve the lead arc alone, then feed its
        // exit state into the trailing arc.
        const auto leadStates = integratePlanarArcRegion(
            glm::dvec3{0.0, 0.0, 0.0},
            identityFrame(),
            leadArc,
            1.0);
        const auto trailStates = integratePlanarArcRegion(
            leadStates.back().position,
            leadStates.back().frame,
            trailArc,
            1.0);

        // The lead exit really carries the authored bank: measured against
        // the unbanked twin's exit orientation so the arc's own turning is
        // not mistaken for roll.
        const PlanarArcRegion unbankedLeadArc{
            leadArc.radius, leadArc.sweptAngle, leadArc.planeTilt, 0.0};
        const auto unbankedLeadStates = integratePlanarArcRegion(
            glm::dvec3{0.0, 0.0, 0.0},
            identityFrame(),
            unbankedLeadArc,
            1.0);
        const double exitBank = signedAngleAbout(
            leadStates.back().frame.tangent,
            unbankedLeadStates.back().frame.lateral,
            leadStates.back().frame.lateral);
        require(std::abs(exitBank - leadArc.bankChange) <= 1e-9,
            "lead construction exits with the authored bank applied");

        // Track output after the lead section equals the trailing solve
        // started from the banked exit state, with distances offset.
        require(trackStates.size()
                == leadStates.size() + trailStates.size() - 1,
            "chained track concatenates without duplicating the joint");

        std::vector<RiderLocalGeometryState> trackSlice(
            trackStates.begin()
                + static_cast<std::ptrdiff_t>(leadStates.size()),
            trackStates.end());

        std::vector<RiderLocalGeometryState> shiftedTrail;
        shiftedTrail.reserve(trailStates.size() - 1);
        for (std::size_t index = 1; index < trailStates.size(); ++index)
        {
            RiderLocalGeometryState state = trailStates[index];
            state.distance +=
                planarArcLength(leadArc);
            shiftedTrail.push_back(std::move(state));
        }

        requireSameStates(shiftedTrail, trackSlice, 0.0,
            "trailing region continues from the banked exit frame");
    }
}

int main()
{
    const std::pair<std::string_view, void(*)()> tests[] = {
        {"validation refuses malformed constructions",
            testValidationRefusals},
        {"flat arc follows authored horizontal circle",
            testFlatArcGeometry},
        {"vertical tilts author arch and dip",
            testVerticalArcOrientations},
        {"intermediate tilt authors inclined planar arc",
            testIntermediateTiltArc},
        {"bank change never moves the centerline",
            testBankChangeDoesNotMoveCenterline},
        {"construction equals rate-solver representation",
            testEquivalenceWithRateSolverRepresentation},
        {"mixed rate and geometry sections chain continuously",
            testMixedTrackChainsContinuously},
        {"rescale adjusts sweep preserving radius",
            testRescaleAdjustsSweptAnglePreservingRadius},
        {"zero bank preserves inherited orientation exactly",
            testZeroBankPreservesInheritedOrientationExactly},
        {"positive and negative bank behave symmetrically",
            testPositiveAndNegativeBankBehaveSymmetrically},
        {"banking accumulates continuously",
            testBankingAccumulatesContinuously},
        {"banked exit frame is inherited by following region",
            testBankedExitFrameIsInheritedByFollowingRegion}
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

    std::cout << "All PlanarArcRegion tests passed.\n";
    return 0;
}
