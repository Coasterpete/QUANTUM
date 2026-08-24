#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/GeometricSection.hpp>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <quantum/geometry/RotationMinimizingFrames.hpp>

#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <iterator>
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
    using quantum::coaster::createDefaultAuthoredTrack;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::integrateAuthoredTrack;
    using quantum::coaster::ProfileSegment;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::coaster::sectionLength;
    using quantum::coaster::setSectionLength;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

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

    // Single-segment channel covering [0, length].
    [[nodiscard]] ChannelProfile singleSegmentChannel(
        const double length,
        const double valueBegin,
        const double valueEnd,
        const TransitionType type)
    {
        ChannelProfile profile;
        profile.segments.push_back(ProfileSegment{
            profile.nextSegmentId,
            ScalarTransition{0.0, length, valueBegin, valueEnd, type}
        });
        ++profile.nextSegmentId;
        return profile;
    }

    [[nodiscard]] AuthoredTrackSection curvedSection(
        const double length,
        const double rollEnd,
        const double pitchBegin,
        const double pitchEnd,
        const double yawEnd)
    {
        AuthoredTrackSection section;
        section.length = length;
        section.rateProfileRegion().rateProfiles.roll =
            singleSegmentChannel(length, 0.0, rollEnd, TransitionType::Smootherstep);
        section.rateProfileRegion().rateProfiles.pitch = singleSegmentChannel(
            length,
            pitchBegin,
            pitchEnd,
            TransitionType::CosineEaseInOut
        );
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(length, 0.004, yawEnd, TransitionType::Smoothstep);
        return section;
    }

    [[nodiscard]] const ScalarTransition& soleTransition(
        const ChannelProfile& channel)
    {
        return channel.segments.front().transition;
    }

    [[nodiscard]] ScalarTransition& soleTransition(ChannelProfile& channel)
    {
        return channel.segments.front().transition;
    }

    void testDefaultDocumentReproducesDemonstrationBehavior()
    {
        const AuthoredTrack track = createDefaultAuthoredTrack();

        require(track.sectionCount() == 1, "default track has one section");

        const GeometricSection& rates = track.section(0).rateProfileRegion().rateProfiles;

        require(track.section(0).length == 180.0, "default section length");
        require(
            soleTransition(rates.roll).domainBegin == 0.0,
            "roll domain begins at zero"
        );
        require(
            soleTransition(rates.roll).domainEnd == 180.0,
            "roll domain ends at 180"
        );
        require(
            soleTransition(rates.pitch).domainBegin == 0.0,
            "pitch domain begins at zero"
        );
        require(
            soleTransition(rates.pitch).domainEnd == 180.0,
            "pitch domain ends at 180"
        );
        require(
            soleTransition(rates.yaw).domainBegin == 0.0,
            "yaw domain begins at zero"
        );
        require(
            soleTransition(rates.yaw).domainEnd == 180.0,
            "yaw domain ends at 180"
        );

        require(
            soleTransition(rates.roll).valueBegin == 0.0,
            "default roll begin"
        );
        require(
            soleTransition(rates.roll).valueEnd == 0.024,
            "default roll end"
        );
        require(
            soleTransition(rates.roll).transitionType
                == TransitionType::Smootherstep,
            "default roll transition"
        );
        require(
            soleTransition(rates.pitch).valueBegin == 0.018,
            "default pitch begin"
        );
        require(
            soleTransition(rates.pitch).valueEnd == -0.010,
            "default pitch end"
        );
        require(
            soleTransition(rates.pitch).transitionType
                == TransitionType::CosineEaseInOut,
            "default pitch transition"
        );
        require(
            soleTransition(rates.yaw).valueBegin == 0.004,
            "default yaw begin"
        );
        require(
            soleTransition(rates.yaw).valueEnd == 0.022,
            "default yaw end"
        );
        require(
            soleTransition(rates.yaw).transitionType
                == TransitionType::Smoothstep,
            "default yaw transition"
        );
    }

    void testAppendPrependOrderAndDefaults()
    {
        AuthoredTrack track;
        require(track.sectionCount() == 0, "fresh track is empty");
        track.appendSection();
        track.appendSection();
        track.prependSection();

        require(track.sectionCount() == 3, "append/prepend counts");

        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            const AuthoredTrackSection& section = track.section(index);
            require(
                sectionLength(section) == defaultNewSectionLength,
                "new sections use the default length"
            );
            require(
                section.rateProfileRegion().rateProfiles.pitch.segments.size() == 1,
                "new sections have one segment per channel"
            );
            const ScalarTransition& pitch =
                soleTransition(section.rateProfileRegion().rateProfiles.pitch);
            require(
                pitch.valueBegin == 0.0 && pitch.valueEnd == 0.0,
                "new sections are straight"
            );
            require(
                pitch.transitionType == TransitionType::Linear,
                "new sections use Linear transitions"
            );
            require(
                section.rateProfileRegion().rateProfiles.pitch.nextSegmentId == 2
                    && section.rateProfileRegion().rateProfiles.yaw.nextSegmentId == 2
                    && section.rateProfileRegion().rateProfiles.roll.nextSegmentId == 2,
                "new sections start their id counters after the first segment"
            );
        }
    }

    void testRemoveSection()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 10.0);
        track.appendSection();
        setSectionLength(track.section(1), 20.0);
        track.appendSection();
        setSectionLength(track.section(2), 30.0);

        track.removeSection(1);
        require(track.sectionCount() == 2, "remove shrinks the track");
        require(
            sectionLength(track.section(0)) == 10.0
                && sectionLength(track.section(1)) == 30.0,
            "remove deletes exactly the addressed section"
        );

        requireThrows<std::out_of_range>(
            [&track] { track.removeSection(2); },
            "remove rejects out-of-range index"
        );

        track.removeSection(0);
        requireThrows<std::invalid_argument>(
            [&track] { track.removeSection(0); },
            "the last section cannot be removed"
        );
        require(track.sectionCount() == 1, "one section remains");
    }

    void testMoveSection()
    {
        AuthoredTrack track;
        for (const double length : {10.0, 20.0, 30.0})
        {
            track.appendSection();
            setSectionLength(track.section(track.sectionCount() - 1), length);
        }

        track.moveSection(0, 0);
        require(
            sectionLength(track.section(0)) == 10.0,
            "self-move leaves ordering unchanged"
        );

        track.moveSection(0, 2);
        require(
            sectionLength(track.section(0)) == 20.0
                && sectionLength(track.section(1)) == 30.0
                && sectionLength(track.section(2)) == 10.0,
            "forward move produces final-position semantics"
        );

        track.moveSection(2, 0);
        require(
            sectionLength(track.section(0)) == 10.0
                && sectionLength(track.section(1)) == 20.0
                && sectionLength(track.section(2)) == 30.0,
            "backward move restores ordering"
        );

        requireThrows<std::out_of_range>(
            [&track] { track.moveSection(0, 3); },
            "move rejects out-of-range destination"
        );
        requireThrows<std::out_of_range>(
            [&track] { track.moveSection(3, 0); },
            "move rejects out-of-range source"
        );
    }

    void testSetSectionLengthRebasesDomains()
    {
        AuthoredTrackSection section =
            curvedSection(90.0, 0.02, 0.01, -0.02, 0.03);

        setSectionLength(section, 45.0);

        require(
            soleTransition(section.rateProfileRegion().rateProfiles.roll).domainBegin == 0.0,
            "begin stays zero"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.roll).domainEnd == 45.0,
            "roll rebased"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.pitch).domainEnd == 45.0,
            "pitch rebased"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.yaw).domainEnd == 45.0,
            "yaw rebased"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.roll).valueBegin == 0.0,
            "roll begin kept"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.roll).valueEnd == 0.02,
            "roll end kept"
        );
        require(
            soleTransition(section.rateProfileRegion().rateProfiles.pitch).valueEnd == -0.02,
            "pitch end kept"
        );

        require(
            sectionLength(section) == 45.0,
            "sectionLength reports the rebased length"
        );

        requireThrows<std::invalid_argument>(
            [&section] { setSectionLength(section, 0.0); },
            "zero length rejected"
        );
        requireThrows<std::invalid_argument>(
            [&section] { setSectionLength(section, -1.0); },
            "negative length rejected"
        );
        requireThrows<std::invalid_argument>(
            [&section]
            {
                setSectionLength(
                    section,
                    std::numeric_limits<double>::quiet_NaN()
                );
            },
            "NaN length rejected"
        );
        requireThrows<std::invalid_argument>(
            [&section]
            {
                setSectionLength(
                    section,
                    std::numeric_limits<double>::infinity()
                );
            },
            "infinite length rejected"
        );

        require(
            sectionLength(section) == 45.0,
            "rejected edits leave the length unchanged"
        );
    }

    void testSetSectionLengthRescalesMultiSegmentChannels()
    {
        AuthoredTrackSection section;
        section.length = 90.0;

        // Pitch chains two segments joined at distance 30 with value 0.02.
        ChannelProfile& pitch = section.rateProfileRegion().rateProfiles.pitch;
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId++,
            ScalarTransition{0.0, 30.0, 0.0, 0.02, TransitionType::Linear}
        });
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId++,
            ScalarTransition{
                30.0,
                90.0,
                0.02,
                -0.04,
                TransitionType::Smoothstep}
        });
        section.rateProfileRegion().rateProfiles.yaw = singleSegmentChannel(
            90.0,
            0.004,
            0.022,
            TransitionType::Smoothstep
        );
        section.rateProfileRegion().rateProfiles.roll = singleSegmentChannel(
            90.0,
            0.0,
            0.024,
            TransitionType::Smootherstep
        );

        setSectionLength(section, 30.0);

        constexpr double scaleFactor = 30.0 / 90.0;

        require(section.length == 30.0, "length field updated");

        const ScalarTransition& pitchFirst = pitch.segments[0].transition;
        const ScalarTransition& pitchSecond = pitch.segments[1].transition;
        require(
            pitchFirst.domainBegin == 0.0,
            "first boundary pinned to zero"
        );
        require(
            pitchFirst.domainEnd == 30.0 * scaleFactor,
            "interior boundary scaled proportionally"
        );
        require(
            pitchSecond.domainBegin == 30.0 * scaleFactor,
            "shared boundary stays contiguous after scaling"
        );
        require(
            pitchSecond.domainEnd == 30.0,
            "last boundary lands exactly on the new length"
        );
        require(
            pitchFirst.valueBegin == 0.0 && pitchFirst.valueEnd == 0.02
                && pitchSecond.valueEnd == -0.04,
            "authored values survive rescaling"
        );
        require(
            pitch.segments[0].id == 1 && pitch.segments[1].id == 2
                && pitch.nextSegmentId == 3,
            "segment ids stay stable across rescaling"
        );

        require(
            sectionLength(section) == 30.0,
            "rescaled section validates against its new length"
        );
    }

    void testSectionAccessAndValidation()
    {
        AuthoredTrack track;
        track.appendSection();

        requireThrows<std::out_of_range>(
            [&track] { static_cast<void>(track.section(1)); },
            "const access rejects out-of-range index"
        );
        requireThrows<std::out_of_range>(
            [&track] { static_cast<void>(track.section(1).rateProfileRegion().rateProfiles); },
            "mutable access rejects out-of-range index"
        );

        // A malformed section (a channel no longer covering its declared
        // length) must be rejected by generation.
        track.appendSection();
        soleTransition(track.section(1).rateProfileRegion().rateProfiles.yaw).domainEnd = 11.0;

        requireThrows<std::invalid_argument>(
            [&track]
            {
                static_cast<void>(integrateAuthoredTrack(track, 0.75));
            },
            "generation rejects incomplete channel coverage"
        );
    }

    void testSingleSectionMatchesDirectIntegration()
    {
        const AuthoredTrack track = createDefaultAuthoredTrack();

        const std::vector<RiderLocalGeometryState> chained =
            integrateAuthoredTrack(track, 0.75);

        const GeometricSection& rates = track.section(0).rateProfileRegion().rateProfiles;
        const std::vector<RiderLocalGeometryState> direct =
            quantum::coaster::integrateLocalRollPitchYawRateProfiles(
                {0.0, 0.0, 0.0},
                {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
                soleTransition(rates.roll),
                soleTransition(rates.pitch),
                soleTransition(rates.yaw),
                0.75
            );

        require(chained.size() == direct.size(), "sample counts match");
        require(chained.size() >= 2u, "at least two samples generated");

        for (std::size_t i = 0; i < chained.size(); ++i)
        {
            require(
                chained[i].distance == direct[i].distance,
                "distances match direct integration"
            );
            require(
                chained[i].position == direct[i].position,
                "positions match direct integration"
            );
        }
    }

    void testMultiSegmentSectionIntegrationSpansKinks()
    {
        AuthoredTrackSection section;
        section.length = 90.0;

        // Triangular pitch-rate profile with a C0 kink at the midpoint; yaw
        // breaks at 22.5 so the merged breakpoints come from different
        // channels; roll stays zero throughout.
        ChannelProfile& pitch = section.rateProfileRegion().rateProfiles.pitch;
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId++,
            ScalarTransition{0.0, 45.0, 0.0, 0.03, TransitionType::Linear}
        });
        pitch.segments.push_back(ProfileSegment{
            pitch.nextSegmentId++,
            ScalarTransition{45.0, 90.0, 0.03, 0.0, TransitionType::Linear}
        });

        ChannelProfile& yaw = section.rateProfileRegion().rateProfiles.yaw;
        yaw.segments.push_back(ProfileSegment{
            yaw.nextSegmentId++,
            ScalarTransition{0.0, 22.5, 0.004, 0.004, TransitionType::Linear}
        });
        yaw.segments.push_back(ProfileSegment{
            yaw.nextSegmentId++,
            ScalarTransition{22.5, 90.0, 0.004, 0.004, TransitionType::Linear}
        });

        section.rateProfileRegion().rateProfiles.roll = singleSegmentChannel(
            90.0,
            0.0,
            0.0,
            TransitionType::Linear
        );

        AuthoredTrack track;
        track.appendSection();
        track.section(0) = section;

        const std::vector<RiderLocalGeometryState> states =
            integrateAuthoredTrack(track, 0.75);

        // Each span contributes ceil(span / spacing) + 1 states and shares
        // exactly one joint state with its neighbor: 60 + 1 spans over 45
        // and 67.5 units produce 61 + 61 - 1 samples overall.
        constexpr std::size_t expectedStates = 121u;
        require(
            states.size() == expectedStates,
            "sample grids restart at every profile breakpoint"
        );
        require(states.front().distance == 0.0, "integration starts at zero");

        for (std::size_t i = 1; i < states.size(); ++i)
        {
            require(
                states[i].distance > states[i - 1].distance,
                "distances strictly increase across kinks"
            );
        }

        require(
            std::abs(states.back().distance - 90.0) < 1e-9,
            "multi-segment integration covers the whole section"
        );

        // The kink joins C0-continuously: the frame stays continuous, so
        // consecutive positions remain within one spacing step everywhere,
        // including across both profile breakpoints.
        for (std::size_t i = 1; i < states.size(); ++i)
        {
            const glm::dvec3 step = states[i].position - states[i - 1].position;
            require(
                glm::length(step) <= 0.75 * (1.0 + 1e-9),
                "consecutive samples remain within one integration spacing"
            );
        }
    }

    void testMultiSectionChainIsContinuous()
    {
        AuthoredTrack track;
        track.appendSection();
        track.section(0) =
            curvedSection(90.0, 0.024, 0.018, -0.010, 0.022);
        track.appendSection();
        track.section(1) =
            curvedSection(45.0, 0.0, 0.006, 0.006, 0.0);

        const std::vector<RiderLocalGeometryState> states =
            integrateAuthoredTrack(track, 0.75);

        constexpr double totalLength = 135.0;
        require(states.size() >= 3u, "chain produced multiple samples");
        require(states.front().distance == 0.0, "chain starts at distance zero");

        for (std::size_t i = 1; i < states.size(); ++i)
        {
            require(
                states[i].distance > states[i - 1].distance,
                "distances strictly increase across sections"
            );
            require(
                states[i].distance <= totalLength + 1e-9,
                "distances stay within the total track length"
            );
        }

        require(
            std::abs(states.back().distance - totalLength) < 1e-9,
            "chain ends at the summed section lengths"
        );

        // Consecutive samples are spaced by no more than the requested
        // spacing, including across section joints.
        for (std::size_t i = 1; i < states.size(); ++i)
        {
            const glm::dvec3 step = states[i].position - states[i - 1].position;
            require(
                glm::length(step) <= 0.75 * (1.0 + 1e-9),
                "consecutive samples remain within one integration spacing"
            );
        }

        // The second section starts from the first section's final frame, so
        // the tangent must be continuous across the joint.
        std::size_t jointIndex = 0;
        while (jointIndex + 1 < states.size()
            && states[jointIndex].distance < 90.0)
        {
            ++jointIndex;
        }

        require(jointIndex + 1 < states.size(), "joint sample pair exists");

        // The chaining must be exactly equivalent to manually feeding each
        // section's final position and frame into the next integration.
        std::vector<RiderLocalGeometryState> reference;
        glm::dvec3 position{0.0, 0.0, 0.0};
        quantum::geometry::CurveFrame frame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        double offset = 0.0;

        for (std::size_t index = 0; index < track.sectionCount(); ++index)
        {
            const AuthoredTrackSection& section = track.section(index);
            const std::vector<RiderLocalGeometryState> local =
                quantum::coaster::integrateLocalRollPitchYawRateProfiles(
                    position,
                    frame,
                    soleTransition(section.rateProfileRegion().rateProfiles.roll),
                    soleTransition(section.rateProfileRegion().rateProfiles.pitch),
                    soleTransition(section.rateProfileRegion().rateProfiles.yaw),
                    0.75
                );

            for (std::size_t i = reference.empty() ? 0u : 1u;
                i < local.size();
                ++i)
            {
                RiderLocalGeometryState state = local[i];
                state.distance += offset;
                reference.push_back(std::move(state));
            }

            position = local.back().position;
            frame = local.back().frame;
            offset += sectionLength(section);
        }

        require(reference.size() == states.size(), "reference sizes match");
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            require(
                states[i].distance == reference[i].distance
                    && states[i].position == reference[i].position,
                "chained state matches manual section chaining"
            );
            require(
                states[i].frame.tangent == reference[i].frame.tangent,
                "chained frame matches manual section chaining"
            );
        }
    }

    void testIntegrateRejectsInvalidInput()
    {
        requireThrows<std::invalid_argument>(
            []
            {
                static_cast<void>(
                    integrateAuthoredTrack(AuthoredTrack{}, 0.75));
            },
            "empty track rejected"
        );

        const AuthoredTrack track = createDefaultAuthoredTrack();

        requireThrows<std::invalid_argument>(
            [&track] { static_cast<void>(integrateAuthoredTrack(track, 0.0)); },
            "zero spacing rejected"
        );
        requireThrows<std::invalid_argument>(
            [&track]
            {
                static_cast<void>(
                    integrateAuthoredTrack(track, std::nan("")));
            },
            "NaN spacing rejected"
        );
        requireThrows<std::invalid_argument>(
            [&track]
            {
                static_cast<void>(
                    integrateAuthoredTrack(track, -0.5));
            },
            "negative spacing rejected"
        );
    }

    void testSectionLengthRejectsMalformedSection()
    {
        AuthoredTrackSection section;
        section.length = 60.0;
        section.rateProfileRegion().rateProfiles.pitch =
            singleSegmentChannel(60.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfileRegion().rateProfiles.yaw =
            singleSegmentChannel(61.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfileRegion().rateProfiles.roll =
            singleSegmentChannel(60.0, 0.0, 0.0, TransitionType::Linear);

        requireThrows<std::invalid_argument>(
            [&section] { static_cast<void>(sectionLength(section)); },
            "sectionLength validates channel coverage"
        );
    }
}

int main()
{
    const std::pair<std::string_view, std::function<void()>> tests[] = {
        {
            "default document reproduces demonstration behavior",
            testDefaultDocumentReproducesDemonstrationBehavior
        },
        {
            "append and prepend order and defaults",
            testAppendPrependOrderAndDefaults
        },
        {"remove section", testRemoveSection},
        {"move section", testMoveSection},
        {"set section length rebases domains", testSetSectionLengthRebasesDomains},
        {
            "set section length rescales multi-segment channels",
            testSetSectionLengthRescalesMultiSegmentChannels
        },
        {"section access and validation", testSectionAccessAndValidation},
        {
            "single section matches direct integration",
            testSingleSectionMatchesDirectIntegration
        },
        {
            "multi-segment section integration spans kinks",
            testMultiSegmentSectionIntegrationSpansKinks
        },
        {
            "multi-section chain is continuous",
            testMultiSectionChainIsContinuous
        },
        {"integration rejects invalid input", testIntegrateRejectsInvalidInput},
        {
            "sectionLength rejects malformed section",
            testSectionLengthRejectsMalformedSection
        }
    };

    int failures = 0;

    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            ++failures;
            std::cerr << "[FAIL] " << name << ": "
                      << exception.what() << '\n';
        }
        catch (...)
        {
            ++failures;
            std::cerr << "[FAIL] " << name
                      << ": unknown exception\n";
        }
    }

    if (failures != 0)
    {
        std::cerr << failures << " test group(s) failed.\n";
        return 1;
    }

    std::cout << std::size(tests) << " test groups passed.\n";
    return 0;
}
