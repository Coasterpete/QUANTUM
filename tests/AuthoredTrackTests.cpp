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
    using quantum::coaster::createDefaultAuthoredTrack;
    using quantum::coaster::defaultNewSectionLength;
    using quantum::coaster::GeometricSection;
    using quantum::coaster::integrateAuthoredTrack;
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

    [[nodiscard]] ScalarTransition transition(
        const double domainEnd,
        const double valueBegin,
        const double valueEnd,
        const TransitionType type)
    {
        return {0.0, domainEnd, valueBegin, valueEnd, type};
    }

    [[nodiscard]] AuthoredTrackSection curvedSection(
        const double length,
        const double rollEnd,
        const double pitchBegin,
        const double pitchEnd,
        const double yawEnd)
    {
        AuthoredTrackSection section;
        section.rateProfiles.roll =
            transition(length, 0.0, rollEnd, TransitionType::Smootherstep);
        section.rateProfiles.pitch = transition(
            length,
            pitchBegin,
            pitchEnd,
            TransitionType::CosineEaseInOut
        );
        section.rateProfiles.yaw =
            transition(length, 0.004, yawEnd, TransitionType::Smoothstep);
        return section;
    }

    void testDefaultDocumentReproducesDemonstrationBehavior()
    {
        const AuthoredTrack track = createDefaultAuthoredTrack();

        require(track.sectionCount() == 1, "default track has one section");

        const GeometricSection& rates = track.section(0).rateProfiles;

        require(rates.roll.domainBegin == 0.0, "roll domain begins at zero");
        require(rates.roll.domainEnd == 180.0, "roll domain ends at 180");
        require(rates.pitch.domainBegin == 0.0, "pitch domain begins at zero");
        require(rates.pitch.domainEnd == 180.0, "pitch domain ends at 180");
        require(rates.yaw.domainBegin == 0.0, "yaw domain begins at zero");
        require(rates.yaw.domainEnd == 180.0, "yaw domain ends at 180");

        require(rates.roll.valueBegin == 0.0, "default roll begin");
        require(rates.roll.valueEnd == 0.024, "default roll end");
        require(
            rates.roll.transitionType == TransitionType::Smootherstep,
            "default roll transition"
        );
        require(rates.pitch.valueBegin == 0.018, "default pitch begin");
        require(rates.pitch.valueEnd == -0.010, "default pitch end");
        require(
            rates.pitch.transitionType == TransitionType::CosineEaseInOut,
            "default pitch transition"
        );
        require(rates.yaw.valueBegin == 0.004, "default yaw begin");
        require(rates.yaw.valueEnd == 0.022, "default yaw end");
        require(
            rates.yaw.transitionType == TransitionType::Smoothstep,
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
                section.rateProfiles.pitch.valueBegin == 0.0
                    && section.rateProfiles.pitch.valueEnd == 0.0,
                "new sections are straight"
            );
            require(
                section.rateProfiles.pitch.transitionType
                    == TransitionType::Linear,
                "new sections use Linear transitions"
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

        require(section.rateProfiles.roll.domainBegin == 0.0, "begin stays zero");
        require(section.rateProfiles.roll.domainEnd == 45.0, "roll rebased");
        require(section.rateProfiles.pitch.domainEnd == 45.0, "pitch rebased");
        require(section.rateProfiles.yaw.domainEnd == 45.0, "yaw rebased");
        require(section.rateProfiles.roll.valueBegin == 0.0, "roll begin kept");
        require(section.rateProfiles.roll.valueEnd == 0.02, "roll end kept");
        require(section.rateProfiles.pitch.valueEnd == -0.02, "pitch end kept");

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

    void testSectionAccessAndValidation()
    {
        AuthoredTrack track;
        track.appendSection();

        requireThrows<std::out_of_range>(
            [&track] { static_cast<void>(track.section(1)); },
            "const access rejects out-of-range index"
        );
        requireThrows<std::out_of_range>(
            [&track] { static_cast<void>(track.section(1).rateProfiles); },
            "mutable access rejects out-of-range index"
        );

        // A malformed section (mismatched channel domains) must be rejected
        // by generation.
        track.appendSection();
        track.section(1).rateProfiles.yaw.domainEnd = 11.0;

        requireThrows<std::invalid_argument>(
            [&track]
            {
                static_cast<void>(integrateAuthoredTrack(track, 0.75));
            },
            "generation rejects mismatched channel domains"
        );
    }

    void testSingleSectionMatchesDirectIntegration()
    {
        const AuthoredTrack track = createDefaultAuthoredTrack();

        const std::vector<RiderLocalGeometryState> chained =
            integrateAuthoredTrack(track, 0.75);

        const GeometricSection& rates = track.section(0).rateProfiles;
        const std::vector<RiderLocalGeometryState> direct =
            quantum::coaster::integrateLocalRollPitchYawRateProfiles(
                {0.0, 0.0, 0.0},
                {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
                rates.roll,
                rates.pitch,
                rates.yaw,
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

    void testMultiSectionChainIsContinuous()
    {
        AuthoredTrack track;
        track.appendSection();
        setSectionLength(track.section(0), 90.0);
        track.section(0).rateProfiles =
            curvedSection(90.0, 0.024, 0.018, -0.010, 0.022).rateProfiles;
        track.appendSection();
        setSectionLength(track.section(1), 45.0);
        track.section(1).rateProfiles =
            curvedSection(45.0, 0.0, 0.006, 0.006, 0.0).rateProfiles;

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
                    section.rateProfiles.roll,
                    section.rateProfiles.pitch,
                    section.rateProfiles.yaw,
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
        section.rateProfiles.pitch = transition(60.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfiles.yaw = transition(61.0, 0.0, 0.0, TransitionType::Linear);
        section.rateProfiles.roll = transition(60.0, 0.0, 0.0, TransitionType::Linear);

        requireThrows<std::invalid_argument>(
            [&section] { static_cast<void>(sectionLength(section)); },
            "sectionLength validates channel domains"
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
        {"section access and validation", testSectionAccessAndValidation},
        {
            "single section matches direct integration",
            testSingleSectionMatchesDirectIntegration
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

