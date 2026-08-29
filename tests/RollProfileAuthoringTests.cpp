#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/editor/TransitionEditorModel.hpp>

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
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

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string_view message)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(message) + ": expected "
                + std::to_string(expected) + ", got "
                + std::to_string(actual)
            );
        }
    }

    [[nodiscard]] bool sameChannelProfile(
        const quantum::coaster::ChannelProfile& first,
        const quantum::coaster::ChannelProfile& second)
    {
        if (first.nextSegmentId != second.nextSegmentId
            || first.segments.size() != second.segments.size())
        {
            return false;
        }

        for (std::size_t index = 0; index < first.segments.size(); ++index)
        {
            const quantum::coaster::ProfileSegment& a =
                first.segments[index];
            const quantum::coaster::ProfileSegment& b =
                second.segments[index];
            if (a.id != b.id
                || a.transition.domainBegin != b.transition.domainBegin
                || a.transition.domainEnd != b.transition.domainEnd
                || a.transition.valueBegin != b.transition.valueBegin
                || a.transition.valueEnd != b.transition.valueEnd
                || a.transition.transitionType
                    != b.transition.transitionType)
            {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool finiteVector(const glm::dvec3& value)
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    void authorOneRevolutionRollProfile()
    {
        using namespace quantum;

        constexpr double length = 15.0;
        constexpr double peakRollDegreesPerMeter = 48.0;

        coaster::AuthoredTrack track;
        track.appendSection();
        coaster::setSectionLength(track.section(0), length);

        coaster::GeometricSection& rates =
            track.section(0).rateProfileRegion().rateProfiles;
        const coaster::ChannelProfile pitchBefore = rates.pitch;
        const coaster::ChannelProfile yawBefore = rates.yaw;
        coaster::ChannelProfile& roll = rates.roll;
        coaster::ProfileSegment& rollSegment = roll.segments.front();
        rollSegment.transition.transitionType =
            math::TransitionType::Smootherstep;

        const double peakRollRadiansPerMeter = editor::
            angularRateDegreesToRadians(peakRollDegreesPerMeter);
        coaster::setChannelSegmentValue(
            roll,
            rollSegment.id,
            coaster::ProfileBoundary::Begin,
            0.0
        );
        coaster::setChannelSegmentValue(
            roll,
            rollSegment.id,
            coaster::ProfileBoundary::End,
            peakRollRadiansPerMeter
        );

        requireNear(
            editor::angularRateRadiansToDegrees(
                rollSegment.transition.valueEnd
            ),
            peakRollDegreesPerMeter,
            1e-12,
            "numeric Roll Rate conversion preserves 48 deg/m"
        );
        requireNear(
            editor::angularRateRadiansToDegrees(
                coaster::evaluateChannelProfile(roll, length * 0.5)
            ),
            24.0,
            1e-11,
            "symmetric eased Roll Rate reaches 24 deg/m at mid-region"
        );

        const std::array<double, 2> rollEndpoints{
            0.0,
            peakRollRadiansPerMeter
        };
        const editor::GraphValueRange rollRange =
            editor::fitSymmetricGraphRange(
                rollEndpoints,
                editor::defaultGraphMagnitude(editor::RateChannel::Roll)
            );
        require(
            rollRange.maximum > peakRollRadiansPerMeter,
            "Roll graph fit does not clamp a 48 deg/m endpoint"
        );
        const double screenValue = editor::graphValueToNormalized(
            peakRollRadiansPerMeter,
            rollRange
        );
        requireNear(
            editor::normalizedToGraphValue(screenValue, rollRange),
            peakRollRadiansPerMeter,
            1e-15,
            "large Roll Rate graph transform round trip"
        );

        const double integratedRollDegrees =
            editor::computeChannelNetRotationDegrees(roll);
        requireNear(
            integratedRollDegrees,
            360.0,
            1e-10,
            "analytic eased profile integral accumulates one revolution"
        );
        require(
            integratedRollDegrees
                != peakRollDegreesPerMeter * length,
            "integrated Roll diagnostic is not endpoint rate times length"
        );
        require(
            sameChannelProfile(rates.pitch, pitchBefore),
            "Roll authoring leaves Pitch data unchanged"
        );
        require(
            sameChannelProfile(rates.yaw, yawBefore),
            "Roll authoring leaves Yaw data unchanged"
        );

        const std::string serialized =
            coaster::serializeCoasterDocument(track);
        auto restored = coaster::deserializeCoasterDocument(serialized);
        require(restored.has_value(),
            "large Roll Rate document round-trip succeeds");
        const coaster::GeometricSection& restoredRates = restored->section(0)
            .rateProfileRegion().rateProfiles;
        requireNear(
            editor::computeChannelNetRotationDegrees(restoredRates.roll),
            360.0,
            1e-10,
            "serialized Roll profile preserves accumulated angle"
        );
        require(
            sameChannelProfile(restoredRates.pitch, pitchBefore),
            "serialized large-Roll profile preserves Pitch"
        );
        require(
            sameChannelProfile(restoredRates.yaw, yawBefore),
            "serialized large-Roll profile preserves Yaw"
        );

        const auto states = coaster::integrateAuthoredTrack(*restored, 0.25);
        require(states.size() >= 2,
            "authoritative integration returns generated geometry");
        for (const coaster::RiderLocalGeometryState& state : states)
        {
            require(
                std::isfinite(state.distance)
                    && finiteVector(state.position)
                    && finiteVector(state.frame.tangent)
                    && finiteVector(state.frame.lateral)
                    && finiteVector(state.frame.up),
                "one-revolution Roll profile generates finite geometry"
            );
        }
    }
}

int main()
{
    try
    {
        authorOneRevolutionRollProfile();
        std::cout << "[PASS] one-revolution Roll profile authoring\n";
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] one-revolution Roll profile authoring: "
            << exception.what() << '\n';
        return 1;
    }

    std::cout << "All Roll profile authoring tests passed.\n";
    return 0;
}
