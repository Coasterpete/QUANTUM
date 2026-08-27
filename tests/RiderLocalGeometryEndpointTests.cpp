#include <quantum/coaster/RiderLocalGeometry.hpp>
#include <quantum/coaster/TrackTopology.hpp>
#include <quantum/coaster/detail/RiderLocalGeometryDetail.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::coaster::TopologyTolerances;
    using quantum::coaster::detail::CoupledIntegrationSchedule;
    using quantum::coaster::detail::integrateCoupledRateProfilesEndpoint;
    using quantum::coaster::detail::integrateCoupledRateProfilesNumerically;
    using quantum::coaster::detail::makeCoupledIntegrationSchedule;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;
    using quantum::math::ScalarTransition;
    using quantum::math::TransitionType;

    constexpr std::size_t baselineInternalPanelCount = 1'024;

    const glm::dvec3 startingPosition{2.75, -4.5, 1.25};
    const CurveFrame startingFrame = applyLocalYaw(
        applyLocalPitch(
            applyRoll(
                CurveFrame{
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0}
                },
                0.37
            ),
            -0.29
        ),
        0.41
    );

    struct Profiles
    {
        ScalarTransition roll;
        ScalarTransition pitch;
        ScalarTransition yaw;
    };

    struct EndpointMeasurement
    {
        std::string name;
        double positionDifference;
        double tangentDifference;
        double lateralDifference;
        double upDifference;
        std::size_t internalPanelCount;
        std::size_t actualSubstepCount;
    };

    struct SpecializedMeasurement
    {
        std::string name;
        double positionDifference;
        double tangentDifference;
        double upDifference;
        double tangentDifferenceDegrees;
        double upDifferenceDegrees;
    };

    std::vector<EndpointMeasurement> endpointMeasurements;
    std::vector<SpecializedMeasurement> specializedMeasurements;

    [[noreturn]] void fail(const std::string& message)
    {
        throw std::runtime_error(message);
    }

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
        {
            fail(message);
        }
    }

    [[nodiscard]] double vectorDifference(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        return glm::length(first - second);
    }

    [[nodiscard]] double angularDifferenceDegrees(
        const glm::dvec3& first,
        const glm::dvec3& second
    )
    {
        const double cosine = glm::clamp(glm::dot(first, second), -1.0, 1.0);
        return std::acos(cosine) * 180.0 / std::numbers::pi_v<double>;
    }

    [[nodiscard]] Profiles makeProfiles(
        const double length,
        const double rollBegin,
        const double rollEnd,
        const TransitionType rollType,
        const double pitchBegin,
        const double pitchEnd,
        const TransitionType pitchType,
        const double yawBegin,
        const double yawEnd,
        const TransitionType yawType
    )
    {
        return {
            {0.0, length, rollBegin, rollEnd, rollType},
            {0.0, length, pitchBegin, pitchEnd, pitchType},
            {0.0, length, yawBegin, yawEnd, yawType}
        };
    }

    [[nodiscard]] CoupledIntegrationSchedule makeSchedule(
        const Profiles& profiles,
        const double spacing
    )
    {
        return makeCoupledIntegrationSchedule(
            &profiles.roll,
            profiles.pitch,
            profiles.yaw,
            spacing
        );
    }

    void requireExplicitSchedule(
        const CoupledIntegrationSchedule& schedule,
        const std::string_view context
    )
    {
        require(
            !schedule.substepLengths.empty(),
            std::string(context) + " schedule has no substeps"
        );
        require(
            !schedule.outputBoundaries.empty(),
            std::string(context) + " schedule has no output boundaries"
        );
        require(
            schedule.outputBoundaries.back().completedSubstepCount
                == schedule.substepLengths.size(),
            std::string(context) + " final boundary omits substeps"
        );
        require(
            schedule.outputBoundaries.back().distance
                == schedule.profileLength,
            std::string(context) + " final output distance is not exact"
        );

        std::size_t previousSubstepCount = 0;
        double previousOutputDistance = 0.0;
        double walkedDistance = 0.0;
        for (const auto& boundary : schedule.outputBoundaries)
        {
            require(
                boundary.completedSubstepCount > previousSubstepCount
                    && boundary.completedSubstepCount
                        <= schedule.substepLengths.size(),
                std::string(context) + " invalid substep boundary"
            );
            require(
                boundary.distance > previousOutputDistance,
                std::string(context) + " output distances are not increasing"
            );

            for (std::size_t index = previousSubstepCount;
                 index < boundary.completedSubstepCount;
                 ++index)
            {
                require(
                    std::isfinite(schedule.substepLengths[index])
                        && schedule.substepLengths[index] > 0.0,
                    std::string(context) + " invalid explicit substep length"
                );
                walkedDistance += schedule.substepLengths[index];
            }

            const double tolerance = 8.0
                * std::numeric_limits<double>::epsilon()
                * std::max(1.0, schedule.profileLength);
            require(
                std::abs(walkedDistance - boundary.distance) <= tolerance,
                std::string(context) + " substeps do not reproduce boundary"
            );
            walkedDistance = boundary.distance;
            previousSubstepCount = boundary.completedSubstepCount;
            previousOutputDistance = boundary.distance;
        }
    }

    void checkEndpointEquivalence(
        const std::string_view name,
        const Profiles& profiles,
        const double spacing,
        const bool requireBeyondBaseline = false
    )
    {
        const CoupledIntegrationSchedule schedule =
            makeSchedule(profiles, spacing);
        requireExplicitSchedule(schedule, name);

        if (requireBeyondBaseline)
        {
            require(
                schedule.internalPanelCount > baselineInternalPanelCount,
                std::string(name)
                    + " did not exceed the baseline internal panel regime"
            );
        }

        const std::vector<RiderLocalGeometryState> full =
            integrateCoupledRateProfilesNumerically(
                startingPosition,
                startingFrame,
                &profiles.roll,
                profiles.pitch,
                profiles.yaw,
                schedule
            );
        const RiderLocalGeometryState endpoint =
            integrateCoupledRateProfilesEndpoint(
                startingPosition,
                startingFrame,
                &profiles.roll,
                profiles.pitch,
                profiles.yaw,
                schedule
            );
        const RiderLocalGeometryState& fullEndpoint = full.back();

        const EndpointMeasurement measurement{
            std::string(name),
            vectorDifference(endpoint.position, fullEndpoint.position),
            vectorDifference(
                endpoint.frame.tangent,
                fullEndpoint.frame.tangent
            ),
            vectorDifference(
                endpoint.frame.lateral,
                fullEndpoint.frame.lateral
            ),
            vectorDifference(endpoint.frame.up, fullEndpoint.frame.up),
            schedule.internalPanelCount,
            schedule.substepLengths.size()
        };
        endpointMeasurements.push_back(measurement);

        const double positionTolerance = 64.0
            * std::numeric_limits<double>::epsilon()
            * std::max({
                1.0,
                schedule.profileLength,
                glm::length(startingPosition)
            });
        constexpr double frameTolerance = 64.0
            * std::numeric_limits<double>::epsilon();

        require(
            endpoint.distance == fullEndpoint.distance
                && endpoint.distance == schedule.profileLength,
            std::string(name) + " endpoint distance differs"
        );
        require(
            measurement.positionDifference <= positionTolerance,
            std::string(name)
                + " endpoint position exceeds roundoff-scale tolerance"
        );
        require(
            measurement.tangentDifference <= frameTolerance,
            std::string(name)
                + " endpoint tangent exceeds roundoff-scale tolerance"
        );
        require(
            measurement.lateralDifference <= frameTolerance,
            std::string(name)
                + " endpoint lateral exceeds roundoff-scale tolerance"
        );
        require(
            measurement.upDifference <= frameTolerance,
            std::string(name)
                + " endpoint up exceeds roundoff-scale tolerance"
        );
    }

    void testEndpointEquivalenceCoverage()
    {
        checkEndpointEquivalence(
            "all-zero short",
            makeProfiles(
                0.125,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            ),
            0.04
        );
        checkEndpointEquivalence(
            "constant positive pitch",
            makeProfiles(
                18.0,
                0.0, 0.0, TransitionType::Linear,
                0.18, 0.18, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            ),
            1.7
        );
        checkEndpointEquivalence(
            "constant negative yaw",
            makeProfiles(
                32.0,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                -0.14, -0.14, TransitionType::Linear
            ),
            2.3
        );
        checkEndpointEquivalence(
            "constant roll",
            makeProfiles(
                25.0,
                0.25, 0.25, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            ),
            4.2
        );
        checkEndpointEquivalence(
            "constant mixed rates",
            makeProfiles(
                40.0,
                -0.11, -0.11, TransitionType::Linear,
                0.17, 0.17, TransitionType::Linear,
                -0.13, -0.13, TransitionType::Linear
            ),
            3.7
        );
        checkEndpointEquivalence(
            "variable pitch",
            makeProfiles(
                12.0,
                0.0, 0.0, TransitionType::Linear,
                -0.08, 0.24, TransitionType::Smoothstep,
                0.0, 0.0, TransitionType::Linear
            ),
            0.85
        );
        checkEndpointEquivalence(
            "variable yaw",
            makeProfiles(
                70.0,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                -0.12, 0.08, TransitionType::SineEaseOut
            ),
            6.1
        );
        checkEndpointEquivalence(
            "variable roll",
            makeProfiles(
                95.0,
                -0.22, 0.19, TransitionType::CosineEaseInOut,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            ),
            8.4
        );
        checkEndpointEquivalence(
            "asymmetric mixed profiles",
            makeProfiles(
                45.0,
                -0.16, 0.21, TransitionType::CosineEaseInOut,
                0.24, -0.09, TransitionType::SeventhOrderSmoothstep,
                -0.18, 0.13, TransitionType::QuadraticEaseIn
            ),
            4.6
        );
        checkEndpointEquivalence(
            "long high-rotation mixed profiles",
            makeProfiles(
                120.0,
                0.25, 0.31, TransitionType::SineEaseIn,
                -0.28, 0.35, TransitionType::QuinticEaseInOut,
                0.22, -0.33, TransitionType::SineEaseOut
            ),
            7.3,
            true
        );
    }

    void checkSpecializedComparison(
        const std::string_view name,
        const Profiles& profiles
    )
    {
        const double length = profiles.pitch.domainEnd
            - profiles.pitch.domainBegin;
        const CoupledIntegrationSchedule schedule =
            makeSchedule(profiles, length);
        const RiderLocalGeometryState fullEndpoint =
            integrateCoupledRateProfilesEndpoint(
                startingPosition,
                startingFrame,
                &profiles.roll,
                profiles.pitch,
                profiles.yaw,
                schedule
            );
        const RiderLocalGeometryState specializedEndpoint =
            quantum::coaster::integrateLocalRollPitchYawRateProfiles(
                startingPosition,
                startingFrame,
                profiles.roll,
                profiles.pitch,
                profiles.yaw,
                length
            ).back();

        const SpecializedMeasurement measurement{
            std::string(name),
            vectorDifference(
                fullEndpoint.position,
                specializedEndpoint.position
            ),
            vectorDifference(
                fullEndpoint.frame.tangent,
                specializedEndpoint.frame.tangent
            ),
            vectorDifference(
                fullEndpoint.frame.up,
                specializedEndpoint.frame.up
            ),
            angularDifferenceDegrees(
                fullEndpoint.frame.tangent,
                specializedEndpoint.frame.tangent
            ),
            angularDifferenceDegrees(
                fullEndpoint.frame.up,
                specializedEndpoint.frame.up
            )
        };
        specializedMeasurements.push_back(measurement);

        const TopologyTolerances topologyTolerances;
        const double topologyAngleChord = 2.0 * std::sin(
            topologyTolerances.angleTolerance
                * std::numbers::pi_v<double> / 360.0
        );
        require(
            measurement.positionDifference
                <= topologyTolerances.closureGapTolerance * 1.0e-5,
            std::string(name)
                + " full/specialized position difference is not comfortably below topology tolerance"
        );
        require(
            measurement.tangentDifference
                <= topologyAngleChord * 1.0e-5,
            std::string(name)
                + " full/specialized tangent difference is not comfortably below topology tolerance"
        );
        require(
            measurement.upDifference <= topologyAngleChord * 1.0e-5,
            std::string(name)
                + " full/specialized up difference is not comfortably below topology tolerance"
        );
    }

    void testSpecializedPathComparisons()
    {
        checkSpecializedComparison(
            "straight/all-zero",
            makeProfiles(
                40.0,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            )
        );
        checkSpecializedComparison(
            "pitch-only",
            makeProfiles(
                30.0,
                0.0, 0.0, TransitionType::Linear,
                0.16, 0.16, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            )
        );
        checkSpecializedComparison(
            "yaw-only",
            makeProfiles(
                35.0,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                -0.12, -0.12, TransitionType::Linear
            )
        );
        checkSpecializedComparison(
            "roll-only",
            makeProfiles(
                25.0,
                0.20, 0.20, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear,
                0.0, 0.0, TransitionType::Linear
            )
        );
        checkSpecializedComparison(
            "constant coupled-rate",
            makeProfiles(
                45.0,
                -0.10, -0.10, TransitionType::Linear,
                0.15, 0.15, TransitionType::Linear,
                0.11, 0.11, TransitionType::Linear
            )
        );
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {
            "full coupled and endpoint-only equivalence coverage",
            testEndpointEquivalenceCoverage
        },
        {
            "specialized path comparisons",
            testSpecializedPathComparisons
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
            std::cerr << "[FAIL] " << name << ": unknown exception\n";
        }
    }

    if (failures != 0)
    {
        std::cerr << failures << " test group(s) failed.\n";
        return 1;
    }

    std::cout << std::setprecision(17);
    for (const EndpointMeasurement& measurement : endpointMeasurements)
    {
        std::cout << "[METRIC] endpoint " << measurement.name
                  << ": position=" << measurement.positionDifference
                  << ", tangent=" << measurement.tangentDifference
                  << ", lateral=" << measurement.lateralDifference
                  << ", up=" << measurement.upDifference
                  << ", panels=" << measurement.internalPanelCount
                  << ", substeps=" << measurement.actualSubstepCount
                  << '\n';
    }
    for (const SpecializedMeasurement& measurement : specializedMeasurements)
    {
        std::cout << "[METRIC] specialized " << measurement.name
                  << ": position=" << measurement.positionDifference
                  << ", tangent=" << measurement.tangentDifference
                  << ", up=" << measurement.upDifference
                  << ", tangent-deg="
                  << measurement.tangentDifferenceDegrees
                  << ", up-deg=" << measurement.upDifferenceDegrees
                  << '\n';
    }
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
