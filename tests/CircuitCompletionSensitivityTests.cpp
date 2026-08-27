#include <quantum/coaster/detail/CircuitCompletionDetail.hpp>
#include <quantum/coaster/detail/RiderLocalGeometryDetail.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/matrix.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using quantum::coaster::detail::CircuitCompletionEndpoint;
    using quantum::coaster::detail::CircuitCompletionIntegrationSchedule;
    using quantum::coaster::detail::CircuitCompletionJacobian;
    using quantum::coaster::detail::CircuitCompletionParameterVector;
    using quantum::coaster::detail::CircuitCompletionRateBasis;
    using quantum::coaster::detail::CircuitCompletionResidual;
    using quantum::coaster::detail::CircuitCompletionSensitivityResult;
    using quantum::coaster::detail::CoupledLocalRateDerivatives;
    using quantum::geometry::applyLocalPitch;
    using quantum::geometry::applyLocalYaw;
    using quantum::geometry::applyRoll;
    using quantum::geometry::CurveFrame;

    // Cube root of double epsilon. The three tested scales exhibited a stable
    // 2.4e-7--3.3e-7 maximum-entry plateau across the complete fixture set.
    constexpr double finiteDifferenceEta =
        6.0554544523933429e-6;
    constexpr double absoluteTolerance = 5.0e-7;
    constexpr double relativeTolerance = 5.0e-7;

    struct Fixture
    {
        std::string name;
        double length;
        CircuitCompletionParameterVector parameters;
        CurveFrame startingFrame;
    };

    struct ComparisonMetrics
    {
        double maximumAbsoluteError = 0.0;
        double maximumMixedNormalizedError = 0.0;
        double worstColumnRelativeL2Error = 0.0;
        double normalizedFrobeniusError = 0.0;
        std::size_t worstColumn = 0;
        std::size_t worstRow = 0;
        double worstAnalyticEntry = 0.0;
        double worstOracleEntry = 0.0;
    };

    struct VerificationSummary
    {
        double maximumNominalPositionDifference = 0.0;
        double maximumNominalFrameDifference = 0.0;
        double maximumStructuralIdentityError = 0.0;
        double maximumAbsoluteJacobianError = 0.0;
        double maximumMixedNormalizedError = 0.0;
        double worstColumnRelativeL2Error = 0.0;
        double maximumNormalizedFrobeniusError = 0.0;
        double maximumRandomizedAbsoluteError = 0.0;
        double maximumRandomizedMixedNormalizedError = 0.0;
        std::array<double, 3> maximumErrorByStepScale{};
        std::string worstFixture;
        std::size_t worstParameter = 0;
        std::size_t worstRow = 0;
        double worstAnalyticEntry = 0.0;
        double worstOracleEntry = 0.0;
        double worstStepScale = 0.0;
        std::array<double, 9> representativeSingularValues{};
        std::size_t representativeRank = 0;
        double productionAwayRelativeDifference = 0.0;
        double productionThresholdRelativeDifference = 0.0;
        double productionDispatchRelativeDifference = 0.0;
    };

    VerificationSummary summary;

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

    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string& context
    )
    {
        if (std::abs(actual - expected) > tolerance)
        {
            fail(
                context + ": expected " + std::to_string(expected)
                + ", got " + std::to_string(actual)
            );
        }
    }

    [[nodiscard]] CurveFrame identityFrame()
    {
        return {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
    }

    [[nodiscard]] CurveFrame noncanonicalFrame(
        const double roll,
        const double pitch,
        const double yaw
    )
    {
        return applyLocalYaw(
            applyLocalPitch(
                applyRoll(identityFrame(), roll),
                pitch
            ),
            yaw
        );
    }

    [[nodiscard]] CircuitCompletionEndpoint startingEndpoint(
        const CurveFrame& frame
    )
    {
        return {
            {2.75, -4.5, 1.25},
            frame.tangent,
            frame.up
        };
    }

    void checkBasis(
        const double coordinate,
        const std::array<double, 3>& expected,
        const std::string_view name
    )
    {
        const CircuitCompletionRateBasis basis =
            quantum::coaster::detail::evaluateCircuitCompletionRateBasis(
                coordinate
            );
        const std::array<double, 3> actual{
            basis.start,
            basis.midpoint,
            basis.end
        };
        for (std::size_t index = 0; index < actual.size(); ++index)
        {
            requireNear(
                actual[index],
                expected[index],
                8.0 * std::numeric_limits<double>::epsilon(),
                std::string(name) + " basis component"
            );
        }
        requireNear(
            actual[0] + actual[1] + actual[2],
            1.0,
            8.0 * std::numeric_limits<double>::epsilon(),
            std::string(name) + " partition of unity"
        );
    }

    void testRateBasisAndAxisMapping()
    {
        const double belowMidpoint = std::nextafter(0.5, 0.0);
        const double aboveMidpoint = std::nextafter(
            0.5,
            std::numeric_limits<double>::infinity()
        );
        checkBasis(0.0, {1.0, 0.0, 0.0}, "u=0");
        checkBasis(0.25, {0.5, 0.5, 0.0}, "u=0.25");
        checkBasis(
            belowMidpoint,
            {1.0 - 2.0 * belowMidpoint, 2.0 * belowMidpoint, 0.0},
            "u=nextbelow(0.5)"
        );
        checkBasis(0.5, {0.0, 1.0, 0.0}, "u=0.5");
        checkBasis(
            aboveMidpoint,
            {0.0, 2.0 - 2.0 * aboveMidpoint,
                2.0 * aboveMidpoint - 1.0},
            "u=nextabove(0.5)"
        );
        checkBasis(0.75, {0.0, 0.5, 0.5}, "u=0.75");
        checkBasis(1.0, {0.0, 0.0, 1.0}, "u=1");
        for (std::size_t sample = 0; sample <= 1'000; ++sample)
        {
            const double coordinate =
                static_cast<double>(sample) / 1'000.0;
            const CircuitCompletionRateBasis sampledBasis =
                quantum::coaster::detail::
                    evaluateCircuitCompletionRateBasis(coordinate);
            requireNear(
                sampledBasis.start
                    + sampledBasis.midpoint
                    + sampledBasis.end,
                1.0,
                8.0 * std::numeric_limits<double>::epsilon(),
                "sampled basis partition of unity"
            );
        }

        const CircuitCompletionRateBasis below =
            quantum::coaster::detail::evaluateCircuitCompletionRateBasis(
                belowMidpoint
            );
        const CircuitCompletionRateBasis above =
            quantum::coaster::detail::evaluateCircuitCompletionRateBasis(
                aboveMidpoint
            );
        require(
            std::abs(below.start - above.start)
                    <= 4.0 * std::numeric_limits<double>::epsilon()
                && std::abs(below.midpoint - above.midpoint)
                    <= 4.0 * std::numeric_limits<double>::epsilon()
                && std::abs(below.end - above.end)
                    <= 4.0 * std::numeric_limits<double>::epsilon(),
            "Rate basis is not continuous at the midpoint"
        );

        CoupledLocalRateDerivatives derivatives{};
        quantum::coaster::detail::evaluateCircuitCompletionLocalRateDerivatives(
            2.0,
            8.0,
            derivatives
        );
        for (std::size_t knot = 0; knot < 3; ++knot)
        {
            require(
                derivatives[knot].x == 0.0
                    && derivatives[knot].z == 0.0,
                "Pitch parameter mapped outside the local pitch axis"
            );
            require(
                derivatives[3 + knot].x == 0.0
                    && derivatives[3 + knot].y == 0.0,
                "Yaw parameter mapped outside the local yaw axis"
            );
            require(
                derivatives[6 + knot].y == 0.0
                    && derivatives[6 + knot].z == 0.0,
                "Roll parameter mapped outside the local roll axis"
            );
        }
        requireNear(derivatives[0].y, 0.5, 0.0, "p0 basis mapping");
        requireNear(derivatives[1].y, 0.5, 0.0, "pm basis mapping");
        requireNear(derivatives[3].z, 0.5, 0.0, "y0 basis mapping");
        requireNear(derivatives[4].z, 0.5, 0.0, "ym basis mapping");
        requireNear(derivatives[6].x, 0.5, 0.0, "r0 basis mapping");
        requireNear(derivatives[7].x, 0.5, 0.0, "rm basis mapping");
    }

    [[nodiscard]] glm::dmat3 rotationMatrix(
        const glm::dvec3& rotationVector
    )
    {
        const double angle = glm::length(rotationVector);
        if (angle == 0.0)
        {
            return glm::dmat3{1.0};
        }
        return glm::mat3_cast(glm::angleAxis(
            angle,
            rotationVector / angle
        ));
    }

    [[nodiscard]] glm::dvec3 finiteDifferenceLeftJacobianAction(
        const glm::dvec3& rotationVector,
        const glm::dvec3& vector,
        const double step
    )
    {
        const glm::dmat3 nominal = rotationMatrix(rotationVector);
        const glm::dmat3 plus = rotationMatrix(
            rotationVector + step * vector
        );
        const glm::dmat3 minus = rotationMatrix(
            rotationVector - step * vector
        );
        const glm::dmat3 derivative = (plus - minus) / (2.0 * step);
        const glm::dmat3 spatialSkew =
            derivative * glm::transpose(nominal);
        return {
            0.5 * (spatialSkew[1][2] - spatialSkew[2][1]),
            0.5 * (spatialSkew[2][0] - spatialSkew[0][2]),
            0.5 * (spatialSkew[0][1] - spatialSkew[1][0])
        };
    }

    void testSo3LeftJacobian()
    {
        const glm::dvec3 vector{0.37, -0.51, 0.29};
        const glm::dvec3 zeroResult =
            quantum::coaster::detail::applySo3LeftJacobian(
                glm::dvec3{0.0},
                vector
            );
        require(
            zeroResult == vector,
            "SO(3) left Jacobian is not exactly identity at zero"
        );

        const glm::dvec3 machineScale{
            std::numeric_limits<double>::epsilon(),
            -2.0 * std::numeric_limits<double>::epsilon(),
            0.5 * std::numeric_limits<double>::epsilon()
        };
        require(
            glm::length(
                quantum::coaster::detail::applySo3LeftJacobian(
                    machineScale,
                    vector
                ) - vector
            ) <= 4.0 * std::numeric_limits<double>::epsilon(),
            "SO(3) left Jacobian is unstable at machine-scale rotation"
        );

        const std::array<glm::dvec3, 4> rotations{
            glm::dvec3{1.0e-9, -2.0e-9, 0.5e-9},
            glm::dvec3{8.0e-5, -3.0e-5, 2.0e-5},
            glm::dvec3{0.43, -0.27, 0.19},
            glm::dvec3{2.4, -1.7, 0.9}
        };
        for (std::size_t index = 0; index < rotations.size(); ++index)
        {
            const glm::dvec3 analytic =
                quantum::coaster::detail::applySo3LeftJacobian(
                    rotations[index],
                    vector
                );
            const glm::dvec3 oracle = finiteDifferenceLeftJacobianAction(
                rotations[index],
                vector,
                2.0e-6
            );
            const double tolerance = index < 2 ? 2.0e-10 : 2.0e-9;
            require(
                glm::length(analytic - oracle) <= tolerance,
                "SO(3) left-Jacobian perturbation check failed for case "
                    + std::to_string(index)
            );
        }
    }

    [[nodiscard]] CircuitCompletionResidual residualForParameters(
        const CircuitCompletionEndpoint& start,
        const CircuitCompletionEndpoint& desired,
        const CircuitCompletionParameterVector& parameters,
        const double length,
        const CircuitCompletionIntegrationSchedule& schedule
    )
    {
        return quantum::coaster::detail::computeCircuitCompletionResidual(
            quantum::coaster::detail::
                evaluateCircuitCompletionFullCoupledEndpoint(
                    start,
                    parameters,
                    length,
                    schedule
                ),
            desired,
            length
        );
    }

    [[nodiscard]] CircuitCompletionJacobian frozenCentralDifferenceJacobian(
        const CircuitCompletionEndpoint& start,
        const CircuitCompletionEndpoint& desired,
        const CircuitCompletionParameterVector& parameters,
        const double length,
        const CircuitCompletionIntegrationSchedule& schedule,
        const double stepScale
    )
    {
        CircuitCompletionJacobian jacobian{};
        for (std::size_t parameter = 0;
             parameter < parameters.size();
             ++parameter)
        {
            const double step = stepScale * finiteDifferenceEta
                * std::max(std::abs(parameters[parameter]), 1.0 / length);
            CircuitCompletionParameterVector plusParameters = parameters;
            CircuitCompletionParameterVector minusParameters = parameters;
            plusParameters[parameter] += step;
            minusParameters[parameter] -= step;

            const CircuitCompletionResidual plusResidual =
                residualForParameters(
                    start,
                    desired,
                    plusParameters,
                    length,
                    schedule
                );
            const CircuitCompletionResidual minusResidual =
                residualForParameters(
                    start,
                    desired,
                    minusParameters,
                    length,
                    schedule
                );
            for (std::size_t row = 0; row < plusResidual.size(); ++row)
            {
                jacobian[parameter][row] =
                    (plusResidual[row] - minusResidual[row])
                    / (2.0 * step);
            }
        }
        return jacobian;
    }

    [[nodiscard]] ComparisonMetrics compareJacobians(
        const CircuitCompletionJacobian& analytic,
        const CircuitCompletionJacobian& oracle
    )
    {
        ComparisonMetrics metrics;
        double analyticSquaredNorm = 0.0;
        double oracleSquaredNorm = 0.0;
        double differenceSquaredNorm = 0.0;

        for (std::size_t parameter = 0;
             parameter < analytic.size();
             ++parameter)
        {
            double analyticColumnSquaredNorm = 0.0;
            double oracleColumnSquaredNorm = 0.0;
            double differenceColumnSquaredNorm = 0.0;
            for (std::size_t row = 0;
                 row < analytic[parameter].size();
                 ++row)
            {
                const double analyticEntry = analytic[parameter][row];
                const double oracleEntry = oracle[parameter][row];
                const double difference =
                    std::abs(analyticEntry - oracleEntry);
                const double mixedTolerance = absoluteTolerance
                    + relativeTolerance
                        * std::max(
                            std::abs(analyticEntry),
                            std::abs(oracleEntry)
                        );
                metrics.maximumAbsoluteError = std::max(
                    metrics.maximumAbsoluteError,
                    difference
                );
                const double normalized = difference / mixedTolerance;
                if (normalized > metrics.maximumMixedNormalizedError)
                {
                    metrics.maximumMixedNormalizedError = normalized;
                    metrics.worstColumn = parameter;
                    metrics.worstRow = row;
                    metrics.worstAnalyticEntry = analyticEntry;
                    metrics.worstOracleEntry = oracleEntry;
                }

                analyticColumnSquaredNorm += analyticEntry * analyticEntry;
                oracleColumnSquaredNorm += oracleEntry * oracleEntry;
                differenceColumnSquaredNorm += difference * difference;
            }

            const double columnScale = std::max({
                std::sqrt(analyticColumnSquaredNorm),
                std::sqrt(oracleColumnSquaredNorm),
                absoluteTolerance
            });
            metrics.worstColumnRelativeL2Error = std::max(
                metrics.worstColumnRelativeL2Error,
                std::sqrt(differenceColumnSquaredNorm) / columnScale
            );
            analyticSquaredNorm += analyticColumnSquaredNorm;
            oracleSquaredNorm += oracleColumnSquaredNorm;
            differenceSquaredNorm += differenceColumnSquaredNorm;
        }

        const double frobeniusScale = std::max({
            std::sqrt(analyticSquaredNorm),
            std::sqrt(oracleSquaredNorm),
            absoluteTolerance
        });
        metrics.normalizedFrobeniusError =
            std::sqrt(differenceSquaredNorm) / frobeniusScale;
        return metrics;
    }

    [[nodiscard]] double relativeFrobeniusDifference(
        const CircuitCompletionJacobian& first,
        const CircuitCompletionJacobian& second
    )
    {
        double firstSquaredNorm = 0.0;
        double secondSquaredNorm = 0.0;
        double differenceSquaredNorm = 0.0;
        for (std::size_t column = 0; column < first.size(); ++column)
        {
            for (std::size_t row = 0; row < first[column].size(); ++row)
            {
                firstSquaredNorm += first[column][row] * first[column][row];
                secondSquaredNorm += second[column][row] * second[column][row];
                const double difference =
                    first[column][row] - second[column][row];
                differenceSquaredNorm += difference * difference;
            }
        }
        return std::sqrt(differenceSquaredNorm) / std::max({
            std::sqrt(firstSquaredNorm),
            std::sqrt(secondSquaredNorm),
            std::numeric_limits<double>::min()
        });
    }

    [[nodiscard]] CircuitCompletionJacobian productionForwardJacobian(
        const CircuitCompletionEndpoint& start,
        const CircuitCompletionEndpoint& desired,
        const CircuitCompletionParameterVector& parameters,
        const double length
    )
    {
        constexpr double productionStep = 1.0e-7;
        const CircuitCompletionResidual nominal =
            quantum::coaster::detail::computeCircuitCompletionResidual(
                quantum::coaster::detail::
                    evaluateCircuitCompletionProductionEndpoint(
                        start,
                        parameters,
                        length
                    ),
                desired,
                length
            );
        CircuitCompletionJacobian jacobian{};
        for (std::size_t parameter = 0;
             parameter < parameters.size();
             ++parameter)
        {
            CircuitCompletionParameterVector perturbed = parameters;
            perturbed[parameter] += productionStep;
            const CircuitCompletionResidual residual =
                quantum::coaster::detail::computeCircuitCompletionResidual(
                    quantum::coaster::detail::
                        evaluateCircuitCompletionProductionEndpoint(
                            start,
                            perturbed,
                            length
                        ),
                    desired,
                    length
                );
            for (std::size_t row = 0; row < nominal.size(); ++row)
            {
                jacobian[parameter][row] =
                    (residual[row] - nominal[row]) / productionStep;
            }
        }
        return jacobian;
    }

    void updateStructuralMetrics(
        const CircuitCompletionSensitivityResult& result
    )
    {
        const CurveFrame& frame = result.endpointSensitivity.endpoint.frame;
        for (std::size_t parameter = 0;
             parameter < result.residualJacobian.size();
             ++parameter)
        {
            const glm::dvec3& rotationSensitivity =
                result.endpointSensitivity.rotationSensitivity[parameter];
            const glm::dvec3 dTangent = glm::cross(
                rotationSensitivity,
                frame.tangent
            );
            const glm::dvec3 dLateral = glm::cross(
                rotationSensitivity,
                frame.lateral
            );
            const glm::dvec3 dUp = glm::cross(
                rotationSensitivity,
                frame.up
            );
            const glm::dvec3 handednessDerivative =
                glm::cross(dTangent, frame.lateral)
                + glm::cross(frame.tangent, dLateral)
                - dUp;
            summary.maximumStructuralIdentityError = std::max({
                summary.maximumStructuralIdentityError,
                std::abs(glm::dot(frame.tangent, dTangent)),
                std::abs(glm::dot(frame.up, dUp)),
                std::abs(
                    glm::dot(frame.tangent, dUp)
                    + glm::dot(frame.up, dTangent)
                ),
                std::abs(glm::dot(frame.lateral, dLateral)),
                glm::length(handednessDerivative)
            });
        }
    }

    [[nodiscard]] std::array<double, 9> singularValues(
        const CircuitCompletionJacobian& jacobian
    )
    {
        std::array<std::array<double, 9>, 9> gram{};
        for (std::size_t row = 0; row < 9; ++row)
        {
            for (std::size_t column = 0; column < 9; ++column)
            {
                for (std::size_t residual = 0; residual < 9; ++residual)
                {
                    gram[row][column] += jacobian[row][residual]
                        * jacobian[column][residual];
                }
            }
        }

        for (std::size_t iteration = 0; iteration < 256; ++iteration)
        {
            std::size_t first = 0;
            std::size_t second = 1;
            double maximumOffDiagonal = 0.0;
            for (std::size_t row = 0; row < 9; ++row)
            {
                for (std::size_t column = row + 1; column < 9; ++column)
                {
                    if (std::abs(gram[row][column]) > maximumOffDiagonal)
                    {
                        maximumOffDiagonal =
                            std::abs(gram[row][column]);
                        first = row;
                        second = column;
                    }
                }
            }
            if (maximumOffDiagonal <= 1.0e-15)
            {
                break;
            }

            const double angle = 0.5 * std::atan2(
                2.0 * gram[first][second],
                gram[second][second] - gram[first][first]
            );
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            const double firstDiagonal = gram[first][first];
            const double secondDiagonal = gram[second][second];
            const double cross = gram[first][second];
            gram[first][first] = cosine * cosine * firstDiagonal
                - 2.0 * sine * cosine * cross
                + sine * sine * secondDiagonal;
            gram[second][second] = sine * sine * firstDiagonal
                + 2.0 * sine * cosine * cross
                + cosine * cosine * secondDiagonal;
            gram[first][second] = 0.0;
            gram[second][first] = 0.0;

            for (std::size_t index = 0; index < 9; ++index)
            {
                if (index == first || index == second)
                {
                    continue;
                }
                const double firstValue = gram[index][first];
                const double secondValue = gram[index][second];
                gram[index][first] =
                    cosine * firstValue - sine * secondValue;
                gram[first][index] = gram[index][first];
                gram[index][second] =
                    sine * firstValue + cosine * secondValue;
                gram[second][index] = gram[index][second];
            }
        }

        std::array<double, 9> values{};
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            values[index] = std::sqrt(std::max(0.0, gram[index][index]));
        }
        std::sort(values.begin(), values.end(), std::greater<>{});
        return values;
    }

    void verifyFixture(const Fixture& fixture)
    {
        const CircuitCompletionEndpoint start =
            startingEndpoint(fixture.startingFrame);
        const CircuitCompletionIntegrationSchedule schedule =
            quantum::coaster::detail::
                makeCircuitCompletionIntegrationSchedule(
                    fixture.parameters,
                    fixture.length
                );
        const CircuitCompletionSensitivityResult sensitivity =
            quantum::coaster::detail::
                evaluateCircuitCompletionEndpointSensitivities(
                    start,
                    fixture.parameters,
                    fixture.length,
                    schedule
                );
        const CircuitCompletionEndpoint fullEndpoint =
            quantum::coaster::detail::
                evaluateCircuitCompletionFullCoupledEndpoint(
                    start,
                    fixture.parameters,
                    fixture.length,
                    schedule
                );
        const auto& augmentedEndpoint =
            sensitivity.endpointSensitivity.endpoint;
        const double positionDifference = glm::length(
            augmentedEndpoint.position - fullEndpoint.position
        );
        const double frameDifference = std::max({
            glm::length(
                augmentedEndpoint.frame.tangent - fullEndpoint.tangent
            ),
            glm::length(
                augmentedEndpoint.frame.lateral
                - glm::normalize(glm::cross(
                    fullEndpoint.up,
                    fullEndpoint.tangent
                ))
            ),
            glm::length(augmentedEndpoint.frame.up - fullEndpoint.up)
        });
        summary.maximumNominalPositionDifference = std::max(
            summary.maximumNominalPositionDifference,
            positionDifference
        );
        summary.maximumNominalFrameDifference = std::max(
            summary.maximumNominalFrameDifference,
            frameDifference
        );
        require(
            augmentedEndpoint.distance == fixture.length,
            fixture.name + " augmented endpoint distance is not exact"
        );
        const double positionTolerance = 64.0
            * std::numeric_limits<double>::epsilon()
            * std::max({
                1.0,
                fixture.length,
                glm::length(start.position)
            });
        require(
            positionDifference <= positionTolerance
                && frameDifference
                    <= 64.0 * std::numeric_limits<double>::epsilon(),
            fixture.name + " augmented nominal endpoint differs"
        );

        updateStructuralMetrics(sensitivity);
        const std::array<double, 3> stepScales{0.5, 1.0, 2.0};
        for (std::size_t scaleIndex = 0;
             scaleIndex < stepScales.size();
             ++scaleIndex)
        {
            const CircuitCompletionJacobian oracle =
                frozenCentralDifferenceJacobian(
                    start,
                    start,
                    fixture.parameters,
                    fixture.length,
                    schedule,
                    stepScales[scaleIndex]
                );
            const ComparisonMetrics metrics = compareJacobians(
                sensitivity.residualJacobian,
                oracle
            );
            summary.maximumErrorByStepScale[scaleIndex] = std::max(
                summary.maximumErrorByStepScale[scaleIndex],
                metrics.maximumAbsoluteError
            );
            summary.maximumAbsoluteJacobianError = std::max(
                summary.maximumAbsoluteJacobianError,
                metrics.maximumAbsoluteError
            );
            summary.worstColumnRelativeL2Error = std::max(
                summary.worstColumnRelativeL2Error,
                metrics.worstColumnRelativeL2Error
            );
            summary.maximumNormalizedFrobeniusError = std::max(
                summary.maximumNormalizedFrobeniusError,
                metrics.normalizedFrobeniusError
            );
            if (fixture.name.starts_with("randomized-"))
            {
                summary.maximumRandomizedAbsoluteError = std::max(
                    summary.maximumRandomizedAbsoluteError,
                    metrics.maximumAbsoluteError
                );
                summary.maximumRandomizedMixedNormalizedError = std::max(
                    summary.maximumRandomizedMixedNormalizedError,
                    metrics.maximumMixedNormalizedError
                );
            }
            if (metrics.maximumMixedNormalizedError
                > summary.maximumMixedNormalizedError)
            {
                summary.maximumMixedNormalizedError =
                    metrics.maximumMixedNormalizedError;
                summary.worstFixture = fixture.name;
                summary.worstParameter = metrics.worstColumn;
                summary.worstRow = metrics.worstRow;
                summary.worstAnalyticEntry = metrics.worstAnalyticEntry;
                summary.worstOracleEntry = metrics.worstOracleEntry;
                summary.worstStepScale = stepScales[scaleIndex];
            }
        }

    }

    [[nodiscard]] std::vector<Fixture> namedFixtures()
    {
        const CurveFrame canonical = identityFrame();
        const CurveFrame rotated = noncanonicalFrame(0.37, -0.29, 0.41);
        constexpr double thresholdLength = 40.0;
        constexpr double halfLength = thresholdLength * 0.5;
        const double maximumPanelAngle = std::numbers::pi_v<double> / 128.0;
        const double baselineThreshold =
            1'024.0 * maximumPanelAngle / halfLength;
        const double substepThreshold =
            1'040.0 * maximumPanelAngle / halfLength;

        return {
            {"all-zero short", 0.25, {}, canonical},
            {"constant pitch", 12.0,
                {0.18, 0.18, 0.18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                rotated},
            {"constant yaw", 15.0,
                {0.0, 0.0, 0.0, -0.14, -0.14, -0.14,
                    0.0, 0.0, 0.0}, rotated},
            {"constant roll", 18.0,
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.22, 0.22, 0.22}, rotated},
            {"constant mixed", 25.0,
                {0.17, 0.17, 0.17, -0.13, -0.13, -0.13,
                    0.11, 0.11, 0.11}, rotated},
            {"variable pitch only", 10.0,
                {-0.08, 0.24, 0.13, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0}, rotated},
            {"variable yaw only", 22.0,
                {0.0, 0.0, 0.0, -0.12, 0.08, 0.19,
                    0.0, 0.0, 0.0}, canonical},
            {"variable roll only", 30.0,
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    -0.22, 0.19, -0.07}, rotated},
            {"mixed positive and negative", 35.0,
                {-0.16, 0.21, -0.09, 0.24, -0.18, 0.13,
                    -0.11, 0.17, -0.20}, rotated},
            {"start dominated", 14.0,
                {0.31, 0.0, 0.0, -0.27, 0.0, 0.0,
                    0.19, 0.0, 0.0}, canonical},
            {"midpoint dominated", 28.0,
                {0.0, -0.29, 0.0, 0.0, 0.23, 0.0,
                    0.0, 0.17, 0.0}, rotated},
            {"end dominated", 16.0,
                {0.0, 0.0, -0.26, 0.0, 0.0, 0.21,
                    0.0, 0.0, -0.18}, rotated},
            {"asymmetric knots", 45.0,
                {-0.05, 0.27, 0.11, 0.19, -0.07, -0.23,
                    0.14, -0.20, 0.09}, rotated},
            {"sign-changing profiles", 32.0,
                {-0.24, 0.13, 0.20, 0.18, -0.26, 0.11,
                    -0.15, 0.22, -0.19}, canonical},
            {"near-straight moderate", 20.0,
                {1.0e-10, -2.0e-10, 1.5e-10,
                    -0.5e-10, 2.5e-10, -1.0e-10,
                    1.0e-10, 0.0, -1.0e-10}, rotated},
            {"long substantial rotation", 120.0,
                {0.25, -0.28, 0.31, -0.22, 0.27, -0.33,
                    0.18, -0.24, 0.29}, rotated},
            {"safely between panel thresholds", thresholdLength,
                {0.93, 0.91, 0.92, 0.08, -0.06, 0.07,
                    -0.04, 0.05, -0.03}, canonical},
            {"immediately below baseline threshold", thresholdLength,
                {std::nextafter(baselineThreshold, 0.0),
                    std::nextafter(baselineThreshold, 0.0),
                    std::nextafter(baselineThreshold, 0.0),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, rotated},
            {"immediately above baseline threshold", thresholdLength,
                {std::nextafter(
                    baselineThreshold,
                    std::numeric_limits<double>::infinity()),
                    std::nextafter(
                        baselineThreshold,
                        std::numeric_limits<double>::infinity()),
                    std::nextafter(
                        baselineThreshold,
                        std::numeric_limits<double>::infinity()),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, rotated},
            {"above-baseline strongly curved", thresholdLength,
                {1.48, 1.39, 1.44, -0.31, 0.28, -0.26,
                    0.22, -0.19, 0.24}, rotated},
            {"nearby substep layout below", thresholdLength,
                {substepThreshold - 1.0e-12,
                    substepThreshold - 1.0e-12,
                    substepThreshold - 1.0e-12,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, canonical},
            {"nearby substep layout above", thresholdLength,
                {substepThreshold + 1.0e-12,
                    substepThreshold + 1.0e-12,
                    substepThreshold + 1.0e-12,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, canonical}
        };
    }

    void testScheduleThresholdCoverage(const std::vector<Fixture>& fixtures)
    {
        const auto findFixture = [&](const std::string_view name)
            -> const Fixture&
        {
            const auto found = std::find_if(
                fixtures.begin(),
                fixtures.end(),
                [name](const Fixture& fixture)
                {
                    return fixture.name == name;
                }
            );
            require(found != fixtures.end(), "Missing schedule fixture");
            return *found;
        };
        const auto scheduleFor = [&](const std::string_view name)
        {
            const Fixture& fixture = findFixture(name);
            return quantum::coaster::detail::
                makeCircuitCompletionIntegrationSchedule(
                    fixture.parameters,
                    fixture.length
                );
        };

        const CircuitCompletionIntegrationSchedule belowBaseline =
            scheduleFor("immediately below baseline threshold");
        const CircuitCompletionIntegrationSchedule aboveBaseline =
            scheduleFor("immediately above baseline threshold");
        require(
            belowBaseline.spans[0].internalPanelCount == 1'024
                && belowBaseline.spans[1].internalPanelCount == 1'024,
            "Below-threshold fixture did not select the baseline regime"
        );
        require(
            aboveBaseline.spans[0].internalPanelCount > 1'024
                && aboveBaseline.spans[1].internalPanelCount > 1'024,
            "Above-threshold fixture did not leave the baseline regime"
        );

        const CircuitCompletionIntegrationSchedule belowSubstep =
            scheduleFor("nearby substep layout below");
        const CircuitCompletionIntegrationSchedule aboveSubstep =
            scheduleFor("nearby substep layout above");
        require(
            belowSubstep.spans[0].substepLengths.size()
                != aboveSubstep.spans[0].substepLengths.size(),
            "Nearby nominal geometries did not exercise distinct substep layouts"
        );
    }

    void testSensitivityVerification()
    {
        std::vector<Fixture> fixtures = namedFixtures();
        testScheduleThresholdCoverage(fixtures);

        std::mt19937_64 generator{0x514E54554DULL};
        std::uniform_real_distribution<double> lengthDistribution(1.0, 80.0);
        std::uniform_real_distribution<double> rateDistribution(-0.55, 0.55);
        std::uniform_real_distribution<double> angleDistribution(-0.7, 0.7);
        for (std::size_t index = 0; index < 10; ++index)
        {
            CircuitCompletionParameterVector parameters{};
            for (double& parameter : parameters)
            {
                parameter = rateDistribution(generator);
            }
            fixtures.push_back({
                "randomized-" + std::to_string(index),
                lengthDistribution(generator),
                parameters,
                noncanonicalFrame(
                    angleDistribution(generator),
                    angleDistribution(generator),
                    angleDistribution(generator)
                )
            });
        }

        for (const Fixture& fixture : fixtures)
        {
            verifyFixture(fixture);
        }

        if (summary.maximumMixedNormalizedError > 1.0)
        {
            std::cerr << std::setprecision(17)
                      << "[DIAGNOSTIC] worst analytic="
                      << summary.worstAnalyticEntry
                      << ", oracle=" << summary.worstOracleEntry
                      << ", absolute="
                      << std::abs(
                          summary.worstAnalyticEntry
                          - summary.worstOracleEntry
                      )
                      << ", global max absolute="
                      << summary.maximumAbsoluteJacobianError
                      << ", scale maxima="
                      << summary.maximumErrorByStepScale[0] << ','
                      << summary.maximumErrorByStepScale[1] << ','
                      << summary.maximumErrorByStepScale[2] << '\n';
        }
        require(
            summary.maximumMixedNormalizedError <= 1.0,
            "Frozen central finite difference mismatch: worst fixture "
                + summary.worstFixture + ", parameter "
                + std::to_string(summary.worstParameter)
                + ", step scale " + std::to_string(summary.worstStepScale)
                + ", row " + std::to_string(summary.worstRow)
                + ", analytic "
                + std::to_string(summary.worstAnalyticEntry)
                + ", oracle "
                + std::to_string(summary.worstOracleEntry)
                + ", normalized error "
                + std::to_string(summary.maximumMixedNormalizedError)
                + ", max absolute "
                + std::to_string(summary.maximumAbsoluteJacobianError)
                + ", worst column relative L2 "
                + std::to_string(summary.worstColumnRelativeL2Error)
                + ", FD scale maxima "
                + std::to_string(summary.maximumErrorByStepScale[0]) + ","
                + std::to_string(summary.maximumErrorByStepScale[1]) + ","
                + std::to_string(summary.maximumErrorByStepScale[2])
        );
        require(
            summary.maximumStructuralIdentityError <= 2.0e-12,
            "Endpoint sensitivity violates a frame structural identity"
        );

        const Fixture& representative = fixtures[12];
        const CircuitCompletionEndpoint representativeStart =
            startingEndpoint(representative.startingFrame);
        const CircuitCompletionIntegrationSchedule representativeSchedule =
            quantum::coaster::detail::
                makeCircuitCompletionIntegrationSchedule(
                    representative.parameters,
                    representative.length
                );
        const CircuitCompletionSensitivityResult representativeResult =
            quantum::coaster::detail::
                evaluateCircuitCompletionEndpointSensitivities(
                    representativeStart,
                    representative.parameters,
                    representative.length,
                    representativeSchedule
                );
        summary.representativeSingularValues = singularValues(
            representativeResult.residualJacobian
        );
        const double rankThreshold =
            summary.representativeSingularValues[0] * 1.0e-7;
        summary.representativeRank = static_cast<std::size_t>(std::count_if(
            summary.representativeSingularValues.begin(),
            summary.representativeSingularValues.end(),
            [rankThreshold](const double value)
            {
                return value > rankThreshold;
            }
        ));
        require(
            summary.representativeRank <= 6,
            "Sensitivity Jacobian numerical rank exceeds six physical DOFs"
        );

        const auto productionComparison = [&](const Fixture& fixture)
        {
            const CircuitCompletionEndpoint start =
                startingEndpoint(fixture.startingFrame);
            const CircuitCompletionIntegrationSchedule schedule =
                quantum::coaster::detail::
                    makeCircuitCompletionIntegrationSchedule(
                        fixture.parameters,
                        fixture.length
                    );
            const CircuitCompletionJacobian analytic =
                quantum::coaster::detail::
                    evaluateCircuitCompletionEndpointSensitivities(
                        start,
                        fixture.parameters,
                        fixture.length,
                        schedule
                    ).residualJacobian;
            return relativeFrobeniusDifference(
                analytic,
                productionForwardJacobian(
                    start,
                    start,
                    fixture.parameters,
                    fixture.length
                )
            );
        };
        summary.productionAwayRelativeDifference =
            productionComparison(fixtures[12]);
        summary.productionThresholdRelativeDifference =
            productionComparison(fixtures[17]);
        summary.productionDispatchRelativeDifference =
            productionComparison(fixtures[0]);
    }
}

int main()
{
    using Test = std::pair<std::string_view, std::function<void()>>;
    const std::vector<Test> tests{
        {"rate basis and parameter-axis mapping",
            testRateBasisAndAxisMapping},
        {"SO(3) left Jacobian action", testSo3LeftJacobian},
        {"frozen-schedule endpoint sensitivities",
            testSensitivityVerification}
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
    std::cout << "[METRIC] nominal position max="
              << summary.maximumNominalPositionDifference
              << ", frame max=" << summary.maximumNominalFrameDifference
              << '\n';
    std::cout << "[METRIC] structural identity max="
              << summary.maximumStructuralIdentityError << '\n';
    std::cout << "[METRIC] frozen central FD max-abs="
              << summary.maximumAbsoluteJacobianError
              << ", max-mixed-normalized="
              << summary.maximumMixedNormalizedError
              << ", worst-column-relative-L2="
              << summary.worstColumnRelativeL2Error
              << ", normalized-Frobenius="
              << summary.maximumNormalizedFrobeniusError
              << ", fixture=" << summary.worstFixture
              << ", parameter=" << summary.worstParameter
              << ", step-scale=" << summary.worstStepScale << '\n';
    std::cout << "[METRIC] FD max-abs by scale h/2,h,2h="
              << summary.maximumErrorByStepScale[0] << ','
              << summary.maximumErrorByStepScale[1] << ','
              << summary.maximumErrorByStepScale[2] << '\n';
    std::cout << "[METRIC] randomized FD max-abs="
              << summary.maximumRandomizedAbsoluteError
              << ", max-mixed-normalized="
              << summary.maximumRandomizedMixedNormalizedError << '\n';
    std::cout << "[METRIC] representative singular values=";
    for (const double value : summary.representativeSingularValues)
    {
        std::cout << ' ' << value;
    }
    std::cout << ", rank=" << summary.representativeRank << '\n';
    std::cout << "[METRIC] production FD relative Frobenius away="
              << summary.productionAwayRelativeDifference
              << ", threshold="
              << summary.productionThresholdRelativeDifference
              << ", dispatch="
              << summary.productionDispatchRelativeDifference << '\n';
    std::cout << tests.size() << " test groups passed.\n";
    return 0;
}
