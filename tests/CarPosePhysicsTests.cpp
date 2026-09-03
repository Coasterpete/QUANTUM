#include <quantum/geometry/RotationMinimizingFrames.hpp>
#include <quantum/physics/CarPose.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::TopologyKind;
    using quantum::coaster::TrackKinematicState;
    using quantum::geometry::CurveFrame;
    using quantum::physics::BogieDefinition;
    using quantum::physics::BogiePose;
    using quantum::physics::CarDefinition;
    using quantum::physics::CarLoadout;
    using quantum::physics::CarPose;
    using quantum::physics::CompiledPhysicsTrack;
    using quantum::physics::TrackLocation;
    using quantum::physics::TravelDirection;
    using quantum::physics::loadedCarCenterOfGravityMeters;
    using quantum::physics::primaryTrackPathId;
    using quantum::physics::solveCarPose;
    using quantum::physics::totalCarMassKilograms;
    using quantum::physics::validateCarDefinition;
    using quantum::physics::validateCarLoadout;

    inline constexpr double positionTolerance = 2.0e-10;
    inline constexpr double directionTolerance = 2.0e-12;

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
        if (!std::isfinite(actual)
            || std::abs(actual - expected) > tolerance)
        {
            throw TestFailure(
                std::string(message)
                + ": expected " + std::to_string(expected)
                + ", got " + std::to_string(actual));
        }
    }

    void requireNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const std::string_view message)
    {
        if (!std::isfinite(actual.x)
            || !std::isfinite(actual.y)
            || !std::isfinite(actual.z)
            || glm::length(actual - expected) > tolerance)
        {
            throw TestFailure(std::string(message));
        }
    }

    template<typename Function>
    void requireInvalidArgument(
        Function&& function,
        const std::string_view message)
    {
        bool threw = false;
        try
        {
            function();
        }
        catch (const std::invalid_argument&)
        {
            threw = true;
        }
        require(threw, message);
    }

    [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
    {
        return std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    [[nodiscard]] bool finite(const glm::dquat& value) noexcept
    {
        return std::isfinite(value.w)
            && std::isfinite(value.x)
            && std::isfinite(value.y)
            && std::isfinite(value.z);
    }

    void requireOrthonormal(
        const CurveFrame& frame,
        const std::string_view message)
    {
        require(finite(frame.tangent)
            && finite(frame.lateral)
            && finite(frame.up), message);
        requireNear(glm::length(frame.tangent), 1.0,
            directionTolerance, message);
        requireNear(glm::length(frame.lateral), 1.0,
            directionTolerance, message);
        requireNear(glm::length(frame.up), 1.0,
            directionTolerance, message);
        requireNear(glm::dot(frame.tangent, frame.lateral), 0.0,
            directionTolerance, message);
        requireNear(glm::dot(frame.tangent, frame.up), 0.0,
            directionTolerance, message);
        requireNear(glm::dot(frame.lateral, frame.up), 0.0,
            directionTolerance, message);
        requireNear(glm::cross(frame.tangent, frame.lateral), frame.up,
            directionTolerance, message);
    }

    [[nodiscard]] TrackLocation locationAt(
        const double stationMeters,
        const TravelDirection direction = TravelDirection::IncreasingStation)
    {
        return {primaryTrackPathId, stationMeters, direction};
    }

    [[nodiscard]] CurveFrame straightFrame(
        const glm::dvec3& tangent,
        const double bankRadians = 0.0)
    {
        const glm::dvec3 unitTangent = glm::normalize(tangent);
        const glm::dvec3 lateral{0.0, 1.0, 0.0};
        const CurveFrame base{
            unitTangent,
            lateral,
            glm::normalize(glm::cross(unitTangent, lateral))
        };
        return quantum::geometry::applyRoll(base, bankRadians);
    }

    [[nodiscard]] std::vector<TrackKinematicState> straightSamples(
        const double lengthCoordinateUnits,
        const glm::dvec3& tangent = glm::dvec3{1.0, 0.0, 0.0},
        const glm::dvec3& startPosition = glm::dvec3{0.0},
        const double bankRadians = 0.0)
    {
        const CurveFrame frame = straightFrame(tangent, bankRadians);
        return {
            {0.0, startPosition, frame, glm::dvec3{0.0}},
            {
                lengthCoordinateUnits,
                startPosition + lengthCoordinateUnits * frame.tangent,
                frame,
                glm::dvec3{0.0}
            }
        };
    }

    [[nodiscard]] CompiledPhysicsTrack straightTrack(
        const double lengthCoordinateUnits = 40.0,
        const glm::dvec3& tangent = glm::dvec3{1.0, 0.0, 0.0},
        const double metersPerCoordinateUnit = 1.0,
        const glm::dvec3& startPosition = glm::dvec3{0.0},
        const double bankRadians = 0.0)
    {
        const auto samples = straightSamples(
            lengthCoordinateUnits, tangent, startPosition, bankRadians);
        return {samples, metersPerCoordinateUnit, TopologyKind::OpenLinear};
    }

    [[nodiscard]] std::vector<TrackKinematicState> horizontalArcSamples(
        const double radiusMeters,
        const double sweepRadians,
        const int segmentCount,
        const double bankRadians = 0.0)
    {
        std::vector<TrackKinematicState> samples;
        samples.reserve(static_cast<std::size_t>(segmentCount + 1));
        for (int index = 0; index <= segmentCount; ++index)
        {
            const double angle = sweepRadians
                * static_cast<double>(index)
                / static_cast<double>(segmentCount);
            CurveFrame frame{
                {std::cos(angle), std::sin(angle), 0.0},
                {-std::sin(angle), std::cos(angle), 0.0},
                {0.0, 0.0, 1.0}
            };
            frame = quantum::geometry::applyRoll(frame, bankRadians);
            samples.push_back({
                radiusMeters * angle,
                {
                    radiusMeters * std::sin(angle),
                    radiusMeters * (1.0 - std::cos(angle)),
                    0.0
                },
                frame,
                {
                    -std::sin(angle) / radiusMeters,
                    std::cos(angle) / radiusMeters,
                    0.0
                }
            });
        }
        return samples;
    }

    [[nodiscard]] CompiledPhysicsTrack horizontalArcTrack(
        const double radiusMeters = 10.0,
        const double sweepRadians = 0.5 * std::numbers::pi,
        const TopologyKind topology = TopologyKind::OpenLinear,
        const double bankRadians = 0.0)
    {
        const auto samples = horizontalArcSamples(
            radiusMeters, sweepRadians, 256, bankRadians);
        return {samples, 1.0, topology};
    }

    [[nodiscard]] std::vector<TrackKinematicState> verticalArcSamples(
        const double radiusMeters,
        const bool crest,
        const int segmentCount)
    {
        constexpr double startAngle = -0.6;
        constexpr double endAngle = 0.6;
        std::vector<TrackKinematicState> samples;
        samples.reserve(static_cast<std::size_t>(segmentCount + 1));
        for (int index = 0; index <= segmentCount; ++index)
        {
            const double angle = startAngle
                + (endAngle - startAngle)
                    * static_cast<double>(index)
                    / static_cast<double>(segmentCount);
            const double verticalSign = crest ? 1.0 : -1.0;
            const glm::dvec3 position{
                radiusMeters * std::sin(angle),
                0.0,
                verticalSign * radiusMeters * std::cos(angle)
            };
            const glm::dvec3 tangent{
                std::cos(angle),
                0.0,
                -verticalSign * std::sin(angle)
            };
            const CurveFrame frame{
                tangent,
                {0.0, 1.0, 0.0},
                glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0})
            };
            samples.push_back({
                radiusMeters * (angle - startAngle),
                position,
                frame,
                {
                    -std::sin(angle) / radiusMeters,
                    0.0,
                    -verticalSign * std::cos(angle) / radiusMeters
                }
            });
        }
        return samples;
    }

    [[nodiscard]] CompiledPhysicsTrack verticalArcTrack(
        const bool crest)
    {
        const auto samples = verticalArcSamples(12.0, crest, 256);
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] std::vector<TrackKinematicState> compoundSamples()
    {
        constexpr double radius = 14.0;
        constexpr double risePerRadian = 4.0;
        constexpr double sweep = 1.2;
        constexpr int segments = 256;
        const double distancePerRadian = std::hypot(radius, risePerRadian);
        std::vector<TrackKinematicState> samples;
        samples.reserve(segments + 1);
        for (int index = 0; index <= segments; ++index)
        {
            const double angle = sweep
                * static_cast<double>(index) / segments;
            const glm::dvec3 tangent = glm::normalize(glm::dvec3{
                radius * std::cos(angle),
                radius * std::sin(angle),
                risePerRadian
            });
            const glm::dvec3 lateral{
                -std::sin(angle), std::cos(angle), 0.0};
            CurveFrame frame{
                tangent,
                lateral,
                glm::cross(tangent, lateral)
            };
            frame = quantum::geometry::applyRoll(
                frame, 0.15 + 0.25 * angle);
            samples.push_back({
                distancePerRadian * angle,
                {
                    radius * std::sin(angle),
                    radius * (1.0 - std::cos(angle)),
                    risePerRadian * angle
                },
                frame,
                {
                    -radius * std::sin(angle)
                        / (distancePerRadian * distancePerRadian),
                    radius * std::cos(angle)
                        / (distancePerRadian * distancePerRadian),
                    0.0
                }
            });
        }
        return samples;
    }

    [[nodiscard]] CompiledPhysicsTrack compoundTrack()
    {
        const auto samples = compoundSamples();
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CompiledPhysicsTrack opposedUpTrack()
    {
        const CurveFrame base = straightFrame({1.0, 0.0, 0.0});
        const std::vector<TrackKinematicState> samples{
            {0.0, {0.0, 0.0, 0.0}, base, {0.0, 0.0, 0.0}},
            {
                1.0,
                {1.0, 0.0, 0.0},
                quantum::geometry::applyRoll(base, 0.5 * std::numbers::pi),
                {0.0, 0.0, 0.0}
            },
            {
                2.0,
                {2.0, 0.0, 0.0},
                quantum::geometry::applyRoll(base, std::numbers::pi),
                {0.0, 0.0, 0.0}
            }
        };
        return {samples, 1.0, TopologyKind::OpenLinear};
    }

    [[nodiscard]] CarDefinition passengerCar()
    {
        CarDefinition definition;
        definition.dryMassKilograms = 800.0;
        definition.dryCenterOfGravityMeters = {0.1, 0.0, 0.55};
        definition.bodyDimensionsMeters = {4.2, 1.3, 1.4};
        definition.frontHitchPositionMeters = {2.25, 0.0, 0.25};
        definition.rearHitchPositionMeters = {-2.15, 0.0, 0.25};
        // Authored order is deliberately rear then front. Solver role
        // selection must use validated longitudinal ordering, not index.
        definition.bogies = {
            BogieDefinition{{-1.25, 0.0, 0.0}},
            BogieDefinition{{1.25, 0.0, 0.0}}
        };
        return definition;
    }

    void validCarDefinitionAndLoadout()
    {
        const CarDefinition definition = passengerCar();
        const CarLoadout loadout{200.0, {0.3, 0.0, 0.75}};
        validateCarDefinition(definition);
        validateCarLoadout(loadout);
        requireNear(totalCarMassKilograms(definition, loadout),
            1000.0, 0.0, "loaded mass");
        requireNear(loadedCarCenterOfGravityMeters(definition, loadout),
            {0.14, 0.0, 0.59}, positionTolerance, "loaded COG");
    }

    void zeroCarCompatibleDefinition()
    {
        CarDefinition leadVehicle;
        leadVehicle.dryMassKilograms = 430.0;
        leadVehicle.dryCenterOfGravityMeters = {-0.2, 0.0, 0.3};
        leadVehicle.bodyDimensionsMeters = {2.7, 1.1, 1.0};
        leadVehicle.frontHitchPositionMeters = {1.5, 0.0, 0.2};
        leadVehicle.rearHitchPositionMeters = {-1.4, 0.0, 0.2};
        leadVehicle.bogies = {{{0.8, 0.0, 0.0}}, {{-0.9, 0.0, 0.0}}};

        validateCarDefinition(leadVehicle);
        const CarPose pose = solveCarPose(
            straightTrack(), leadVehicle, locationAt(10.0), CarLoadout{});
        requireNear(pose.totalMassKilograms(), 430.0, 0.0,
            "zero-load lead vehicle mass");
        requireNear(pose.localCenterOfGravityMeters(),
            leadVehicle.dryCenterOfGravityMeters, 0.0,
            "zero-load lead vehicle COG");
    }

    void invalidMassAndDimensionsAreRejected()
    {
        for (const double invalidMass : {
            0.0,
            -1.0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()})
        {
            CarDefinition invalid = passengerCar();
            invalid.dryMassKilograms = invalidMass;
            requireInvalidArgument(
                [&] { validateCarDefinition(invalid); },
                "invalid dry mass must be rejected");
        }

        CarDefinition invalidDimensions = passengerCar();
        invalidDimensions.bodyDimensionsMeters.y = 0.0;
        requireInvalidArgument(
            [&] { validateCarDefinition(invalidDimensions); },
            "non-positive dimensions must be rejected");

        invalidDimensions = passengerCar();
        invalidDimensions.bodyDimensionsMeters.z =
            std::numeric_limits<double>::infinity();
        requireInvalidArgument(
            [&] { validateCarDefinition(invalidDimensions); },
            "non-finite dimensions must be rejected");

        CarDefinition invalidPoint = passengerCar();
        invalidPoint.dryCenterOfGravityMeters.x =
            std::numeric_limits<double>::quiet_NaN();
        requireInvalidArgument(
            [&] { validateCarDefinition(invalidPoint); },
            "non-finite COG must be rejected");

        invalidPoint = passengerCar();
        invalidPoint.frontHitchPositionMeters.y =
            std::numeric_limits<double>::infinity();
        requireInvalidArgument(
            [&] { validateCarDefinition(invalidPoint); },
            "non-finite hitch position must be rejected");

        requireInvalidArgument(
            []
            {
                validateCarLoadout(CarLoadout{
                    -1.0, glm::dvec3{0.0}});
            },
            "negative load mass must be rejected");
        requireInvalidArgument(
            []
            {
                validateCarLoadout(CarLoadout{
                    std::numeric_limits<double>::infinity(),
                    glm::dvec3{0.0}});
            },
            "non-finite load mass must be rejected");
    }

    void invalidBogieGeometryIsRejected()
    {
        const CompiledPhysicsTrack track = straightTrack();
        CarDefinition invalid = passengerCar();
        invalid.bogies.resize(1);
        requireInvalidArgument(
            [&] { (void)solveCarPose(track, invalid, locationAt(10.0)); },
            "one bogie must be unsupported by Phase 2 solver");

        invalid = passengerCar();
        invalid.bogies.push_back({{0.0, 0.0, 0.0}});
        requireInvalidArgument(
            [&] { (void)solveCarPose(track, invalid, locationAt(10.0)); },
            "three bogies must be unsupported by Phase 2 solver");

        invalid = passengerCar();
        invalid.bogies[1].referencePositionMeters.x =
            invalid.bogies[0].referencePositionMeters.x;
        requireInvalidArgument(
            [&] { (void)solveCarPose(track, invalid, locationAt(10.0)); },
            "coincident longitudinal bogie geometry must be rejected");

        invalid = passengerCar();
        invalid.bogies[0].referencePositionMeters.z =
            std::numeric_limits<double>::quiet_NaN();
        requireInvalidArgument(
            [&] { validateCarDefinition(invalid); },
            "non-finite bogie position must be rejected");
    }

    void straightTrackAlignsBogiesAndBody()
    {
        const CarPose pose = solveCarPose(
            straightTrack(), passengerCar(), locationAt(10.0));
        requireNear(pose.frontBogie().location().stationMeters,
            11.25, positionTolerance, "front bogie station");
        requireNear(pose.rearBogie().location().stationMeters,
            8.75, positionTolerance, "rear bogie station");
        requireNear(pose.frontBogie().worldPositionMeters(),
            {11.25, 0.0, 0.0}, positionTolerance, "front bogie position");
        requireNear(pose.rearBogie().worldPositionMeters(),
            {8.75, 0.0, 0.0}, positionTolerance, "rear bogie position");
        requireNear(pose.bodyWorldPositionMeters(),
            {10.0, 0.0, 0.0}, positionTolerance, "body position");
        requireNear(pose.bodyFrame().tangent,
            {1.0, 0.0, 0.0}, directionTolerance, "body forward");
        require(pose.frontBogie().definitionIndex() == 1,
            "front role must come from longitudinal ordering");
        require(pose.rearBogie().definitionIndex() == 0,
            "rear role must come from longitudinal ordering");
    }

    void pitchedTrackAlignsBody()
    {
        const glm::dvec3 tangent = glm::normalize(glm::dvec3{3.0, 0.0, 4.0});
        const CarPose pose = solveCarPose(
            straightTrack(40.0, tangent), passengerCar(), locationAt(10.0));
        requireNear(pose.bodyFrame().tangent, tangent,
            directionTolerance, "pitched body tangent");
        requireNear(pose.bodyFrame().up,
            glm::cross(tangent, glm::dvec3{0.0, 1.0, 0.0}),
            directionTolerance, "pitched body up");
    }

    void horizontalCurveArticulatesBogies()
    {
        const CompiledPhysicsTrack track = horizontalArcTrack();
        const CarPose pose = solveCarPose(
            track, passengerCar(), locationAt(7.5));
        require(glm::length(
                pose.frontBogie().trackFrame().tangent
                - pose.rearBogie().trackFrame().tangent) > 0.1,
            "curve bogies must sample different tangents");
        require(pose.frontBogie().bodyRelativeYawRadians() > 0.05,
            "front bogie must yaw toward the curve relative to body");
        require(pose.rearBogie().bodyRelativeYawRadians() < -0.05,
            "rear bogie must yaw away from body forward");
        require(finite(pose.frontBogie().bodyRelativeOrientation())
            && finite(pose.rearBogie().bodyRelativeOrientation()),
            "full bogie/body relative orientations must be finite");
    }

    void bankedTrackBlendsUpDirection()
    {
        constexpr double bank = 0.55;
        const CompiledPhysicsTrack track = straightTrack(
            40.0, {1.0, 0.0, 0.0}, 1.0, glm::dvec3{0.0}, bank);
        const CarPose pose = solveCarPose(
            track, passengerCar(), locationAt(10.0));
        requireNear(pose.bodyFrame().up,
            straightFrame({1.0, 0.0, 0.0}, bank).up,
            directionTolerance, "banked body up");
        requireOrthonormal(pose.bodyFrame(), "banked body frame");
    }

    void compoundTrackProducesOrthonormalFrame()
    {
        const CompiledPhysicsTrack track = compoundTrack();
        const CarPose pose = solveCarPose(
            track, passengerCar(), locationAt(8.0));
        requireOrthonormal(pose.bodyFrame(), "compound body frame");
        require(finite(pose.bodyOrientation()),
            "compound body orientation must be finite");
        require(glm::length(pose.frontBogie().trackFrame().up
                - pose.rearBogie().trackFrame().up) > 0.01,
            "compound bogies must sample different banked frames");
    }

    void crestAndValleyRemainStable()
    {
        const CarDefinition definition = passengerCar();
        for (const bool crest : {true, false})
        {
            const CompiledPhysicsTrack track = verticalArcTrack(crest);
            const CarPose pose = solveCarPose(
                track, definition, locationAt(12.0 * 0.6));
            requireOrthonormal(
                pose.bodyFrame(), crest ? "crest frame" : "valley frame");
            require(std::abs(
                    pose.frontBogie().trackFrame().tangent.z
                    - pose.rearBogie().trackFrame().tangent.z) > 0.1,
                "crest/valley bogies must occupy different local pitches");
            require(finite(pose.worldCenterOfGravityMeters()),
                "crest/valley COG must be finite");
        }
    }

    void verticalTrackHasNoSingularity()
    {
        const CarPose pose = solveCarPose(
            straightTrack(40.0, {0.0, 0.0, 1.0}),
            passengerCar(), locationAt(10.0));
        requireNear(pose.bodyFrame().tangent,
            {0.0, 0.0, 1.0}, directionTolerance, "vertical forward");
        requireOrthonormal(pose.bodyFrame(), "vertical body frame");
        require(finite(pose.bodyOrientation()),
            "vertical orientation must be finite");
    }

    void deterministicFallbackHandlesOpposedUpVectors()
    {
        CarDefinition definition = passengerCar();
        definition.bogies = {{{-1.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}};
        const CompiledPhysicsTrack track = opposedUpTrack();
        const CarPose first = solveCarPose(
            track, definition, locationAt(1.0));
        const CarPose second = solveCarPose(
            track, definition, locationAt(1.0));
        requireOrthonormal(first.bodyFrame(), "fallback body frame");
        requireNear(first.bodyFrame().up, second.bodyFrame().up,
            0.0, "fallback up must be deterministic");
        require(glm::dot(first.bodyFrame().up,
                first.frontBogie().orientedFrame().up) > 0.99,
            "fallback must retain the ordered front sampled up direction");
    }

    void openTrackEndpointsClampThroughTrackAdvance()
    {
        const CompiledPhysicsTrack track = straightTrack();
        const CarDefinition definition = passengerCar();
        const CarPose start = solveCarPose(
            track, definition, locationAt(0.0));
        const CarPose end = solveCarPose(
            track, definition, locationAt(track.lengthMeters()));
        requireNear(start.rearBogie().location().stationMeters,
            0.0, 0.0, "open-start rear bogie clamp");
        requireNear(end.frontBogie().location().stationMeters,
            track.lengthMeters(), 0.0, "open-end front bogie clamp");
        requireOrthonormal(start.bodyFrame(), "open-start body frame");
        requireOrthonormal(end.bodyFrame(), "open-end body frame");
    }

    void circuitSeamIsContinuous()
    {
        constexpr double radius = 20.0;
        const CompiledPhysicsTrack track = horizontalArcTrack(
            radius, 2.0 * std::numbers::pi, TopologyKind::ClosedCircuit);
        const double length = track.lengthMeters();
        const CarDefinition definition = passengerCar();
        const CarPose before = solveCarPose(
            track, definition, locationAt(length - 1.0e-6));
        const CarPose after = solveCarPose(
            track, definition, locationAt(1.0e-6));

        require(before.frontBogie().location().stationMeters < 2.0,
            "front bogie must wrap across circuit seam");
        require(before.rearBogie().location().stationMeters > length - 2.0,
            "rear bogie must remain before circuit seam");
        require(glm::length(
                before.bodyWorldPositionMeters()
                - after.bodyWorldPositionMeters()) < 1.0e-4,
            "body position must be continuous across seam");
        require(glm::length(
                before.bodyFrame().tangent
                - after.bodyFrame().tangent) < 1.0e-4,
            "body orientation must be continuous across seam");
    }

    void reverseTravelUsesPhysicalCarForward()
    {
        const CarPose pose = solveCarPose(
            straightTrack(), passengerCar(),
            locationAt(10.0, TravelDirection::DecreasingStation));
        requireNear(pose.frontBogie().location().stationMeters,
            8.75, positionTolerance, "reverse front station");
        requireNear(pose.rearBogie().location().stationMeters,
            11.25, positionTolerance, "reverse rear station");
        require(pose.frontBogie().location().direction
                == TravelDirection::DecreasingStation
            && pose.rearBogie().location().direction
                == TravelDirection::DecreasingStation,
            "offset lookup must preserve car travel direction");
        requireNear(pose.bodyFrame().tangent,
            {-1.0, 0.0, 0.0}, directionTolerance,
            "reverse body +X must face decreasing station");
        requireNear(pose.frontBogie().trackFrame().tangent,
            {1.0, 0.0, 0.0}, directionTolerance,
            "reverse pose must retain canonical sampled frame");
        requireNear(pose.frontBogie().orientedFrame().tangent,
            {-1.0, 0.0, 0.0}, directionTolerance,
            "reverse bogie frame must face car travel");
    }

    void coordinateScaleIsAppliedOnce()
    {
        const CompiledPhysicsTrack track = straightTrack(
            20.0, {1.0, 0.0, 0.0}, 2.5, {2.0, 3.0, 4.0});
        const CarPose pose = solveCarPose(
            track, passengerCar(), locationAt(10.0));
        requireNear(pose.bodyWorldPositionMeters(),
            {15.0, 7.5, 10.0}, positionTolerance,
            "SI body pose must use compiled coordinate scale once");
        requireNear(pose.frontBogie().worldPositionMeters().x,
            16.25, positionTolerance, "SI front bogie position");
    }

    void centerOfGravityAndHitchesTransform()
    {
        const CarDefinition definition = passengerCar();
        const CarLoadout loadout{200.0, {0.3, 0.0, 0.75}};
        const CarPose pose = solveCarPose(
            straightTrack(), definition, locationAt(10.0), loadout);
        requireNear(pose.localCenterOfGravityMeters(),
            {0.14, 0.0, 0.59}, positionTolerance, "pose local COG");
        requireNear(pose.worldCenterOfGravityMeters(),
            {10.14, 0.0, 0.59}, positionTolerance, "pose world COG");
        requireNear(pose.frontHitchWorldPositionMeters(),
            {12.25, 0.0, 0.25}, positionTolerance, "front hitch world");
        requireNear(pose.rearHitchWorldPositionMeters(),
            {7.85, 0.0, 0.25}, positionTolerance, "rear hitch world");
        requireNear(pose.transformLocalPoint({0.14, 0.0, 0.59}),
            pose.worldCenterOfGravityMeters(), positionTolerance,
            "generic local point transform");
    }

    void differentCarGeometryProducesDifferentPose()
    {
        CarDefinition shortCar = passengerCar();
        shortCar.bogies = {{{-0.7, 0.0, 0.0}}, {{0.7, 0.0, 0.0}}};
        CarDefinition longCar = passengerCar();
        longCar.bogies = {{{-2.0, 0.0, 0.0}}, {{2.0, 0.0, 0.0}}};
        const CompiledPhysicsTrack track = horizontalArcTrack();
        const TrackLocation reference = locationAt(7.5);
        const CarPose shortPose = solveCarPose(track, shortCar, reference);
        const CarPose longPose = solveCarPose(track, longCar, reference);
        require(glm::length(
                shortPose.frontBogie().worldPositionMeters()
                - longPose.frontBogie().worldPositionMeters()) > 1.0,
            "distinct authored bogie spacing must produce distinct samples");
        require(glm::length(
                shortPose.bodyWorldPositionMeters()
                - longPose.bodyWorldPositionMeters()) > 0.05,
            "distinct authored geometry must produce distinct body pose");
    }

    void solveIsDeterministic()
    {
        const CompiledPhysicsTrack track = compoundTrack();
        const CarDefinition definition = passengerCar();
        const CarLoadout loadout{173.0, {0.2, -0.03, 0.8}};
        const TrackLocation reference = locationAt(8.0);
        const CarPose first = solveCarPose(
            track, definition, reference, loadout);
        const CarPose second = solveCarPose(
            track, definition, reference, loadout);
        requireNear(first.bodyWorldPositionMeters(),
            second.bodyWorldPositionMeters(), 0.0, "deterministic body position");
        requireNear(first.bodyFrame().tangent,
            second.bodyFrame().tangent, 0.0, "deterministic body tangent");
        requireNear(first.bodyFrame().lateral,
            second.bodyFrame().lateral, 0.0, "deterministic body lateral");
        requireNear(first.bodyFrame().up,
            second.bodyFrame().up, 0.0, "deterministic body up");
        requireNear(first.worldCenterOfGravityMeters(),
            second.worldCenterOfGravityMeters(), 0.0, "deterministic COG");
        requireNear(first.frontBogie().bodyRelativeYawRadians(),
            second.frontBogie().bodyRelativeYawRadians(), 0.0,
            "deterministic articulation");
    }

    void representativeOutputsRemainFinite()
    {
        const CompiledPhysicsTrack track = compoundTrack();
        const CarDefinition definition = passengerCar();
        for (const TravelDirection direction : {
            TravelDirection::IncreasingStation,
            TravelDirection::DecreasingStation})
        {
            for (int index = 2; index <= 14; ++index)
            {
                const double station = static_cast<double>(index);
                const CarPose pose = solveCarPose(
                    track, definition, locationAt(station, direction));
                require(finite(pose.bodyWorldPositionMeters())
                    && finite(pose.worldCenterOfGravityMeters())
                    && finite(pose.frontHitchWorldPositionMeters())
                    && finite(pose.rearHitchWorldPositionMeters())
                    && finite(pose.frontBogie().worldPositionMeters())
                    && finite(pose.rearBogie().worldPositionMeters())
                    && finite(pose.bodyOrientation())
                    && finite(pose.frontBogie().bodyRelativeOrientation())
                    && finite(pose.rearBogie().bodyRelativeOrientation()),
                    "all representative pose outputs must remain finite");
                requireOrthonormal(
                    pose.bodyFrame(), "representative body frame");
            }
        }
    }

    // This test intentionally constructs a pose from only canonical
    // TrackKinematicState data. No TrackStylePreset, rail count, mesh, tube,
    // tie, spine, or renderer type participates in the public pose path.
    void canonicalTrackDataIsTheOnlyGeometryDependency()
    {
        const std::vector<TrackKinematicState> canonical = straightSamples(20.0);
        const CompiledPhysicsTrack track{
            canonical, 1.0, TopologyKind::OpenLinear};
        const CarPose pose = solveCarPose(
            track, passengerCar(), locationAt(10.0));
        requireNear(pose.bodyWorldPositionMeters(),
            {10.0, 0.0, 0.0}, positionTolerance,
            "canonical-only track pose");
    }
}

int main()
{
    int passed = 0;
    int failed = 0;

    const auto run = [&](const char* name, void (*test)())
    {
        try
        {
            test();
            ++passed;
            std::fprintf(stdout, "  PASS  %s\n", name);
        }
        catch (const std::exception& error)
        {
            ++failed;
            std::fprintf(stderr, "  FAIL  %s: %s\n", name, error.what());
        }
    };

    std::fprintf(stdout, "Car Pose Physics Tests\n");

    run("valid car definition and loadout", validCarDefinitionAndLoadout);
    run("zero-car-compatible definition", zeroCarCompatibleDefinition);
    run("invalid mass and dimensions", invalidMassAndDimensionsAreRejected);
    run("invalid bogie geometry", invalidBogieGeometryIsRejected);
    run("straight track", straightTrackAlignsBogiesAndBody);
    run("pitched track", pitchedTrackAlignsBody);
    run("horizontal curve articulation", horizontalCurveArticulatesBogies);
    run("banked track", bankedTrackBlendsUpDirection);
    run("compound pitch yaw bank", compoundTrackProducesOrthonormalFrame);
    run("hill crest and valley", crestAndValleyRemainStable);
    run("vertical track", verticalTrackHasNoSingularity);
    run("deterministic up fallback", deterministicFallbackHandlesOpposedUpVectors);
    run("open-track endpoint clamping", openTrackEndpointsClampThroughTrackAdvance);
    run("circuit seam", circuitSeamIsContinuous);
    run("reverse travel", reverseTravelUsesPhysicalCarForward);
    run("coordinate scale", coordinateScaleIsAppliedOnce);
    run("COG and hitch transforms", centerOfGravityAndHitchesTransform);
    run("different car geometries", differentCarGeometryProducesDifferentPose);
    run("determinism", solveIsDeterministic);
    run("finite representative outputs", representativeOutputsRemainFinite);
    run("canonical track dependency only", canonicalTrackDataIsTheOnlyGeometryDependency);

    std::fprintf(stdout, "\nResults: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
