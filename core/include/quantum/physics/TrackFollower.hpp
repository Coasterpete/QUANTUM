#pragma once

#include <quantum/coaster/TrackKinematics.hpp>
#include <quantum/coaster/TrackPhysicalSettings.hpp>
#include <quantum/coaster/TrackTopology.hpp>

#include <glm/vec3.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace quantum::physics
{
    inline constexpr double defaultFixedTimeStepSeconds = 1.0 / 240.0;
    inline constexpr double followerRestSpeedToleranceMetersPerSecond = 1.0e-9;

    struct TrackPathId
    {
        std::uint32_t value = 0;

        [[nodiscard]] friend bool operator==(
            const TrackPathId&,
            const TrackPathId&) = default;
    };

    inline constexpr TrackPathId primaryTrackPathId{0};

    enum class TravelDirection : std::int8_t
    {
        DecreasingStation = -1,
        IncreasingStation = 1
    };

    // Phase 1 has one path, but callers do not expose a bare scalar station as
    // the permanent location contract. Direction records the last nonzero
    // direction of travel; signed speed remains relative to increasing station.
    struct TrackLocation
    {
        TrackPathId path = primaryTrackPathId;
        double stationMeters = 0.0;
        TravelDirection direction = TravelDirection::IncreasingStation;

        [[nodiscard]] friend bool operator==(
            const TrackLocation&,
            const TrackLocation&) = default;
    };

    enum class TrackBoundary : std::uint8_t
    {
        None,
        Start,
        End
    };

    struct PhysicsTrackSample
    {
        TrackLocation location;
        glm::dvec3 positionMeters{0.0};
        geometry::CurveFrame frame;
        glm::dvec3 curvaturePerMeter{0.0};
    };

    struct TrackAdvanceResult
    {
        TrackLocation location;
        TrackBoundary boundary = TrackBoundary::None;
        bool wrapped = false;
    };

    // Authored shuttles retain open-endpoint semantics even if their endpoint
    // geometry happens to meet the circuit closure tolerances.
    [[nodiscard]] coaster::TopologyKind physicsTopologyForLayout(
        coaster::LayoutMode layoutMode,
        coaster::TopologyKind derivedTopology);

    // Immutable SI query compiled from QUANTUM's canonical kinematic samples.
    // This is an interpolation/query seam, not a second geometry evaluator.
    class CompiledPhysicsTrack
    {
    public:
        CompiledPhysicsTrack(
            std::span<const coaster::TrackKinematicState> kinematics,
            double metersPerCoordinateUnit,
            coaster::TopologyKind topology);

        // Preferred authored-track boundary: consumes the existing physical
        // scale and resolves Circuit/Shuttle intent against derived topology.
        CompiledPhysicsTrack(
            std::span<const coaster::TrackKinematicState> kinematics,
            const coaster::TrackPhysicalSettings& physicalSettings,
            coaster::LayoutMode layoutMode,
            coaster::TopologyKind derivedTopology);

        [[nodiscard]] double lengthMeters() const noexcept;
        [[nodiscard]] coaster::TopologyKind topology() const noexcept;

        [[nodiscard]] PhysicsTrackSample sample(
            const TrackLocation& location) const;

        // This is the only Phase 1 location-normalization seam. Circuits wrap
        // in both directions; open paths clamp and report the reached boundary.
        [[nodiscard]] TrackAdvanceResult advance(
            const TrackLocation& location,
            double signedDistanceMeters) const;

    private:
        struct Sample
        {
            double stationMeters = 0.0;
            glm::dvec3 positionMeters{0.0};
            geometry::CurveFrame frame;
            glm::dvec3 curvaturePerMeter{0.0};
        };

        void validateLocation(const TrackLocation& location) const;

        std::vector<Sample> samples_;
        double lengthMeters_ = 0.0;
        coaster::TopologyKind topology_ = coaster::TopologyKind::OpenLinear;
    };

    struct BasicResistance
    {
        // Coulomb-like mechanical/bearing force capacity, in N.
        double constantMechanicalForceNewtons = 0.0;

        // Linear force coefficient in N/(m/s), equivalently N*s/m.
        double linearResistanceCoefficientNewtonSecondsPerMeter = 0.0;

        // Zero-wind Phase 1 aerodynamic inputs. CdA is represented directly
        // by dragAreaSquareMeters, with force -0.5 * rho * CdA * v * |v|.
        double airDensityKilogramsPerCubicMeter = 1.225;
        double dragAreaSquareMeters = 0.0;

        // Dimensionless Crr. Until bogie/contact loads exist, supported load
        // is approximated as mass * gravity rather than a resolved normal load.
        double rollingResistanceCoefficient = 0.0;
    };

    void validateBasicResistance(const BasicResistance& resistance);

    // Shared Phase 1/3 aggregate resistance law. impendingForceNewtons is the
    // non-resistance generalized force that dry resistance opposes at rest.
    [[nodiscard]] double evaluateBasicResistanceForceNewtons(
        const BasicResistance& resistance,
        double supportedMassKilograms,
        double gravityAccelerationMetersPerSecondSquared,
        double impendingForceNewtons,
        double velocityMetersPerSecond);

    struct SingleFollowerDefinition
    {
        double massKilograms = 1.0;
        BasicResistance resistance;
    };

    struct PhysicsEnvironment
    {
        double gravityAccelerationMetersPerSecondSquared =
            coaster::standardGravityAcceleration;
    };

    [[nodiscard]] PhysicsEnvironment physicsEnvironmentFrom(
        const coaster::TrackPhysicalSettings& settings);

    struct FixedStepSettings
    {
        double deltaTimeSeconds = defaultFixedTimeStepSeconds;
    };

    enum class FollowerRunState : std::uint8_t
    {
        Running,
        Resting,
        StoppedAtStart,
        StoppedAtEnd
    };

    struct TrackFollowerState
    {
        TrackLocation location;
        double signedVelocityMetersPerSecond = 0.0;
        double longitudinalAccelerationMetersPerSecondSquared = 0.0;
        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;
    };

    // Read-only result for the newly committed state. Force contributions are
    // evaluated from the preceding committed state, as required by the
    // semi-implicit step. constraintForceNewtons records zero-crossing or
    // open-endpoint resolution so the reported total remains physically closed.
    struct TrackFollowerTelemetry
    {
        std::uint64_t tick = 0;
        double simulationTimeSeconds = 0.0;
        TrackLocation location;
        glm::dvec3 worldPositionMeters{0.0};
        geometry::CurveFrame frame;
        double signedSpeedMetersPerSecond = 0.0;
        double longitudinalAccelerationMetersPerSecondSquared = 0.0;
        double massKilograms = 0.0;
        double gravityForceNewtons = 0.0;
        double resistanceForceNewtons = 0.0;
        double constraintForceNewtons = 0.0;
        double totalLongitudinalForceNewtons = 0.0;
        double gravityAccelerationMetersPerSecondSquared = 0.0;
        double resistanceAccelerationMetersPerSecondSquared = 0.0;
        double totalLongitudinalAccelerationMetersPerSecondSquared = 0.0;
        glm::dvec3 curvaturePerMeter{0.0};
        double curvatureMagnitudePerMeter = 0.0;
        double curvatureNormalAccelerationMagnitudeMetersPerSecondSquared = 0.0;
        FollowerRunState runState = FollowerRunState::Resting;
        TrackBoundary boundary = TrackBoundary::None;
        bool wrapped = false;
    };

    struct TrackFollowerStepResult
    {
        TrackFollowerState state;
        TrackFollowerTelemetry telemetry;
    };

    void validateSingleFollowerDefinition(
        const SingleFollowerDefinition& definition);
    void validatePhysicsEnvironment(const PhysicsEnvironment& environment);
    void validateFixedStepSettings(const FixedStepSettings& settings);

    // One deterministic Phase 1 integration path: force evaluation at the
    // current state, semi-implicit velocity update, then location advancement.
    [[nodiscard]] TrackFollowerStepResult stepTrackFollower(
        const CompiledPhysicsTrack& track,
        const SingleFollowerDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrackFollowerState& currentState,
        const FixedStepSettings& step = {});
}
