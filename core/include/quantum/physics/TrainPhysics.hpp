#pragma once

#include <quantum/physics/CarPose.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace quantum::physics
{
    // The tolerance closes authored hitch-to-hitch connector geometry; it is
    // not a rendering tolerance. The Jacobian step is measured in the same
    // generalized SI distance as TrackLocation::stationMeters.
    inline constexpr double connectorLengthToleranceMeters = 1.0e-8;
    inline constexpr double trainKinematicJacobianStepMeters = 1.0e-2;
    // Phase 4 local derivatives use a separate 1 cm car-reference-station
    // step. Axes shorter than 1 micrometre and projections that move less than
    // 1 micrometre along the axis per metre of local coordinate are rejected
    // instead of amplifying closure/finite-difference error into huge loads.
    inline constexpr double connectorLoadLocalDerivativeStepMeters = 1.0e-2;
    inline constexpr double connectorLoadMinimumDirectionalLengthMeters =
        1.0e-6;
    inline constexpr double connectorLoadMinimumAxialProjection = 1.0e-6;
    inline constexpr double connectorLoadClassificationToleranceNewtons =
        1.0e-6;
    // The redundant balance row allows 1 mN absolute error plus 0.01% of the
    // largest projected force-balance scale. This accommodates the canonical
    // sampled track and connector-root finite differences while rejecting a
    // materially inconsistent generalized acceleration.
    inline constexpr double connectorLoadBalanceAbsoluteToleranceNewtons =
        1.0e-3;
    inline constexpr double connectorLoadBalanceRelativeTolerance = 1.0e-4;

    struct TrainCarDefinition
    {
        CarDefinition car;
        CarLoadout loadout;
    };

    struct InterCarConnectionDefinition
    {
        double rigidLengthMeters = 0.0;
    };

    // Cars are ordered from lead to rear. Connection i always joins the rear
    // hitch of car i to the front hitch of car i + 1. Resistance coefficients
    // describe the complete train once; they are not multiplied by car count.
    struct TrainDefinition
    {
        std::vector<TrainCarDefinition> cars;
        std::vector<InterCarConnectionDefinition> connections;
        BasicResistance resistance;
    };

    void validateInterCarConnectionDefinition(
        const InterCarConnectionDefinition& definition);
    void validateTrainDefinition(const TrainDefinition& definition);

    class TrainCarPose
    {
    public:
        TrainCarPose(std::size_t carIndex, CarPose carPose);

        [[nodiscard]] std::size_t carIndex() const noexcept;
        [[nodiscard]] const TrackLocation& referenceLocation() const noexcept;
        [[nodiscard]] const CarPose& carPose() const noexcept;
        [[nodiscard]] double loadedMassKilograms() const noexcept;
        [[nodiscard]] const glm::dvec3& loadedLocalCenterOfGravityMeters()
            const noexcept;
        [[nodiscard]] const glm::dvec3& loadedWorldCenterOfGravityMeters()
            const noexcept;

    private:
        std::size_t carIndex_ = 0;
        CarPose carPose_;
    };

    class InterCarConnectionPose
    {
    public:
        InterCarConnectionPose(
            std::size_t connectionIndex,
            std::size_t leadingCarIndex,
            std::size_t followingCarIndex,
            double authoredRigidLengthMeters,
            glm::dvec3 leadingEndpointWorldPositionMeters,
            glm::dvec3 followingEndpointWorldPositionMeters,
            double actualEndpointDistanceMeters,
            double signedLengthResidualMeters,
            std::optional<glm::dvec3> worldDirection,
            std::optional<glm::dvec3> directionInLeadingBody,
            std::optional<glm::dvec3> directionInFollowingBody,
            glm::dquat followingBodyRelativeOrientation,
            glm::dvec3 relativeYawPitchRollRadians,
            std::size_t solverIterationCount,
            double finalBracketSizeMeters);

        [[nodiscard]] std::size_t connectionIndex() const noexcept;
        [[nodiscard]] std::size_t leadingCarIndex() const noexcept;
        [[nodiscard]] std::size_t followingCarIndex() const noexcept;
        [[nodiscard]] double authoredRigidLengthMeters() const noexcept;
        [[nodiscard]] const glm::dvec3& leadingEndpointWorldPositionMeters()
            const noexcept;
        [[nodiscard]] const glm::dvec3& followingEndpointWorldPositionMeters()
            const noexcept;
        [[nodiscard]] double actualEndpointDistanceMeters() const noexcept;
        [[nodiscard]] double signedLengthResidualMeters() const noexcept;
        [[nodiscard]] double absoluteLengthErrorMeters() const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>& worldDirection()
            const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>&
            directionInLeadingBody() const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>&
            directionInFollowingBody() const noexcept;
        [[nodiscard]] const glm::dquat& followingBodyRelativeOrientation()
            const noexcept;
        // Components are yaw, pitch, and roll diagnostics in radians.
        [[nodiscard]] const glm::dvec3& relativeYawPitchRollRadians()
            const noexcept;
        [[nodiscard]] std::size_t solverIterationCount() const noexcept;
        [[nodiscard]] double finalBracketSizeMeters() const noexcept;

    private:
        std::size_t connectionIndex_ = 0;
        std::size_t leadingCarIndex_ = 0;
        std::size_t followingCarIndex_ = 0;
        double authoredRigidLengthMeters_ = 0.0;
        glm::dvec3 leadingEndpointWorldPositionMeters_{0.0};
        glm::dvec3 followingEndpointWorldPositionMeters_{0.0};
        double actualEndpointDistanceMeters_ = 0.0;
        double signedLengthResidualMeters_ = 0.0;
        std::optional<glm::dvec3> worldDirection_;
        std::optional<glm::dvec3> directionInLeadingBody_;
        std::optional<glm::dvec3> directionInFollowingBody_;
        glm::dquat followingBodyRelativeOrientation_{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 relativeYawPitchRollRadians_{0.0};
        std::size_t solverIterationCount_ = 0;
        double finalBracketSizeMeters_ = 0.0;
    };

    class TrainPose
    {
    public:
        TrainPose(
            TrackLocation generalizedReferenceLocation,
            std::vector<TrainCarPose> cars,
            std::vector<InterCarConnectionPose> connections,
            double totalLoadedMassKilograms,
            glm::dvec3 aggregateWorldCenterOfGravityMeters,
            double maximumAbsoluteConnectorResidualMeters);

        [[nodiscard]] const TrackLocation& generalizedReferenceLocation()
            const noexcept;
        [[nodiscard]] const std::vector<TrainCarPose>& cars() const noexcept;
        [[nodiscard]] const std::vector<InterCarConnectionPose>& connections()
            const noexcept;
        [[nodiscard]] std::size_t carCount() const noexcept;
        [[nodiscard]] std::size_t connectionCount() const noexcept;
        [[nodiscard]] double totalLoadedMassKilograms() const noexcept;
        [[nodiscard]] const glm::dvec3& aggregateWorldCenterOfGravityMeters()
            const noexcept;
        [[nodiscard]] double maximumAbsoluteConnectorResidualMeters()
            const noexcept;

    private:
        TrackLocation generalizedReferenceLocation_;
        std::vector<TrainCarPose> cars_;
        std::vector<InterCarConnectionPose> connections_;
        double totalLoadedMassKilograms_ = 0.0;
        glm::dvec3 aggregateWorldCenterOfGravityMeters_{0.0};
        double maximumAbsoluteConnectorResidualMeters_ = 0.0;
    };

    // The generalized reference is exactly the Phase 2 reference location of
    // car 0. Every following car is a constrained function of this coordinate.
    [[nodiscard]] TrainPose solveTrainPose(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const TrackLocation& generalizedReferenceLocation);

    enum class TrainFiniteDifferenceKind : std::uint8_t
    {
        Central,
        Forward,
        Backward
    };

    struct TrainCarKinematics
    {
        std::size_t carIndex = 0;
        glm::dvec3 worldCenterOfGravityDerivativePerGeneralizedMeter{0.0};
        double generalizedGravityForceNewtons = 0.0;
    };

    struct TrainKinematicEvaluation
    {
        TrainPose pose;
        std::vector<TrainCarKinematics> cars;
        double effectiveGeneralizedMassKilograms = 0.0;
        double effectiveGeneralizedMassDerivativeKilogramsPerMeter = 0.0;
        double generalizedGravityForceNewtons = 0.0;
        double finiteDifferenceStepMeters = trainKinematicJacobianStepMeters;
        TrainFiniteDifferenceKind finiteDifferenceKind =
            TrainFiniteDifferenceKind::Central;
    };

    // Only translational COG kinetic energy participates in Phase 3. Body and
    // bogie rotational inertia are intentionally deferred.
    [[nodiscard]] TrainKinematicEvaluation evaluateTrainKinematics(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrackLocation& generalizedReferenceLocation);

    struct TrainDynamicsState
    {
        TrackLocation generalizedReferenceLocation;
        double signedVelocityMetersPerSecond = 0.0;
        double generalizedAccelerationMetersPerSecondSquared = 0.0;
        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;
    };

    enum class RigidConnectorLoadClassification : std::uint8_t
    {
        Tension,
        Compression,
        NearZero,
        Unavailable
    };

    enum class RigidConnectorLoadRecoveryStatus : std::uint8_t
    {
        Available,
        AggregateResistanceUnderdetermined,
        UndefinedConnectorAxis,
        IllConditioned,
        InconsistentBalance
    };

    // Positive axial force is tension/draft. For connection direction n from
    // the leading rear hitch to the following front hitch, the force on the
    // leading car is +T*n and the force on the following car is -T*n.
    // This is a translational reduced-model load, not a connector moment or a
    // complete rigid-body/bogie structural solution.
    class RigidConnectorLoad
    {
    public:
        RigidConnectorLoad(
            std::size_t connectionIndex,
            std::size_t leadingCarIndex,
            std::size_t followingCarIndex,
            RigidConnectorLoadClassification classification,
            std::optional<double> axialForceNewtons,
            std::optional<double> absoluteAxialLoadNewtons,
            std::optional<glm::dvec3> worldDirection,
            std::optional<glm::dvec3> worldForceOnLeadingCarNewtons,
            std::optional<glm::dvec3> worldForceOnFollowingCarNewtons,
            double connectorClosureResidualMeters);

        [[nodiscard]] std::size_t connectionIndex() const noexcept;
        [[nodiscard]] std::size_t leadingCarIndex() const noexcept;
        [[nodiscard]] std::size_t followingCarIndex() const noexcept;
        [[nodiscard]] RigidConnectorLoadClassification classification()
            const noexcept;
        [[nodiscard]] const std::optional<double>& axialForceNewtons()
            const noexcept;
        [[nodiscard]] const std::optional<double>& absoluteAxialLoadNewtons()
            const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>& worldDirection()
            const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>&
            worldForceOnLeadingCarNewtons() const noexcept;
        [[nodiscard]] const std::optional<glm::dvec3>&
            worldForceOnFollowingCarNewtons() const noexcept;
        [[nodiscard]] double connectorClosureResidualMeters() const noexcept;

    private:
        std::size_t connectionIndex_ = 0;
        std::size_t leadingCarIndex_ = 0;
        std::size_t followingCarIndex_ = 0;
        RigidConnectorLoadClassification classification_ =
            RigidConnectorLoadClassification::Unavailable;
        std::optional<double> axialForceNewtons_;
        std::optional<double> absoluteAxialLoadNewtons_;
        std::optional<glm::dvec3> worldDirection_;
        std::optional<glm::dvec3> worldForceOnLeadingCarNewtons_;
        std::optional<glm::dvec3> worldForceOnFollowingCarNewtons_;
        double connectorClosureResidualMeters_ = 0.0;
    };

    class RigidConnectorLoadAnalysis
    {
    public:
        RigidConnectorLoadAnalysis(
            std::vector<RigidConnectorLoad> connectorLoads,
            RigidConnectorLoadRecoveryStatus status,
            std::optional<double> maximumAbsoluteLoadNewtons,
            std::optional<std::size_t> maximumAbsoluteLoadConnectionIndex,
            double maximumTensionNewtons,
            std::optional<std::size_t> maximumTensionConnectionIndex,
            double maximumCompressionMagnitudeNewtons,
            std::optional<std::size_t> maximumCompressionConnectionIndex,
            std::optional<double> balanceResidualNewtons,
            double balanceToleranceNewtons,
            double minimumUsedAxialProjection,
            double localDerivativeStepMeters,
            std::vector<TrainFiniteDifferenceKind> localDerivativeKinds,
            TrainFiniteDifferenceKind constrainedDerivativeKind);

        [[nodiscard]] const std::vector<RigidConnectorLoad>& connectorLoads()
            const noexcept;
        [[nodiscard]] bool exactRecoveryAvailable() const noexcept;
        [[nodiscard]] RigidConnectorLoadRecoveryStatus status() const noexcept;
        [[nodiscard]] const std::optional<double>& maximumAbsoluteLoadNewtons()
            const noexcept;
        [[nodiscard]] const std::optional<std::size_t>&
            maximumAbsoluteLoadConnectionIndex() const noexcept;
        [[nodiscard]] double maximumTensionNewtons() const noexcept;
        [[nodiscard]] const std::optional<std::size_t>&
            maximumTensionConnectionIndex() const noexcept;
        [[nodiscard]] double maximumCompressionMagnitudeNewtons()
            const noexcept;
        [[nodiscard]] const std::optional<std::size_t>&
            maximumCompressionConnectionIndex() const noexcept;
        // Residual of the one omitted/redundant car equation, expressed as
        // inertial-minus-known-external generalized force minus connector
        // generalized force. A valid recovery keeps it within tolerance.
        [[nodiscard]] const std::optional<double>& balanceResidualNewtons()
            const noexcept;
        [[nodiscard]] double balanceToleranceNewtons() const noexcept;
        [[nodiscard]] double minimumUsedAxialProjection() const noexcept;
        [[nodiscard]] double localDerivativeStepMeters() const noexcept;
        [[nodiscard]] const std::vector<TrainFiniteDifferenceKind>&
            localDerivativeKinds() const noexcept;
        [[nodiscard]] TrainFiniteDifferenceKind constrainedDerivativeKind()
            const noexcept;

    private:
        std::vector<RigidConnectorLoad> connectorLoads_;
        RigidConnectorLoadRecoveryStatus status_ =
            RigidConnectorLoadRecoveryStatus::Available;
        std::optional<double> maximumAbsoluteLoadNewtons_;
        std::optional<std::size_t> maximumAbsoluteLoadConnectionIndex_;
        double maximumTensionNewtons_ = 0.0;
        std::optional<std::size_t> maximumTensionConnectionIndex_;
        double maximumCompressionMagnitudeNewtons_ = 0.0;
        std::optional<std::size_t> maximumCompressionConnectionIndex_;
        std::optional<double> balanceResidualNewtons_;
        double balanceToleranceNewtons_ = 0.0;
        double minimumUsedAxialProjection_ = 0.0;
        double localDerivativeStepMeters_ =
            connectorLoadLocalDerivativeStepMeters;
        std::vector<TrainFiniteDifferenceKind> localDerivativeKinds_;
        TrainFiniteDifferenceKind constrainedDerivativeKind_ =
            TrainFiniteDifferenceKind::Central;
    };

    // Recovers gravity/inertia-consistent rigid connector axial loads from an
    // already defined train state. Aggregate resistance has no authored
    // per-car distribution, so any force-producing aggregate resistance model
    // makes a multi-car result explicitly unavailable.
    [[nodiscard]] RigidConnectorLoadAnalysis evaluateRigidConnectorLoads(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state);

    struct TrainTelemetry
    {
        std::uint64_t tick = 0;
        double simulationTimeSeconds = 0.0;
        TrackLocation generalizedReferenceLocation;
        TrainPose pose;
        std::vector<TrainCarKinematics> cars;
        double signedSpeedMetersPerSecond = 0.0;
        double generalizedAccelerationMetersPerSecondSquared = 0.0;
        double totalLoadedMassKilograms = 0.0;
        double effectiveGeneralizedMassKilograms = 0.0;
        double generalizedGravityForceNewtons = 0.0;
        double resistanceForceNewtons = 0.0;
        double kinematicMassGradientForceNewtons = 0.0;
        double constraintForceNewtons = 0.0;
        double totalGeneralizedForceNewtons = 0.0;
        double finiteDifferenceStepMeters = trainKinematicJacobianStepMeters;
        TrainFiniteDifferenceKind finiteDifferenceKind =
            TrainFiniteDifferenceKind::Central;
        std::size_t carCount = 0;
        std::size_t connectionCount = 0;
        double maximumAbsoluteConnectorResidualMeters = 0.0;
        FollowerRunState runState = FollowerRunState::Resting;
        TrackBoundary boundary = TrackBoundary::None;
        bool wrapped = false;
        bool boundaryIntervention = false;
    };

    struct TrainStepResult
    {
        TrainDynamicsState state;
        TrainTelemetry telemetry;
    };

    [[nodiscard]] TrainStepResult stepTrain(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& currentState,
        const FixedStepSettings& step = {});
}
