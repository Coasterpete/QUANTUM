#pragma once

#include <quantum/physics/CarPose.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace quantum::physics
{
    // The tolerance closes authored hitch-to-hitch connector geometry; it is
    // not a rendering tolerance. The Jacobian step is measured in the same
    // generalized SI distance as TrackLocation::stationMeters.
    inline constexpr double connectorLengthToleranceMeters = 1.0e-8;
    inline constexpr double trainKinematicJacobianStepMeters = 1.0e-2;
    // A sampled relative rotation this close to pi has an ambiguous shortest
    // logarithm axis and is rejected instead of producing unstable rates.
    inline constexpr double angularDerivativeNearPiToleranceRadians = 1.0e-6;
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
    // Phase 7 uses the same balance tolerance policy as connector recovery.
    // Bogie separations below one micrometre are reported as singular for a
    // future moment-based split instead of implying numerically useful load
    // leverage from the much smaller Phase 2 pose-only threshold.
    inline constexpr double bogieReactionBalanceAbsoluteToleranceNewtons =
        1.0e-3;
    inline constexpr double bogieReactionBalanceRelativeTolerance = 1.0e-4;
    inline constexpr double bogieReactionMinimumSeparationMeters = 1.0e-6;
    inline constexpr double bogieReactionMomentBalanceAbsoluteToleranceNewtonMeters =
        1.0e-3;
    inline constexpr double bogieReactionMomentBalanceRelativeTolerance =
        1.0e-4;
    inline constexpr double bogieReactionRankRelativeTolerance = 1.0e-10;
    inline constexpr double bogieReactionMaximumConditionEstimate = 1.0e5;

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

    // Runtime physical input, intentionally separate from authored vehicle
    // configuration. The application point uses the target car's physical
    // +X-forward, +Y-lateral, +Z-up coordinates; the force is world-space.
    struct ExternalForceApplication
    {
        std::size_t carIndex = 0;
        glm::dvec3 localApplicationPointMeters{0.0};
        glm::dvec3 worldForceNewtons{0.0};
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
        glm::dvec3
            worldCenterOfGravitySecondDerivativePerGeneralizedMeterSquared{
                0.0};
        // World angular velocity is Jw*qdot. Its derivative supplies the
        // geometry term in alpha=Jw*qdd+Jw'*qdot^2.
        glm::dvec3 worldAngularRateJacobianRadiansPerGeneralizedMeter{0.0};
        glm::dvec3
            worldAngularRateJacobianDerivativeRadiansPerGeneralizedMeterSquared{
                0.0};
        glm::dvec3 bodyAngularRateJacobianRadiansPerGeneralizedMeter{0.0};
        glm::dvec3
            bodyAngularRateJacobianDerivativeRadiansPerGeneralizedMeterSquared{
                0.0};
        glm::dmat3 loadedInertiaTensorBodyKgM2{1.0};
        double translationalEffectiveMassKilograms = 0.0;
        double rotationalEffectiveMassKilograms = 0.0;
        double translationalEffectiveMassDerivativeKilogramsPerMeter = 0.0;
        double rotationalEffectiveMassDerivativeKilogramsPerMeter = 0.0;
        double generalizedGravityForceNewtons = 0.0;
    };

    enum class CarAngularKinematicsStatus : std::uint8_t
    {
        Available
    };

    // Runtime angular state derived from immutable q-based car kinematics.
    // All angular vectors are physical vectors, never Euler-angle rates.
    class CarAngularKinematics
    {
    public:
        CarAngularKinematics(
            std::size_t carIndex,
            CarAngularKinematicsStatus status,
            glm::dvec3 worldAngularVelocityRadiansPerSecond,
            glm::dvec3 worldAngularAccelerationRadiansPerSecondSquared,
            glm::dvec3 bodyAngularVelocityRadiansPerSecond,
            glm::dvec3 bodyAngularAccelerationRadiansPerSecondSquared,
            glm::dvec3 worldAngularRateJacobianRadiansPerGeneralizedMeter,
            glm::dvec3
                worldAngularRateJacobianDerivativeRadiansPerGeneralizedMeterSquared,
            double rotationalKineticEnergyJoules,
            double rotationalEffectiveMassKilograms,
            double rotationalEffectiveMassDerivativeKilogramsPerMeter);

        [[nodiscard]] std::size_t carIndex() const noexcept;
        [[nodiscard]] CarAngularKinematicsStatus status() const noexcept;
        [[nodiscard]] const glm::dvec3&
            worldAngularVelocityRadiansPerSecond() const noexcept;
        [[nodiscard]] const glm::dvec3&
            worldAngularAccelerationRadiansPerSecondSquared() const noexcept;
        [[nodiscard]] const glm::dvec3&
            bodyAngularVelocityRadiansPerSecond() const noexcept;
        [[nodiscard]] const glm::dvec3&
            bodyAngularAccelerationRadiansPerSecondSquared() const noexcept;
        [[nodiscard]] const glm::dvec3&
            worldAngularRateJacobianRadiansPerGeneralizedMeter() const noexcept;
        [[nodiscard]] const glm::dvec3&
            worldAngularRateJacobianDerivativeRadiansPerGeneralizedMeterSquared()
                const noexcept;
        [[nodiscard]] double rotationalKineticEnergyJoules() const noexcept;
        [[nodiscard]] double rotationalEffectiveMassKilograms() const noexcept;
        [[nodiscard]] double
            rotationalEffectiveMassDerivativeKilogramsPerMeter() const noexcept;
        [[nodiscard]] bool available() const noexcept;

    private:
        std::size_t carIndex_ = 0;
        CarAngularKinematicsStatus status_ =
            CarAngularKinematicsStatus::Available;
        glm::dvec3 worldAngularVelocityRadiansPerSecond_{0.0};
        glm::dvec3 worldAngularAccelerationRadiansPerSecondSquared_{0.0};
        glm::dvec3 bodyAngularVelocityRadiansPerSecond_{0.0};
        glm::dvec3 bodyAngularAccelerationRadiansPerSecondSquared_{0.0};
        glm::dvec3 worldAngularRateJacobianRadiansPerGeneralizedMeter_{0.0};
        glm::dvec3
            worldAngularRateJacobianDerivativeRadiansPerGeneralizedMeterSquared_{
                0.0};
        double rotationalKineticEnergyJoules_ = 0.0;
        double rotationalEffectiveMassKilograms_ = 0.0;
        double rotationalEffectiveMassDerivativeKilogramsPerMeter_ = 0.0;
    };

    struct TrainAngularKinematicEvaluation
    {
        std::vector<CarAngularKinematics> cars;
        double totalRotationalKineticEnergyJoules = 0.0;
        double totalRotationalEffectiveMassKilograms = 0.0;
        double rotationalEffectiveMassDerivativeKilogramsPerMeter = 0.0;
    };

    struct ExternalForceApplicationEvaluation
    {
        std::size_t carIndex = 0;
        glm::dvec3 worldApplicationPointMeters{0.0};
        glm::dvec3 worldApplicationPointDerivativePerGeneralizedMeter{0.0};
        double generalizedForceNewtons = 0.0;
    };

    struct TrainKinematicEvaluation
    {
        TrainPose pose;
        std::vector<TrainCarKinematics> cars;
        double translationalEffectiveGeneralizedMassKilograms = 0.0;
        double rotationalEffectiveGeneralizedMassKilograms = 0.0;
        double effectiveGeneralizedMassKilograms = 0.0;
        double translationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter =
            0.0;
        double rotationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter =
            0.0;
        double effectiveGeneralizedMassDerivativeKilogramsPerMeter = 0.0;
        double generalizedGravityForceNewtons = 0.0;
        std::vector<ExternalForceApplicationEvaluation> externalForces;
        double generalizedExternalForceNewtons = 0.0;
        double finiteDifferenceStepMeters = trainKinematicJacobianStepMeters;
        TrainFiniteDifferenceKind finiteDifferenceKind =
            TrainFiniteDifferenceKind::Central;
    };

    // COG translation and body rotation both participate. Bogie rotational
    // inertia and arbitrary applied torque sources remain deferred.
    [[nodiscard]] TrainKinematicEvaluation evaluateTrainKinematics(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrackLocation& generalizedReferenceLocation,
        std::span<const ExternalForceApplication> externalForces = {});

    // Converts the q-based orientation derivatives in an existing evaluation
    // into deterministic angular velocity, acceleration, and kinetic energy
    // without resolving any train poses.
    [[nodiscard]] TrainAngularKinematicEvaluation
        evaluateTrainAngularKinematics(
            const TrainKinematicEvaluation& kinematics,
            double signedVelocityMetersPerSecond,
            double generalizedAccelerationMetersPerSecondSquared);

    struct TrainDynamicsState
    {
        TrackLocation generalizedReferenceLocation;
        double signedVelocityMetersPerSecond = 0.0;
        double generalizedAccelerationMetersPerSecondSquared = 0.0;
        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;
    };

    struct ExplicitResistanceTelemetry
    {
        std::size_t generatedApplicationCount = 0;
        std::size_t aerodynamicApplicationCount = 0;
        double generalizedAerodynamicForceNewtons = 0.0;
        double totalGeneralizedExplicitResistanceForceNewtons = 0.0;
        double finiteDifferenceStepMeters = trainKinematicJacobianStepMeters;
        TrainFiniteDifferenceKind finiteDifferenceKind =
            TrainFiniteDifferenceKind::Central;
    };

    // Generates one ordinary ExternalForceApplication for each car with
    // nonzero authored CdA. Zero-CdA cars are omitted. outputForces is cleared
    // but retains its capacity so callers can reuse storage each physics tick.
    // The application-point velocity is (dp/dq) * qdot, using the same legal
    // full-consist finite-difference poses as Phase 5 virtual work.
    [[nodiscard]] ExplicitResistanceTelemetry
        generateExplicitResistanceForces(
            const CompiledPhysicsTrack& track,
            const TrainDefinition& definition,
            const PhysicsEnvironment& environment,
            const TrainDynamicsState& state,
            std::vector<ExternalForceApplication>& outputForces);

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

    // Recovers gravity/inertia/external-force-consistent rigid connector axial
    // loads from an already defined train state. Aggregate resistance has no
    // authored per-car distribution, so any force-producing aggregate
    // resistance model makes a multi-car result explicitly unavailable.
    [[nodiscard]] RigidConnectorLoadAnalysis evaluateRigidConnectorLoads(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state,
        std::span<const ExternalForceApplication> externalForces = {});

    enum class BogieRole : std::uint8_t
    {
        Front,
        Rear
    };

    // This status applies to the uniquely recoverable sum of a car's two
    // track reactions. Invalid authored/runtime input continues to use the
    // existing validation exceptions; unavailable statuses describe valid
    // inputs whose current reduced mechanics do not determine a result.
    enum class CarTrackReactionRecoveryStatus : std::uint8_t
    {
        Available,
        AggregateResistanceUnderdetermined,
        ConnectorLoadRecoveryUnavailable,
        InconsistentGeneralizedBalance,
        KinematicsUnavailable
    };

    // A bogie reaction is available only when the two ideal constraint-plane
    // resultants are uniquely and consistently recovered from force and moment
    // balance. MomentBalanceNotImplemented is retained for source compatibility
    // with Phase 7/8 callers but is no longer produced by Phase 9.
    enum class BogieReactionRecoveryStatus : std::uint8_t
    {
        Available,
        AggregateCarReactionUnavailable,
        MomentBalanceNotImplemented,
        SingularGeometry,
        MissingConstraintSubspace,
        RankDeficient,
        IllConditioned,
        ForceBalanceInconsistent,
        MomentBalanceInconsistent,
        RotationalKinematicsUnavailable,
        NonFiniteSystem
    };

    struct BogieReaction
    {
        std::size_t carIndex = 0;
        std::size_t bogieDefinitionIndex = 0;
        BogieRole role = BogieRole::Front;
        TrackLocation location;
        glm::dvec3 worldPositionMeters{0.0};
        geometry::CurveFrame trackFrame;
        BogieReactionRecoveryStatus status =
            BogieReactionRecoveryStatus::AggregateCarReactionUnavailable;
        std::optional<glm::dvec3> worldReactionNewtons;
        std::optional<double> magnitudeNewtons;
        // (x, y, z) are canonical track-frame tangent, lateral, and up
        // components. They are not running-, guide-, or upstop-wheel loads.
        std::optional<glm::dvec3> trackFrameComponentsNewtons;
        // Body-frame (+X forward, +Y lateral, +Z up) projection of the same
        // ideal aggregate bogie resultant.
        std::optional<glm::dvec3> bodyFrameComponentsNewtons;
        std::size_t reactionSolveRank = 0;
        std::optional<double> reactionSolveConditionEstimate;
        // Moment rows are multiplied by this inverse-metre scale only inside
        // the numerical solve. Published residuals retain N and N*m units.
        double momentRowScalePerMeter = 0.0;
    };

    struct CarTrackReaction
    {
        std::size_t carIndex = 0;
        CarTrackReactionRecoveryStatus status =
            CarTrackReactionRecoveryStatus::KinematicsUnavailable;
        glm::dvec3 worldCenterOfGravityMeters{0.0};
        std::optional<glm::dvec3>
            worldCenterOfGravityAccelerationMetersPerSecondSquared;
        // This is R_front + R_rear. It is a free-vector resultant for
        // translational balance and has no inferred application point.
        std::optional<glm::dvec3> aggregateWorldBogieReactionNewtons;
        std::optional<double> aggregateMagnitudeNewtons;
        // (x, y, z) are car-body tangent/forward, lateral, and up components.
        // They do not identify which physical wheel contacts carry the force.
        std::optional<glm::dvec3> aggregateBodyFrameComponentsNewtons;
        BogieReaction frontBogie;
        BogieReaction rearBogie;
        // R_front + R_rear + F_known - m*a_COG. When a finite conditioned
        // split candidate exists this diagnoses that candidate; otherwise it
        // remains the exact aggregate Phase 7 closure residual.
        std::optional<glm::dvec3> forceBalanceResidualNewtons;
        double forceBalanceToleranceNewtons = 0.0;
        std::optional<glm::dvec3> momentBalanceResidualNewtonMeters;
        double momentBalanceToleranceNewtonMeters = 0.0;
        std::optional<glm::dvec3> rotationalInertialMomentNewtonMeters;
        std::optional<glm::dvec3> knownAppliedMomentNewtonMeters;
    };

    struct BogieReactionAnalysis
    {
        CarTrackReactionRecoveryStatus status =
            CarTrackReactionRecoveryStatus::KinematicsUnavailable;
        RigidConnectorLoadRecoveryStatus connectorLoadStatus =
            RigidConnectorLoadRecoveryStatus::Available;
        std::vector<CarTrackReaction> cars;
        // M_eff*qdd + 0.5*M_eff'*qdot^2 - (Q_gravity + Q_external).
        // With aggregate resistance disabled, an ideal ordinary track
        // constraint requires this to close within the reported tolerance.
        std::optional<double> generalizedBalanceResidualNewtons;
        double generalizedBalanceToleranceNewtons = 0.0;
        double finiteDifferenceStepMeters = trainKinematicJacobianStepMeters;
        TrainFiniteDifferenceKind finiteDifferenceKind =
            TrainFiniteDifferenceKind::Central;

        [[nodiscard]] bool exactAggregateRecoveryAvailable() const noexcept
        {
            return status == CarTrackReactionRecoveryStatus::Available;
        }
    };

    // Recovers the exact Phase 7 per-car sum and conditionally splits it into
    // front/rear ideal constraint-plane resultants using Phase 8 rotational
    // inertia and force/moment balance about the loaded COG.
    [[nodiscard]] BogieReactionAnalysis evaluateBogieReactions(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const TrainDynamicsState& state,
        std::span<const ExternalForceApplication> externalForces = {});

    struct TrainTelemetry
    {
        std::uint64_t tick = 0;
        double simulationTimeSeconds = 0.0;
        TrackLocation generalizedReferenceLocation;
        TrainPose pose;
        std::vector<TrainCarKinematics> cars;
        std::vector<CarAngularKinematics> angularKinematics;
        double signedSpeedMetersPerSecond = 0.0;
        double generalizedAccelerationMetersPerSecondSquared = 0.0;
        double totalLoadedMassKilograms = 0.0;
        double totalRotationalKineticEnergyJoules = 0.0;
        double translationalEffectiveGeneralizedMassKilograms = 0.0;
        double rotationalEffectiveGeneralizedMassKilograms = 0.0;
        double effectiveGeneralizedMassKilograms = 0.0;
        double rotationalEffectiveGeneralizedMassDerivativeKilogramsPerMeter =
            0.0;
        double generalizedGravityForceNewtons = 0.0;
        double generalizedExternalForceNewtons = 0.0;
        std::size_t externalForceApplicationCount = 0;
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
        const FixedStepSettings& step = {},
        std::span<const ExternalForceApplication> externalForces = {});
}
