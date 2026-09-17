#pragma once

#include <quantum/physics/TrainPhysics.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace quantum::physics
{
    inline constexpr double dynamicContactGapToleranceMeters = 1.0e-9;
    inline constexpr double dynamicContactComplementarityToleranceMeters =
        1.0e-9;
    inline constexpr double dynamicContactMaximumConditionEstimate = 1.0e8;
    inline constexpr std::size_t dynamicContactMaximumSubdivisions = 8;

    // The q step deliberately reuses the established train-pose derivative
    // policy. The 10 micrometre transverse step is large compared with rigid
    // closure tolerance and small compared with authored clearance, balancing
    // cancellation and truncation error for metre-valued double coordinates.
    inline constexpr double dynamicContactLongitudinalDerivativeStepMeters =
        trainKinematicJacobianStepMeters;
    inline constexpr double dynamicContactTransverseDerivativeStepMeters =
        1.0e-5;

    struct DynamicContactState
    {
        TrackLocation generalizedReferenceLocation;
        double signedLongitudinalVelocityMetersPerSecond = 0.0;
        double frontVerticalOffsetMeters = 0.0;
        double frontVerticalVelocityMetersPerSecond = 0.0;
        double rearVerticalOffsetMeters = 0.0;
        double rearVerticalVelocityMetersPerSecond = 0.0;
        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;
    };

    struct DynamicBogiePose
    {
        std::size_t definitionIndex = 0;
        TrackLocation location;
        glm::dvec3 nominalWorldPositionMeters{0.0};
        glm::dvec3 worldPositionMeters{0.0};
        geometry::CurveFrame trackFrame;
        geometry::CurveFrame orientedFrame;
    };

    struct DynamicWorldContactPose
    {
        std::size_t sourceContactIndex = 0;
        BogieContactRole role = BogieContactRole::Running;
        glm::dvec3 worldPositionMeters{0.0};
        glm::dvec3 worldNormal{0.0};
        double signedGapMeters = 0.0;
    };

    struct DynamicContactCarPose
    {
        TrackLocation referenceLocation;
        glm::dvec3 bodyWorldPositionMeters{0.0};
        geometry::CurveFrame bodyFrame;
        glm::dquat bodyOrientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 localCenterOfGravityMeters{0.0};
        glm::dvec3 worldCenterOfGravityMeters{0.0};
        double totalMassKilograms = 0.0;
        glm::dvec3 frontHitchWorldPositionMeters{0.0};
        glm::dvec3 rearHitchWorldPositionMeters{0.0};
        DynamicBogiePose frontBogie;
        DynamicBogiePose rearBogie;
        std::vector<DynamicWorldContactPose> frontContacts;
        std::vector<DynamicWorldContactPose> rearContacts;

        [[nodiscard]] glm::dvec3 transformLocalPoint(
            const glm::dvec3& localPointMeters) const noexcept;
    };

    enum class DynamicContactStepStatus : std::uint8_t
    {
        Available,
        UnsupportedCarCount,
        UnsupportedBogieCount,
        UnsupportedConnector,
        UnsupportedTrackGeometry,
        UnsupportedBogieReferenceGeometry,
        UnsupportedGuideContact,
        UnsupportedContactNormal,
        UnsupportedAsymmetricContacts,
        UnsupportedNonidentifiableContacts,
        UnsupportedAggregateResistance,
        UnsupportedGeneratedAerodynamics,
        UnsupportedFixedTimeStep,
        InvalidInitialPenetration,
        RigidClosureFailure,
        MassMatrixNonFinite,
        MassMatrixNotPositiveDefinite,
        MassMatrixIllConditioned,
        ContactSystemIllConditioned,
        ContactSolveNonConverged,
        NonlinearPenetration,
        TrackBoundaryReached,
        NonFiniteInput
    };

    enum class DynamicContactConditionStatus : std::uint8_t
    {
        NotEvaluated,
        WellConditioned,
        Singular,
        IllConditioned,
        NonFinite
    };

    struct DynamicContactTelemetry
    {
        TrackLocation generalizedReferenceLocation;
        double signedLongitudinalVelocityMetersPerSecond = 0.0;
        double frontVerticalOffsetMeters = 0.0;
        double frontVerticalVelocityMetersPerSecond = 0.0;
        double rearVerticalOffsetMeters = 0.0;
        double rearVerticalVelocityMetersPerSecond = 0.0;
        std::array<double, 3> freeGeneralizedVelocity{0.0, 0.0, 0.0};
        std::array<double, 3> committedGeneralizedVelocity{0.0, 0.0, 0.0};
        double minimumSignedGapMeters = 0.0;
        double maximumSignedGapMeters = 0.0;
        std::size_t candidateContactCount = 0;
        std::size_t impulseCarryingContactCount = 0;
        double aggregateFrontBogieNormalImpulseNewtonSeconds = 0.0;
        double aggregateRearBogieNormalImpulseNewtonSeconds = 0.0;
        double stepAverageFrontBogieNormalForceNewtons = 0.0;
        double stepAverageRearBogieNormalForceNewtons = 0.0;
        double complementarityResidualMeters = 0.0;
        double nonlinearCommittedGapResidualMeters = 0.0;
        DynamicContactConditionStatus massMatrixStatus =
            DynamicContactConditionStatus::NotEvaluated;
        double massMatrixConditionEstimate = 0.0;
        DynamicContactConditionStatus contactSystemStatus =
            DynamicContactConditionStatus::NotEvaluated;
        double contactSystemConditionEstimate = 0.0;
        std::size_t solverIterationCount = 0;
        std::size_t retryCount = 0;
        std::size_t substepCount = 0;
        double kineticEnergyBeforeContactJoules = 0.0;
        double kineticEnergyAfterContactJoules = 0.0;
        std::size_t dynamicPoseEvaluationCount = 0;
        std::size_t massMatrixSize = 3;
        std::size_t contactSystemSize = 0;
    };

    struct DynamicContactStepResult
    {
        DynamicContactStepStatus status = DynamicContactStepStatus::Available;
        DynamicContactState state;
        std::optional<DynamicContactCarPose> pose;
        DynamicContactTelemetry telemetry;

        [[nodiscard]] bool available() const noexcept
        {
            return status == DynamicContactStepStatus::Available;
        }
    };

    // Separate opt-in M1A path. The existing TrainDynamicsState/stepTrain path
    // is neither called nor modified by this solver.
    [[nodiscard]] DynamicContactStepResult stepDynamicContact(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const DynamicContactState& currentState,
        const FixedStepSettings& step = {},
        std::span<const ExternalForceApplication> externalForces = {});

    //=========================================================================
    // M1B: Coupled Multi-Car Planar Vertical Dynamic Contact
    //=========================================================================

    inline constexpr std::size_t dynamicContactPgsMaximumIterations = 32;
    inline constexpr std::size_t dynamicContactExhaustiveSolverMaximumRows = 4;

    // Maps between the reduced generalized-coordinate vector and individual
    // car/bogie DOF indices. The layout is:
    //   [s, z_0f, z_0r, z_1f, z_1r, ..., z_Nf, z_Nr]
    // Total DOFs: 1 + 2 * carCount
    struct DynamicContactDofLayout
    {
        std::size_t carCount = 0;
        std::size_t dofCount = 0;

        [[nodiscard]] static DynamicContactDofLayout create(
            std::size_t carCount) noexcept;

        [[nodiscard]] std::size_t longitudinalDof() const noexcept;
        [[nodiscard]] std::size_t frontVerticalDof(
            std::size_t carIndex) const noexcept;
        [[nodiscard]] std::size_t rearVerticalDof(
            std::size_t carIndex) const noexcept;
        [[nodiscard]] std::size_t carIndexForVerticalDof(
            std::size_t dofIndex) const noexcept;
        [[nodiscard]] bool isLongitudinalDof(
            std::size_t dofIndex) const noexcept;
    };

    // Per-car result data for multi-car output.
    struct DynamicContactMultiCarCarResult
    {
        DynamicContactCarPose pose;
        std::size_t carIndex = 0;
        double aggregateFrontBogieNormalImpulseNewtonSeconds = 0.0;
        double aggregateRearBogieNormalImpulseNewtonSeconds = 0.0;
    };

    // Connector closure diagnostic for one connection.
    struct DynamicContactConnectorClosure
    {
        std::size_t connectionIndex = 0;
        double signedResidualMeters = 0.0;
        std::size_t refinementIterations = 0;
        double finalBracketSizeMeters = 0.0;
    };

    // Multi-car dynamic contact telemetry, extending M1A telemetry with
    // per-car and connector data.
    struct DynamicContactMultiCarTelemetry
    {
        // Aggregate M1A-compatible telemetry (single-car equivalent).
        double minimumSignedGapMeters = 0.0;
        double maximumSignedGapMeters = 0.0;
        std::size_t candidateContactCount = 0;
        std::size_t impulseCarryingContactCount = 0;
        double complementarityResidualMeters = 0.0;
        double nonlinearCommittedGapResidualMeters = 0.0;
        DynamicContactConditionStatus massMatrixStatus =
            DynamicContactConditionStatus::NotEvaluated;
        double massMatrixConditionEstimate = 0.0;
        DynamicContactConditionStatus contactSystemStatus =
            DynamicContactConditionStatus::NotEvaluated;
        double contactSystemConditionEstimate = 0.0;
        std::size_t solverIterationCount = 0;
        std::size_t retryCount = 0;
        std::size_t substepCount = 0;
        double kineticEnergyBeforeContactJoules = 0.0;
        double kineticEnergyAfterContactJoules = 0.0;
        std::size_t dynamicPoseEvaluationCount = 0;
        std::size_t massMatrixSize = 0;
        std::size_t contactSystemSize = 0;

        // PGS-specific telemetry.
        std::size_t pgsIterationCount = 0;
        double pgsFinalPenetrationMeters = 0.0;
        double pgsFinalImpulseDeltaNewtonSeconds = 0.0;
        bool pgsConverged = false;

        // Connector closure telemetry.
        std::vector<DynamicContactConnectorClosure> connectorClosures;
        double maximumConnectorResidualMeters = 0.0;
    };

    // Multi-car dynamic contact step result.
    struct DynamicContactMultiCarStepResult
    {
        DynamicContactStepStatus status = DynamicContactStepStatus::Available;
        DynamicContactMultiCarTelemetry telemetry;
        std::vector<DynamicContactMultiCarCarResult> carResults;
        DynamicContactDofLayout dofLayout;

        // The generalized velocity vector (1 + 2N).
        std::vector<double> generalizedVelocity;

        // The committed generalized-coordinate vector (1 + 2N).
        std::vector<double> generalizedCoordinates;

        // The lead car's reference location after the step.
        TrackLocation leadCarReferenceLocation;

        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;

        [[nodiscard]] bool available() const noexcept
        {
            return status == DynamicContactStepStatus::Available;
        }

        [[nodiscard]] std::size_t carCount() const noexcept
        {
            return dofLayout.carCount;
        }
    };

    // Multi-car dynamic contact state: generalized coordinates and velocities
    // for the reduced-coordinate formulation.
    struct DynamicContactMultiCarState
    {
        // Lead car's reference location (the single longitudinal DOF).
        TrackLocation leadCarReferenceLocation;

        // Per-car vertical offsets and velocities.
        struct CarVerticalState
        {
            double frontVerticalOffsetMeters = 0.0;
            double frontVerticalVelocityMetersPerSecond = 0.0;
            double rearVerticalOffsetMeters = 0.0;
            double rearVerticalVelocityMetersPerSecond = 0.0;
        };

        std::vector<CarVerticalState> cars;

        // Longitudinal velocity (shared across all cars).
        double signedLongitudinalVelocityMetersPerSecond = 0.0;

        std::uint64_t tick = 0;
        FollowerRunState runState = FollowerRunState::Resting;

        [[nodiscard]] std::size_t carCount() const noexcept
        {
            return cars.size();
        }

        [[nodiscard]] bool isValid() const noexcept;
    };

    // Multi-car opt-in path. M1B: coupled multi-car planar vertical dynamic
    // contact with rigid connectors. The existing stepTrain and
    // stepDynamicContact (M1A) paths are neither called nor modified.
    [[nodiscard]] DynamicContactMultiCarStepResult stepDynamicContactMultiCar(
        const CompiledPhysicsTrack& track,
        const TrainDefinition& definition,
        const PhysicsEnvironment& environment,
        const DynamicContactMultiCarState& currentState,
        const FixedStepSettings& step = {},
        std::span<const ExternalForceApplication> externalForces = {});
}
