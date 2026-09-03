#pragma once

#include <quantum/physics/TrackFollower.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace quantum::physics
{
    // Car-local physical coordinates use +X forward, +Y lateral, and +Z up.
    // A bogie's X coordinate selects its nominal station offset. Its complete
    // position identifies the corresponding body reference point, allowing
    // the body origin to differ from the track centerline (for example, for a
    // suspended vehicle) without depending on rendered rail geometry.
    struct BogieDefinition
    {
        glm::dvec3 referencePositionMeters{0.0};
    };

    // Reusable authored vehicle data. Bogies use contiguous variable-length
    // storage so the data model does not permanently impose a two-bogie car;
    // the Phase 2 pose solver below deliberately accepts exactly two.
    struct CarDefinition
    {
        double dryMassKilograms = 1.0;
        glm::dvec3 dryCenterOfGravityMeters{0.0};
        glm::dvec3 bodyDimensionsMeters{1.0};
        glm::dvec3 frontHitchPositionMeters{0.5, 0.0, 0.0};
        glm::dvec3 rearHitchPositionMeters{-0.5, 0.0, 0.0};
        std::vector<BogieDefinition> bogies;

        // Effective drag area CdA for this complete car. The aerodynamic
        // center is a car-local physical point (+X forward, +Y lateral,
        // +Z up); it is not assumed to coincide with the loaded COG.
        double aerodynamicDragAreaSquareMeters = 0.0;
        glm::dvec3 aerodynamicCenterLocalMeters{0.0};
    };

    // Scenario load is intentionally separate from reusable authored car
    // mass. The aggregate load acts at one car-local center of mass; seats and
    // individual riders are outside Phase 2.
    struct CarLoadout
    {
        double massKilograms = 0.0;
        glm::dvec3 centerOfMassMeters{0.0};
    };

    void validateBogieDefinition(const BogieDefinition& definition);
    void validateCarDefinition(const CarDefinition& definition);
    void validateCarLoadout(const CarLoadout& loadout);

    [[nodiscard]] double totalCarMassKilograms(
        const CarDefinition& definition,
        const CarLoadout& loadout = {});

    [[nodiscard]] glm::dvec3 loadedCarCenterOfGravityMeters(
        const CarDefinition& definition,
        const CarLoadout& loadout = {});

    class CarPose;

    // Read-only solved bogie state. trackFrame() is the canonical increasing-
    // station frame returned by CompiledPhysicsTrack. orientedFrame() faces
    // the car's travel direction; on reverse travel its tangent and lateral
    // axes are negated while up, handedness, and banking are preserved.
    class BogiePose
    {
    public:
        [[nodiscard]] std::size_t definitionIndex() const noexcept;
        [[nodiscard]] const TrackLocation& location() const noexcept;
        [[nodiscard]] const glm::dvec3& worldPositionMeters() const noexcept;
        [[nodiscard]] const geometry::CurveFrame& trackFrame() const noexcept;
        [[nodiscard]] const geometry::CurveFrame& orientedFrame() const noexcept;
        [[nodiscard]] const glm::dquat& bodyRelativeOrientation() const noexcept;
        [[nodiscard]] double bodyRelativeYawRadians() const noexcept;

    private:
        BogiePose(
            std::size_t definitionIndex,
            TrackLocation location,
            glm::dvec3 worldPositionMeters,
            geometry::CurveFrame trackFrame,
            geometry::CurveFrame orientedFrame,
            glm::dquat bodyRelativeOrientation,
            double bodyRelativeYawRadians);

        std::size_t definitionIndex_ = 0;
        TrackLocation location_;
        glm::dvec3 worldPositionMeters_{0.0};
        geometry::CurveFrame trackFrame_;
        geometry::CurveFrame orientedFrame_;
        glm::dquat bodyRelativeOrientation_{1.0, 0.0, 0.0, 0.0};
        double bodyRelativeYawRadians_ = 0.0;

        friend CarPose solveCarPose(
            const CompiledPhysicsTrack&,
            const CarDefinition&,
            const TrackLocation&,
            const CarLoadout&);
    };

    // Immutable/read-only result for one physical car. The two solved bogies
    // have named accessors so callers never infer front/rear semantics from an
    // authored vector index.
    class CarPose
    {
    public:
        [[nodiscard]] const TrackLocation& referenceLocation() const noexcept;
        [[nodiscard]] const glm::dvec3& bodyWorldPositionMeters() const noexcept;
        [[nodiscard]] const geometry::CurveFrame& bodyFrame() const noexcept;
        [[nodiscard]] const glm::dquat& bodyOrientation() const noexcept;
        [[nodiscard]] const glm::dvec3& localCenterOfGravityMeters() const noexcept;
        [[nodiscard]] const glm::dvec3& worldCenterOfGravityMeters() const noexcept;
        [[nodiscard]] double totalMassKilograms() const noexcept;
        [[nodiscard]] const glm::dvec3& frontHitchWorldPositionMeters() const noexcept;
        [[nodiscard]] const glm::dvec3& rearHitchWorldPositionMeters() const noexcept;
        [[nodiscard]] const BogiePose& frontBogie() const noexcept;
        [[nodiscard]] const BogiePose& rearBogie() const noexcept;

        [[nodiscard]] glm::dvec3 transformLocalPoint(
            const glm::dvec3& localPointMeters) const noexcept;

    private:
        CarPose(
            TrackLocation referenceLocation,
            glm::dvec3 bodyWorldPositionMeters,
            geometry::CurveFrame bodyFrame,
            glm::dquat bodyOrientation,
            glm::dvec3 localCenterOfGravityMeters,
            glm::dvec3 worldCenterOfGravityMeters,
            double totalMassKilograms,
            glm::dvec3 frontHitchWorldPositionMeters,
            glm::dvec3 rearHitchWorldPositionMeters,
            std::array<BogiePose, 2> bogies);

        TrackLocation referenceLocation_;
        glm::dvec3 bodyWorldPositionMeters_{0.0};
        geometry::CurveFrame bodyFrame_;
        glm::dquat bodyOrientation_{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 localCenterOfGravityMeters_{0.0};
        glm::dvec3 worldCenterOfGravityMeters_{0.0};
        double totalMassKilograms_ = 0.0;
        glm::dvec3 frontHitchWorldPositionMeters_{0.0};
        glm::dvec3 rearHitchWorldPositionMeters_{0.0};
        // Solver-owned order is [front, rear]; authored indices are retained
        // by each BogiePose and never used to infer the roles.
        std::array<BogiePose, 2> bogies_;

        friend CarPose solveCarPose(
            const CompiledPhysicsTrack&,
            const CarDefinition&,
            const TrackLocation&,
            const CarLoadout&);
    };

    // The reference location corresponds to car-local X = 0. Bogie stations
    // are found only through CompiledPhysicsTrack::advance(), with local +X
    // following the car's recorded travel direction.
    [[nodiscard]] CarPose solveCarPose(
        const CompiledPhysicsTrack& track,
        const CarDefinition& definition,
        const TrackLocation& referenceLocation,
        const CarLoadout& loadout = {});
}
