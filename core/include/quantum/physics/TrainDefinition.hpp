#pragma once

#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace quantum::physics
{
    // Contact roles describe intended mechanical function only. The authored
    // normal remains authoritative; roles do not imply direction or engagement.
    enum class BogieContactRole : std::uint8_t
    {
        Running,
        Guide,
        Upstop
    };

    // Rigid physics contact geometry in the travel-oriented bogie frame:
    // +X forward/rolling tangent, +Y lateral, +Z up. A positive scalar lambda
    // applies lambda * contactNormalLocal from the rail to the bogie. Contacts
    // are intrinsically unilateral, but engagement and lambda >= 0 are deferred.
    struct BogieContactDefinition
    {
        BogieContactRole role = BogieContactRole::Running;
        glm::dvec3 localPositionMeters{0.0};
        glm::dvec3 contactNormalLocal{0.0};
        // Nominal separation along the authored contact normal. Zero retains
        // the original rigid-contact geometry.
        double clearanceMeters = 0.0;
    };

    // Prescribed displacement of the rigid bogie/contact assembly relative
    // to its nominal Phase 2 travel-oriented frame. M0 exposes this only as a
    // contact query; these transverse coordinates are not dynamically solved.
    struct BogieContactClearanceState
    {
        double lateralDisplacementMeters = 0.0;
        double verticalDisplacementMeters = 0.0;
    };

    // Car-local physical coordinates use +X forward, +Y lateral, and +Z up.
    // A bogie's X coordinate seeds its nominal station offset; the two-bogie
    // solver adjusts those stations locally to preserve rigid pivot spacing.
    // Its complete position identifies the corresponding body reference
    // point, allowing the body origin to differ from the track centerline
    // (for example, for a suspended vehicle) without depending on rendered
    // rail geometry.
    struct BogieDefinition
    {
        glm::dvec3 referencePositionMeters{0.0};
        std::vector<BogieContactDefinition> contacts;
    };

    // Reusable authored vehicle data. Bogies use contiguous variable-length
    // storage so the data model does not permanently impose a two-bogie car;
    // the Phase 2 pose solver below deliberately accepts exactly two.
    struct CarDefinition
    {
        double dryMassKilograms = 1.0;
        glm::dvec3 dryCenterOfGravityMeters{0.0};
        // Full symmetric inertia tensor about the dry COG, expressed in the
        // physical car frame (+X forward, +Y lateral, +Z up), in kg*m^2.
        // This is authoritative authored physics data; body dimensions do not
        // implicitly replace it with a box approximation.
        glm::dmat3 dryInertiaTensorBodyKgM2{1.0};
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

    struct TrainCarDefinition
    {
        CarDefinition car;
        CarLoadout loadout;
        std::uint64_t authoredCarId = 0;
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
        std::uint64_t trainId = 1;
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

    // Convenience approximation for authored defaults and tests only. Real
    // vehicle inertia should be authored directly on CarDefinition.
    [[nodiscard]] glm::dmat3 makeUniformBoxInertiaTensorBodyKgM2(
        double massKilograms,
        const glm::dvec3& dimensionsMeters);

    // Combines dry inertia about the dry COG with CarLoadout treated as a
    // point mass at its authored COM, returning inertia about the loaded COG.
    [[nodiscard]] glm::dmat3 loadedCarInertiaTensorBodyKgM2(
        const CarDefinition& definition,
        const CarLoadout& loadout = {});

    void validateInterCarConnectionDefinition(const InterCarConnectionDefinition& definition);
    void validateTrainDefinition(const TrainDefinition& definition);
    void validateBasicResistance(const BasicResistance& resistance);
}
