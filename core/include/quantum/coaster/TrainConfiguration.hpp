#pragma once

#include <quantum/physics/TrainDefinition.hpp>

#include <cstddef>
#include <string>
#include <string_view>

namespace quantum::coaster
{
    using TrainCarId = std::uint64_t;
    enum class TrainCarRole { Lead, Middle, Rear, Zero, Trailer, Custom };
    [[nodiscard]] const char* trainCarRoleName(TrainCarRole role);
    [[nodiscard]] TrainCarRole trainCarRoleFromName(std::string_view name);

    // Alignment affects the shell only. SI metres and XYZ Euler degrees in
    // the physical car frame; rotation composes Rz * Ry * Rx. No implicit scale.
    struct TrainVisualAlignment
    {
        glm::dvec3 translationMeters{0.0};
        glm::dvec3 rotationDegrees{0.0};
    };
    struct AuthoredTrainCar
    {
        TrainCarId id = 0;
        std::string name;
        TrainCarRole role = TrainCarRole::Custom;
        physics::TrainCarDefinition physical;
        // Connection travels with its leading car when the consist is reordered.
        // The last car retains this editable default but has no active connection.
        physics::InterCarConnectionDefinition rearConnection{0.5};
        std::string visualAsset;
        TrainVisualAlignment visualAlignment;
    };

    struct TrainConfiguration
    {
        // Legacy repeated input/default for callers and newly added cars. An
        // ordered document always has cars; an empty vector resolves this input.
        physics::TrainCarDefinition repeatedCar;
        std::size_t carCount = 1;
        physics::InterCarConnectionDefinition repeatedConnection;
        physics::BasicResistance resistance;
        std::uint64_t trainId = 1;
        TrainCarId nextCarId = 1;
        std::string name = "Train 1";
        std::vector<AuthoredTrainCar> cars;
    };

    struct TrainDiagnostic
    {
        bool error = false;
        TrainCarId carId = 0;
        std::string message;
    };
    struct TrainCarSummary
    {
        TrainCarId id = 0;
        double dryMassKilograms = 0, loadedMassKilograms = 0;
        glm::dvec3 loadedCenterOfGravityMeters{0.0};
        double referenceXMeters = 0, gapToNextMeters = 0;
    };
    struct TrainSummary
    {
        std::size_t carCount = 0, bogieCount = 0, connectorCount = 0;
        double dryMassKilograms = 0, loadedMassKilograms = 0;
        double bodyEnvelopeLengthMeters = 0, frontExtentMeters = 0, rearExtentMeters = 0;
        glm::dvec3 straightLoadedCenterOfGravityMeters{0.0};
        bool straightAssemblyAvailable = false;
        std::vector<TrainCarSummary> cars;
        std::vector<TrainDiagnostic> diagnostics;
    };

    [[nodiscard]] TrainConfiguration createDefaultTrainConfiguration();
    [[nodiscard]] TrainConfiguration createDefaultOrderedTrainConfiguration(std::size_t count = 4);
    [[nodiscard]] AuthoredTrainCar createTrainCarPrototype(TrainCarRole role);
    void resizeTrainConsist(TrainConfiguration& configuration, std::size_t count);
    [[nodiscard]] TrainCarId insertTrainCar(TrainConfiguration& configuration,
        std::size_t index, AuthoredTrainCar car);
    void removeTrainCar(TrainConfiguration& configuration, TrainCarId id);
    void reorderTrainCar(TrainConfiguration& configuration, TrainCarId id, std::size_t index);
    [[nodiscard]] TrainSummary summarizeTrainConfiguration(const TrainConfiguration& configuration);
    void validateTrainConfiguration(const TrainConfiguration& configuration);
    [[nodiscard]] std::string serializeTrainConfiguration(const TrainConfiguration& configuration);
    [[nodiscard]] TrainConfiguration deserializeTrainConfiguration(const std::string& text, bool validate = true);

    // Returns a complete validated runtime physics definition. The authored
    // configuration remains independent of editor and renderer state.
    [[nodiscard]] physics::TrainDefinition resolveTrainConfiguration(
        const TrainConfiguration& configuration);
}
