#pragma once

#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/renderer/Renderer.hpp>

#include <string>
#include <string_view>
#include <span>
#include <vector>

namespace quantum::editor
{
    inline constexpr std::string_view placeholderTrainCarAssetId =
        "assets://train/placeholder-car-shell.glb";
    inline constexpr std::string_view placeholderTrainLeadAssetId =
        "assets://train/placeholder-lead-car-shell.glb";
    inline constexpr std::string_view placeholderTrainRearAssetId =
        "assets://train/placeholder-rear-car-shell.glb";

    // Editor presentation data, independent of authored/serialized physics.
    // The placeholder is authored in meters about the physical body origin.
    struct TrainVisualPrototype
    {
        std::string assetIdentifier{placeholderTrainCarAssetId};
        glm::dmat4 localAssetTransform{1.0};
    };

    // Entry i describes render-pose car i; it does not define physical cars.
    struct TrainVisualConsist
    {
        std::vector<TrainVisualPrototype> cars;
    };

    // Temporary presentation policy, to be replaced by authored visual inputs:
    // 0: empty; 1: lead; 2: lead/rear; 3+: lead/middle(s)/rear.
    [[nodiscard]] TrainVisualConsist makeDefaultTrainVisualConsist(std::size_t carCount);

    // Reuses output storage. Poses are borrowed only during this call; each
    // result owns its identifier and matrix, with no physics or GPU handles.
    // A count mismatch throws invalid_argument before changing output values.
    void updateTrainMeshInstances(std::vector<renderer::StaticMeshInstance>& instances,
        const physics::TrainPose& renderPose, std::span<const TrainVisualPrototype> prototypes,
        double coordinateUnitsPerMeter);
}
