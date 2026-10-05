#pragma once

#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/renderer/Renderer.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace quantum::editor
{
    inline constexpr std::string_view placeholderTrainCarAssetId =
        "assets://train/placeholder-car-shell.glb";

    // Editor presentation data, independent of authored/serialized physics.
    // The placeholder is authored in meters about the physical body origin.
    struct TrainVisualPrototype
    {
        std::string assetIdentifier{placeholderTrainCarAssetId};
        glm::dmat4 localAssetTransform{1.0};
    };

    // Reuses output storage. Poses are borrowed only during this call; each
    // result owns its identifier and matrix, with no physics or GPU handles.
    void updateTrainMeshInstances(std::vector<renderer::StaticMeshInstance>& instances,
        const physics::TrainPose& renderPose, const TrainVisualPrototype& prototype,
        double coordinateUnitsPerMeter);
}
