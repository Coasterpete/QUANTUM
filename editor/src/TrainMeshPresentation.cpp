#include <quantum/editor/TrainMeshPresentation.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <stdexcept>

namespace quantum::editor
{
    TrainVisualConsist makeDefaultTrainVisualConsist(const std::size_t carCount)
    {
        TrainVisualConsist consist;
        consist.cars.resize(carCount);
        if (carCount > 0)
            consist.cars.front().assetIdentifier = placeholderTrainLeadAssetId;
        if (carCount > 1)
            consist.cars.back().assetIdentifier = placeholderTrainRearAssetId;
        return consist;
    }

    void updateTrainMeshInstances(std::vector<renderer::StaticMeshInstance>& instances,
        const physics::TrainPose& renderPose, const std::span<const TrainVisualPrototype> prototypes,
        const double coordinateUnitsPerMeter)
    {
        if (prototypes.size() != renderPose.carCount())
            throw std::invalid_argument("Train visual consist has "
                + std::to_string(prototypes.size()) + " entries for "
                + std::to_string(renderPose.carCount()) + " render-pose cars.");
        instances.resize(renderPose.carCount());
        const auto worldUnits = glm::scale(glm::dmat4{1.0},
            glm::dvec3{coordinateUnitsPerMeter});
        for (std::size_t index = 0; index < renderPose.carCount(); ++index)
        {
            const auto& prototype = prototypes[index];
            const auto& car = renderPose.cars()[index].carPose();
            const auto bodyWorld = glm::translate(glm::dmat4{1.0},
                car.bodyWorldPositionMeters()) * glm::mat4_cast(car.bodyOrientation());
            auto& instance = instances[index];
            instance.assetIdentifier = prototype.assetIdentifier;
            instance.transform = glm::mat4{
                worldUnits * bodyWorld * prototype.localAssetTransform};
        }
    }
}
