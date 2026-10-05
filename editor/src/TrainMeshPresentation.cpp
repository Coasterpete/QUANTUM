#include <quantum/editor/TrainMeshPresentation.hpp>

#include <glm/gtc/matrix_transform.hpp>

namespace quantum::editor
{
    void updateTrainMeshInstances(std::vector<renderer::StaticMeshInstance>& instances,
        const physics::TrainPose& renderPose, const TrainVisualPrototype& prototype,
        const double coordinateUnitsPerMeter)
    {
        instances.resize(renderPose.carCount());
        const auto worldUnits = glm::scale(glm::dmat4{1.0},
            glm::dvec3{coordinateUnitsPerMeter});
        for (std::size_t index = 0; index < renderPose.carCount(); ++index)
        {
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
