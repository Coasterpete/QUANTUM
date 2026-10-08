#include <quantum/editor/TrainMeshPresentation.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

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
    std::optional<std::uint64_t> pickTrainPreviewCar(std::span<const TrainPreviewCar> cars,
        const glm::dvec3& rayOrigin, const glm::dvec3& rayDirection)
    {
        double nearest = std::numeric_limits<double>::infinity();
        std::optional<std::uint64_t> selected;
        for (const auto& car : cars)
        {
            const auto inverse = glm::conjugate(car.orientation);
            const auto origin = inverse * (rayOrigin - car.center);
            const auto direction = inverse * rayDirection;
            double begin = 0, end = nearest;
            bool hit = true;
            for (int axis = 0; axis < 3; ++axis)
            {
                const double half = car.dimensions[axis] * .5;
                if (std::abs(direction[axis]) < 1e-12)
                {
                    if (std::abs(origin[axis]) > half) { hit = false; break; }
                }
                else
                {
                    auto a = (-half - origin[axis]) / direction[axis];
                    auto b = (half - origin[axis]) / direction[axis];
                    if (a > b) std::swap(a, b);
                    begin = std::max(begin, a); end = std::min(end, b);
                    if (end < begin) { hit = false; break; }
                }
            }
            if (hit && begin < nearest) { selected = car.id; nearest = begin; }
        }
        return selected;
    }

    std::optional<std::pair<glm::dvec3, glm::dvec3>> trainPreviewBounds(
        std::span<const TrainPreviewCar> cars, std::optional<std::uint64_t> selected)
    {
        auto minimum = glm::dvec3{std::numeric_limits<double>::infinity()};
        auto maximum = -minimum;
        for (const auto& car : cars)
        {
            if (selected && car.id != *selected) continue;
            for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1})
            {
                const auto point = car.center + car.orientation * (car.dimensions * glm::dvec3{x, y, z} * .5);
                minimum = glm::min(minimum, point); maximum = glm::max(maximum, point);
            }
        }
        if (!std::isfinite(minimum.x)) return std::nullopt;
        return std::pair{minimum, maximum};
    }
}
