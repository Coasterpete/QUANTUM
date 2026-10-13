#include "NativeViewport.hpp"
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <algorithm>
#include <limits>
#include <utility>

void NativeViewport::captureTrainAuthoring(uint64_t revision, bool active, uint64_t selected, uint32_t overlays,
    const quantum::coaster::TrainConfiguration& train, const quantum::physics::TrainPose* pose, double metersPerUnit)
{
    trainActive_ = active; selectedCar_ = selected; trainOverlays_ = overlays; trainAuthoringRevision_ = revision;
    previewCars_.clear();
    if (!active || !pose || pose->carCount() != train.cars.size()) return;
    for (std::size_t i = 0; i < train.cars.size(); ++i)
    {
        const auto& car = pose->cars()[i].carPose();
        previewCars_.push_back({train.cars[i].id, i, car.bodyWorldPositionMeters() / metersPerUnit,
            train.cars[i].physical.car.bodyDimensionsMeters / metersPerUnit,
            pose->cars()[i].loadedWorldCenterOfGravityMeters() / metersPerUnit, car.bodyOrientation()});
    }
}

void NativeViewport::pickTrainAt(double x, double y)
{
    if (!trainActive_) return;
    const auto ray = camera_.viewportRay(x, y, static_cast<double>(width_) / height_);
    if (const auto selected = quantum::editor::pickTrainPreviewCar(previewCars_, ray.origin, ray.direction))
        trainPick_ = TrainPickIntent{trainAuthoringRevision_, *selected};
}

std::optional<NativeViewport::TrainPickIntent> NativeViewport::takeTrainPick() noexcept
{
    return std::exchange(trainPick_, std::nullopt);
}

void NativeViewport::frameTrain(uint32_t action)
{
    if (!trainActive_ || previewCars_.empty()) return;
    std::optional<uint64_t> selected;
    if (action == 15) selected = selectedCar_;
    if (action == 16) selected = previewCars_.front().id;
    if (action == 17) selected = previewCars_.back().id;
    if (const auto bounds = quantum::editor::trainPreviewBounds(previewCars_, selected))
        camera_.frameBounds(bounds->first, bounds->second, static_cast<double>(width_) / height_);
}

void NativeViewport::drawTrainOverlay()
{
    if (!trainActive_) return;
    auto* draw = ImGui::GetBackgroundDrawList();
    const auto area = ImGui::GetIO().DisplaySize;
    const auto label = [&](const glm::dvec3& position, const std::string& text, ImU32 color) {
        const auto point = quantum::editor::projectViewportPoint(camera_, position, static_cast<double>(width_) / height_);
        if (point && point->normalizedPosition.x >= 0 && point->normalizedPosition.x <= 1 && point->normalizedPosition.y >= 0 && point->normalizedPosition.y <= 1)
            draw->AddText({static_cast<float>(point->normalizedPosition.x) * area.x,
                static_cast<float>(point->normalizedPosition.y) * area.y}, color, text.c_str());
    };
    for (const auto& car : previewCars_) if (car.id == selectedCar_)
    {
        label(car.center + car.orientation * glm::dvec3{0, 0, car.dimensions.z * .5},
            "Car " + std::to_string(car.index) + " / #" + std::to_string(car.id), IM_COL32(255, 220, 30, 255));
        if (trainOverlays_ & 8) label(car.cog, "  Loaded COG", IM_COL32(255, 80, 255, 255));
    }
}
