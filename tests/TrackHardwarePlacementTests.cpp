#include <quantum/coaster/TrackStyle.hpp>

#include <glm/geometric.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireNear(
        const glm::dvec3& actual,
        const glm::dvec3& expected,
        const double tolerance,
        const char* const message)
    {
        if (glm::length(actual - expected) > tolerance)
        {
            throw std::runtime_error(message);
        }
    }

    [[nodiscard]] glm::dvec3 translation(const glm::mat4& transform)
    {
        return glm::dvec3{transform[3]};
    }

    [[nodiscard]] std::vector<RiderLocalGeometryState> straightSamples()
    {
        const quantum::geometry::CurveFrame frame{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        std::vector<RiderLocalGeometryState> samples;
        for (int index = 0; index <= 6; ++index)
        {
            samples.push_back({
                static_cast<double>(index),
                {static_cast<double>(index), 0.0, 0.0},
                frame
            });
        }
        return samples;
    }

    [[nodiscard]] TrackStylePreset placementStyle()
    {
        TrackStylePreset style = createStandardDualRailPreset();
        RepeatingHardwareStyle& hardware = style.repeatingHardware.front();
        hardware.spacing = 1.5;
        hardware.startOffset = 0.0;
        hardware.localPosition = glm::dvec3{0.0};
        hardware.localRotation = glm::dvec3{0.0};
        hardware.localScale = glm::dvec3{1.0};
        return style;
    }

    void placementIsDeterministicAndSpaced()
    {
        const auto samples = straightSamples();
        const auto style = placementStyle();
        const auto first = generateRenderableTrack(samples, style);
        const auto second = generateRenderableTrack(samples, style);
        const auto& firstInstances = first.hardwareBatches.front().instances;
        const auto& secondInstances = second.hardwareBatches.front().instances;
        require(firstInstances.size() == 5,
            "six-unit track gets instances at 0, 1.5, 3, 4.5 and 6");
        require(firstInstances.size() == secondInstances.size(),
            "deterministic instance count");
        for (std::size_t index = 0; index < firstInstances.size(); ++index)
        {
            require(firstInstances[index].transform
                    == secondInstances[index].transform,
                "deterministic instance transform");
            require(firstInstances[index].objectId
                    == secondInstances[index].objectId,
                "deterministic object id");
            requireNear(translation(firstInstances[index].transform),
                {static_cast<double>(index) * 1.5, 0.0, 0.0},
                1.0e-6, "requested longitudinal placement");
            if (index > 0)
            {
                require(std::abs(glm::length(
                    translation(firstInstances[index].transform)
                    - translation(firstInstances[index - 1].transform))
                    - 1.5) < 1.0e-6,
                    "requested spacing is respected");
            }
        }
    }

    void placementFollowsCenterlineAndFrame()
    {
        auto style = placementStyle();
        RepeatingHardwareStyle& hardware = style.repeatingHardware.front();
        hardware.spacing = 1.0;
        hardware.localPosition = {0.2, 0.3, 0.4};

        const quantum::geometry::CurveFrame banked{
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0},
            {0.0, -1.0, 0.0}
        };
        const std::vector<RiderLocalGeometryState> samples{
            {0.0, {0.0, 0.0, 0.0}, banked},
            {2.0, {2.0, 2.0, 0.0}, banked}
        };
        const auto result = generateRenderableTrack(samples, style);
        const auto& middle = result.hardwareBatches.front().instances[1];
        requireNear(translation(middle.transform),
            glm::dvec3{1.0, 1.0, 0.0}
                + banked.tangent * 0.2
                + banked.lateral * 0.3
                + banked.up * 0.4,
            1.0e-6,
            "hardware follows interpolated centerline and local position"
        );
        requireNear(glm::normalize(glm::dvec3{middle.transform[1]}),
            banked.lateral, 1.0e-6,
            "banking rotates hardware lateral orientation");
        requireNear(glm::normalize(glm::dvec3{middle.transform[2]}),
            banked.up, 1.0e-6,
            "banking rotates hardware up orientation");
    }

    void localRotationAndScaleAreApplied()
    {
        auto style = placementStyle();
        auto& hardware = style.repeatingHardware.front();
        hardware.spacing = 100.0;
        hardware.localRotation = {
            0.0, 0.0, 0.5 * 3.14159265358979323846};
        hardware.localScale = {2.0, 3.0, 4.0};
        const auto result = generateRenderableTrack(straightSamples(), style);
        const glm::mat4 transform =
            result.hardwareBatches.front().instances.front().transform;
        requireNear(glm::dvec3{transform[0]}, {0.0, 2.0, 0.0}, 1.0e-6,
            "local rotation adjustment rotates scaled X");
        requireNear(glm::dvec3{transform[1]}, {-3.0, 0.0, 0.0}, 1.0e-6,
            "local rotation adjustment rotates scaled Y");
        require(std::abs(glm::length(glm::dvec3{transform[2]}) - 4.0)
                < 1.0e-6,
            "local Z scale is respected");
    }

    void invalidHardwareIsRejectedAndOutputIsFinite()
    {
        auto style = placementStyle();
        style.repeatingHardware.front().spacing = 0.0;
        bool spacingRejected = false;
        try { validateTrackStyle(style); }
        catch (const std::invalid_argument&) { spacingRejected = true; }
        require(spacingRejected, "zero hardware spacing is rejected");

        style = placementStyle();
        style.repeatingHardware.front().localPosition.x =
            std::numeric_limits<double>::quiet_NaN();
        bool offsetRejected = false;
        try { validateTrackStyle(style); }
        catch (const std::invalid_argument&) { offsetRejected = true; }
        require(offsetRejected, "non-finite hardware offset is rejected");

        style = placementStyle();
        style.repeatingHardware.front().localRotation.y =
            std::numeric_limits<double>::infinity();
        bool rotationRejected = false;
        try { validateTrackStyle(style); }
        catch (const std::invalid_argument&) { rotationRejected = true; }
        require(rotationRejected, "non-finite hardware rotation is rejected");

        style = placementStyle();
        style.repeatingHardware.front().localScale.z = 0.0;
        bool scaleRejected = false;
        try { validateTrackStyle(style); }
        catch (const std::invalid_argument&) { scaleRejected = true; }
        require(scaleRejected, "non-positive hardware scale is rejected");

        const auto result = generateRenderableTrack(
            straightSamples(), placementStyle());
        for (const HardwareInstance& instance
            : result.hardwareBatches.front().instances)
        {
            for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
            {
                require(std::isfinite(instance.transform[column][row]),
                    "generated hardware transform is finite");
            }
        }
    }
}

int main()
{
    try
    {
        placementIsDeterministicAndSpaced();
        placementFollowsCenterlineAndFrame();
        localRotationAndScaleAreApplied();
        invalidHardwareIsRejectedAndOutputIsFinite();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Track hardware placement test failure: "
            << error.what() << '\n';
        return 1;
    }
    std::cout << "Track hardware placement tests passed.\n";
    return 0;
}
