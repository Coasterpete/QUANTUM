#pragma once
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/physics/RideOperations.hpp>
#include <quantum/coaster/SupportSolidGeometry.hpp>
#include <span>

namespace quantum::editor
{
    struct InfrastructureSolid
    {
        std::uint64_t id = 0;
        bool access = false;
        coaster::SupportSolidPresentation solid;
    };
    [[nodiscard]] std::vector<InfrastructureSolid> createInfrastructurePresentation(
        const coaster::AuthoredTrack& track, std::span<const coaster::RiderLocalGeometryState> states);
    [[nodiscard]] std::vector<InfrastructureSolid> createAccessPresentation(
        const coaster::AuthoredTrack& track, std::span<const coaster::RiderLocalGeometryState> states,
        std::span<const physics::HardwareRuntime> runtime = {});
    struct InfrastructureHit { std::uint64_t id; bool access; double distance; };
    [[nodiscard]] std::optional<InfrastructureHit> pickInfrastructureSolid(
        std::span<const InfrastructureSolid> objects, const glm::dvec3& origin,
        const glm::dvec3& direction);
}
