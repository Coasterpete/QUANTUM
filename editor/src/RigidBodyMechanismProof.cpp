#include <quantum/editor/RigidBodyMechanismProof.hpp>

#include <glm/gtc/matrix_transform.hpp>

namespace quantum::editor
{
    renderer::StaticMeshInstance RigidBodyMeshBinding::instance(
        const physics::RigidBodyWorld& world) const
    {
        const auto state = world.bodyState(body);
        const auto bodyTransform = glm::translate(glm::dmat4{1.0}, state.positionMeters)
            * glm::mat4_cast(state.orientation);
        return {assetIdentifier, glm::mat4{bodyTransform * localAssetTransform}};
    }

    RigidBodyMechanismProof::RigidBodyMechanismProof()
    {
        // Support sits behind the arm's X/Z rotation plane, allowing a full
        // revolution without collisions between the two box colliders.
        physics::RigidBodyBoxSettings box;
        box.positionMeters = {0.0, -11.35, 6.0};
        box.halfExtentsMeters = {0.4, 0.3, 0.4};
        box.massKilograms = 0.0;
        halfExtents_[0] = box.halfExtentsMeters;
        bodies_[0] = world_.createBox(box);

        box.positionMeters = {2.0, -12.0, 6.0};
        box.halfExtentsMeters = {2.0, 0.22, 0.22};
        box.massKilograms = 4.0;
        halfExtents_[1] = box.halfExtentsMeters;
        bodies_[1] = world_.createBox(box);

        // The asset origin is its pivot end. The box body origin is its center,
        // two meters along +X from the hinge. Keep that physics setup intact.
        meshBindings_[0] = {bodies_[1], std::string{mechanicalArmAssetId},
            glm::translate(glm::dmat4{1.0}, glm::dvec3{-2.0, 0.0, 0.0})};

        // Keep the center of mass directly below the shared hinge. The proof's
        // isolated world excludes contact between these connected box colliders.
        box.positionMeters = {4.0, -12.0, 5.0};
        box.halfExtentsMeters = {0.65, 0.28, 1.0};
        box.massKilograms = 1.0;
        halfExtents_[2] = box.halfExtentsMeters;
        bodies_[2] = world_.createBox(box);
        // Asset origin is the upper hinge, not the carrier's center of mass.
        meshBindings_[1] = {bodies_[2], std::string{mechanicalGondolaAssetId},
            glm::translate(glm::dmat4{1.0}, glm::dvec3{0.0, 0.0, 1.0})};

        physics::RigidBodyHingeSettings joint;
        joint.body = bodies_[1];
        joint.anchorPositionMeters = {0.0, -12.0, 6.0};
        joint.axis = {0.0, 1.0, 0.0};
        const auto hinge = world_.createHinge(joint);
        world_.setHingeMotor(hinge, {true, 1.5, 200.0});

        joint.body = bodies_[2];
        joint.connectedBody = bodies_[1];
        joint.anchorPositionMeters = {4.0, -12.0, 6.0};
        // A new hinge defaults to motor Off: gravity/inertia drive the carrier.
        (void)world_.createHinge(joint);
    }

    std::array<RigidBodyProofBox, 3> RigidBodyMechanismProof::snapshot() const
    {
        constexpr std::array roles{RigidBodyProofRole::Support,
            RigidBodyProofRole::DrivenArm, RigidBodyProofRole::PassiveGondola};
        std::array<RigidBodyProofBox, 3> boxes;
        for (std::size_t index = 0; index < bodies_.size(); ++index)
        {
            const auto state = world_.bodyState(bodies_[index]);
            boxes[index] = {state.positionMeters, state.orientation,
                halfExtents_[index], roles[index]};
        }
        return boxes;
    }

    double RigidBodyMechanismProof::armAngularSpeedRadiansPerSecond() const
    {
        return world_.bodyState(bodies_[1]).angularVelocityRadiansPerSecond.y;
    }

    renderer::StaticMeshInstance RigidBodyMechanismProof::armMeshInstance() const
    {
        return meshBindings_[0].instance(world_);
    }

    std::array<renderer::StaticMeshInstance, 2> RigidBodyMechanismProof::meshInstances() const
    {
        std::array<renderer::StaticMeshInstance, 2> instances;
        for (std::size_t index = 0; index < meshBindings_.size(); ++index)
            instances[index] = meshBindings_[index].instance(world_);
        return instances;
    }

    void appendRigidBodyProofVertices(std::vector<renderer::LineVertex>& vertices,
        const std::span<const RigidBodyProofBox> boxes)
    {
        // Fixed topology, centered unit-box corners. Scaling by half extents
        // makes the edge-to-edge dimensions exactly twice the collider extents.
        constexpr std::array<glm::dvec3, 8> corners{{
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
        }};
        constexpr std::array<std::array<std::size_t, 2>, 12> edges{{
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        }};
        vertices.reserve(vertices.size() + boxes.size() * edges.size() * 2);
        for (const auto& box : boxes)
        {
            const std::array<float, 4> color = box.role == RigidBodyProofRole::Support
                ? std::array<float, 4>{0.75F, 0.78F, 0.82F, 1.0F}
                : box.role == RigidBodyProofRole::DrivenArm
                    ? std::array<float, 4>{1.0F, 0.48F, 0.08F, 1.0F}
                    : std::array<float, 4>{0.08F, 0.75F, 1.0F, 1.0F};
            std::array<glm::dvec3, 8> worldCorners;
            for (std::size_t index = 0; index < corners.size(); ++index)
            {
                worldCorners[index] = box.positionMeters
                    + box.orientation * (corners[index] * box.halfExtentsMeters);
            }
            for (const auto& edge : edges)
            {
                for (const auto index : edge)
                {
                    const auto& point = worldCorners[index];
                    vertices.push_back({static_cast<float>(point.x),
                        static_cast<float>(point.y), static_cast<float>(point.z), color});
                }
            }
        }
    }
}
