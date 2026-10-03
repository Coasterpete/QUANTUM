#include <quantum/physics/RigidBodyWorld.hpp>
#include <quantum/physics/TrackFollower.hpp>

// Jolt requires this header before every other Jolt header.
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace quantum::physics
{
    namespace
    {
        constexpr JPH::ObjectLayer staticLayer = 0;
        constexpr JPH::ObjectLayer dynamicLayer = 1;
        constexpr JPH::uint layerCount = 2;
        // Deliberately bounded M0 capacities, not production scene limits.
        constexpr JPH::uint maximumBodies = 1024;
        constexpr JPH::uint maximumBodyPairs = 1024;
        constexpr JPH::uint maximumContacts = 1024;
        constexpr JPH::uint temporaryBytes = 10 * 1024 * 1024;

        // Jolt's allocator, factory and collision registration are process-wide.
        // Initialize before any world members allocate; keep registration alive
        // across simultaneous worlds and release it after local worlds die.
        class JoltRuntime
        {
        public:
            JoltRuntime()
            {
                if (JPH::Factory::sInstance != nullptr)
                {
                    throw std::logic_error(
                        "RigidBodyWorld requires ownership of Jolt registration.");
                }
                JPH::RegisterDefaultAllocator();
                factory_ = std::make_unique<JPH::Factory>();
                JPH::Factory::sInstance = factory_.get();
                JPH::RegisterTypes();
            }

            ~JoltRuntime()
            {
                JPH::UnregisterTypes();
                JPH::Factory::sInstance = nullptr;
            }

        private:
            std::unique_ptr<JPH::Factory> factory_;
        };

        void initializeJolt()
        {
            static JoltRuntime runtime;
        }

        [[nodiscard]] float toFloat(const double value)
        {
            if (!std::isfinite(value)
                || std::abs(value) > std::numeric_limits<float>::max())
            {
                throw std::invalid_argument(
                    "Rigid-body input must be finite and fit single precision.");
            }
            return static_cast<float>(value);
        }

        [[nodiscard]] JPH::Vec3 toVector(const glm::dvec3& value)
        {
            return {toFloat(value.x), toFloat(value.y), toFloat(value.z)};
        }

        [[nodiscard]] glm::dvec3 fromVector(const JPH::Vec3& value)
        {
            return {value.GetX(), value.GetY(), value.GetZ()};
        }
    }

    struct RigidBodyWorld::Impl
    {
        // PhysicsSystem borrows these tables, so it must be destroyed first.
        JPH::BroadPhaseLayerInterfaceTable broadPhase{layerCount, layerCount};
        JPH::ObjectLayerPairFilterTable layerPairs{layerCount};
        std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> broadPhaseFilter;
        JPH::TempAllocatorImpl temporaryAllocator{temporaryBytes};
        JPH::JobSystemSingleThreaded jobs{JPH::cMaxPhysicsJobs};
        JPH::PhysicsSystem system;
        std::vector<JPH::BodyID> bodies;
        std::uint64_t tick = 0;

        Impl()
        {
            broadPhase.MapObjectToBroadPhaseLayer(
                staticLayer, JPH::BroadPhaseLayer{0});
            broadPhase.MapObjectToBroadPhaseLayer(
                dynamicLayer, JPH::BroadPhaseLayer{1});
            layerPairs.EnableCollision(staticLayer, dynamicLayer);
            layerPairs.EnableCollision(dynamicLayer, dynamicLayer);
            broadPhaseFilter =
                std::make_unique<JPH::ObjectVsBroadPhaseLayerFilterTable>(
                    broadPhase, layerCount, layerPairs, layerCount);
            system.Init(maximumBodies, 0, maximumBodyPairs, maximumContacts,
                broadPhase, *broadPhaseFilter, layerPairs);
            system.SetGravity({0.0f, 0.0f,
                -static_cast<float>(coaster::standardGravityAcceleration)});
            // No vector allocation can fail after a library body is created.
            bodies.reserve(maximumBodies);
        }

        ~Impl()
        {
            auto& interface = system.GetBodyInterface();
            for (const auto id : bodies)
            {
                interface.RemoveBody(id);
                interface.DestroyBody(id);
            }
        }
    };

    RigidBodyWorld::RigidBodyWorld()
    {
        initializeJolt();
        impl_ = std::make_unique<Impl>();
    }

    RigidBodyWorld::~RigidBodyWorld() = default;

    RigidBodyHandle RigidBodyWorld::createBox(
        const RigidBodyBoxSettings& settings)
    {
        const auto halfExtents = toVector(settings.halfExtentsMeters);
        if (halfExtents.GetX() <= 0.0f || halfExtents.GetY() <= 0.0f
            || halfExtents.GetZ() <= 0.0f)
        {
            throw std::invalid_argument("Rigid-body box half extents must be positive.");
        }
        const auto position = toVector(settings.positionMeters);
        const float mass = toFloat(settings.massKilograms);
        if (settings.massKilograms < 0.0
            || (settings.massKilograms > 0.0 && mass == 0.0f))
        {
            throw std::invalid_argument("Rigid-body mass must be zero or positive.");
        }
        // GLM constructs (w,x,y,z); Jolt constructs (x,y,z,w). No axis swap.
        JPH::Quat orientation{toFloat(settings.orientation.x),
            toFloat(settings.orientation.y), toFloat(settings.orientation.z),
            toFloat(settings.orientation.w)};
        const float quaternionLength = orientation.Length();
        if (!std::isfinite(quaternionLength) || quaternionLength <= 0.0f)
        {
            throw std::invalid_argument("Rigid-body orientation must be nonzero and finite.");
        }
        orientation = orientation.Normalized();
        if (impl_->bodies.size() == maximumBodies)
        {
            throw std::runtime_error("RigidBodyWorld M0 body capacity exceeded.");
        }

        // Zero convex radius keeps the proof's collider dimensions exact.
        const auto shape = JPH::BoxShapeSettings{halfExtents, 0.0f}.Create();
        if (shape.HasError())
        {
            throw std::runtime_error("Rigid-body box shape creation failed: "
                + std::string(shape.GetError().c_str()));
        }
        const bool dynamic = mass > 0.0f;
        JPH::BodyCreationSettings bodySettings{shape.Get(),
            JPH::RVec3{position}, orientation,
            dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
            dynamic ? dynamicLayer : staticLayer};
        if (dynamic)
        {
            bodySettings.mOverrideMassProperties =
                JPH::EOverrideMassProperties::CalculateInertia;
            bodySettings.mMassPropertiesOverride.mMass = mass;
        }
        const auto id = impl_->system.GetBodyInterface().CreateAndAddBody(
            bodySettings,
            dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
        if (id.IsInvalid())
        {
            throw std::runtime_error("Rigid-body box creation failed.");
        }
        impl_->bodies.push_back(id);
        return {this, impl_->bodies.size() - 1};
    }

    RigidBodyState RigidBodyWorld::bodyState(const RigidBodyHandle handle) const
    {
        if (handle.world != this || handle.index >= impl_->bodies.size())
        {
            throw std::invalid_argument("Rigid-body handle does not belong to this world.");
        }
        const auto id = impl_->bodies[handle.index];
        const auto& interface = impl_->system.GetBodyInterface();
        const auto position = interface.GetPosition(id);
        const auto orientation = interface.GetRotation(id);
        return {
            {position.GetX(), position.GetY(), position.GetZ()},
            {orientation.GetW(), orientation.GetX(), orientation.GetY(), orientation.GetZ()},
            fromVector(interface.GetLinearVelocity(id)),
            interface.IsActive(id)
        };
    }

    void RigidBodyWorld::stepFixed()
    {
        // The preview already accumulates wall time. One collision step here
        // advances exactly one host tick, without another accumulator/substep.
        const auto error = impl_->system.Update(
            static_cast<float>(defaultFixedTimeStepSeconds), 1,
            &impl_->temporaryAllocator, &impl_->jobs);
        if (error != JPH::EPhysicsUpdateError::None)
        {
            throw std::runtime_error("RigidBodyWorld update failed (Jolt error "
                + std::to_string(static_cast<unsigned>(error)) + ").");
        }
        ++impl_->tick;
    }

    std::uint64_t RigidBodyWorld::tick() const noexcept
    {
        return impl_->tick;
    }
}
