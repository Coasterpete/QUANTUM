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
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
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
        // Handles never reuse slots, so these bound total successful creations.
        constexpr std::size_t maximumConstraints = 128;
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
        struct ConstraintEntry
        {
            JPH::Ref<JPH::TwoBodyConstraint> constraint;
            JPH::BodyID connectedBody;
            JPH::BodyID body;
        };

        // PhysicsSystem borrows these tables, so it must be destroyed first.
        JPH::BroadPhaseLayerInterfaceTable broadPhase{layerCount, layerCount};
        JPH::ObjectLayerPairFilterTable layerPairs{layerCount};
        std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> broadPhaseFilter;
        JPH::TempAllocatorImpl temporaryAllocator{temporaryBytes};
        JPH::JobSystemSingleThreaded jobs{JPH::cMaxPhysicsJobs};
        JPH::PhysicsSystem system;
        std::vector<JPH::BodyID> bodies;
        std::vector<ConstraintEntry> constraints;
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
            constraints.reserve(maximumConstraints);
        }

        ~Impl()
        {
            // Constraints borrow their bodies. Drop system and local references
            // before destroying any body that a constraint can access.
            for (auto& entry : constraints)
            {
                if (entry.constraint != nullptr)
                {
                    system.RemoveConstraint(entry.constraint);
                    entry.constraint = nullptr;
                }
            }
            auto& interface = system.GetBodyInterface();
            for (const auto id : bodies)
            {
                if (!id.IsInvalid())
                {
                    interface.RemoveBody(id);
                    interface.DestroyBody(id);
                }
            }
        }

        [[nodiscard]] JPH::BodyID requireBody(const RigidBodyWorld* world,
            const RigidBodyHandle handle) const
        {
            if (handle.world != world || handle.index >= bodies.size()
                || bodies[handle.index].IsInvalid())
            {
                throw std::invalid_argument("Rigid-body handle is invalid, foreign or removed.");
            }
            return bodies[handle.index];
        }

        [[nodiscard]] const ConstraintEntry& requireConstraint(const RigidBodyWorld* world,
            const RigidBodyConstraintHandle handle) const
        {
            if (handle.world != world || handle.index >= constraints.size()
                || constraints[handle.index].constraint == nullptr)
            {
                throw std::invalid_argument("Rigid-body constraint handle is invalid, foreign or removed.");
            }
            return constraints[handle.index];
        }

        [[nodiscard]] JPH::HingeConstraint& requireHinge(const RigidBodyWorld* world,
            const RigidBodyConstraintHandle handle) const
        {
            const auto& entry = requireConstraint(world, handle);
            if (entry.constraint->GetSubType() != JPH::EConstraintSubType::Hinge)
            {
                throw std::invalid_argument("Rigid-body constraint is not a hinge.");
            }
            return static_cast<JPH::HingeConstraint&>(*entry.constraint);
        }

        [[nodiscard]] JPH::SliderConstraint& requireSlider(const RigidBodyWorld* world,
            const RigidBodyConstraintHandle handle) const
        {
            const auto& entry = requireConstraint(world, handle);
            if (entry.constraint->GetSubType() != JPH::EConstraintSubType::Slider)
            {
                throw std::invalid_argument("Rigid-body constraint is not a slider.");
            }
            return static_cast<JPH::SliderConstraint&>(*entry.constraint);
        }

        void removeConstraint(ConstraintEntry& entry)
        {
            // Removing a support must let a sleeping body respond to gravity.
            system.GetBodyInterface().ActivateConstraint(entry.constraint);
            system.RemoveConstraint(entry.constraint);
            entry.constraint = nullptr;
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
            throw std::runtime_error("RigidBodyWorld lifetime body creation capacity exceeded.");
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
        const auto id = impl_->requireBody(this, handle);
        const auto& interface = impl_->system.GetBodyInterface();
        const auto position = interface.GetPosition(id);
        const auto orientation = interface.GetRotation(id);
        return {
            {position.GetX(), position.GetY(), position.GetZ()},
            {orientation.GetW(), orientation.GetX(), orientation.GetY(), orientation.GetZ()},
            fromVector(interface.GetLinearVelocity(id)),
            interface.IsActive(id),
            fromVector(interface.GetAngularVelocity(id))
        };
    }

    void RigidBodyWorld::removeBody(const RigidBodyHandle handle)
    {
        const auto id = impl_->requireBody(this, handle);
        for (auto& entry : impl_->constraints)
        {
            if (entry.constraint != nullptr
                && (entry.body == id || entry.connectedBody == id))
            {
                impl_->removeConstraint(entry);
            }
        }
        auto& interface = impl_->system.GetBodyInterface();
        interface.RemoveBody(id);
        interface.DestroyBody(id);
        impl_->bodies[handle.index] = JPH::BodyID{};
    }

    RigidBodyConstraintHandle RigidBodyWorld::createHinge(
        const RigidBodyHingeSettings& settings)
    {
        const auto body = impl_->requireBody(this, settings.body);
        const auto connectedBody = settings.connectedBody
            ? impl_->requireBody(this, *settings.connectedBody) : JPH::BodyID{};
        auto& interface = impl_->system.GetBodyInterface();
        if (body == connectedBody)
        {
            throw std::invalid_argument("A hinge requires distinct bodies.");
        }
        if (interface.GetMotionType(body) != JPH::EMotionType::Dynamic
            && (connectedBody.IsInvalid()
                || interface.GetMotionType(connectedBody) != JPH::EMotionType::Dynamic))
        {
            throw std::invalid_argument("A hinge requires at least one dynamic body.");
        }
        const auto anchor = toVector(settings.anchorPositionMeters);
        (void)toVector(settings.axis);
        const double axisLength = glm::length(settings.axis);
        if (axisLength <= 0.0 || !std::isfinite(axisLength))
        {
            throw std::invalid_argument("Hinge axis must be finite and nonzero.");
        }
        const auto axis = toVector(settings.axis / axisLength).Normalized();
        // Jolt needs a perpendicular reference to define zero angle. Any one
        // suffices for an unlimited velocity hinge; keep it private here.
        const auto normal = axis.GetNormalizedPerpendicular();
        if (impl_->constraints.size() == maximumConstraints)
        {
            throw std::runtime_error("RigidBodyWorld lifetime constraint creation capacity exceeded.");
        }
        JPH::HingeConstraintSettings hingeSettings;
        hingeSettings.mPoint1 = hingeSettings.mPoint2 = JPH::RVec3{anchor};
        hingeSettings.mHingeAxis1 = hingeSettings.mHingeAxis2 = axis;
        hingeSettings.mNormalAxis1 = hingeSettings.mNormalAxis2 = normal;
        JPH::Ref<JPH::HingeConstraint> hinge = static_cast<JPH::HingeConstraint*>(
            interface.CreateConstraint(&hingeSettings, connectedBody, body));
        if (hinge == nullptr)
        {
            throw std::runtime_error("Rigid-body hinge creation failed.");
        }
        // The reserved vector cannot allocate after the system takes a reference.
        // The local Ref also releases the hinge if AddConstraint throws.
        impl_->system.AddConstraint(hinge);
        impl_->constraints.push_back({hinge.GetPtr(), connectedBody, body});
        interface.ActivateConstraint(hinge);
        return {this, impl_->constraints.size() - 1};
    }

    void RigidBodyWorld::setHingeMotor(const RigidBodyConstraintHandle handle,
        const RigidBodyHingeMotorSettings& settings)
    {
        auto& hinge = impl_->requireHinge(this, handle);
        const float velocity = toFloat(settings.targetAngularVelocityRadiansPerSecond);
        const float torque = toFloat(settings.maximumTorqueNewtonMeters);
        if (settings.maximumTorqueNewtonMeters < 0.0
            || (settings.maximumTorqueNewtonMeters > 0.0 && torque == 0.0f))
        {
            throw std::invalid_argument("Hinge motor torque limit must be zero or positive.");
        }
        hinge.GetMotorSettings().SetTorqueLimit(torque);
        hinge.SetTargetAngularVelocity(velocity);
        hinge.SetMotorState(settings.enabled
            ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        impl_->system.GetBodyInterface().ActivateConstraint(&hinge);
    }

    RigidBodyConstraintHandle RigidBodyWorld::createSlider(
        const RigidBodySliderSettings& settings)
    {
        const auto body = impl_->requireBody(this, settings.body);
        const auto connectedBody = settings.connectedBody
            ? impl_->requireBody(this, *settings.connectedBody) : JPH::BodyID{};
        auto& interface = impl_->system.GetBodyInterface();
        if (body == connectedBody)
        {
            throw std::invalid_argument("A slider requires distinct bodies.");
        }
        if (interface.GetMotionType(body) != JPH::EMotionType::Dynamic
            && (connectedBody.IsInvalid()
                || interface.GetMotionType(connectedBody) != JPH::EMotionType::Dynamic))
        {
            throw std::invalid_argument("A slider requires at least one dynamic body.");
        }
        const auto anchor = toVector(settings.anchorPositionMeters);
        (void)toVector(settings.axis);
        // hypot avoids overflow/underflow when normalizing a finite direction.
        const double axisLength = std::hypot(settings.axis.x, settings.axis.y, settings.axis.z);
        if (axisLength <= 0.0 || !std::isfinite(axisLength))
        {
            throw std::invalid_argument("Slider axis must be finite and nonzero.");
        }
        const auto axis = toVector(settings.axis / axisLength).Normalized();
        const float minimum = settings.minimumTranslationMeters
            ? toFloat(*settings.minimumTranslationMeters) : -std::numeric_limits<float>::max();
        const float maximum = settings.maximumTranslationMeters
            ? toFloat(*settings.maximumTranslationMeters) : std::numeric_limits<float>::max();
        // Jolt's hard slider limits require zero within the range. Coincident
        // attachment points make the creation pose that zero reference.
        if (minimum > 0.0f || maximum < 0.0f || minimum >= maximum
            || (settings.minimumTranslationMeters
                && *settings.minimumTranslationMeters != 0.0 && minimum == 0.0f)
            || (settings.maximumTranslationMeters
                && *settings.maximumTranslationMeters != 0.0 && maximum == 0.0f))
        {
            throw std::invalid_argument(
                "Slider limits must include zero and have representable nonzero travel.");
        }
        if (impl_->constraints.size() == maximumConstraints)
        {
            throw std::runtime_error("RigidBodyWorld lifetime constraint creation capacity exceeded.");
        }
        JPH::SliderConstraintSettings sliderSettings;
        sliderSettings.mPoint1 = sliderSettings.mPoint2 = JPH::RVec3{anchor};
        sliderSettings.SetSliderAxis(axis);
        sliderSettings.mLimitsMin = minimum;
        sliderSettings.mLimitsMax = maximum;
        JPH::Ref<JPH::SliderConstraint> slider = static_cast<JPH::SliderConstraint*>(
            interface.CreateConstraint(&sliderSettings, connectedBody, body));
        if (slider == nullptr)
        {
            throw std::runtime_error("Rigid-body slider creation failed.");
        }
        // Retain before registration; reserved storage cannot allocate afterward.
        impl_->system.AddConstraint(slider);
        impl_->constraints.push_back({slider.GetPtr(), connectedBody, body});
        interface.ActivateConstraint(slider);
        return {this, impl_->constraints.size() - 1};
    }

    void RigidBodyWorld::setSliderMotor(const RigidBodyConstraintHandle handle,
        const RigidBodySliderMotorSettings& settings)
    {
        auto& slider = impl_->requireSlider(this, handle);
        const float velocity = toFloat(settings.targetLinearVelocityMetersPerSecond);
        const float force = toFloat(settings.maximumForceNewtons);
        if (settings.maximumForceNewtons < 0.0
            || (settings.maximumForceNewtons > 0.0 && force == 0.0f))
        {
            throw std::invalid_argument("Slider motor force limit must be zero or positive.");
        }
        slider.GetMotorSettings().SetForceLimit(force);
        slider.SetTargetVelocity(velocity);
        slider.SetMotorState(settings.enabled
            ? JPH::EMotorState::Velocity : JPH::EMotorState::Off);
        impl_->system.GetBodyInterface().ActivateConstraint(&slider);
    }

    RigidBodySliderState RigidBodyWorld::sliderState(
        const RigidBodyConstraintHandle handle) const
    {
        const auto& slider = impl_->requireSlider(this, handle);
        const auto& referenceBody = *slider.GetBody1();
        const auto& body = *slider.GetBody2();
        const auto axis = referenceBody.GetRotation()
            * slider.GetConstraintToBody1Matrix().GetAxisX();
        const auto attachment = body.GetCenterOfMassTransform()
            * slider.GetConstraintToBody2Matrix().GetTranslation();
        // Evaluate both point velocities at body 2's attachment. This includes
        // rotation of the reference frame and matches Jolt's slider motor axis.
        const auto relativeVelocity = body.GetPointVelocity(attachment)
            - referenceBody.GetPointVelocity(attachment);
        return {slider.GetCurrentPosition(), relativeVelocity.Dot(axis)};
    }

    void RigidBodyWorld::removeConstraint(const RigidBodyConstraintHandle handle)
    {
        (void)impl_->requireConstraint(this, handle);
        impl_->removeConstraint(impl_->constraints[handle.index]);
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
