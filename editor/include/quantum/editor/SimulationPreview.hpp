#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/physics/TrainPhysics.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace quantum::editor
{
    // Editor-owned integration state for one temporary diagnostic train.
    // Core remains authoritative for track compilation, dynamics, car spacing,
    // and every car/bogie pose.
    class SimulationPreview
    {
    public:
        enum class PlaybackState : std::uint8_t
        {
            Stopped,
            Playing,
            Paused
        };

        // Replaces every track-dependent value. A failed rebuild leaves the
        // preview unavailable and retains only a user-facing error message.
        [[nodiscard]] bool rebuild(
            const coaster::AuthoredTrack& authoredTrack) noexcept;

        void play() noexcept;
        void pause() noexcept;
        void reset() noexcept;
        void update(double frameDeltaSeconds) noexcept;

        [[nodiscard]] bool isAvailable() const noexcept;
        [[nodiscard]] PlaybackState playbackState() const noexcept;
        [[nodiscard]] double speedMetersPerSecond() const noexcept;
        [[nodiscard]] const std::string& error() const noexcept;
        [[nodiscard]] std::span<const renderer::LineVertex> vertices()
            const noexcept;
        [[nodiscard]] std::uint64_t vertexGeneration() const noexcept;

        // Exposed for focused non-render tests and compact UI telemetry.
        [[nodiscard]] const physics::TrainDefinition& trainDefinition()
            const noexcept;
        [[nodiscard]] const physics::TrainDynamicsState* dynamicsState()
            const noexcept;
        [[nodiscard]] const physics::TrainPose* pose() const noexcept;

        static constexpr std::size_t maximumStepsPerFrame = 60;

    private:
        void setUnavailable(std::string error) noexcept;
        void rebuildVertices();

        std::optional<physics::CompiledPhysicsTrack> compiledTrack_;
        physics::PhysicsEnvironment environment_;
        physics::TrainDefinition trainDefinition_;
        std::optional<physics::TrainDynamicsState> initialState_;
        std::optional<physics::TrainDynamicsState> dynamicsState_;
        std::optional<physics::TrainPose> initialPose_;
        std::optional<physics::TrainPose> pose_;
        std::vector<renderer::LineVertex> vertices_;
        PlaybackState playbackState_ = PlaybackState::Stopped;
        double accumulatorSeconds_ = 0.0;
        double coordinateUnitsPerMeter_ = 1.0;
        std::uint64_t vertexGeneration_ = 0;
        std::string error_;
    };
}
