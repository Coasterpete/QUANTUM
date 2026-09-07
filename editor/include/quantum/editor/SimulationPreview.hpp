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
    struct SimulationPreviewFrameTelemetry
    {
        double rawDeltaMilliseconds = 0.0;
        double accumulatorBeforeMilliseconds = 0.0;
        double accumulatorAfterIncomingMilliseconds = 0.0;
        double accumulatorRemainingMilliseconds = 0.0;
        double discardedWallTimeMilliseconds = 0.0;
        std::size_t requestedStepCount = 0;
        std::size_t fixedStepCount = 0;
        bool maximumStepsHit = false;
        bool boundaryStopped = false;
        double minimumStepMilliseconds = 0.0;
        double averageStepMilliseconds = 0.0;
        double maximumStepMilliseconds = 0.0;
        double physicsMilliseconds = 0.0;
        std::size_t consecutiveCatchUpFrameCount = 0;
        double interpolationMilliseconds = 0.0;
        double renderPoseSolveMilliseconds = 0.0;
        std::size_t renderPoseSolveCount = 0;
        std::size_t renderPoseFailureCount = 0;
        double vertexPreparationMilliseconds = 0.0;
        physics::TrainSolveCounters solveCounters;
    };

    // Presentation only: alpha=0 is previous, alpha=1 is current. Between
    // adjacent ticks, use the nearest circuit displacement and retain physical
    // orientation, including at zero speed and during backward travel.
    [[nodiscard]] physics::TrainPose interpolateTrainPreviewPose(
        const physics::CompiledPhysicsTrack& track,
        const physics::TrainDefinition& train,
        const physics::TrainPose& previous,
        const physics::TrainPose& current,
        double alpha,
        SimulationPreviewFrameTelemetry* telemetry = nullptr);

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
        // Clears counters accumulated by preview work for the next rendered
        // frame. This does not alter simulation state.
        void beginFrameTelemetry() noexcept;
        // A known host interruption is not preview playback time. Keep the
        // fractional tick and committed/render poses when discarding it.
        void update(double frameDeltaSeconds,
            bool timingDiscontinuity = false) noexcept;

        [[nodiscard]] bool isAvailable() const noexcept;
        [[nodiscard]] PlaybackState playbackState() const noexcept;
        [[nodiscard]] double speedMetersPerSecond() const noexcept;
        [[nodiscard]] const std::string& error() const noexcept;
        [[nodiscard]] std::span<const renderer::LineVertex> vertices()
            const noexcept;
        [[nodiscard]] std::uint64_t vertexGeneration() const noexcept;
        [[nodiscard]] const SimulationPreviewFrameTelemetry& frameTelemetry()
            const noexcept;

        // Exposed for focused non-render tests and compact UI telemetry.
        [[nodiscard]] const physics::TrainDefinition& trainDefinition()
            const noexcept;
        [[nodiscard]] const physics::TrainDynamicsState* dynamicsState()
            const noexcept;
        [[nodiscard]] const physics::TrainPose* pose() const noexcept;
        [[nodiscard]] const physics::TrainPose* renderPose() const noexcept;
        [[nodiscard]] double interpolationAlpha() const noexcept;

        static constexpr std::size_t maximumStepsPerFrame = 60;
        // Eight steps is 33.3 ms of demand at 240 Hz and is deliberately
        // diagnostic only; it never changes accumulator or playback behavior.
        static constexpr std::size_t catchUpStepThreshold = 8;

    private:
        void setUnavailable(std::string error) noexcept;
        void rebuildVertices();

        std::optional<physics::CompiledPhysicsTrack> compiledTrack_;
        physics::PhysicsEnvironment environment_;
        physics::TrainDefinition trainDefinition_;
        std::optional<physics::TrainDynamicsState> initialState_;
        std::optional<physics::TrainDynamicsState> dynamicsState_;
        std::optional<physics::TrainDynamicsState> previousState_;
        std::optional<physics::TrainPose> initialPose_;
        std::optional<physics::TrainPose> pose_;
        std::optional<physics::TrainPose> previousPose_;
        std::optional<physics::TrainPose> renderPose_;
        std::vector<renderer::LineVertex> vertices_;
        PlaybackState playbackState_ = PlaybackState::Stopped;
        double accumulatorSeconds_ = 0.0;
        double renderAlpha_ = 0.0;
        double coordinateUnitsPerMeter_ = 1.0;
        std::uint64_t vertexGeneration_ = 0;
        SimulationPreviewFrameTelemetry frameTelemetry_;
        std::size_t consecutiveCatchUpFrameCount_ = 0;
        std::string error_;
    };
}
