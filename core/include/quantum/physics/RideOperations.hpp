#pragma once
#include <quantum/physics/TrackDeviceForces.hpp>
#include <string>
#include <variant>

namespace quantum::physics
{
    using OperationTrainId = std::uint64_t;
    struct BogieObservation { std::size_t car = 0, bogie = 0; double stationMeters = 0; };
    struct TrainObservation
    {
        OperationTrainId id = 1;
        double referenceStationMeters = 0, signedSpeedMetersPerSecond = 0;
        std::vector<BogieObservation> bogies;
    };
    [[nodiscard]] TrainObservation observeTrain(const TrainPose& pose,
        double signedSpeedMetersPerSecond, OperationTrainId id = 1);
    struct OccupiedBogie { OperationTrainId train; std::size_t car, bogie; };
    struct OccupyingTrain
    {
        OperationTrainId id;
        std::size_t bogieCount, carCount;
        bool complete;
    };
    struct HardwareOccupancy
    {
        std::vector<OccupiedBogie> bogies;
        std::vector<OccupyingTrain> trains;
        std::vector<OperationTrainId> entered, exited;
        [[nodiscard]] bool clear() const noexcept { return trains.empty(); }
    };
    struct ChainRuntime { enum class Phase { Available, Engaging, Pulling, Releasing }; Phase phase = Phase::Available; bool available = true; };
    struct CableRuntime
    {
        enum class Phase { Available, Engaging, Pulling, Releasing, Returning, Unavailable };
        Phase phase = Phase::Available;
        double returnRemainingSeconds = 0;
        bool available = true;
    };
    struct StationRuntime { bool dispatchRequested = false, dispatchPermitted = false, departing = false; OperationTrainId train = 0; };
    struct TransferRuntime
    {
        enum class Phase { Locked, Unlocking, Moving, Locking, Interlocked };
        Phase phase = Phase::Locked;
        std::uint64_t alignment = 0, target = 0;
        double positionMeters = 0;
        bool moveRequested = false;
    };
    struct AccessRuntime
    {
        enum class Phase { Closed, Opening, Open, Closing };
        Phase phase = Phase::Closed;
        double fraction = 0;
        bool locked = true, interlocked = false, protectedByOperation = false;
    };
    using HardwareMechanismState = std::variant<std::monostate, ChainRuntime, CableRuntime,
        StationRuntime, TransferRuntime, AccessRuntime>;
    struct HardwareRuntime
    {
        coaster::TrackDeviceId id = 0;
        bool enabled = true;
        HardwareOccupancy occupancy;
        HardwareMechanismState mechanism;
        std::string state = "Ready", interlockReason;
        double commandedSpeedMetersPerSecond = 0;
        double commandedAccelerationMetersPerSecondSquared = 0;
        double actualSpeedMetersPerSecond = 0;
        double appliedForceNewtons = 0;
        double profileAccelerationMetersPerSecondSquared = 0;
        bool targetSpeedLimited = false;
        bool aligned = false, stopped = false, movementPermitted = false;
        TrackDeviceRuntimeState forceControl;
    };
    enum class HardwareCommand { SetEnabled, SetAvailable, RequestDispatch, MoveTransfer,
        OpenAccess, CloseAccess, LockAccess, UnlockAccess, SetAccessInterlocked };

    // Simulation owns this value state. Commands address stable document IDs;
    // no document/history mutation and no renderer or general-physics backend.
    class RideOperations
    {
    public:
        void reset(std::span<const coaster::TrackDevice> devices);
        void step(std::span<const coaster::TrackDevice> devices,
            std::span<const TrainObservation> trains, double lengthMeters, bool circuit,
            double seconds = defaultFixedTimeStepSeconds);
        void command(coaster::TrackDeviceId id, HardwareCommand command, double value = 0, std::uint64_t target = 0);
        void recordForces(const TrackDeviceForceResult& forces);
        [[nodiscard]] const std::vector<HardwareRuntime>& hardware() const noexcept { return hardware_; }
        [[nodiscard]] std::vector<TrackDeviceRuntimeState> forceControls() const;
    private:
        [[nodiscard]] HardwareRuntime& find(coaster::TrackDeviceId id);
        std::vector<HardwareRuntime> hardware_;
        // Only configuration needed to validate commands between fixed steps.
        std::vector<coaster::TrackDevice> authored_;
    };
}
