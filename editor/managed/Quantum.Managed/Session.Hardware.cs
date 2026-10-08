using System.Collections.ObjectModel;

using System.Text.Json;



namespace Quantum.Managed;

public enum HardwareCommand : uint { SetEnabled, SetAvailable, RequestDispatch, MoveTransfer, OpenAccess, CloseAccess, LockAccess, UnlockAccess, SetAccessInterlocked }

public sealed record LaunchSettings(double? TargetSpeedMetersPerSecond = null, int Direction = 1);

public sealed record BrakeSettings(double TargetSpeedMetersPerSecond = 0, bool Holding = false);

public sealed record TransportSettings(double SpeedMetersPerSecond = 2, int Direction = 1);

public sealed record ChainSettings(double SpeedMetersPerSecond = 3.5, bool AntiRollback = true);

public sealed record CableSettings(double SpeedMetersPerSecond = 5, double ReturnSeconds = 8, bool InitiallyAvailable = true);

public sealed record StationSettings(double StopStationMeters, double AlignmentToleranceMeters = .15,

    double StoppedToleranceMetersPerSecond = .05, double TransportSpeedMetersPerSecond = 2,

    ulong DownstreamBlock = 0, ulong[]? AccessGates = null, bool RequireLockedGates = true);

public sealed record AlignmentSettings(ulong Id, string Name, double LateralOffsetMeters, uint Destination)

{

    public override string ToString() => $"{Name} · #{Id} · {LateralOffsetMeters:F2} m";

}

public sealed record TransferSettings(AlignmentSettings[] Alignments, ulong HomeAlignment = 1,

    double MovementSpeedMetersPerSecond = .5, double StoppedToleranceMetersPerSecond = .05, ulong[]? AccessGates = null, ulong NextAlignmentId = 3);

public sealed record AccessSettings(uint Role = 0, uint Motion = 0, int TravelDirection = 1,

    double OpenAngleDegrees = 90, double TravelDistanceMeters = 1, double MovementSeconds = 1,

    double LateralOffsetMeters = 2, double VerticalOffsetMeters = 0, double YawDegrees = 0,

    bool InitiallyOpen = false, bool InitiallyLocked = true, bool InitiallyInterlocked = false, string MeshAsset = "", uint Infill = 0, double WidthMeters = 1, double HeightMeters = 1.1, SupportSurfaceSettings? Surface = null);

public sealed record RailingSettings(bool Inner = false, bool Outer = true, double HeightMeters = 1.1, double PostSpacingMeters = 1.5);

public sealed record InfrastructureSettings(ulong Id, string Name, uint Kind,

    double StartStationMeters, double EndStationMeters, int Side = 1,

    double LateralOffsetMeters = 1.5, double VerticalOffsetMeters = 0, double WidthMeters = .8,

    uint Deck = 1, RailingSettings? Railings = null, ulong[]? AccessGates = null, string MeshAsset = "",
    SupportSurfaceSettings? Surface = null, bool WorldPlacement = false,
    double[]? WorldStartMeters = null, double[]? WorldEndMeters = null, double StepRiseMeters = .18)
{
    public override string ToString() => $"{Name} · #{Id}";
}

public sealed record HardwareEntry(ulong Id, JsonElement Configuration, string State = "Preview unavailable",

    string InterlockReason = "", bool RuntimeEnabled = false, uint OccupiedBogieCount = 0, uint OccupiedCarCount = 0,

    ulong[]? TrainIds = null, ulong[]? Entered = null, ulong[]? Exited = null,

    double CommandedSpeed = 0, double CommandedAcceleration = 0, double ActualSpeed = 0, double AppliedForce = 0,

    bool Aligned = false, bool Stopped = false, bool MovementPermitted = false,

    bool DispatchRequested = false, bool DispatchPermitted = false, ulong Alignment = 0, ulong TargetAlignment = 0,

    double Position = 0, bool Locked = false, bool Interlocked = false, int Direction = 0,
    double ProfileAcceleration = 0, bool TargetSpeedLimited = false)

{

    public T Settings<T>(T fallback) => Configuration.ValueKind == JsonValueKind.Null ? fallback

        : Configuration.Deserialize<T>(Session.HardwareJson) ?? fallback;

    public string Telemetry => $"{State} · {OccupiedBogieCount} bogies / {OccupiedCarCount} cars · {ActualSpeed:F2} m/s · {(Direction == 0 ? "stopped" : Direction > 0 ? "forward" : "reverse")}\n"

        + $"Command {CommandedSpeed:F2} m/s / {CommandedAcceleration:F2} m/s² · force {AppliedForce:F0} N\n"

        + (InterlockReason.Length == 0 ? "" : InterlockReason);

}

public sealed record HardwareSnapshot(ulong Revision, HardwareEntry[] Hardware, ulong NextInfrastructureId, InfrastructureSettings[] Infrastructure, ulong SelectedInfrastructure = 0, ulong SelectedAccess = 0);

public sealed unsafe partial class Session

{

    internal static readonly JsonSerializerOptions HardwareJson = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, PropertyNameCaseInsensitive = true };

    private byte[] hardwareBuffer = new byte[4096];

    private HardwareSnapshot ReadHardware(ulong revision)

    {

        for (int attempt = 0; attempt < 3; ++attempt)

        {

            int result; uint required; NativeError error;

            fixed (byte* buffer = hardwareBuffer)

                result = Native.qn_hardware_snapshot(handle, revision, buffer, (uint)hardwareBuffer.Length, out required, out error);

            if (result == 3) { hardwareBuffer = new byte[checked((int)required)]; continue; }

            Check(result, error);

            return JsonSerializer.Deserialize<HardwareSnapshot>(hardwareBuffer.AsSpan(0, checked((int)required - 1)), HardwareJson)

                ?? throw new InvalidOperationException("Native hardware snapshot is empty.");

        }

        throw new InvalidOperationException("Hardware snapshot changed during sizing.");

    }

    public void SetHardwareConfiguration<T>(ulong revision, ulong id, T configuration)

    {

        RequireOpen();

        Check(Native.qn_hardware_configuration(handle, revision, id, JsonSerializer.Serialize<object?>(configuration switch
        {
            StationSettings c => c with { AccessGates = c.AccessGates ?? [] },
            TransferSettings c => c with { AccessGates = c.AccessGates ?? [] },
            _ => configuration
        }, HardwareJson), out var error), error);

    }

    public void HardwareCommand(ulong revision, ulong id, HardwareCommand command, double value = 0, ulong target = 0)

    {

        RequireOpen(); Check(Native.qn_hardware_command(handle, revision, id, (uint)command, value, target, out var error), error);

    }

    public void SetInfrastructure(ulong revision, IReadOnlyList<InfrastructureSettings> infrastructure, ulong nextId)

    {

        RequireOpen(); Check(Native.qn_infrastructure_configuration(handle, revision,

            JsonSerializer.Serialize(infrastructure.Select(c => c with { Railings = c.Railings ?? new(), AccessGates = c.AccessGates ?? [], WorldStartMeters = c.WorldStartMeters ?? [0,0,0], WorldEndMeters = c.WorldEndMeters ?? [3,0,1.5] }), HardwareJson), nextId, out var error), error);

    }

}
