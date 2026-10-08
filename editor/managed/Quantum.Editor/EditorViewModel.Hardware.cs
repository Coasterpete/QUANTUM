using System.Globalization;
using Quantum.Managed;
namespace Quantum.Editor;
public sealed record HardwareChoice(DeviceKind Kind, string Name) { public override string ToString() => Name; }
public sealed partial class EditorViewModel
{
    public IReadOnlyList<HardwareChoice> HardwareChoices { get; } = [
        new(DeviceKind.Launch, "LSM launch"), new(DeviceKind.Brake, "Brake / holding brake"),
        new(DeviceKind.Transport, "Drive tires / transport"), new(DeviceKind.ChainLift, "Chain lift"),
        new(DeviceKind.CableLift, "Cable lift"), new(DeviceKind.Station, "Station zone"),
        new(DeviceKind.Block, "Block / occupancy sensor"), new(DeviceKind.Transfer, "Transfer table"), new(DeviceKind.Access, "Gate / door / barrier")];
    public HardwareChoice SelectedHardwareChoice { get; set; } = new(DeviceKind.Transport, "Drive tires / transport");
    public EditorCommand CreateHardwareCommand { get; private set; } = null!;
    public EditorCommand ApplyHardwareCommand { get; private set; } = null!;
    public EditorCommand DispatchCommand { get; private set; } = null!;
    public EditorCommand TransferMoveCommand { get; private set; } = null!;
    public EditorCommand OpenGateCommand { get; private set; } = null!;
    public EditorCommand CloseGateCommand { get; private set; } = null!;
    public EditorCommand LockGateCommand { get; private set; } = null!;
    public EditorCommand UnlockGateCommand { get; private set; } = null!;
    public EditorCommand AddAlignmentCommand { get; private set; } = null!;
    public EditorCommand ApplyAlignmentCommand { get; private set; } = null!;
    public EditorCommand DeleteAlignmentCommand { get; private set; } = null!;
    public EditorCommand HardwareAvailableCommand { get; private set; } = null!;
    public EditorCommand HardwareUnavailableCommand { get; private set; } = null!;
    private DeviceKind? HardwareKind => SelectedDevice is { } d ? (DeviceKind)d.Kind : null;
    public bool IsLaunchHardware => HardwareKind == DeviceKind.Launch;
    public bool IsBrakeHardware => HardwareKind == DeviceKind.Brake;
    public bool IsTransportHardware => HardwareKind == DeviceKind.Transport;
    public bool IsChainHardware => HardwareKind == DeviceKind.ChainLift;
    public bool IsCableHardware => HardwareKind == DeviceKind.CableLift;
    public bool IsStationHardware => HardwareKind == DeviceKind.Station;
    public bool IsBlockHardware => HardwareKind == DeviceKind.Block;
    public bool IsTransferHardware => HardwareKind == DeviceKind.Transfer;
    public bool IsAccessHardware => HardwareKind == DeviceKind.Access;
    public bool HasSpeedControllerHardware => IsTransportHardware || IsChainHardware || IsStationHardware || IsTransferHardware;
    public bool HasHardwareSpeed => IsTransportHardware || IsChainHardware || IsCableHardware || IsStationHardware || IsTransferHardware;
    public bool HasHardwareDirection => IsLaunchHardware || IsTransportHardware || IsAccessHardware;
    public bool HasGateReferences => IsStationHardware || IsTransferHardware;
    public bool HasForceHardware => HasDevice && !IsBlockHardware && !IsAccessHardware;
    public bool HasAccelerationProfileHardware => IsLaunchHardware || IsBrakeHardware || IsCableHardware;
    public bool ShowProfileEditor => !IsDeviceTimeline || HasAccelerationProfileHardware;
    public bool ShowHardwareState => IsDeviceTimeline && !HasAccelerationProfileHardware;
    public bool HasLiftHardware => IsChainHardware || IsCableHardware;
    public HardwareEntry? SelectedHardwareRuntime => Snapshot?.Hardware.Hardware.FirstOrDefault(h => h.Id == SelectedDevice?.Id);
    public string HardwareTelemetry => SelectedHardwareRuntime is { } h ? h.Telemetry
        + (IsStationHardware ? $"\nAligned {h.Aligned} · stopped {h.Stopped} · dispatch requested {h.DispatchRequested} / permitted {h.DispatchPermitted}" : "")
        + (IsTransferHardware ? $"\nAlignment #{h.Alignment} → #{h.TargetAlignment} · offset {h.Position:F2} m · locked {h.Locked} · movement permitted {h.MovementPermitted}" : "")
        + (IsAccessHardware ? $"\nOpen {h.Position:P0} · locked {h.Locked} · interlocked {h.Interlocked}" : "") : "Select ride hardware";
    public string HardwareContentDescription => IsStationHardware ? "Station positioning, stopping and dispatch interlocks"
        : IsTransferHardware ? "Transfer mechanism / storage alignments · alternate-track train routing pending"
        : IsAccessHardware ? "Operable access hardware · swing/slide mechanism"
        : IsBlockHardware ? "Bogie occupancy / entry / exit sensor" : "Native hardware commands and force telemetry";
    private string hardwareSpeed = "2";
    public string HardwareSpeed { get => hardwareSpeed; set { hardwareSpeed = value; Changed(); } }
    private string hardwareTarget = "";
    public string HardwareTarget { get => hardwareTarget; set { hardwareTarget = value; Changed(); } }
    private string hardwareStop = "0";
    public string HardwareStop { get => hardwareStop; set { hardwareStop = value; Changed(); } }
    private string hardwareAlignmentTolerance = ".15";
    public string HardwareAlignmentTolerance { get => hardwareAlignmentTolerance; set { hardwareAlignmentTolerance = value; Changed(); } }
    private string hardwareStoppedTolerance = ".05";
    public string HardwareStoppedTolerance { get => hardwareStoppedTolerance; set { hardwareStoppedTolerance = value; Changed(); } }
    private string hardwareReturn = "8";
    public string HardwareReturn { get => hardwareReturn; set { hardwareReturn = value; Changed(); } }
    private string hardwareBlock = "0";
    public string HardwareBlock { get => hardwareBlock; set { hardwareBlock = value; Changed(); } }
    private string hardwareGates = "";
    public string HardwareGates { get => hardwareGates; set { hardwareGates = value; Changed(); } }
    private int hardwareDirection = 0;
    public int HardwareDirection { get => hardwareDirection; set { hardwareDirection = value; Changed(); } }
    private bool hardwareHolding = false;
    public bool HardwareHolding { get => hardwareHolding; set { hardwareHolding = value; Changed(); } }
    private bool hardwareAntiRollback = true;
    public bool HardwareAntiRollback { get => hardwareAntiRollback; set { hardwareAntiRollback = value; Changed(); } }
    private bool hardwareAvailable = true;
    public bool HardwareAvailable { get => hardwareAvailable; set { hardwareAvailable = value; Changed(); } }
    private bool hardwareRequireLocked = true;
    public bool HardwareRequireLocked { get => hardwareRequireLocked; set { hardwareRequireLocked = value; Changed(); } }
    private int gateRole = 0;
    public int GateRole { get => gateRole; set { gateRole = value; Changed(); } }
    private int gateMotion = 0;
    public int GateMotion { get => gateMotion; set { gateMotion = value; Changed(); } }
    private string gateAngle = "90";
    public string GateAngle { get => gateAngle; set { gateAngle = value; Changed(); } }
    private string gateTravel = "1";
    public string GateTravel { get => gateTravel; set { gateTravel = value; Changed(); } }
    private string gateDuration = "1";
    public string GateDuration { get => gateDuration; set { gateDuration = value; Changed(); } }
    private string gateLateral = "2";
    public string GateLateral { get => gateLateral; set { gateLateral = value; Changed(); } }
    private string gateVertical = "0";
    public string GateVertical { get => gateVertical; set { gateVertical = value; Changed(); } }
    private string gateYaw = "0";
    public string GateYaw { get => gateYaw; set { gateYaw = value; Changed(); } }
    private bool gateInitiallyOpen = false;
    public bool GateInitiallyOpen { get => gateInitiallyOpen; set { gateInitiallyOpen = value; Changed(); } }
    private bool gateInitiallyLocked = true;
    public bool GateInitiallyLocked { get => gateInitiallyLocked; set { gateInitiallyLocked = value; Changed(); } }
    private bool gateInitiallyInterlocked = false;
    public bool GateInitiallyInterlocked { get => gateInitiallyInterlocked; set { gateInitiallyInterlocked = value; Changed(); } }
    private string gateMesh = "";
    public string GateMesh { get => gateMesh; set { gateMesh = value; Changed(); } }
    private string alignmentName = "";
    public string AlignmentName { get => alignmentName; set { alignmentName = value; Changed(); } }
    private string alignmentOffset = "0";
    public string AlignmentOffset { get => alignmentOffset; set { alignmentOffset = value; Changed(); } }
    private int alignmentKind = 0;
    public int AlignmentKind { get => alignmentKind; set { alignmentKind = value; Changed(); } }
    public IReadOnlyList<AlignmentSettings> TransferAlignments { get; private set; } = [];
    private AlignmentSettings? selectedAlignment, selectedHomeAlignment;
    private ulong nextAlignmentId = 3;
    public AlignmentSettings? SelectedHomeAlignment { get => selectedHomeAlignment; set { selectedHomeAlignment = value; Changed(); } }
    public AlignmentSettings? SelectedAlignment
    {
        get => selectedAlignment;
        set { selectedAlignment = value; Changed(); AlignmentName = value?.Name ?? "";
            AlignmentOffset = Number(value?.LateralOffsetMeters ?? 0); AlignmentKind = (int)(value?.Destination ?? 0); }
    }
    private static string Number(double value) => value.ToString("G12", CultureInfo.CurrentCulture);
    private static ulong Id(string text, string name) => ulong.TryParse(text, out var id) ? id : throw new ArgumentException($"{name} must be a whole nonnegative ID.");
    private ulong[] GateIds() => HardwareGates.Split([',', ' '], StringSplitOptions.RemoveEmptyEntries).Select(text => Id(text, "Gate ID")).ToArray();
    private int Direction => HardwareDirection == 0 ? 1 : -1;
    private void InitializeHardwareCommands()
    {
        CreateHardwareCommand = Add(() => CreateDevice(SelectedHardwareChoice.Kind), () => Snapshot?.TotalLength > 0);
        ApplyHardwareCommand = Add(() => Run(ApplyHardware), () => HasDevice && !IsBlockHardware);
        DispatchCommand = Add(() => RuntimeHardware(HardwareCommand.RequestDispatch), () => IsStationHardware);
        TransferMoveCommand = Add(() => RuntimeHardware(HardwareCommand.MoveTransfer, target: SelectedAlignment?.Id ?? 0), () => IsTransferHardware && SelectedAlignment is not null);
        OpenGateCommand = Add(() => RuntimeHardware(HardwareCommand.OpenAccess), () => IsAccessHardware);
        CloseGateCommand = Add(() => RuntimeHardware(HardwareCommand.CloseAccess), () => IsAccessHardware);
        LockGateCommand = Add(() => RuntimeHardware(HardwareCommand.LockAccess), () => IsAccessHardware);
        UnlockGateCommand = Add(() => RuntimeHardware(HardwareCommand.UnlockAccess), () => IsAccessHardware);
        HardwareAvailableCommand = Add(() => RuntimeHardware(HardwareCommand.SetAvailable, 1), () => HasLiftHardware);
        HardwareUnavailableCommand = Add(() => RuntimeHardware(HardwareCommand.SetAvailable, 0), () => HasLiftHardware);
        AddAlignmentCommand = Add(() => Run(async () =>
        {
            var next = nextAlignmentId;
            nextAlignmentId = checked(next + 1);
            var alignment = new AlignmentSettings(next, $"Storage {next - 1}", next * 2, 1);
            TransferAlignments = TransferAlignments.Append(alignment).ToArray(); SelectedAlignment = alignment;
            await ApplyHardware();
        }), () => IsTransferHardware);
        ApplyAlignmentCommand = Add(() => Run(async () =>
        {
            if (SelectedAlignment is not { } a) return;
            TransferAlignments = TransferAlignments.Select(row => row.Id == a.Id ? new AlignmentSettings(a.Id,
                AlignmentName, Finite(AlignmentOffset, "Alignment offset"), (uint)AlignmentKind) : row).ToArray();
            await ApplyHardware();
        }), () => IsTransferHardware && SelectedAlignment is not null);
        DeleteAlignmentCommand = Add(() => Run(async () =>
        {
            TransferAlignments = TransferAlignments.Where(a => a.Id != SelectedAlignment?.Id).ToArray();
            await ApplyHardware();
        }), () => IsTransferHardware && SelectedAlignment is not null && SelectedAlignment.Id != SelectedHomeAlignment?.Id);
    }
    private Task RuntimeHardware(HardwareCommand command, double value = 0, ulong target = 0) => Run(async () =>
    {
        if (Snapshot is { } snapshot && SelectedDevice is { } d)
            await Controller.Submit(s => s.HardwareCommand(snapshot.Revision, d.Id, command, value, target));
    });
    private Task ApplyHardware()
    {
        if (Snapshot is not { } snapshot || SelectedDevice is not { } d) return Task.CompletedTask;
        object configuration = HardwareKind switch
        {
            DeviceKind.Launch => new LaunchSettings(HardwareTarget.Trim().Length == 0 ? null : Finite(HardwareTarget, "Target speed"), Direction),
            DeviceKind.Brake => new BrakeSettings(Finite(HardwareTarget, "Trim speed"), HardwareHolding),
            DeviceKind.Transport => new TransportSettings(Finite(HardwareSpeed, "Transport speed"), Direction),
            DeviceKind.ChainLift => new ChainSettings(Finite(HardwareSpeed, "Chain speed"), HardwareAntiRollback),
            DeviceKind.CableLift => new CableSettings(Finite(HardwareSpeed, "Cable speed"), Finite(HardwareReturn, "Return duration"), HardwareAvailable),
            DeviceKind.Station => new StationSettings(Finite(HardwareStop, "Stop station"), Finite(HardwareAlignmentTolerance, "Alignment tolerance"),
                Finite(HardwareStoppedTolerance, "Stopped tolerance"), Finite(HardwareSpeed, "Transport speed"), Id(HardwareBlock, "Downstream block"), GateIds(), HardwareRequireLocked),
            DeviceKind.Transfer => new TransferSettings(TransferAlignments.ToArray(), SelectedHomeAlignment?.Id ?? 0,
                Finite(HardwareSpeed, "Movement speed"), Finite(HardwareStoppedTolerance, "Stopped tolerance"), GateIds(), nextAlignmentId),
            DeviceKind.Access => new AccessSettings((uint)GateRole, (uint)GateMotion, Direction, Finite(GateAngle, "Open angle"), Finite(GateTravel, "Travel distance"),
                Finite(GateDuration, "Movement duration"), Finite(GateLateral, "Lateral offset"), Finite(GateVertical, "Vertical offset"), Finite(GateYaw, "Yaw"),
                GateInitiallyOpen, GateInitiallyLocked, GateInitiallyInterlocked, GateMesh),
            _ => throw new ArgumentException("Select configurable hardware.")
        };
        return Controller.Submit(s => s.SetHardwareConfiguration(snapshot.Revision, d.Id, configuration));
    }
    private void SeedHardware()
    {
        var runtime = SelectedHardwareRuntime;
        if (runtime is not null) switch (HardwareKind)
        {
        case DeviceKind.Launch:
            var launch = runtime.Settings(new LaunchSettings()); HardwareTarget = launch.TargetSpeedMetersPerSecond is { } target ? Number(target) : ""; HardwareDirection = launch.Direction == 1 ? 0 : 1; break;
        case DeviceKind.Brake:
            var brake = runtime.Settings(new BrakeSettings()); HardwareTarget = Number(brake.TargetSpeedMetersPerSecond); HardwareHolding = brake.Holding; break;
        case DeviceKind.Transport:
            var transport = runtime.Settings(new TransportSettings()); HardwareSpeed = Number(transport.SpeedMetersPerSecond); HardwareDirection = transport.Direction == 1 ? 0 : 1; break;
        case DeviceKind.ChainLift:
            var chain = runtime.Settings(new ChainSettings()); HardwareSpeed = Number(chain.SpeedMetersPerSecond); HardwareAntiRollback = chain.AntiRollback; break;
        case DeviceKind.CableLift:
            var cable = runtime.Settings(new CableSettings()); HardwareSpeed = Number(cable.SpeedMetersPerSecond); HardwareReturn = Number(cable.ReturnSeconds); HardwareAvailable = cable.InitiallyAvailable; break;
        case DeviceKind.Station:
            var station = runtime.Settings(new StationSettings(0)); HardwareSpeed = Number(station.TransportSpeedMetersPerSecond); HardwareStop = Number(station.StopStationMeters);
            HardwareAlignmentTolerance = Number(station.AlignmentToleranceMeters); HardwareStoppedTolerance = Number(station.StoppedToleranceMetersPerSecond);
            HardwareBlock = station.DownstreamBlock.ToString(); HardwareGates = string.Join(", ", station.AccessGates ?? []); HardwareRequireLocked = station.RequireLockedGates; break;
        case DeviceKind.Transfer:
            var transfer = runtime.Settings(new TransferSettings([])); HardwareSpeed = Number(transfer.MovementSpeedMetersPerSecond); HardwareStoppedTolerance = Number(transfer.StoppedToleranceMetersPerSecond);
            HardwareGates = string.Join(", ", transfer.AccessGates ?? []);
            var selectedId = SelectedAlignment?.Id;
            nextAlignmentId = transfer.NextAlignmentId;
            TransferAlignments = transfer.Alignments; Changed(nameof(TransferAlignments));
            SelectedAlignment = TransferAlignments.FirstOrDefault(a => a.Id == selectedId) ?? TransferAlignments.FirstOrDefault();
            SelectedHomeAlignment = TransferAlignments.FirstOrDefault(a => a.Id == transfer.HomeAlignment); break;
        case DeviceKind.Access:
            var gate = runtime.Settings(new AccessSettings()); GateRole = (int)gate.Role; GateMotion = (int)gate.Motion; HardwareDirection = gate.TravelDirection == 1 ? 0 : 1;
            GateAngle = Number(gate.OpenAngleDegrees); GateTravel = Number(gate.TravelDistanceMeters); GateDuration = Number(gate.MovementSeconds);
            GateLateral = Number(gate.LateralOffsetMeters); GateVertical = Number(gate.VerticalOffsetMeters); GateYaw = Number(gate.YawDegrees);
            GateInitiallyOpen = gate.InitiallyOpen; GateInitiallyLocked = gate.InitiallyLocked; GateInitiallyInterlocked = gate.InitiallyInterlocked; GateMesh = gate.MeshAsset; break;
        }
        foreach (var name in new[] { nameof(IsLaunchHardware), nameof(IsBrakeHardware), nameof(IsTransportHardware), nameof(IsChainHardware), nameof(IsCableHardware),
            nameof(IsStationHardware), nameof(IsBlockHardware), nameof(IsTransferHardware), nameof(IsAccessHardware), nameof(HasHardwareSpeed), nameof(HasSpeedControllerHardware), nameof(HasHardwareDirection),
            nameof(HasGateReferences), nameof(HasForceHardware), nameof(HasAccelerationProfileHardware), nameof(HasLiftHardware), nameof(ShowProfileEditor), nameof(ShowHardwareState), nameof(HardwareContentDescription) }) Changed(name);
        Changed(nameof(HardwareTelemetry));
    }
}
