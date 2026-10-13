namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    // Copy the text, including invalid/unfinished input. Applying a different
    // group must not validate or silently discard this configuration draft.
    private Action PreserveHardwareConfiguration()
    {
        var draft = new
        {
            HardwareSpeed, HardwareTarget, HardwareStop, HardwareAlignmentTolerance, HardwareStoppedTolerance,
            HardwareReturn, HardwareBlock, HardwareGates, HardwareDirection, HardwareHolding, HardwareAntiRollback,
            HardwareAvailable, HardwareRequireLocked, GateRole, GateMotion, GateAngle, GateTravel, GateDuration,
            GateLateral, GateVertical, GateYaw, GateInitiallyOpen, GateInitiallyLocked, GateInitiallyInterlocked,
            GateMesh, GateInfill, GateWidth, GateHeight, TransferAlignments, SelectedHomeAlignment, SelectedAlignment,
            AlignmentName, AlignmentOffset, AlignmentKind
        };
        return () =>
        {
            HardwareSpeed = draft.HardwareSpeed; HardwareTarget = draft.HardwareTarget; HardwareStop = draft.HardwareStop;
            HardwareAlignmentTolerance = draft.HardwareAlignmentTolerance; HardwareStoppedTolerance = draft.HardwareStoppedTolerance;
            HardwareReturn = draft.HardwareReturn; HardwareBlock = draft.HardwareBlock; HardwareGates = draft.HardwareGates;
            HardwareDirection = draft.HardwareDirection; HardwareHolding = draft.HardwareHolding; HardwareAntiRollback = draft.HardwareAntiRollback;
            HardwareAvailable = draft.HardwareAvailable; HardwareRequireLocked = draft.HardwareRequireLocked;
            GateRole = draft.GateRole; GateMotion = draft.GateMotion; GateAngle = draft.GateAngle; GateTravel = draft.GateTravel;
            GateDuration = draft.GateDuration; GateLateral = draft.GateLateral; GateVertical = draft.GateVertical; GateYaw = draft.GateYaw;
            GateInitiallyOpen = draft.GateInitiallyOpen; GateInitiallyLocked = draft.GateInitiallyLocked; GateInitiallyInterlocked = draft.GateInitiallyInterlocked;
            GateMesh = draft.GateMesh; GateInfill = draft.GateInfill; GateWidth = draft.GateWidth; GateHeight = draft.GateHeight;
            TransferAlignments = draft.TransferAlignments; Changed(nameof(TransferAlignments));
            SelectedHomeAlignment = draft.SelectedHomeAlignment; SelectedAlignment = draft.SelectedAlignment;
            AlignmentName = draft.AlignmentName; AlignmentOffset = draft.AlignmentOffset; AlignmentKind = draft.AlignmentKind;
        };
    }
    private Action PreserveHardwarePlacement()
    {
        var start = DeviceStart; var end = DeviceEnd; var force = DeviceForce; var enabled = DeviceEnabled;
        return () => { DeviceStart = start; DeviceEnd = end; DeviceForce = force; DeviceEnabled = enabled; };
    }
    private Action PreserveHardwareAcceleration()
    {
        var acceleration = DeviceAcceleration; var mode = DeviceMode;
        return () => { DeviceAcceleration = acceleration; DeviceMode = mode; };
    }
    private async Task ApplyHardwareConfiguration()
    {
        var id = SelectedDevice?.Id;
        var restorePlacement = PreserveHardwarePlacement(); var restoreAcceleration = PreserveHardwareAcceleration();
        await Run(ApplyHardware);
        if (Error.Length == 0 && SelectedDevice?.Id == id) { restorePlacement(); restoreAcceleration(); }
    }
}
