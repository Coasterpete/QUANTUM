using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    public WorkbenchLayout Layout { get; }
    public bool IsOperationsWorkspace => WorkspaceMode == 3;
    public string ViewportHeading => IsTrainWorkspace ? "TRAIN PREVIEW" : "DESIGN VIEW";
    public string PlaybackSummary => Snapshot is { } s ? $"{s.PlaybackLabel} · {s.Speed:F2} m/s" : "Loading…";
    public bool IsTrackStartSelected => Snapshot?.Placement.Selected == true;
    public bool ShowRegionInspector => HasSelection && !IsTrackStartSelected;
    public bool ShowSupportEmpty => !HasSupportStructure && !InspectInfrastructure;
    public int SupportsBrowserTab { get => InspectInfrastructure ? 1 : 0; set => InspectInfrastructure = value == 1; }
    public bool HasAccelerationOrResponse => HasAccelerationProfileHardware || HasSpeedControllerHardware;
    public EditorCommand OpenHardwareProfileCommand { get; private set; } = null!;
    public EditorCommand OpenOperationsCommand { get; private set; } = null!;
    public EditorCommand NewInfrastructureDraftCommand { get; private set; } = null!;
    public EditorCommand ToggleDockCommand { get; private set; } = null!;
    public EditorCommand OpenTimelineCommand { get; private set; } = null!;
    public EditorCommand ResetLayoutCommand { get; private set; } = null!;
    public EditorCommand RevertCarCommand { get; private set; } = null!;
    public EditorCommand RevertTrainDefaultsCommand { get; private set; } = null!;
    public EditorCommand RevertHardwareCommand { get; private set; } = null!;
    private bool inspectInfrastructure;
    public bool InspectInfrastructure
    {
        get => inspectInfrastructure;
        set { inspectInfrastructure = value; Changed(); Changed(nameof(SupportsBrowserTab)); Changed(nameof(SupportHeading)); Changed(nameof(SupportContext)); Changed(nameof(ShowSupportEmpty)); Changed(nameof(ShowSupportStructure)); Changed(nameof(ShowSupportNode)); Changed(nameof(ShowSupportMember)); }
    }
    public bool ShowSupportStructure => !InspectInfrastructure && HasSupportStructure && !HasSupportNode && !HasSupportMember;
    public bool ShowSupportNode => !InspectInfrastructure && HasSupportNode;
    public bool ShowSupportMember => !InspectInfrastructure && HasSupportMember;
    public EditorCommand InspectInfrastructureCommand { get; private set; } = null!;
    private string hardwareFilter = "";
    public string HardwareFilter { get => hardwareFilter; set { hardwareFilter = value ?? ""; Changed(); Changed(nameof(FilteredHardware)); } }
    public IReadOnlyList<TrackDevice> FilteredHardware => Snapshot?.Devices.Where(d =>
        HardwareFilter.Length == 0 || d.Name.Contains(HardwareFilter, StringComparison.OrdinalIgnoreCase)
        || ((DeviceKind)d.Kind).ToString().Contains(HardwareFilter, StringComparison.OrdinalIgnoreCase)
        || d.Summary.Contains(HardwareFilter, StringComparison.OrdinalIgnoreCase)).ToArray() ?? [];
    public TrackDevice? SelectedBrowserHardware { get => SelectedDevice; set { if (value is not null) SelectedDevice = value; } }
    private string carBaseline = "", defaultsBaseline = "", configurationBaseline = "", placementBaseline = "", accelerationBaseline = "";
    public bool CarPending => HasCar && CarStamp() != carBaseline;
    public bool TrainDefaultsPending => DefaultsStamp() != defaultsBaseline;
    public bool HardwareConfigurationPending => HasDevice && ConfigurationStamp() != configurationBaseline;
    public bool AlignmentPending => SelectedAlignment is { } a && (AlignmentName != a.Name
        || AlignmentOffset != Number(a.LateralOffsetMeters) || AlignmentKind != a.Destination);
    public bool HardwarePlacementPending => HasDevice && PlacementStamp() != placementBaseline;
    public bool HardwareAccelerationPending => HasDevice && AccelerationStamp() != accelerationBaseline;
    public string HardwarePendingStatus => !HasDevice ? "" : string.Join(" · ", new[] {
        HardwareConfigurationPending ? "● Configuration modified" : null,
        HardwarePlacementPending ? "● Placement modified" : null,
        HardwareAccelerationPending ? "● Profile mode modified" : null }.Where(s => s is not null));
    private string CarStamp() => string.Join("\u001f", new[] { TrainCarName, TrainCarRole, TrainCarAsset }
        .Concat(CarFields.Select(f => f.Value)).Concat(CarAlignmentFields.Select(f => f.Value)));
    private string DefaultsStamp() => string.Join("\u001f", new[] { AuthoredTrainName, DefaultConnectorLength }.Concat(TrainDefaultFields.Select(f => f.Value)));
    private string PlacementStamp() => $"{DeviceStart}\u001f{DeviceEnd}\u001f{DeviceForce}\u001f{DeviceEnabled}";
    private string AccelerationStamp() => $"{DeviceMode}\u001f{DeviceAcceleration}";
    private string ConfigurationStamp() => string.Join("\u001f", new object?[] {
        HardwareSpeed, HardwareTarget, HardwareStop, HardwareAlignmentTolerance, HardwareStoppedTolerance,
        HardwareReturn, HardwareBlock, HardwareGates, HardwareDirection, HardwareHolding, HardwareAntiRollback,
        HardwareAvailable, HardwareRequireLocked, GateRole, GateMotion, GateAngle, GateTravel, GateDuration,
        GateLateral, GateVertical, GateYaw, GateInitiallyOpen, GateInitiallyLocked, GateInitiallyInterlocked,
        GateMesh, GateInfill, GateWidth, GateHeight, SelectedHomeAlignment?.Id });
    private void NotifyPending()
    {
        foreach (var name in new[] { nameof(CarPending), nameof(TrainDefaultsPending), nameof(HardwareConfigurationPending),
            nameof(HardwarePlacementPending), nameof(HardwareAccelerationPending), nameof(HardwarePendingStatus), nameof(AlignmentPending) })
            PropertyChanged?.Invoke(this, new(name));
    }
    private void TrainCellChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e) => NotifyPending();
    private void SeedHardwareBaselines()
    {
        configurationBaseline = ConfigurationStamp(); placementBaseline = PlacementStamp(); accelerationBaseline = AccelerationStamp(); NotifyPending();
    }
    private void InitializeWorkbenchCommands()
    {
        ToggleDockCommand = Add(() => { Layout.DockOpen = !Layout.DockOpen; return Task.CompletedTask; });
        OpenTimelineCommand = Add(() => { Layout.OpenTab("Timeline"); return Task.CompletedTask; });
        OpenHardwareProfileCommand = Add(() => { WorkspaceMode = 3; TimelineSource = 1; Layout.OpenTab("Profile"); return Task.CompletedTask; });
        OpenOperationsCommand = Add(() => { WorkspaceMode = 3; return Task.CompletedTask; });
        NewInfrastructureDraftCommand = Add(() => { SelectedInfrastructure = null; InspectInfrastructure = true; return Task.CompletedTask; }, () => IsSupportsWorkspace);
        ResetLayoutCommand = Add(() => { Layout.ResetWorkspace(); return Task.CompletedTask; });
        RevertCarCommand = Add(() => { SeedCar(selectedCar); return Task.CompletedTask; }, () => HasCar);
        RevertTrainDefaultsCommand = Add(() => { SeedTrainDefaults(); return Task.CompletedTask; });
        RevertHardwareCommand = Add(() => { SelectedDevice = selectedDevice; return Task.CompletedTask; }, () => HasDevice);
        InspectInfrastructureCommand = Add(() => { InspectInfrastructure = true; Layout.OpenTab("Infrastructure"); return Task.CompletedTask; }, () => IsSupportsWorkspace);
    }
    private void SeedTrainDefaults()
    {
        if (Snapshot?.Train.Configuration is not { } configuration) return;
        foreach (var field in TrainDefaultFields) field.PropertyChanged -= TrainCellChanged;
        AuthoredTrainName = configuration.Name; DefaultConnectorLength = configuration.DefaultConnectorLengthMeters.ToString("G", System.Globalization.CultureInfo.CurrentCulture);
        TrainDefaultFields = PhysicalFields(configuration.DefaultPhysical);
        foreach (var field in TrainDefaultFields) field.PropertyChanged += TrainCellChanged;
        defaultsBaseline = DefaultsStamp(); Changed(nameof(TrainDefaultFields)); NotifyPending();
    }
    private async void ClearTrackStartSelection()
    {
        if (Snapshot is not { } snapshot) return;
        try { await Controller.Submit(s => s.SelectTrackStart(snapshot.Revision, false, TransformMode.Select)); }
        catch (ObjectDisposedException) { }
        catch (TaskCanceledException) { }
        catch (QuantumException failure) { if (failure.Code != 8) Error = failure.Message; }
    }
}
