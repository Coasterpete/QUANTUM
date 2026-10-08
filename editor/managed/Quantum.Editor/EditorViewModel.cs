using System.ComponentModel;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Windows.Input;
using Quantum.Managed;

namespace Quantum.Editor;

public sealed class EditorCommand(Func<Task> action, Func<bool> enabled) : ICommand
{
    public event EventHandler? CanExecuteChanged;
    public bool CanExecute(object? parameter) => enabled();
    public async void Execute(object? parameter) => await ExecuteAsync();
    public Task ExecuteAsync() => CanExecute(null) ? action() : Task.CompletedTask;
    public void Refresh() => CanExecuteChanged?.Invoke(this, EventArgs.Empty);
}
public enum UnsavedChoice { Cancel, Save, Discard }

public sealed partial class EditorViewModel : INotifyPropertyChanged
{
    public SessionController Controller { get; }
    public event PropertyChangedEventHandler? PropertyChanged;
    public Func<bool, Task<string?>>? PickFile { get; set; }
    public Func<Task<UnsavedChoice>>? ConfirmUnsaved { get; set; }
    public SessionSnapshot? Snapshot { get; private set; }
    private Region? selected;
    private string length = "", initialSpeed = "", error = "", path = "";
    private bool busy;
    private ProfileSegment? selectedProfile;
    private TrackDevice? selectedDevice;
    private int timelineSource;
    private string profileBegin = "", profileEnd = "", profileDistance = "", deviceAcceleration = "";
    private TransitionChoice? selectedTransition;
    private int deviceMode;
    private string deviceStart = "", deviceEnd = "", deviceForce = "";
    private bool deviceEnabled;
    public IReadOnlyList<TransitionChoice> TransitionTypes => Controller.Session.TransitionTypes;
    public IReadOnlyList<ProfileSegment> VisibleProfiles { get; private set; } = [];
    public EditorCommand ApplyProfileBeginCommand { get; }
    public EditorCommand ApplyProfileEndCommand { get; }
    public EditorCommand ApplyTransitionCommand { get; }
    public EditorCommand SplitProfileCommand { get; }
    public EditorCommand RemoveProfileCommand { get; }
    public EditorCommand MoveProfileBeginCommand { get; }
    public EditorCommand MoveProfileEndCommand { get; }
    public EditorCommand ApplyDeviceCommand { get; }
    public EditorCommand CreateLaunchCommand { get; }
    public EditorCommand CreateBrakeCommand { get; }
    public EditorCommand DeleteDeviceCommand { get; }
    public EditorCommand ApplyDevicePropertiesCommand { get; }
    private readonly List<EditorCommand> commands = [];
    public EditorCommand NewCommand { get; }
    public EditorCommand OpenCommand { get; }
    public EditorCommand SaveCommand { get; }
    public EditorCommand SaveAsCommand { get; }
    public EditorCommand UndoCommand { get; }
    public EditorCommand RedoCommand { get; }
    public EditorCommand PlayCommand { get; }
    public EditorCommand PauseCommand { get; }
    public EditorCommand ResetCommand { get; }
    public EditorCommand ApplyLengthCommand { get; }
    public EditorCommand ApplySpeedCommand { get; }
    public EditorCommand FrameCommand { get; }
    public EditorViewModel(SessionController controller)
    {
        Controller = controller;
        NewCommand = Add(() => Run(async () => { if (await AllowReplacement()) { await Controller.Submit(s => s.New()); path = ""; } }));
        OpenCommand = Add(() => Run(async () =>
        {
            if (!await AllowReplacement()) return;
            var chosen = PickFile is null ? null : await PickFile(false);
            if (chosen is null) return;
            await Controller.Submit(s => s.Open(chosen)); path = chosen;
        }));
        SaveCommand = Add(() => Run(async () => { await Save(false); }));
        SaveAsCommand = Add(() => Run(async () => { await Save(true); }));
        UndoCommand = Add(() => Send(SessionCommand.Undo), () => Snapshot?.CanUndo == true);
        RedoCommand = Add(() => Send(SessionCommand.Redo), () => Snapshot?.CanRedo == true);
        PlayCommand = Add(() => Send(SessionCommand.Play), () => Snapshot is { Available: true, Playback: not 1 });
        PauseCommand = Add(() => Send(SessionCommand.Pause), () => Snapshot?.Playback == 1);
        ResetCommand = Add(() => Send(SessionCommand.Reset), () => Snapshot?.Available == true);
        ApplyLengthCommand = Add(() => Run(async () =>
        {
            if (Selected is not { } region || Snapshot is not { } snapshot) return;
            var value = ParsePositive(Length, "Region length");
            await Controller.Submit(s => s.Command(SessionCommand.SetRegionLength, snapshot.Revision, value, region.Index));
        }), () => Selected is not null);
        ApplySpeedCommand = Add(() => Run(async () =>
        {
            if (Snapshot is not { } snapshot) return;
            var value = ParsePositive(InitialSpeed, "Initial speed", allowZero: true);
            await Controller.Submit(s => s.Command(SessionCommand.SetInitialSpeed, snapshot.Revision, value));
        }), () => Snapshot is not null);
        FrameCommand = Add(() => Run(() => { Controller.Session.Camera(ViewportAction.Frame); return Task.CompletedTask; }));
        ApplyProfileBeginCommand = Add(() => EditProfile(ProfileOperation.BeginValue, ProfileBegin), () => HasProfile);
        ApplyProfileEndCommand = Add(() => EditProfile(ProfileOperation.EndValue, ProfileEnd), () => HasProfile);
        ApplyTransitionCommand = Add(() => EditProfile(ProfileOperation.TransitionType), () => HasProfile && SelectedTransition is not null);
        SplitProfileCommand = Add(() => EditProfile(ProfileOperation.Split, ProfileDistance), () => HasProfile);
        RemoveProfileCommand = Add(() => EditProfile(ProfileOperation.Remove), () => CanRemoveProfile);
        MoveProfileBeginCommand = Add(() => EditProfile(ProfileOperation.MoveBegin, ProfileDistance), () => CanMoveProfileBegin);
        MoveProfileEndCommand = Add(() => EditProfile(ProfileOperation.MoveEnd, ProfileDistance), () => CanMoveProfileEnd);
        ApplyDeviceCommand = Add(() => Run(async () =>
        {
            if (SelectedDevice is not { } device || Snapshot is not { } snapshot) return;
            var acceleration = ParsePositive(DeviceAcceleration, "Acceleration magnitude");
            await Controller.Submit(s => s.SetDeviceAcceleration(device.Id, snapshot.Revision, DeviceMode == 1, acceleration));
        }), () => SelectedDevice is not null);
        CreateLaunchCommand = Add(() => CreateDevice(DeviceKind.Launch), () => Snapshot?.TotalLength > 0);
        CreateBrakeCommand = Add(() => CreateDevice(DeviceKind.Brake), () => Snapshot?.TotalLength > 0);
        DeleteDeviceCommand = Add(() => Run(async () =>
        {
            if (SelectedDevice is { } device && Snapshot is { } snapshot)
                await Controller.Submit(s => s.DeleteDevice(snapshot.Revision, device.Id));
        }), () => HasDevice);
        ApplyDevicePropertiesCommand = Add(() => Run(async () =>
        {
            if (SelectedDevice is not { } device || Snapshot is not { } snapshot) return;
            var start = ParsePositive(DeviceStart, "Start station", true);
            var end = ParsePositive(DeviceEnd, "End station", true);
            var force = ParsePositive(DeviceForce, "Force limit");
            var enabled = DeviceEnabled;
            await Controller.Submit(s => s.SetDeviceProperties(snapshot.Revision, device.Id, enabled, start, end, force));
        }), () => HasDevice);
    }
    private EditorCommand Add(Func<Task> action, Func<bool>? enabled = null)
    {
        var command = new EditorCommand(action, () => !Busy && !InteractionActive && (enabled?.Invoke() ?? true));
        commands.Add(command); return command;
    }
    public bool Busy { get => busy; private set { busy = value; Changed(); Changed(nameof(CanInteract)); RefreshCommands(); Changed(nameof(Activity)); } }
    public bool CanInteract => !Busy;
    public string Activity => Busy ? "Working…" : InteractionActive ? "Profile preview" : "Ready";
    public string Error { get => error; set { error = value; Changed(); Changed(nameof(HasError)); } }
    public bool HasError => Error.Length != 0;
    public string Title => $"QUANTUM — {DocumentName}{(Snapshot?.Dirty == true ? " *" : "")}";
    public string DocumentName => path.Length == 0 ? "Untitled" : System.IO.Path.GetFileName(path);
    public string RouteSummary => Snapshot is { } s ? $"{s.Route.Count} Regions · {s.TotalLength:F2} m" : "Loading document…";
    public string TrainSummary => Snapshot is { } s ? $"{s.CarCount} cars · {s.TrainMass:F0} kg" : "—";
    public string Telemetry => Snapshot is { } s ? $"{s.PlaybackLabel}   |   {s.TimeSeconds:F3} s   |   tick {s.Tick}   |   {s.Speed:F2} m/s   |   station {s.Station:F2} m" : "Preparing native preview…";
    public string Length { get => length; set { length = value; Changed(); } }
    public string InitialSpeed { get => initialSpeed; set { initialSpeed = value; Changed(); } }
    public Region? Selected
    {
        get => selected;
        set { if (selected?.Index != value?.Index) CancelForSelection(); selected = value; Changed(); Changed(nameof(HasSelection)); Length = value?.Length.ToString("G12", CultureInfo.CurrentCulture) ?? ""; RefreshProfiles(); RefreshCommands(); }
    }
    public bool HasSelection => Selected is not null;
    public int TimelineSource { get => timelineSource; set { if (timelineSource != value) CancelForSelection(); timelineSource = value; Changed(); Changed(nameof(IsDeviceTimeline)); RefreshProfiles(); SynchronizeDeviceSelection(); } }
    public bool IsDeviceTimeline => TimelineSource == 1;
    public string TimelineContext => IsDeviceTimeline ? SelectedDevice?.Summary ?? "No authored Track Devices in this document"
        : Selected is { } region ? $"{region.Name} · track {region.Station} · local distance in metres" : "Select an authored Region";
    public bool HasProfile => SelectedProfile is not null;
    public bool HasDevice => SelectedDevice is not null;
    private IEnumerable<ProfileSegment> SelectedChannel => VisibleProfiles.Where(p => p.Channel == SelectedProfile?.Channel);
    public bool CanMoveProfileBegin => SelectedProfile is { } p && SelectedChannel.Any(other => other.Begin < p.Begin);
    public bool CanMoveProfileEnd => SelectedProfile is { } p && SelectedChannel.Any(other => other.End > p.End);
    public bool CanRemoveProfile => SelectedChannel.Count() > 1;
    public string ProfileDescription => SelectedProfile is { } p
        ? $"{p.ChannelName} · #{p.Id} · {TransitionTypes.FirstOrDefault(t => t.Type == p.TransitionType)?.Name} · {(p.ValueBegin == p.ValueEnd ? "flat endpoints" : p.ValueEnd > p.ValueBegin ? "rising" : "falling")}"
        : "Select a segment in the graph or list";
    public string ProfileBegin { get => profileBegin; set { profileBegin = value; Changed(); } }
    public string ProfileEnd { get => profileEnd; set { profileEnd = value; Changed(); } }
    public string ProfileDistance { get => profileDistance; set { profileDistance = value; Changed(); } }
    public string DeviceAcceleration { get => deviceAcceleration; set { deviceAcceleration = value; Changed(); } }
    public int DeviceMode { get => deviceMode; set { deviceMode = value; Changed(); } }
    public string DeviceStart { get => deviceStart; set { deviceStart = value; Changed(); } }
    public string DeviceEnd { get => deviceEnd; set { deviceEnd = value; Changed(); } }
    public string DeviceForce { get => deviceForce; set { deviceForce = value; Changed(); } }
    public bool DeviceEnabled { get => deviceEnabled; set { deviceEnabled = value; Changed(); } }
    public TransitionChoice? SelectedTransition { get => selectedTransition; set { selectedTransition = value; Changed(); RefreshCommands(); } }
    public TrackDevice? SelectedDevice
    {
        get => selectedDevice;
        set
        {
            if (selectedDevice?.Id != value?.Id) CancelForSelection();
            selectedDevice = value; Changed(); Changed(nameof(HasDevice));
            DeviceMode = value?.Custom == true ? 1 : 0;
            DeviceAcceleration = value?.Acceleration.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            DeviceStart = value?.Start.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            DeviceEnd = value?.End.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            DeviceForce = value?.MaximumForce.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            DeviceEnabled = value?.Enabled == true;
            RefreshProfiles(); RefreshCommands();
            SynchronizeDeviceSelection();
        }
    }
    public ProfileSegment? SelectedProfile
    {
        get => selectedProfile;
        set
        {
            if (!SameTarget(selectedProfile, value)) CancelForSelection();
            selectedProfile = value; Changed(); Changed(nameof(HasProfile)); Changed(nameof(ProfileDescription));
            Changed(nameof(CanMoveProfileBegin)); Changed(nameof(CanMoveProfileEnd)); Changed(nameof(CanRemoveProfile));
            ProfileBegin = value?.ValueBegin.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            ProfileEnd = value?.ValueEnd.ToString("G12", CultureInfo.CurrentCulture) ?? "";
            ProfileDistance = value is null ? "" : ((value.Begin + value.End) / 2).ToString("G12", CultureInfo.CurrentCulture);
            SelectedTransition = value is null ? null : TransitionTypes.FirstOrDefault(t => t.Type == value.TransitionType);
            RefreshCommands();
        }
    }
    private void RefreshProfiles(ProfileSegment? preserve = null)
    {
        var previous = preserve ?? SelectedProfile;
        VisibleProfiles = Snapshot?.Profiles.Where(p => IsDeviceTimeline ? p.OwnerKind == 1 && p.OwnerId == SelectedDevice?.Id
            : p.OwnerKind == 0 && p.OwnerId == Selected?.Index).ToArray() ?? [];
        Changed(nameof(VisibleProfiles)); Changed(nameof(TimelineContext));
        Changed(nameof(GraphProfiles));
        SelectedProfile = VisibleProfiles.FirstOrDefault(p => previous is not null && p.OwnerKind == previous.OwnerKind
            && p.OwnerId == previous.OwnerId && p.Channel == previous.Channel && p.Id == previous.Id)
            ?? VisibleProfiles.FirstOrDefault(p => previous is not null && p.OwnerKind == previous.OwnerKind
                && p.OwnerId == previous.OwnerId && p.Channel == previous.Channel)
            ?? VisibleProfiles.FirstOrDefault();
    }
    private Task EditProfile(ProfileOperation operation, string? input = null) => Run(async () =>
    {
        if (SelectedProfile is not { } target || Snapshot is not { } snapshot) return;
        var value = 0.0;
        if (input is not null && (!double.TryParse(input, NumberStyles.Float, CultureInfo.CurrentCulture, out value) || !double.IsFinite(value)))
            throw new ArgumentException("Profile input must be a finite number.");
        var type = SelectedTransition?.Type ?? 0;
        await Controller.Submit(s => s.EditProfile(target, snapshot.Revision, operation, value, type));
    });
    public void Publish(SessionSnapshot next)
    {
        if (InteractionActive && endingDrag && Busy) return;
        if (InteractionActive && next.Revision != dragRevision)
        { CancelForSelection(); Error = "Document changed. The Timeline drag was canceled."; }
        var changedRoute = Snapshot?.Revision != next.Revision;
        // ItemsSource replacement can synchronously clear ListBox selection.
        // Capture its identity before notifying the Route binding.
        var selectedIndex = Selected?.Index ?? 0;
        var profile = SelectedProfile;
        var deviceId = SelectedDevice?.Id;
        Snapshot = next;
        Changed(nameof(Snapshot)); Changed(nameof(Title)); Changed(nameof(RouteSummary)); Changed(nameof(TrainSummary)); Changed(nameof(Telemetry));
        if (changedRoute)
        {
            Selected = next.Route.FirstOrDefault(r => r.Index == selectedIndex) ?? next.Route.FirstOrDefault();
            InitialSpeed = next.InitialSpeed.ToString("G12", CultureInfo.CurrentCulture);
            SelectedDevice = next.Devices.FirstOrDefault(d => d.Id == deviceId) ?? next.Devices.FirstOrDefault();
            RefreshProfiles(profile);
        }
        RefreshCommands();
    }
    private static double ParsePositive(string text, string name, bool allowZero = false)
    {
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out var value)
            || !double.IsFinite(value) || value < 0 || (!allowZero && value == 0))
            throw new ArgumentException($"{name} must be a finite {(allowZero ? "nonnegative" : "positive")} number.");
        return value;
    }
    private Task Send(SessionCommand command) => Run(() => Controller.Submit(s => s.Command(command)));
    private async Task Run(Func<Task> operation)
    {
        if (Busy) return;
        Busy = true; Error = "";
        try { await operation(); if (Controller.Latest is { } latest) Publish(latest); Changed(nameof(DocumentName)); Changed(nameof(Title)); }
        catch (Exception failure)
        {
            Error = failure.Message;
            if (Controller.Latest is { } latest) Publish(latest);
            SelectedProfile = selectedProfile;
            SelectedDevice = selectedDevice;
        }
        finally { Busy = false; }
    }
    private async Task<bool> Save(bool choosePath)
    {
        var destination = !choosePath && path.Length != 0 ? path : PickFile is null ? null : await PickFile(true);
        if (destination is null) return false;
        if (System.IO.Path.GetExtension(destination).Length == 0) destination += ".quantum";
        await Controller.Submit(s => s.Save(destination)); path = destination;
        return true;
    }
    private async Task<bool> AllowReplacement()
    {
        // Barrier against a pending command before deciding whether to prompt.
        await Controller.Submit(_ => { });
        if (Controller.Latest?.Dirty != true) return true;
        var choice = ConfirmUnsaved is null ? UnsavedChoice.Cancel : await ConfirmUnsaved();
        return choice == UnsavedChoice.Discard || (choice == UnsavedChoice.Save && await Save(false));
    }
    public async Task<bool> ConfirmClose()
    {
        if (Busy) { Error = "Wait for the current operation to finish before closing."; return false; }
        if (InteractionActive) await EndProfileDragAsync(false);
        Busy = true;
        try { return await AllowReplacement(); }
        catch (Exception failure) { Error = failure.Message; return false; }
        finally { Busy = false; }
    }
    private void RefreshCommands() { foreach (var command in commands) command.Refresh(); }
    private Task CreateDevice(DeviceKind kind) => Run(async () =>
    {
        if (Snapshot is not { } snapshot) return;
        ulong id = 0;
        await Controller.Submit(s => id = s.CreateDevice(snapshot.Revision, kind));
        if (Controller.Latest is { } latest)
        { Publish(latest); SelectedDevice = latest.Devices.First(d => d.Id == id); }
    });
    private async void SynchronizeDeviceSelection()
    {
        if (Snapshot is not { } snapshot) return;
        var id = IsDeviceTimeline ? SelectedDevice?.Id ?? 0 : 0;
        try { await Controller.Submit(s => s.SelectDevice(snapshot.Revision, id)); }
        catch (QuantumException failure) { if (failure.Code != 8) Error = failure.Message; }
        catch (ObjectDisposedException) { }
        catch (TaskCanceledException) { }
    }
    private void Changed([CallerMemberName] string? name = null) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
