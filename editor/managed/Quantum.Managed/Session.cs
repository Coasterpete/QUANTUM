using System.Collections.ObjectModel;

namespace Quantum.Managed;

public enum SessionCommand : uint { Play = 1, Pause, Reset, SetInitialSpeed, Undo, Redo, SetRegionLength }
public enum ViewportAction : uint { Orbit = 1, Pan, Zoom, Frame, Move, Focus, FrameSupport, PickSupport, FrameTrackStart, TransformDown, TransformMove, TransformUp, TransformCancel }
public enum ProfileOperation : uint { BeginValue, EndValue, TransitionType, Split, Remove, MoveBegin, MoveEnd }
public enum DeviceKind : uint { Launch, Brake, Transport, ChainLift, CableLift, Station, Block, Transfer, Access }
public sealed record TransitionChoice(uint Type, string Name) { public override string ToString() => Name; }
public sealed record ProfileSegment(uint OwnerKind, ulong OwnerId, uint Channel, uint Id, uint TransitionType,
    uint Unit, double Begin, double End, double ValueBegin, double ValueEnd, ReadOnlyCollection<double> Samples)
{
    public string ChannelName => Channel == 3 ? "Acceleration" : Unit == 1
        ? Channel == 1 ? "Normal G" : "Lateral G" : Channel switch { 0 => "Roll", 1 => "Pitch", _ => "Yaw" };
    public string Units => Unit switch { 0 => "deg/m", 1 => "G", _ => "m/s²" };
    public string Summary => $"{ChannelName} · #{Id} · {Begin:F2} → {End:F2} m";
}
public sealed record TrackDevice(ulong Id, string Name, uint Kind, bool Enabled, bool Custom,
    double Start, double End, double Acceleration, double MaximumForce)
{
    public string Label => $"{(Name.Length == 0 ? "Device" : Name)} · #{Id} · {((DeviceKind)Kind).ToString()}";
    public string Summary => $"{Start:F2} → {End:F2} m · {(Enabled ? "Enabled" : "Disabled")} · {MaximumForce:F0} N limit";
    public override string ToString() => Label;
}
public sealed class QuantumException(int code, string message) : Exception(message)
{
    public int Code { get; } = code;
}
public sealed record Region(uint Index, uint Kind, uint Construction, double StartStation, double Length, double EndStation,
    double RollDegrees, double PitchDegrees, double YawDegrees,
    double StartX, double StartY, double StartZ, double EndX, double EndY, double EndZ)
{
    public string Name => $"Region {Index + 1:00}";
    public string Type => Construction switch { 1 => "Planar arc", 2 => "Force driven", _ => "Rate profiles" };
    public string Summary => $"{Type}  ·  {Length:F2} m";
    public string Station => $"{StartStation:F2} → {EndStation:F2} m";
    public string Start => $"{StartX:F2}, {StartY:F2}, {StartZ:F2} m";
    public string End => $"{EndX:F2}, {EndY:F2}, {EndZ:F2} m";
    public string Rotation => Kind == 0 ? $"{RollDegrees:F2}° / {PitchDegrees:F2}° / {YawDegrees:F2}°" : "Defined by construction";
}
public sealed record SessionSnapshot(ulong Revision, ulong Tick, double Speed, double Station,
    double InitialSpeed, uint CarCount, double TrainMass, double TotalLength, uint Playback,
    bool Available, bool Dirty, bool CanUndo, bool CanRedo, double PhysicsMs, ReadOnlyCollection<Region> Route,
    ReadOnlyCollection<ProfileSegment> Profiles, ReadOnlyCollection<TrackDevice> Devices, SupportSnapshot Supports, PlacementSnapshot Placement, HardwareSnapshot Hardware)
{
    public double TimeSeconds => Tick / 240.0;
    public string PlaybackLabel => Available ? Playback switch { 1 => "Playing", 2 => "Paused", _ => "Stopped" } : "Preview unavailable";
}

// Snapshot/command calls have one controller owner. Viewport methods may run
// concurrently on the UI thread; the native session serializes CPU access.
public sealed unsafe partial class Session : IDisposable
{
    private readonly SessionHandle handle;
    private NativeRegion[] regionBuffer = new NativeRegion[1];
    private ReadOnlyCollection<Region> route = Array.AsReadOnly(Array.Empty<Region>());
    private ulong routeRevision;
    private ulong authoringRevision;
    private NativeProfile[] profileBuffer = [];
    private NativeDevice[] deviceBuffer = [];
    private NativeProfile[] dragBuffer = [];
    private ReadOnlyCollection<ProfileSegment> profiles = Array.AsReadOnly(Array.Empty<ProfileSegment>());
    private ReadOnlyCollection<TrackDevice> devices = Array.AsReadOnly(Array.Empty<TrackDevice>());
    public ReadOnlyCollection<TransitionChoice> TransitionTypes { get; }
    private bool disposed;
    public Session()
    {
        if (nint.Size != 8 || Native.qn_abi_version() != 5 || sizeof(NativeSnapshot) != 104 || sizeof(NativeRegion) != 112 || sizeof(NativeCommand) != 24
            || sizeof(NativeProfile) != 328 || sizeof(NativeDevice) != 184 || sizeof(NativeAuthoringSnapshot) != 24 || sizeof(NativeProfileCommand) != 48
            || sizeof(NativeSupportSnapshot) != 64 || sizeof(NativeSupportStructure) != 272 || sizeof(NativeSupportNode) != 88
            || sizeof(NativeSupportMember) != 1096 || sizeof(NativeSupportConnection) != 520 || sizeof(NativeSupportCommand) != 88
            || sizeof(NativePlacement) != 184 || sizeof(NativePose) != 56 || sizeof(NativeTransformCommand) != 32 || sizeof(TransformIntent) != 48)
            throw new NotSupportedException("QUANTUM requires a matching ABI 5 in a 64-bit process. Rebuild and stage the native runtime.");
        Check(Native.qn_create(out var id, out var error), error);
        handle = new SessionHandle(id);
        try
        {
            var sizing = Native.qn_transition_types(null, 0, out var count, out error);
            if (sizing != 3) Check(sizing, error);
            var catalog = new NativeTransitionType[count];
            fixed (NativeTransitionType* buffer = catalog)
                Check(Native.qn_transition_types(buffer, count, out _, out error), error);
            TransitionTypes = Array.AsReadOnly(catalog.Select(t => new TransitionChoice(t.Type, t.Label())).ToArray());
        }
        catch { handle.Dispose(); throw; }
    }
    private static void Check(int result, NativeError error)
    {
        if (result != 0) throw new QuantumException(result, error.ToString());
    }
    private void RequireOpen() => ObjectDisposedException.ThrowIf(disposed, this);
    public void New() { RequireOpen(); Check(Native.qn_new(handle, out var error), error); }
    public void Open(string path) { RequireOpen(); ArgumentException.ThrowIfNullOrEmpty(path); Check(Native.qn_open(handle, path, out var error), error); }
    public void Save(string path) { RequireOpen(); ArgumentException.ThrowIfNullOrEmpty(path); Check(Native.qn_save(handle, path, out var error), error); }
    public SessionSnapshot Snapshot()
    {
        RequireOpen();
        NativeSnapshot summary = new() { Size = (uint)sizeof(NativeSnapshot), Version = 1 };
        for (var attempt = 0; attempt < 3; ++attempt)
        {
            int result;
            NativeError error;
            fixed (NativeRegion* buffer = regionBuffer)
                result = Native.qn_snapshot(handle, ref summary, buffer, (uint)regionBuffer.Length, routeRevision, out error);
            if (result == 3) { regionBuffer = new NativeRegion[checked((int)summary.RegionCount)]; continue; }
            Check(result, error);
            if (routeRevision != summary.Revision)
            {
                var mapped = new Region[summary.RegionCount];
                for (var i = 0; i < mapped.Length; ++i)
                {
                    var r = regionBuffer[i];
                    mapped[i] = new(r.Index, r.Kind, r.Construction, r.StartStation, r.Length, r.EndStation,
                        r.Roll, r.Pitch, r.Yaw, r.StartX, r.StartY, r.StartZ, r.EndX, r.EndY, r.EndZ);
                }
                route = Array.AsReadOnly(mapped);
                routeRevision = summary.Revision;
            }
            RefreshAuthoring(summary.Revision);
            return new(summary.Revision, summary.Tick, summary.Speed, summary.Station, summary.InitialSpeed,
                summary.CarCount, summary.TrainMass, summary.TotalLength, summary.Playback, summary.Available != 0,
                summary.Dirty != 0, summary.CanUndo != 0, summary.CanRedo != 0, summary.PhysicsMs, route, profiles, devices, ReadSupports(summary.Revision), ReadPlacement(), ReadHardware(summary.Revision));
        }
        throw new InvalidOperationException("Document changed repeatedly during snapshot sizing.");
    }
    public void Command(SessionCommand command, ulong revision = 0, double value = 0, uint regionIndex = 0)
    {
        RequireOpen();
        NativeCommand request = new() { Type = (uint)command, ExpectedRevision = revision, Value = value, RegionIndex = regionIndex };
        Check(Native.qn_command(handle, in request, out var error), error);
    }
    private void RefreshAuthoring(ulong revision)
    {
        if (authoringRevision == revision) return;
        NativeAuthoringSnapshot summary = new() { Size = (uint)sizeof(NativeAuthoringSnapshot), Version = 1 };
        for (var attempt = 0; attempt < 3; ++attempt)
        {
            int result; NativeError error;
            fixed (NativeProfile* profileData = profileBuffer)
            fixed (NativeDevice* deviceData = deviceBuffer)
                result = Native.qn_authoring_snapshot(handle, ref summary, profileData, (uint)profileBuffer.Length,
                    deviceData, (uint)deviceBuffer.Length, authoringRevision, out error);
            if (result == 3)
            {
                profileBuffer = new NativeProfile[checked((int)summary.ProfileCount)];
                deviceBuffer = new NativeDevice[checked((int)summary.DeviceCount)];
                continue;
            }
            Check(result, error);
            if (summary.Revision != revision) throw new InvalidOperationException("Document changed during authoring snapshot.");
            profiles = MapProfiles(profileBuffer, summary.ProfileCount);
            devices = Array.AsReadOnly(deviceBuffer.Take((int)summary.DeviceCount).Select(d => new TrackDevice(
                d.Id, d.Label(), d.Kind, d.Enabled != 0, d.Custom != 0, d.Start, d.End, d.Acceleration, d.MaximumForce)).ToArray());
            authoringRevision = summary.Revision; return;
        }
        throw new InvalidOperationException("Authoring snapshot capacity changed repeatedly.");
    }
    public void EditProfile(ProfileSegment target, ulong revision, ProfileOperation operation, double value = 0, uint transitionType = 0)
    {
        RequireOpen();
        NativeProfileCommand command = new() { OwnerKind = target.OwnerKind, OwnerId = target.OwnerId,
            Channel = target.Channel, SegmentId = target.Id, ExpectedRevision = revision, Operation = (uint)operation,
            Value = value, TransitionType = transitionType };
        Check(Native.qn_profile_command(handle, in command, out var error), error);
    }
    public void SetDeviceAcceleration(ulong device, ulong revision, bool custom, double acceleration)
    {
        RequireOpen(); Check(Native.qn_device_acceleration(handle, device, revision, custom ? 1u : 0u, acceleration, out var error), error);
    }
    private static ReadOnlyCollection<ProfileSegment> MapProfiles(NativeProfile[] buffer, uint count) =>
        Array.AsReadOnly(buffer.Take((int)count).Select(p => new ProfileSegment(
            p.OwnerKind, p.OwnerId, p.Channel, p.SegmentId, p.TransitionType, p.Unit, p.Begin, p.End,
            p.ValueBegin, p.ValueEnd, p.CopySamples())).ToArray());
    public ulong BeginProfileDrag(ProfileSegment target, ulong revision, ProfileOperation operation)
    {
        RequireOpen();
        NativeProfileCommand command = new() { OwnerKind = target.OwnerKind, OwnerId = target.OwnerId,
            Channel = target.Channel, SegmentId = target.Id, ExpectedRevision = revision, Operation = (uint)operation };
        Check(Native.qn_profile_drag_begin(handle, in command, out var token, out var error), error);
        return token;
    }
    public ReadOnlyCollection<ProfileSegment> PreviewProfileDrag(ulong token, double value, double snapIncrement = 0)
    {
        RequireOpen();
        for (var attempt = 0; attempt < 2; ++attempt)
        {
            int result; uint count; NativeError error;
            fixed (NativeProfile* buffer = dragBuffer)
                result = Native.qn_profile_drag_preview(handle, token, value, snapIncrement, buffer, (uint)dragBuffer.Length, out count, out error);
            if (result == 3) { dragBuffer = new NativeProfile[count]; continue; }
            Check(result, error);
            return MapProfiles(dragBuffer, count);
        }
        throw new InvalidOperationException("Drag preview capacity changed repeatedly.");
    }
    public void FinishProfileDrag(ulong token, bool commit)
    {
        RequireOpen(); Check(Native.qn_profile_drag_finish(handle, token, commit ? 1u : 0u, out var error), error);
    }
    public ulong CreateDevice(ulong revision, DeviceKind kind)
    {
        RequireOpen(); Check(Native.qn_device_create(handle, revision, (uint)kind, out var device, out var error), error);
        return device;
    }
    public void DeleteDevice(ulong revision, ulong device)
    {
        RequireOpen(); Check(Native.qn_device_delete(handle, revision, device, out var error), error);
    }
    public void SetDeviceProperties(ulong revision, ulong device, bool enabled, double start, double end, double force)
    {
        RequireOpen(); Check(Native.qn_device_properties(handle, revision, device, enabled ? 1u : 0u, start, end, force, out var error), error);
    }
    public void SelectDevice(ulong revision, ulong device)
    {
        RequireOpen(); Check(Native.qn_select_device(handle, revision, device, out var error), error);
    }
    public void Update(double seconds) { RequireOpen(); Check(Native.qn_update(handle, seconds, out var error), error); }
    public void OpenViewport(string? descriptor, nint window)
    {
        RequireOpen();
        var platform = descriptor switch { "HWND" => 1u, "XID" or "X11" => 2u,
            _ => throw new NotSupportedException($"Unsupported native viewport host: {descriptor}") };
        Check(Native.qn_viewport_open(handle, platform, (ulong)window, out var error), error);
    }
    public bool RenderViewport(uint width, uint height)
    {
        RequireOpen();
        var result = Native.qn_viewport_frame(handle, width, height, out var error);
        if (result == 6) return false;
        Check(result, error);
        return true;
    }
    public void CloseViewport() { RequireOpen(); Check(Native.qn_viewport_close(handle, out var error), error); }
    public uint TakeViewportShortcuts()
    {
        RequireOpen();
        var result = Native.qn_viewport_shortcuts(handle, out var keys, out var error);
        if (result == 6) return 0;
        Check(result, error);
        return keys;
    }
    public void SetViewportPresentation(uint mode) { RequireOpen(); Check(Native.qn_viewport_presentation(handle, mode, out var error), error); }
    public void Camera(ViewportAction action, double x = 0, double y = 0, double z = 0, double seconds = 0)
    { RequireOpen(); Check(Native.qn_viewport_input(handle, (uint)action, x, y, z, seconds, out var error), error); }
    public void Dispose()
    {
        if (disposed) return;
        Check(Native.qn_destroy((ulong)handle.DangerousGetHandle(), out var error), error);
        handle.SetHandleAsInvalid();
        handle.Dispose();
        disposed = true;
    }
}
