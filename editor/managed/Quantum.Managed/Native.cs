using System.Runtime.InteropServices;

namespace Quantum.Managed;

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeError
{
    public fixed byte Message[512];
    public override string ToString()
    {
        fixed (byte* text = Message) return Marshal.PtrToStringUTF8((nint)text) ?? "Native operation failed.";
    }
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeRegion
{
    public uint Index, Kind, Construction, Reserved;
    public double StartStation, Length, EndStation, Roll, Pitch, Yaw;
    public double StartX, StartY, StartZ, EndX, EndY, EndZ;
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSnapshot
{
    public uint Size, Version;
    public ulong Revision, Tick;
    public uint RegionCount, SampleCount, CarCount, Playback, Available, Dirty, CanUndo, CanRedo;
    public double Speed, Station, InitialSpeed, TrainMass, TotalLength, PhysicsMs;
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeCommand
{
    public uint Type, RegionIndex;
    public ulong ExpectedRevision;
    public double Value;
}
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeProfile
{
    public uint OwnerKind, Channel, SegmentId, TransitionType, Unit, Reserved;
    public ulong OwnerId;
    public double Begin, End, ValueBegin, ValueEnd;
    public fixed double Samples[33];
    public System.Collections.ObjectModel.ReadOnlyCollection<double> CopySamples()
    {
        var copy = new double[33];
        fixed (double* samples = Samples) new ReadOnlySpan<double>(samples, 33).CopyTo(copy);
        return Array.AsReadOnly(copy);
    }
}
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeDevice
{
    public ulong Id;
    public uint Kind, Enabled, Custom, Reserved;
    public double Start, End, Acceleration, MaximumForce;
    public fixed byte Name[128];
    public string Label() { fixed (byte* name = Name) return Marshal.PtrToStringUTF8((nint)name) ?? ""; }
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeAuthoringSnapshot
{
    public uint Size, Version;
    public ulong Revision;
    public uint ProfileCount, DeviceCount;
}
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeTransitionType
{
    public uint Type;
    public fixed byte Name[64];
    public string Label() { fixed (byte* name = Name) return Marshal.PtrToStringUTF8((nint)name) ?? ""; }
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeProfileCommand
{
    public uint OwnerKind, Channel, SegmentId, Operation, TransitionType, Reserved;
    public ulong OwnerId, ExpectedRevision;
    public double Value;
}
internal sealed class SessionHandle : SafeHandle
{
    public SessionHandle(ulong id) : base(nint.Zero, true) => SetHandle(checked((nint)id));
    public override bool IsInvalid => handle == nint.Zero;
    // Headless fallback only. The application explicitly closes its viewport
    // and joins the worker before Dispose; finalization cannot do UI shutdown.
    protected override bool ReleaseHandle() => Native.qn_destroy((ulong)handle, out _) == 0;
}

internal static unsafe partial class Native
{
    private const string Library = "QuantumNative";
    [LibraryImport(Library)] internal static partial uint qn_abi_version();
    [LibraryImport(Library)] internal static partial int qn_create(out ulong session, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_destroy(ulong session, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_new(SessionHandle session, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_open(SessionHandle session, string path, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_save(SessionHandle session, string path, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_snapshot(SessionHandle session,
        ref NativeSnapshot snapshot, NativeRegion* regions, uint capacity, ulong knownRevision, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_command(SessionHandle session, in NativeCommand command, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_authoring_snapshot(SessionHandle session,
        ref NativeAuthoringSnapshot snapshot, NativeProfile* profiles, uint profileCapacity,
        NativeDevice* devices, uint deviceCapacity, ulong knownRevision, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_transition_types(NativeTransitionType* types,
        uint capacity, out uint count, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_profile_command(SessionHandle session,
        in NativeProfileCommand command, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_device_acceleration(SessionHandle session,
        ulong device, ulong revision, uint custom, double acceleration, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_profile_drag_begin(SessionHandle session,
        in NativeProfileCommand command, out ulong token, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_profile_drag_preview(SessionHandle session,
        ulong token, double value, double snap, NativeProfile* profiles, uint capacity, out uint count, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_profile_drag_finish(SessionHandle session,
        ulong token, uint commit, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_device_create(SessionHandle session,
        ulong revision, uint kind, out ulong device, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_device_delete(SessionHandle session,
        ulong revision, ulong device, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_device_properties(SessionHandle session,
        ulong revision, ulong device, uint enabled, double start, double end, double force, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_select_device(SessionHandle session,
        ulong revision, ulong device, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_update(SessionHandle session, double seconds, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_open(SessionHandle session, uint platform, ulong window, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_frame(SessionHandle session, uint width, uint height, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_close(SessionHandle session, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_shortcuts(SessionHandle session, out uint shortcuts, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_presentation(SessionHandle session, uint mode, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_input(SessionHandle session,
        uint action, double x, double y, double z, double seconds, out NativeError error);
}
