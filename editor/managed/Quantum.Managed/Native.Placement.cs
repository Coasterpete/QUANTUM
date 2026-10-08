using System.Runtime.InteropServices;

namespace Quantum.Managed;

[StructLayout(LayoutKind.Sequential)]
internal struct NativePose { public double X, Y, Z, Qw, Qx, Qy, Qz; }
[StructLayout(LayoutKind.Sequential)]
internal struct NativePlacement
{
    public uint Size, Version;
    public ulong Revision, SelectionRevision, PresentationRevision;
    public uint Selected, Mode, Active, Preview;
    public NativePose Authored, Candidate;
    public double Pitch, Yaw, Roll;
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeTransformCommand
{
    public ulong Revision, SelectionRevision;
    public uint Target, Mode, Axis, Reserved;
}
[StructLayout(LayoutKind.Sequential)]
public struct TransformIntent
{
    public ulong Id, Revision, SelectionRevision;
    public uint Phase, Target, Mode, Axis;
    public double Amount;
}
internal static partial class Native
{
    [LibraryImport(Library)] internal static partial int qn_placement_snapshot(SessionHandle session, ref NativePlacement snapshot, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_placement_edit(SessionHandle session, ulong revision, uint operation,
        double x, double y, double z, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_track_start_select(SessionHandle session, ulong revision, uint selected, uint mode, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_transform_begin(SessionHandle session, in NativeTransformCommand command, out ulong token, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_transform_preview(SessionHandle session, ulong token, double amount, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_transform_finish(SessionHandle session, ulong token, uint commit, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_transform_intent(SessionHandle session, out TransformIntent intent, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_viewport_cancel_transform(SessionHandle session, out NativeError error);
}
